#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/sensor2_rotating.py  (timebase 版)

- 串口行读取 → 聚合 CH0..CH{N-1} 成一帧
- 统一时间戳：MonoNS/EstNS（以及其秒制版本 MonoS/EstS）
- 按日期/大小轮换 CSV；支持按行自动 flush
"""

import os, sys, csv, time, signal, threading
from datetime import datetime, timedelta
from uwnav.io.timebase import stamp  # ★ 统一时间基：返回 (mono_ns:int, est_ns:int)
from uwnav.drivers.imu.WitHighModbus.serial_io_tools import InputListenerThread, SerialReaderThread
from uwnav.io.data_paths import get_sensor_outdir


# ===================== 配置 =====================
SERIAL_PORT   = '/dev/ttyUSB0'   # Windows: 'COM3'
SERIAL_BAUD   = 115200
N_CHANNELS    = 16
FILENAME_PREF = "motor_data"
OUT_DIR       = None
ROTATE_MB     = 50
KEEP_DAYS     = 7
FLUSH_SEC     = 60   # 仍保留后台定时 flush
# —— 调试与写盘可视化 ——
DEBUG_RAW_SNIFF = 20     # 仅打印前 N 行原始数据，0 关闭
DEBUG_RAW_ECHO  = False  # True 打印所有原始行（高频会刷屏）
DEBUG_WRITE_LOG = True   # 每写入一帧就打印一条提示
AUTOFLUSH_EVERY = 1      # 每写入多少“帧”自动 flush；1=每帧都刷
JOIN_TIMEOUT_S  = 2.0    # 退出时 join 的超时时间（秒）
# ===============================================

# ---------- 日志写入：按日期/大小轮换 ----------
class RollingCSVWriter:
    """线程安全的CSV轮换写入器：跨天或达大小阈值即换新文件；支持按行自动flush。"""
    def __init__(self, out_dir, prefix, header, rotate_mb=5, keep_days=7, autoflush_every=1):
        self.out_dir = out_dir or os.path.dirname(os.path.abspath(__file__))
        self.prefix, self.header = prefix, header
        self.rotate_mb, self.keep_days = rotate_mb, keep_days
        self.autoflush_every = max(0, int(autoflush_every))
        os.makedirs(self.out_dir, exist_ok=True)
        self._file = None
        self._csv  = None
        self._open_date = None
        self._lock = threading.Lock()
        self._row_counter = 0
        self.current_path = None
        self._open_new_file()

    def _build_filepath(self, now: datetime) -> str:
        name = f"{self.prefix}_{now.strftime('%Y%m%d_%H%M%S')}.csv"
        return os.path.join(self.out_dir, name)

    def _open_new_file(self):
        if self._file:
            try: self._file.flush(); self._file.close()
            except Exception: pass
        now = datetime.now()
        path = self._build_filepath(now)
        self._file = open(path, 'a', newline='', encoding='utf-8')
        self._csv  = csv.writer(self._file)
        self._csv.writerow(self.header); self._file.flush()
        self._open_date = now.date()
        self.current_path = path
        self._row_counter = 0
        self._cleanup_old_files()
        print(f"[LOG] 新日志文件：{path}")

    def _need_rotate_nolock(self) -> bool:
        if datetime.now().date() != self._open_date:
            return True
        if self.rotate_mb and self.rotate_mb > 0 and self._file:
            try:
                import os as _os
                if _os.fstat(self._file.fileno()).st_size >= self.rotate_mb * 1024 * 1024:
                    return True
            except Exception:
                pass
        return False

    def _cleanup_old_files(self):
        if not self.keep_days or self.keep_days <= 0: return
        cutoff = datetime.now() - timedelta(days=self.keep_days)
        for fname in os.listdir(self.out_dir):
            if not (fname.startswith(self.prefix + "_") and fname.endswith(".csv")):
                continue
            fpath = os.path.join(self.out_dir, fname)
            try:
                if datetime.fromtimestamp(os.path.getmtime(fpath)) < cutoff:
                    os.remove(fpath)
                    print(f"[LOG] 清理旧文件：{fname}")
            except Exception:
                pass

    def writerow(self, row):
        with self._lock:
            if self._need_rotate_nolock():
                self._open_new_file()
            self._csv.writerow(row)
            self._row_counter += 1
            if self.autoflush_every and (self._row_counter % self.autoflush_every == 0):
                try: self._file.flush()
                except Exception: pass

    def flush(self):
        with self._lock:
            try: self._file.flush()
            except Exception: pass

    def close(self):
        with self._lock:
            try: self._file.flush(); self._file.close()
            except Exception: pass


# ---------- 解析与聚合 ----------
class ChannelBuffer:
    """聚合 CH0..CH{N-1} 的一帧数据，齐则返回一行，否则返回 None。"""
    def __init__(self, n_channels: int):
        self.n = n_channels
        self._buf = {f'CH{i}': None for i in range(self.n)}
        self._lock = threading.Lock()

    @staticmethod
    def parse_line(line: str):
        # 兼容“多空格/Tab”分隔
        if not line: return None
        s = line.rstrip('\r\n').strip()
        if not s: return None
        toks = s.split()
        if len(toks) < 2: return None
        channel_part, value_part = toks[0], toks[-1]
        if ':' not in channel_part: return None
        ch_key, _ = channel_part.split(':', 1)
        ch_key = ch_key.strip().upper()
        if not ch_key.startswith('CH'): return None
        return ch_key, value_part

    def update(self, ch_key: str, value_str: str):
        if ch_key not in self._buf: return None
        with self._lock:
            self._buf[ch_key] = value_str
            if all(self._buf[f'CH{i}'] is not None for i in range(self.n)):
                row = [self._buf[f'CH{i}'] for i in range(self.n)]
                for i in range(self.n):
                    self._buf[f'CH{i}'] = None
                return row
        return None


# ---------- 周期性刷盘 ----------
class PeriodicFlusher(threading.Thread):
    def __init__(self, writer: RollingCSVWriter, shutdown_event: threading.Event, interval_sec: int):
        super().__init__(daemon=True)
        self.writer = writer
        self.shutdown_event = shutdown_event
        self.interval = interval_sec

    def run(self):
        while not self.shutdown_event.wait(timeout=self.interval):
            try:
                self.writer.flush()
                print("[LOG] 周期性刷新完成")
            except Exception as e:
                print(f"[LOG] 刷新失败：{e}")


# ---------- 主流程 ----------
def main():
    # OUT_DIR 视为数据根目录（可为 None）
    # 最终结果形如：<data_root>/YYYY-MM-DD/volt/
    sensor_outdir = get_sensor_outdir("volt", OUT_DIR)

    header  = ['Timestamp'] + [f'CH{i}' for i in range(N_CHANNELS)]
    writer  = RollingCSVWriter(str(sensor_outdir), FILENAME_PREF, header,
                               rotate_mb=ROTATE_MB, keep_days=KEEP_DAYS,
                               autoflush_every=AUTOFLUSH_EVERY)

    shutdown = threading.Event()
    buf = ChannelBuffer(N_CHANNELS)
    sniff_count = {"n": 0}
    frames_written = {"n": 0}


    def on_serial_line(payload: str):
        # ---- 调试：打印原始数据 ----
        if DEBUG_RAW_ECHO or (DEBUG_RAW_SNIFF and sniff_count["n"] < DEBUG_RAW_SNIFF):
            print(f"[RAW ] {repr(payload)}")
            sniff_count["n"] += 1

        parsed = buf.parse_line(payload)
        if not parsed:
            return
        ch_key, value_str = parsed
        row_values = buf.update(ch_key, value_str)
        if row_values is not None:
            mono_ns, est_ns = stamp()          # ★ 统一取样时刻
            mono_s, est_s   = mono_ns/1e9, est_ns/1e9
            writer.writerow([mono_ns, est_ns, mono_s, est_s] + row_values)
            frames_written["n"] += 1
            if DEBUG_WRITE_LOG:
                print(f"[WRITE] est_s={est_s:.6f}  写入1帧（{N_CHANNELS}通道）  → {writer.current_path}")

    # 串口读取线程
    serial_th = SerialReaderThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUD,
        shutdown_event=shutdown,
        data_callback=on_serial_line,
        line_mode=True,
        encoding='utf-8',
        eol=b'\n'  # 兼容 \r\n
    )
    serial_th.start()
    print(f"[SERIAL] 已启动：{SERIAL_PORT} @ {SERIAL_BAUD}")

    # 周期性刷盘
    flush_th = PeriodicFlusher(writer, shutdown, FLUSH_SEC)
    flush_th.start()

    # 输入监听（按 's' 退出）
    input_th = InputListenerThread(shutdown_event=shutdown, shutdown_callback=shutdown.set)
    input_th.start()
    print("程序运行中。按 's' 回车退出，或 Ctrl+C。")

    def _signal_handler(sig, frame):
        print("\n[SYS] 收到退出信号，准备关闭…")
        shutdown.set()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        while not shutdown.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        shutdown.set()

    # 收尾
    print("[SYS] 正在收尾…")
    serial_th.join(timeout=JOIN_TIMEOUT_S)
    flush_th.join(timeout=JOIN_TIMEOUT_S)
    writer.flush()
    writer.close()
    print(f"[STAT] 总写入帧数：{frames_written['n']}  输出文件：{writer.current_path}")
    print("[SYS] 已退出。")

if __name__ == "__main__":
    main()
