# sensor2_rotating.py
# coding: UTF-8
import os
import sys
import csv
import time
import signal
import threading
from datetime import datetime, timedelta

# 复用你之前的工具线程（已提供）
from serial_io_tools import InputListenerThread, SerialReaderThread

# ===================== 配置 =====================
SERIAL_PORT   = '/dev/ttyUSB1'   # Windows: 'COM3'
SERIAL_BAUD   = 115200
N_CHANNELS    = 16               # CH0..CH15
FILENAME_PREF = "motor_data"     # 输出文件前缀
OUT_DIR       = None             # None 表示脚本所在目录
ROTATE_MB     = 50                # 单文件最大大小（MB）；<=0 表示不按大小轮换
KEEP_DAYS     = 7                # 保留最近 N 天日志文件
FLUSH_SEC     = 60               # 周期性 flush 到磁盘（秒）
# ===============================================


# ---------- 日志写入：按日期/大小轮换 ----------
class RollingCSVWriter:
    """
    按“日期变化”或“超过大小阈值”轮换 CSV。
    文件名示例： motor_data_20250902_153025.csv
    - 每个新文件都会写入表头
    - 清理 KEEP_DAYS 之前的旧文件
    """
    def __init__(self, out_dir, prefix, header, rotate_mb=5, keep_days=7):
        self.out_dir   = out_dir or os.path.dirname(os.path.abspath(__file__))
        self.prefix    = prefix
        self.header    = header
        self.rotate_mb = rotate_mb
        self.keep_days = keep_days

        os.makedirs(self.out_dir, exist_ok=True)
        self._file = None
        self._csv  = None
        self._open_date = None  # 用于跨天轮换
        self._open_new_file()

    def _build_filepath(self, ts: datetime) -> str:
        name = f"{self.prefix}_{ts.strftime('%Y%m%d_%H%M%S')}.csv"
        return os.path.join(self.out_dir, name)

    def _open_new_file(self):
        if self._file:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
        now = datetime.now()
        path = self._build_filepath(now)
        self._file = open(path, 'a', newline='', encoding='utf-8')
        self._csv  = csv.writer(self._file)
        self._csv.writerow(self.header)
        self._file.flush()
        self._open_date = now.date()
        self._cleanup_old_files()
        print(f"[LOG] 新日志文件：{path}")

    def _need_rotate(self) -> bool:
        # 日期变化
        if datetime.now().date() != self._open_date:
            return True
        # 大小超过阈值
        if self.rotate_mb and self.rotate_mb > 0:
            try:
                sz = self._file.tell()
                if sz >= self.rotate_mb * 1024 * 1024:
                    return True
            except Exception:
                pass
        return False

    def _cleanup_old_files(self):
        if not self.keep_days or self.keep_days <= 0:
            return
        cutoff = datetime.now() - timedelta(days=self.keep_days)
        for fname in os.listdir(self.out_dir):
            if not fname.startswith(self.prefix + "_") or not fname.endswith(".csv"):
                continue
            fpath = os.path.join(self.out_dir, fname)
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(fpath))
                if mtime < cutoff:
                    os.remove(fpath)
                    print(f"[LOG] 清理旧文件：{fname}")
            except Exception:
                pass

    def writerow(self, row):
        if self._need_rotate():
            self._open_new_file()
        self._csv.writerow(row)

    def flush(self):
        try:
            self._file.flush()
        except Exception:
            pass

    def close(self):
        try:
            self._file.flush()
            self._file.close()
        except Exception:
            pass


# ---------- 解析与聚合 ----------
class ChannelBuffer:
    """聚合 CH0..CH{N-1} 的一帧数据，齐则返回一行，否则返回 None。"""
    def __init__(self, n_channels: int):
        self.n = n_channels
        self._buf = {f'CH{i}': None for i in range(self.n)}
        self._lock = threading.Lock()

    @staticmethod
    def parse_line(line: str):
        """
        预期格式：'CH7:2025\\t1.631A' 或 'CH8:2200\\t1.773V'
        返回 (channel_key, value_str) 或 None
        """
        if not line:
            return None
        s = line.strip()
        parts = s.split('\t')
        if len(parts) != 2:
            # print(f"[WARN] 格式不符：{s}")
            return None
        channel_part, value_part = parts[0].strip(), parts[1].strip()
        if ':' not in channel_part:
            return None
        ch_key, _adc = channel_part.split(':', 1)
        ch_key = ch_key.strip().upper()  # 'CH7'
        if not ch_key.startswith('CH'):
            return None
        # 保留单位字符串（如 '1.631A' / '1.773V'）
        value_str = value_part
        return ch_key, value_str

    def update(self, ch_key: str, value_str: str):
        """更新某通道；若齐则返回列表[CH0..CH{N-1}]，并清空缓冲；否则返回 None。"""
        if ch_key not in self._buf:
            # print(f"[INFO] 忽略未知通道：{ch_key}")
            return None
        with self._lock:
            self._buf[ch_key] = value_str
            if all(self._buf[f'CH{i}'] is not None for i in range(self.n)):
                row = [self._buf[f'CH{i}'] for i in range(self.n)]
                # 清空下一轮
                for i in range(self.n):
                    self._buf[f'CH{i}'] = None
                return row
        return None


# ---------- 周期性刷盘 ----------
class PeriodicFlusher(threading.Thread):
    """每 FLUSH_SEC 秒调用 writer.flush()。"""
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
    out_dir = OUT_DIR or os.path.dirname(os.path.abspath(__file__))
    header  = ['Timestamp'] + [f'CH{i}' for i in range(N_CHANNELS)]
    writer  = RollingCSVWriter(out_dir, FILENAME_PREF, header, rotate_mb=ROTATE_MB, keep_days=KEEP_DAYS)

    shutdown = threading.Event()
    buf = ChannelBuffer(N_CHANNELS)

    # 串口到 CSV 的回调
    def on_serial_line(payload: str):
        """
        payload 来自 SerialReaderThread：
        - 文本优先，decode 失败则是十六进制字符串
        - 我们只处理文本行
        """
        parsed = buf.parse_line(payload)
        if not parsed:
            return
        ch_key, value_str = parsed
        row_values = buf.update(ch_key, value_str)
        if row_values is not None:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            writer.writerow([ts] + row_values)

    # 线程：串口读取（按行）
    serial_th = SerialReaderThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUD,
        shutdown_event=shutdown,
        data_callback=on_serial_line,
        line_mode=True,           # 按行读取
        encoding='utf-8'
    )
    serial_th.start()
    print(f"[SERIAL] 已启动：{SERIAL_PORT} @ {SERIAL_BAUD}")

    # 线程：周期性刷新
    flush_th = PeriodicFlusher(writer, shutdown, FLUSH_SEC)
    flush_th.start()

    # 线程：输入监听（按 's' 回车退出）
    input_th = InputListenerThread(shutdown_event=shutdown, shutdown_callback=shutdown.set)
    input_th.start()
    print("程序运行中。按 's' 回车退出，或 Ctrl+C。")

    # 信号处理
    def _signal_handler(sig, frame):
        print("\n[SYS] 收到退出信号，准备关闭…")
        shutdown.set()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # 主循环等待退出
    try:
        while not shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        shutdown.set()

    # 收尾
    print("[SYS] 正在收尾…")
    serial_th.join()
    input_th.join()
    flush_th.join()
    writer.close()
    print("[SYS] 已退出。")

if __name__ == "__main__":
    main()
