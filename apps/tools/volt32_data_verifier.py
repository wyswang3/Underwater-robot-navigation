#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/sensor2_rotating.py  (timebase 统一版)

功能：
- 串口读取 16/32 通道电压采集板行数据
- 聚合 CH0..CH{N-1} 成一帧
- 使用统一时间基 uwnav.io.timebase：
      ts = stamp("volt0", SensorKind.OTHER)
      → ts.host_time_ns
      → ts.corrected_time_ns
- 输出 CSV 列格式：
      MonoNS, EstNS, MonoS, EstS, CH0..CH{N-1}
"""

import os
import sys
import csv
import time
import signal
import threading
from datetime import datetime, timedelta

from uwnav.io.timebase import stamp, SensorKind
from uwnav.drivers.imu.WitHighModbus.serial_io_tools import (
    InputListenerThread,
    SerialReaderThread,
)
from uwnav.io.data_paths import get_sensor_outdir


# ===================== 配置 =====================
SERIAL_PORT   = "/dev/ttyUSB0"
SERIAL_BAUD   = 115200
N_CHANNELS    = 16

FILENAME_PREF = "motor_data"
OUT_DIR       = None

ROTATE_MB        = 50
KEEP_DAYS        = 7
FLUSH_SEC        = 60
DEBUG_RAW_SNIFF  = 20
DEBUG_RAW_ECHO   = False
DEBUG_WRITE_LOG  = True
AUTOFLUSH_EVERY  = 1
JOIN_TIMEOUT_S   = 2.0
# =================================================


# ================= RollingCSVWriter =================
class RollingCSVWriter:
    """线程安全的 CSV 轮换写入器：跨天/大小轮换；可自动 flush。"""

    def __init__(self, out_dir, prefix, header,
                 rotate_mb=50, keep_days=7, autoflush_every=1):
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        self.prefix = prefix
        self.header = header
        self.rotate_mb   = rotate_mb
        self.keep_days   = keep_days
        self.autoflush_every = max(1, autoflush_every)

        self._file = None
        self._csv  = None
        self._lock = threading.Lock()
        self._open_date = None
        self._row_counter = 0
        self.current_path = None

        self._open_new_file()

    def _new_filepath(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.out_dir, f"{self.prefix}_{ts}.csv")

    def _open_new_file(self):
        if self._file:
            try:
                self._file.flush()
                self._file.close()
            except:
                pass

        path = self._new_filepath()
        self._file = open(path, "a", newline="", encoding="utf-8")
        self._csv  = csv.writer(self._file)

        self._csv.writerow(self.header)
        self._file.flush()

        self.current_path = path
        self._open_date   = datetime.now().date()
        self._row_counter = 0

        self._cleanup_old()
        print(f"[LOG] 创建新文件：{path}")

    def _need_rotate(self):
        now = datetime.now()
        if now.date() != self._open_date:
            return True

        try:
            size = os.fstat(self._file.fileno()).st_size
            if size >= self.rotate_mb * 1024 * 1024:
                return True
        except:
            pass
        return False

    def _cleanup_old(self):
        cutoff = datetime.now() - timedelta(days=self.keep_days)
        for fname in os.listdir(self.out_dir):
            if not fname.startswith(self.prefix + "_"):
                continue
            f = os.path.join(self.out_dir, fname)
            try:
                if datetime.fromtimestamp(os.path.getmtime(f)) < cutoff:
                    os.remove(f)
                    print(f"[LOG] 清理旧文件：{fname}")
            except:
                pass

    def writerow(self, row):
        with self._lock:
            if self._need_rotate():
                self._open_new_file()
            self._csv.writerow(row)
            self._row_counter += 1
            if self._row_counter % self.autoflush_every == 0:
                try:
                    self._file.flush()
                except:
                    pass

    def flush(self):
        with self._lock:
            try:
                self._file.flush()
            except:
                pass

    def close(self):
        with self._lock:
            try:
                self._file.flush()
                self._file.close()
            except:
                pass


# ================= ChannelBuffer =================
class ChannelBuffer:
    """聚合 CH0..CH{N-1}，齐则返回一行值列表。"""

    def __init__(self, n):
        self.n = n
        self._buf = {f"CH{i}": None for i in range(n)}
        self._lock = threading.Lock()

    @staticmethod
    def parse_line(line):
        if not line:
            return None
        s = line.strip()
        if not s:
            return None

        toks = s.split()
        if len(toks) < 2:
            return None

        key = toks[0]
        if ":" not in key:
            return None
        ch_key = key.split(":")[0].strip().upper()
        if not ch_key.startswith("CH"):
            return None

        return ch_key, toks[-1]

    def update(self, ch_key, value_str):
        if ch_key not in self._buf:
            return None

        with self._lock:
            self._buf[ch_key] = value_str
            if all(self._buf[f"CH{i}"] is not None for i in range(self.n)):
                row = [self._buf[f"CH{i}"] for i in range(self.n)]
                for i in range(self.n):
                    self._buf[f"CH{i}"] = None
                return row
        return None


# ================= PeriodicFlusher =================
class PeriodicFlusher(threading.Thread):
    def __init__(self, writer, shutdown_event, interval):
        super().__init__(daemon=True)
        self.writer = writer
        self.shutdown_event = shutdown_event
        self.interval = interval

    def run(self):
        while not self.shutdown_event.wait(self.interval):
            try:
                self.writer.flush()
                print("[LOG] 定时 flush 完成")
            except Exception as e:
                print(f"[LOG] flush 失败：{e}")


# ==================== 主流程 ====================
def main():
    sensor_outdir = get_sensor_outdir("volt", OUT_DIR)

    header = ["MonoNS", "EstNS", "MonoS", "EstS"] + [f"CH{i}" for i in range(N_CHANNELS)]
    writer = RollingCSVWriter(
        sensor_outdir, FILENAME_PREF, header,
        rotate_mb=ROTATE_MB,
        keep_days=KEEP_DAYS,
        autoflush_every=AUTOFLUSH_EVERY,
    )

    shutdown = threading.Event()
    buf = ChannelBuffer(N_CHANNELS)
    sniff_n = 0
    total_frames = 0

    def on_serial_line(payload):
        nonlocal sniff_n, total_frames

        if DEBUG_RAW_ECHO or (DEBUG_RAW_SNIFF and sniff_n < DEBUG_RAW_SNIFF):
            print(f"[RAW] {payload!r}")
            sniff_n += 1

        parsed = buf.parse_line(payload)
        if not parsed:
            return
        ch_key, v = parsed
        row_vals = buf.update(ch_key, v)
        if row_vals is None:
            return

        # ---------- ★统一时间基★ ----------
        ts = stamp("volt0", SensorKind.OTHER)
        mono_ns = ts.host_time_ns
        est_ns  = ts.corrected_time_ns
        mono_s  = mono_ns * 1e-9
        est_s   = est_ns * 1e-9

        writer.writerow([mono_ns, est_ns, mono_s, est_s] + row_vals)
        total_frames += 1

        if DEBUG_WRITE_LOG:
            print(f"[WRITE] {total_frames}: est_s={est_s:.6f} → {writer.current_path}")

    # 串口线程
    serial_th = SerialReaderThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUD,
        shutdown_event=shutdown,
        data_callback=on_serial_line,
        line_mode=True,
        encoding="utf-8",
        eol=b"\n",
    )
    serial_th.start()
    print(f"[SERIAL] 启动：{SERIAL_PORT} @ {SERIAL_BAUD}")

    # 周期刷新
    flush_th = PeriodicFlusher(writer, shutdown, FLUSH_SEC)
    flush_th.start()

    # 退出键监听
    input_th = InputListenerThread(shutdown_event=shutdown, shutdown_callback=shutdown.set)
    input_th.start()
    print("运行中。按 's' 回车或 Ctrl+C 退出。")

    def _sig(_s, _f):
        print("\n[SYS] 收到信号，准备退出…")
        shutdown.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        while not shutdown.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        shutdown.set()

    print("[SYS] 收尾中…")
    serial_th.join(JOIN_TIMEOUT_S)
    flush_th.join(JOIN_TIMEOUT_S)
    writer.flush()
    writer.close()

    print(f"[STAT] 写入帧数：{total_frames}")
    print("[SYS] 已退出。")


if __name__ == "__main__":
    main()
