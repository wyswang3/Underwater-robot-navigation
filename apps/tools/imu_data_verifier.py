#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/imu_data_verifier.py

功能：
- 以 100 Hz 目标采样调用 HWT9073-485（底层仍为 Modbus 回调）
- 回调线程只做采样入队；后台线程做滤波、落盘、打印
- 时间戳：Unix 秒 (float)
- CSV：原始/滤波分别写，半秒 flush 一次
"""

from __future__ import annotations

import os
import csv
import time
import math
import signal
import argparse
import threading
from dataclasses import dataclass
from queue import Queue, Empty
from typing import Optional, Tuple

import numpy as np

from uwnav.drivers.imu.WitHighModbus import device_model
from uwnav.drivers.imu.WitHighModbus.filters import RealTimeIMUFilter

# ---- 默认参数 ----
DEFAULT_FS = 100.0       # 目标采样频率（Hz）
DEFAULT_CUTOFF = 6.0     # 低通截止（Hz）
QUEUE_SIZE = 2000        # 队列容量（丢最老策略）
WARN_IF_IDLE_SEC = 3.0   # N 秒没数据报警
FLUSH_PERIOD_SEC = 0.5   # flush 周期
DEBUG_PRINT_EVERY = 25   # 每 N 帧打印一次
# -------------------

@dataclass
class IMUSample:
    t_unix: float          # Unix 时间戳（秒）
    acc: np.ndarray        # (3,)
    gyr: np.ndarray        # (3,) deg/s

class IMURecorder:
    def __init__(self, port: str, baud: int, addr: int,
                 fs: float = DEFAULT_FS, cutoff: float = DEFAULT_CUTOFF,
                 outdir: Optional[str] = None):
        self.port = port
        self.baud = baud
        self.addr = addr

        # 实时滤波器（含 1s 静置校准 + 低通 + 航向角积分解缠）
        self.filter = RealTimeIMUFilter(fs=fs, cutoff=cutoff, calibrate=True)

        # 设备与线程
        self.dev: Optional[device_model.DeviceModel] = None
        self.q: Queue[IMUSample] = Queue(maxsize=QUEUE_SIZE)
        self.stop_evt = threading.Event()
        self.worker: Optional[threading.Thread] = None

        # 统计
        self.last_rx_ts = 0.0
        self.n_recv = 0
        self.n_drop = 0
        self.n_wraw = 0
        self.n_wfilt = 0

        # CSV
        self.raw_f = None; self.raw_w = None
        self.flt_f = None; self.flt_w = None
        if outdir:
            os.makedirs(outdir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            raw_path = os.path.join(outdir, f"imu_raw_{ts}.csv")
            flt_path = os.path.join(outdir, f"imu_filt_{ts}.csv")
            self.raw_f = open(raw_path, "a", newline="", encoding="utf-8")
            self.flt_f = open(flt_path, "a", newline="", encoding="utf-8")
            self.raw_w = csv.writer(self.raw_f)
            self.flt_w = csv.writer(self.flt_f)
            if self.raw_f.tell() == 0:
                self.raw_w.writerow(["Timestamp(s)","AccX","AccY","AccZ",
                                     "GyrX(deg/s)","GyrY(deg/s)","GyrZ(deg/s)"])
            if self.flt_f.tell() == 0:
                self.flt_w.writerow(["Timestamp(s)","AccX_f","AccY_f","AccZ_f",
                                     "GyrX_f(deg/s)","GyrY_f(deg/s)","GyrZ_f(deg/s)",
                                     "Yaw_f(deg)"])

    # ---------- 设备 ----------
    def open(self):
        if self.dev: return
        self.dev = device_model.DeviceModel(
            deviceName="IMUDevice",
            portName=self.port,
            baud=self.baud,
            ADDR=self.addr,
            callback_method=self._on_sample  # 底层回调
        )
        self.dev.openDevice()
        self.dev.startLoopRead()
        print(f"[IMU] open {self.port} @ {self.baud}")

    def close(self):
        if self.dev:
            try:
                self.dev.stopLoopRead()
                self.dev.closeDevice()
            finally:
                self.dev = None
        if self.raw_f:
            try: self.raw_f.flush()
            except: pass
            self.raw_f.close()
            self.raw_f = None
        if self.flt_f:
            try: self.flt_f.flush()
            except: pass
            self.flt_f.close()
            self.flt_f = None

    # ---------- 回调：轻 ----------
    def _on_sample(self, data: dict):
        if self.stop_evt.is_set():
            return
        t = time.time()
        try:
            acc = np.array([data.get("AccX"), data.get("AccY"), data.get("AccZ")], dtype=float)
            gyr = np.array([data.get("AsX"),  data.get("AsY"),  data.get("AsZ")], dtype=float)  # deg/s
        except Exception:
            return

        s = IMUSample(t_unix=t, acc=acc, gyr=gyr)

        # 非阻塞入队；队满丢最老
        try:
            self.q.put_nowait(s)
        except:
            try:
                _ = self.q.get_nowait()
                self.n_drop += 1
                self.q.put_nowait(s)
            except:
                self.n_drop += 1
                return

        self.n_recv += 1
        self.last_rx_ts = t

        if DEBUG_PRINT_EVERY and (self.n_recv % DEBUG_PRINT_EVERY == 0):
            print(f"[RECV] n={self.n_recv} acc={np.round(acc,3)} gyr={np.round(gyr,3)}")

    # ---------- 后台线程 ----------
    def start(self):
        if self.worker and self.worker.is_alive():
            return
        self.stop_evt.clear()
        self.worker = threading.Thread(target=self._loop, name="IMURecorder.worker", daemon=True)
        self.worker.start()

    def stop(self, timeout: float = 1.0):
        self.stop_evt.set()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=timeout)

    def _loop(self):
        last_flush = time.time()
        while not self.stop_evt.is_set():
            try:
                s = self.q.get(timeout=0.1)
            except Empty:
                if self.last_rx_ts and (time.time() - self.last_rx_ts > WARN_IF_IDLE_SEC):
                    print(f"[IMU] idle > {WARN_IF_IDLE_SEC:.1f}s without data. Check port/baud/mode.")
                continue

            # 调滤波器：校准期返回 None
            out = self.filter.process_sample(s.t_unix, s.acc, s.gyr)
            # 原始必写
            if self.raw_w:
                self.raw_w.writerow([s.t_unix, *s.acc.tolist(), *s.gyr.tolist()])
                self.n_wraw += 1

            if out is not None and self.flt_w:
                acc_f, gyr_f, yaw_deg = out  # gyr_f 为 deg/s（与滤波器说明保持一致）
                self.flt_w.writerow([s.t_unix, *acc_f.tolist(), *gyr_f.tolist(), float(yaw_deg)])
                self.n_wfilt += 1

            # 定期 flush
            if (time.time() - last_flush) > FLUSH_PERIOD_SEC:
                try:
                    if self.raw_f: self.raw_f.flush()
                    if self.flt_f: self.flt_f.flush()
                except Exception:
                    pass
                last_flush = time.time()

    # ---------- 统计 ----------
    def stats(self) -> dict:
        return {"recv": self.n_recv, "drop": self.n_drop, "wraw": self.n_wraw, "wfilt": self.n_wfilt}


def make_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="IMU HWT9073-485 logger (raw + filtered)")
    ap.add_argument("--port", default="/dev/ttyUSB1", help="Serial port, e.g., /dev/ttyUSB0 or COM6")
    ap.add_argument("--baud", type=int, default=230400, help="Baudrate (default 230400)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x50, help="Modbus address (hex like 0x50)")
    ap.add_argument("--fs", type=float, default=DEFAULT_FS, help="Target sample freq for filter (Hz)")
    ap.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF, help="Low-pass cutoff (Hz)")
    ap.add_argument("--outdir", default="./data", help="Output directory")
    ap.add_argument("--log-every", type=int, default=DEBUG_PRINT_EVERY, help="Print every N frames (0=off)")
    return ap


def main():
    args = make_argparser().parse_args()
    global DEBUG_PRINT_EVERY
    DEBUG_PRINT_EVERY = max(0, int(args.log_every))

    rec = IMURecorder(
        port=args.port, baud=args.baud, addr=args.addr,
        fs=args.fs, cutoff=args.cutoff, outdir=args.outdir
    )

    stop_evt = threading.Event()
    def _on_signal(sig, frame):
        print("signal received, exiting...")
        stop_evt.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    rec.open()
    rec.start()

    try:
        while not stop_evt.is_set():
            time.sleep(1.0)
            st = rec.stats()
            age = float("inf") if rec.last_rx_ts == 0 else (time.time() - rec.last_rx_ts)
            print(f"[STAT] recv={st['recv']} drop={st['drop']} wraw={st['wraw']} wfilt={st['wfilt']} age={age:.2f}s")
    finally:
        rec.stop()
        rec.close()
        print("[IMU] bye.")

if __name__ == "__main__":
    main()
