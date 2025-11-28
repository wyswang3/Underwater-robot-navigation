#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/imu_data_verifier.py   (timebase 统一版)

更新要点：
- IMU 采样时间全部改为：
      ts = stamp("imu0", SensorKind.IMU)
      mono_ns = ts.host_time_ns
      est_ns  = ts.corrected_time_ns
- 滤波器 dt 使用 mono_s（稳定、连续、无回拨）
- CSV 输出格式统一：
      MonoNS, EstNS, MonoS, EstS, ...
"""

from __future__ import annotations

import os
import csv
import time
import signal
import argparse
import threading
from dataclasses import dataclass
from queue import Queue, Empty
from typing import Optional

import numpy as np

from uwnav.io.timebase import stamp, SensorKind   # ★ 新 API
from uwnav.drivers.imu.WitHighModbus import device_model
from uwnav.drivers.imu.WitHighModbus.filters import RealTimeIMUFilter
from uwnav.io.data_paths import get_sensor_outdir

# ---- 默认参数 ----
DEFAULT_FS = 100.0       # 滤波器参考频率
DEFAULT_CUTOFF = 6.0
QUEUE_SIZE = 2000
WARN_IF_IDLE_SEC = 3.0
FLUSH_PERIOD_SEC = 0.5
DEBUG_PRINT_EVERY = 25
# -------------------

@dataclass
class IMUSample:
    mono_ns: int
    est_ns: int
    acc: np.ndarray
    gyr: np.ndarray

    @property
    def mono_s(self):
        return self.mono_ns * 1e-9

    @property
    def est_s(self):
        return self.est_ns * 1e-9


class IMURecorder:
    def __init__(self, port: str, baud: int, addr: int,
                 fs: float = DEFAULT_FS, cutoff: float = DEFAULT_CUTOFF,
                 outdir: Optional[str] = None):

        self.port = port
        self.baud = baud
        self.addr = addr

        # 实时滤波器
        self.filter = RealTimeIMUFilter(fs=fs, cutoff=cutoff, calibrate=True)

        # 数据结构
        self.dev = None
        self.q: Queue[IMUSample] = Queue(maxsize=QUEUE_SIZE)
        self.stop_evt = threading.Event()
        self.worker = None

        self.last_rx_est_s = 0.0
        self.n_recv = 0
        self.n_drop = 0
        self.n_wraw = 0
        self.n_wfilt = 0

        # 输出
        self.raw_f = None
        self.raw_w = None
        self.flt_f = None
        self.flt_w = None

        if outdir:
            os.makedirs(outdir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            raw_path = os.path.join(outdir, f"imu_raw_{ts}.csv")
            flt_path = os.path.join(outdir, f"imu_filt_{ts}.csv")

            self.raw_f = open(raw_path, "a", newline="", encoding="utf-8")
            self.raw_w = csv.writer(self.raw_f)

            self.flt_f = open(flt_path, "a", newline="", encoding="utf-8")
            self.flt_w = csv.writer(self.flt_f)

            if self.raw_f.tell() == 0:
                self.raw_w.writerow([
                    "MonoNS","EstNS","MonoS","EstS",
                    "AccX","AccY","AccZ",
                    "GyrX(deg/s)","GyrY(deg/s)","GyrZ(deg/s)"
                ])
            if self.flt_f.tell() == 0:
                self.flt_w.writerow([
                    "MonoNS","EstNS","MonoS","EstS",
                    "AccX_f","AccY_f","AccZ_f",
                    "GyrX_f","GyrY_f","GyrZ_f",
                    "Yaw_f(deg)"
                ])

    # ---------- 设备 ----------
    def open(self):
        if self.dev:
            return

        self.dev = device_model.DeviceModel(
            deviceName="IMUDevice",
            portName=self.port,
            baud=self.baud,
            ADDR=self.addr,
            callback_method=self._on_sample,
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

    # ---------- 轻量回调 ----------
    def _on_sample(self, data: dict):
        if self.stop_evt.is_set():
            return

        # ★ 新的时间 API
        ts = stamp("imu0", SensorKind.IMU)
        mono_ns = ts.host_time_ns
        est_ns  = ts.corrected_time_ns

        try:
            acc = np.array([data.get("AccX"), data.get("AccY"), data.get("AccZ")], float)
            gyr = np.array([data.get("AsX"),  data.get("AsY"),  data.get("AsZ")],  float)
        except Exception:
            return

        s = IMUSample(mono_ns, est_ns, acc, gyr)

        # 非阻塞入队（满了丢最老）
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
        self.last_rx_est_s = s.est_s

        if DEBUG_PRINT_EVERY and (self.n_recv % DEBUG_PRINT_EVERY == 0):
            print(f"[RECV] n={self.n_recv} acc={np.round(acc,3)} gyr={np.round(gyr,3)} est_s={s.est_s:.6f}")

    # ---------- 后台处理线程 ----------
    def start(self):
        if self.worker and self.worker.is_alive():
            return
        self.stop_evt.clear()
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()

    def stop(self, timeout=1.0):
        self.stop_evt.set()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=timeout)

    def _loop(self):
        last_flush = time.time()

        while not self.stop_evt.is_set():
            try:
                s = self.q.get(timeout=0.1)
            except Empty:
                if self.last_rx_est_s and (time.time() - self.last_rx_est_s > WARN_IF_IDLE_SEC):
                    print(f"[IMU] idle > {WARN_IF_IDLE_SEC}s — check device.")
                continue

            # 滤波器使用单调秒 dt
            out = self.filter.process_sample(s.mono_s, s.acc, s.gyr)

            # 写 raw
            if self.raw_w:
                self.raw_w.writerow([
                    s.mono_ns, s.est_ns, s.mono_s, s.est_s,
                    *s.acc.tolist(), *s.gyr.tolist(),
                ])
                self.n_wraw += 1

            # 写 filtered
            if out is not None and self.flt_w:
                acc_f, gyr_f, yaw_deg = out
                self.flt_w.writerow([
                    s.mono_ns, s.est_ns, s.mono_s, s.est_s,
                    *acc_f.tolist(), *gyr_f.tolist(), float(yaw_deg),
                ])
                self.n_wfilt += 1

            # 定期 flush
            if (time.time() - last_flush) >= FLUSH_PERIOD_SEC:
                try:
                    if self.raw_f: self.raw_f.flush()
                    if self.flt_f: self.flt_f.flush()
                except:
                    pass
                last_flush = time.time()

    # ---------- 统计 ----------
    def stats(self):
        return {
            "recv": self.n_recv,
            "drop": self.n_drop,
            "wraw": self.n_wraw,
            "wfilt": self.n_wfilt,
        }


# ---------- CLI ----------
def make_argparser():
    ap = argparse.ArgumentParser(
        description="IMU HWT9073-485 logger (raw + filtered) [timebase unified]"
    )
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x50)
    ap.add_argument("--fs", type=float, default=DEFAULT_FS)
    ap.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF)
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--log-every", type=int, default=DEBUG_PRINT_EVERY)
    return ap


def main():
    args = make_argparser().parse_args()
    global DEBUG_PRINT_EVERY
    DEBUG_PRINT_EVERY = max(0, int(args.log_every))

    # 保存路径（统一）
    sensor_outdir = get_sensor_outdir("imu", args.outdir)

    rec = IMURecorder(
        port=args.port,
        baud=args.baud,
        addr=args.addr,
        fs=args.fs,
        cutoff=args.cutoff,
        outdir=str(sensor_outdir),
    )

    stop_evt = threading.Event()
    def _on_sig(sig, frame):
        print("[SYS] Signal received, exiting...")
        stop_evt.set()
    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    rec.open()
    rec.start()

    try:
        while not stop_evt.is_set():
            time.sleep(1.0)
            st = rec.stats()
            age = float("inf") if rec.last_rx_est_s == 0 else (time.time() - rec.last_rx_est_s)
            print(f"[STAT] recv={st['recv']} drop={st['drop']} wraw={st['wraw']} wfilt={st['wfilt']} idle={age:.2f}s")
    finally:
        rec.stop()
        rec.close()
        print("[IMU] bye.")


if __name__ == "__main__":
    main()
