# uwnav/sensors/imu.py
# -*- coding: utf-8 -*-

"""
IMU 采集与实时滤波（HWT9073-485 / Modbus）
- 串口读：底层仍用 uwnav.drivers.imu.WitHighModbus.device_model（厂家示例改造）
- 滤波：uwnav.drivers.imu.WitHighModbus.filters.RealTimeIMUFilter
- 时间戳：Unix 时间戳（time.time）+ 单调时钟（time.perf_counter）用于稳定 dt
- 线程模型：驱动回调 → 线程安全队列 → 后台处理线程（滤波、回调、落盘）
- 回调：把“原始 + 滤波 + 航向角”打包回传上层（不阻塞驱动回调）
"""

from __future__ import annotations

import csv
import os
import time
import math
import threading
from dataclasses import dataclass
from queue import Queue, Empty
from typing import Callable, Optional, Tuple

import numpy as np

from uwnav.drivers.imu.WitHighModbus import device_model
from uwnav.drivers.imu.WitHighModbus.filters import RealTimeIMUFilter


# ---------------------- 配置常量 ----------------------
DEFAULT_FS = 100.0     # 目标采样频率（Hz）
DEFAULT_CUTOFF = 6.0   # 低通截止（Hz），建议 0.05~0.2*FS
QUEUE_SIZE = 2000      # 环形缓冲（足够覆盖2~3秒）
WARN_IF_IDLE_SEC = 3.0 # 收不到数据的警告时间
# ---------------------------------------------------


@dataclass
class IMUSample:
    """线程间传递的最小单元（原始 IMU 数据 + 时间戳）"""
    t_unix: float                # Unix时间戳（秒）
    t_mono: float                # 单调时钟（秒），用于估计 dt
    acc: np.ndarray              # 形状(3,) 加速度，单位：按设备输出（一般 g 或 m/s^2）
    gyr: np.ndarray              # 形状(3,) 角速度，单位：deg/s（与 filters.py 对齐）


@dataclass
class IMUResult:
    """传给上层（回调/上游模块）的数据包：原始 + 滤波 + 航向角"""
    t_unix: float
    acc_raw: np.ndarray          # (3,)
    gyr_raw: np.ndarray          # (3,)
    acc_filt: Optional[np.ndarray]  # (3,) or None（校准期 None）
    gyr_filt: Optional[np.ndarray]  # (3,) or None（校准期 None，单位 deg/s）
    yaw_deg: Optional[float]        # None 表示尚未输出（校准期 or 首帧）


class IMUReader:
    """
    - open()/close(): 管理底层设备
    - start()/stop(): 启动/停止后台处理线程
    - on_result: 回调，签名 on_result(IMUResult)
    - 可选 CSV 落盘（原始/滤波）
    """

    def __init__(self,
                 port: str,
                 baud: int = 230400,
                 addr: int = 0x50,
                 fs: float = DEFAULT_FS,
                 cutoff: float = DEFAULT_CUTOFF,
                 on_result: Optional[Callable[[IMUResult], None]] = None,
                 csv_dir: Optional[str] = None):
        self.port = port
        self.baud = baud
        self.addr = addr
        self.on_result = on_result

        # 实时滤波器：在 filters.py 内部完成“1秒静置校准 + 低通 + 航向角积分解缠”
        self.filter = RealTimeIMUFilter(fs=fs, cutoff=cutoff, calibrate=True)

        # 设备与线程控制
        self._dev: Optional[device_model.DeviceModel] = None
        self._q: Queue[IMUSample] = Queue(maxsize=QUEUE_SIZE)
        self._stop_evt = threading.Event()
        self._worker: Optional[threading.Thread] = None

        # 统计
        self._last_rx_unix = 0.0
        self._n_rx = 0
        self._n_drop = 0
        self._n_out = 0

        # 可选 CSV 落盘（轻量、非阻塞地 flush）
        self._csv_raw = None
        self._csv_flt = None
        self._raw_writer = None
        self._flt_writer = None
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._csv_raw = open(os.path.join(csv_dir, f"imu_raw_{ts}.csv"), "a", newline="", encoding="utf-8")
            self._csv_flt = open(os.path.join(csv_dir, f"imu_filt_{ts}.csv"), "a", newline="", encoding="utf-8")
            self._raw_writer = csv.writer(self._csv_raw)
            self._flt_writer = csv.writer(self._csv_flt)
            if self._csv_raw.tell() == 0:
                self._raw_writer.writerow(["Timestamp(s)","AccX","AccY","AccZ","GyrX(deg/s)","GyrY","GyrZ"])
            if self._csv_flt.tell() == 0:
                self._flt_writer.writerow(["Timestamp(s)","AccX_f","AccY_f","AccZ_f","GyrX_f(deg/s)","GyrY_f","GyrZ_f","Yaw(deg)"])

    # ---------------- 设备生命周期 ----------------
    def open(self) -> None:
        """打开底层设备并开始回调采集（非阻塞）"""
        if self._dev:
            return
        self._dev = device_model.DeviceModel(
            deviceName="IMUDevice",
            portName=self.port,
            baud=self.baud,
            ADDR=self.addr,
            callback_method=self._on_device_sample  # 底层回调
        )
        self._dev.openDevice()
        self._dev.startLoopRead()

    def close(self) -> None:
        """关闭底层设备"""
        if self._dev:
            try:
                self._dev.stopLoopRead()
                self._dev.closeDevice()
            finally:
                self._dev = None

    # ---------------- 线程控制 ----------------
    def start(self) -> None:
        """启动后台处理线程（滤波/回调/落盘）"""
        if self._worker and self._worker.is_alive():
            return
        self._stop_evt.clear()
        self._worker = threading.Thread(target=self._worker_loop, name="IMUReader.worker", daemon=True)
        self._worker.start()

    def stop(self, timeout: float = 1.0) -> None:
        """停止后台线程并关闭设备"""
        self._stop_evt.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=timeout)
        self.close()
        # 关文件
        if self._csv_raw:
            try: self._csv_raw.flush()
            except: pass
            self._csv_raw.close()
            self._csv_raw = None
        if self._csv_flt:
            try: self._csv_flt.flush()
            except: pass
            self._csv_flt.close()
            self._csv_flt = None

    # ---------------- 底层回调 ----------------
    def _on_device_sample(self, data: dict) -> None:
        """
        由底层串口线程调用，千万不要做重计算！
        只做：取时间戳 + 取字段 + 入队（队满丢最老，避免阻塞）
        """
        t_unix = time.time()
        t_mono = time.perf_counter()
        try:
            acc = np.array([data.get("AccX"), data.get("AccY"), data.get("AccZ")], dtype=float)
            gyr = np.array([data.get("AsX"),  data.get("AsY"),  data.get("AsZ")], dtype=float)  # deg/s
        except Exception:
            return

        sample = IMUSample(t_unix=t_unix, t_mono=t_mono, acc=acc, gyr=gyr)

        # 非阻塞入队：队满则丢弃最老，保证最新数据优先
        try:
            self._q.put_nowait(sample)
        except:
            try:
                _ = self._q.get_nowait()
                self._n_drop += 1
            except:
                pass
            try:
                self._q.put_nowait(sample)
            except:
                self._n_drop += 1
                return

        self._n_rx += 1
        self._last_rx_unix = t_unix

    # ---------------- 后台处理线程 ----------------
    def _worker_loop(self) -> None:
        last_flush = time.time()
        while not self._stop_evt.is_set():
            # 无新数据时小睡，避免空转
            try:
                item = self._q.get(timeout=0.1)
            except Empty:
                # 空闲告警
                if self._last_rx_unix and (time.time() - self._last_rx_unix > WARN_IF_IDLE_SEC):
                    print(f"[IMU] idle > {WARN_IF_IDLE_SEC:.1f}s without data. Check port/baud/mode.")
                continue

            # 计算滤波（含 1s 静置校准阶段）
            res = self._process_one(item)
            if res is not None:
                # 回调（非阻塞设计：上层若耗时，请自建队列/线程）
                if self.on_result:
                    try:
                        self.on_result(res)
                    except Exception:
                        pass
                self._n_out += 1

                # 轻量落盘
                if self._raw_writer:
                    self._raw_writer.writerow([res.t_unix, *res.acc_raw.tolist(), *res.gyr_raw.tolist()])
                if self._flt_writer and (res.acc_filt is not None) and (res.gyr_filt is not None) and (res.yaw_deg is not None):
                    self._flt_writer.writerow([res.t_unix, *res.acc_filt.tolist(), *res.gyr_filt.tolist(), res.yaw_deg])

                # 定期 flush，降低 I/O 抖动
                if (time.time() - last_flush) > 0.5:
                    try:
                        if self._csv_raw: self._csv_raw.flush()
                        if self._csv_flt: self._csv_flt.flush()
                    except Exception:
                        pass
                    last_flush = time.time()

    # ---------------- 核心处理 ----------------
    def _process_one(self, s: IMUSample) -> Optional[IMUResult]:
        """
        把 IMUSample 送入实时滤波器：
          - 滤波器内部会使用时间戳估计 dt 并完成校准与低通/航向角积分
          - 你可以将 t_unix 作为全局时间戳记录到日志与对齐用
        """
        out = self.filter.process_sample(s.t_unix, s.acc, s.gyr)
        if out is None:
            # 校准期：只返回原始数据，滤波/航向角 None
            return IMUResult(t_unix=s.t_unix,
                             acc_raw=s.acc, gyr_raw=s.gyr,
                             acc_filt=None, gyr_filt=None, yaw_deg=None)
        acc_f, gyr_f, yaw_deg = out
        return IMUResult(t_unix=s.t_unix,
                         acc_raw=s.acc, gyr_raw=s.gyr,
                         acc_filt=acc_f, gyr_filt=gyr_f, yaw_deg=yaw_deg)

    # ---------------- 工具 ----------------
    def stats(self) -> dict:
        return {"rx": self._n_rx, "drop": self._n_drop, "out": self._n_out}

    def last_rx_age(self) -> float:
        return float("inf") if not self._last_rx_unix else (time.time() - self._last_rx_unix)


# ---------------- 使用示例 ----------------
if __name__ == "__main__":
    # 直接运行本文件做快速连通性测试
    def _printer(res: IMUResult):
        # 每收到一帧，简要打印（航向角可能为 None，表示还在校准）
        if res.yaw_deg is not None:
            print(f"{res.t_unix:.6f} | acc_f={np.round(res.acc_filt,3)} gyr_f={np.round(res.gyr_filt,3)} yaw={res.yaw_deg:7.3f}°")
        else:
            print(f"{res.t_unix:.6f} | acc={np.round(res.acc_raw,3)} gyr={np.round(res.gyr_raw,3)} (calibrating...)")

    imu = IMUReader(
        port="COM6",             # Linux: "/dev/ttyUSB0"
        baud=230400,
        addr=0x50,
        fs=100.0,
        cutoff=6.0,
        on_result=_printer,
        csv_dir="./data"         # 也可 None 关闭落盘
    )
    try:
        imu.open()
        imu.start()
        while True:
            time.sleep(1.0)
            st = imu.stats()
            print(f"[STAT] rx={st['rx']} drop={st['drop']} out={st['out']} age={imu.last_rx_age():.2f}s")
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
