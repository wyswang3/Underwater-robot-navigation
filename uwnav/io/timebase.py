# uwnav/io/timebase.py
# -*- coding: utf-8 -*-
"""
统一时间基（Python 版）

与 C++ 版 uwnav::timebase 对齐的最小实现：

- 仅使用单调时钟 time.monotonic_ns() 作为唯一时间源
- 不再维护 real/epoch 语义，也不再做“真实墙上时间”估计
- 提供按传感器类型区分的默认延迟（LatencyDefaults）
- 提供统一时间戳结构 Stamp：
    * host_time_ns      —— 采集/解析瞬间的主机单调时间
    * corrected_time_ns —— host_time_ns - latency_ns
    * sensor_time_ns    —— 可选，预留给将来使用传感器内部时间戳
    * latency_ns        —— 本次使用的延迟估计

推荐用法（采集脚本中）：
    from uwnav.io.timebase import SensorKind, stamp

    ts = stamp("imu0", SensorKind.IMU)
    packet.mono_ns = ts.host_time_ns
    packet.est_ns  = ts.corrected_time_ns
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Optional

__all__ = [
    "SensorKind",
    "Stamp",
    "LatencyDefaults",
    "latency_defaults",
    "now_ns",
    "stamp",
    "to_sec",
]


# ========================= 基础时间源 =========================


def now_ns() -> int:
    """
    返回当前单调时间（ns）。
    对应 C++ 的 uwnav::timebase::now_ns()
    """
    return time.monotonic_ns()


# ========================= 传感器类型 & 时间戳结构 =========================


class SensorKind(enum.Enum):
    IMU = "imu"
    DVL = "dvl"
    USBL = "usbl"
    OTHER = "other"


@dataclass
class Stamp:
    """
    与 C++ 版 Stamp 对齐的 Python 结构：
    - sensor_id: 传感器实例标识，例如 "imu0", "dvl0"
    - kind:      传感器类型（IMU/DVL/USBL/OTHER）
    - host_time_ns:      调用 stamp() 时的主机单调时间
    - corrected_time_ns: host_time_ns - latency_ns
    - sensor_time_ns:    可选，预留将来用传感器内部时间戳对齐
    - latency_ns:        本次使用的延迟估计（根据 kind 或显式传入）
    """

    sensor_id: str
    kind: SensorKind
    host_time_ns: int
    corrected_time_ns: int
    sensor_time_ns: Optional[int] = None
    latency_ns: int = 0


# ========================= 默认延迟配置 =========================


@dataclass
class LatencyDefaults:
    """
    默认延迟（单位：ns）

    注意：这些值是工程经验参数，需在下水联调时根据实际情况微调：
      - imu_ns:   IMU 从测量到到达主机的典型时延
      - dvl_ns:   DVL 帧从测量到到达主机的典型时延
      - usbl_ns:  USBL/其它慢速定位传感器的典型时延
      - other_ns: 其它未分类传感器的默认时延
    """
    imu_ns: int = 2_000_000        # 2 ms
    dvl_ns: int = 50_000_000       # 50 ms
    usbl_ns: int = 150_000_000     # 150 ms
    other_ns: int = 0              # 默认不补偿


_latency_defaults = LatencyDefaults()


def latency_defaults() -> LatencyDefaults:
    """
    获取可修改的默认延迟配置对象。

    示例：
        from uwnav.io.timebase import latency_defaults
        cfg = latency_defaults()
        cfg.dvl_ns = 80_000_000
    """
    return _latency_defaults


def _default_latency_ns(kind: SensorKind) -> int:
    d = _latency_defaults
    if kind is SensorKind.IMU:
        return d.imu_ns
    if kind is SensorKind.DVL:
        return d.dvl_ns
    if kind is SensorKind.USBL:
        return d.usbl_ns
    return d.other_ns


# ========================= 核心 API：stamp =========================


def stamp(
    sensor_id: str,
    kind: SensorKind,
    sensor_time_ns: Optional[int] = None,
    latency_ns: Optional[int] = None,
) -> Stamp:
    """
    统一打时间戳的核心函数（Python 版）。

    参数：
        sensor_id:      传感器实例 ID，如 "imu0", "dvl0"
        kind:           传感器类型（SensorKind.IMU/DVL/USBL/OTHER）
        sensor_time_ns: 可选，传感器内部时间（将来做更精细对齐时使用）
        latency_ns:     可选，显式指定延迟；若为 None，则使用默认值

    返回：
        Stamp 对象，包含：
            host_time_ns
            corrected_time_ns
            sensor_time_ns
            latency_ns

    当前实现：
        host_time_ns      = time.monotonic_ns()
        latency_ns        = 显式传入 或 默认延迟
        corrected_time_ns = host_time_ns - latency_ns
    """
    host = now_ns()
    lat = latency_ns if latency_ns is not None else _default_latency_ns(kind)
    corrected = host - lat

    return Stamp(
        sensor_id=sensor_id,
        kind=kind,
        host_time_ns=host,
        corrected_time_ns=corrected,
        sensor_time_ns=sensor_time_ns,
        latency_ns=lat,
    )


# ========================= 工具函数 =========================


def to_sec(ns: int) -> float:
    """
    ns → 秒（float）
    """
    return ns / 1e9
