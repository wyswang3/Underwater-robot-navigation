# uwnav/io/timebase.py
# -*- coding: utf-8 -*-
"""
统一时间基：面向“离线无网络/无NTP”的水下场景
- 双时间戳：单调时钟 mono_ns（对齐/融合唯一真相）+ 估计实时时间 est_ns（展示/文件）
- est_ns = epoch_real_ns + (monotonic_ns - epoch_mono_ns)
- 提供线程安全的全局实例与便捷函数
"""

from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from typing import Tuple, Optional

__all__ = [
    "TimeBase", "get_timebase",
    "stamp", "stamp_s",
    "mono_ns", "est_ns",
    "to_sec", "to_iso8601",
    "resync_epoch"
]

@dataclass(frozen=True)
class _Epoch:
    real_ns: int
    mono_ns: int

class TimeBase:
    """
    会话级时间基（线程安全）：
    - 初始化时拍下 (real_ns, mono_ns)
    - 可在必要时 resync（例如人工校时/收到上位机广播时间）
    - 通常融合只用 mono_ns；est_ns 用于 CSV 展示/离线可读性
    """
    def __init__(self, epoch_real_ns: Optional[int] = None, epoch_mono_ns: Optional[int] = None):
        r = time.time_ns() if epoch_real_ns is None else int(epoch_real_ns)
        m = time.monotonic_ns() if epoch_mono_ns is None else int(epoch_mono_ns)
        self._lock = threading.RLock()
        self._epoch = _Epoch(real_ns=r, mono_ns=m)

    # ---- 基本API ----
    def stamp(self) -> Tuple[int, int]:
        """
        返回 (mono_ns, est_ns)
        mono_ns：单调时钟； est_ns：估计实时时间
        """
        with self._lock:
            now_mono = time.monotonic_ns()
            est = self._epoch.real_ns + (now_mono - self._epoch.mono_ns)
            return now_mono, est

    def mono_ns(self) -> int:
        return time.monotonic_ns()

    def est_ns(self) -> int:
        with self._lock:
            now_mono = time.monotonic_ns()
            return self._epoch.real_ns + (now_mono - self._epoch.mono_ns)

    def stamp_s(self) -> Tuple[float, float]:
        m, e = self.stamp()
        return m / 1e9, e / 1e9

    # ---- 工具 ----
    @staticmethod
    def to_sec(ns: int) -> float:
        return ns / 1e9

    @staticmethod
    def to_iso8601(est_ns: int) -> str:
        # 仅用于日志/展示；避免在融合里频繁调用（有开销）
        import datetime as _dt
        s = est_ns / 1e9
        return _dt.datetime.utcfromtimestamp(s).strftime("%Y-%m-%dT%H:%M:%S.%fZ")

    # ---- 纠偏/重同步 ----
    def resync(self, new_real_ns: int) -> None:
        """
        当收到“更可信”的实时时间（例如人工校时/外部基准）：
        仅调整 real_ns，使 est_ns 平移；mono 基准不变，保证融合连续
        """
        with self._lock:
            self._epoch = _Epoch(real_ns=int(new_real_ns), mono_ns=self._epoch.mono_ns)


# ---- 全局单例与便捷函数 ----
_global_tb: Optional[TimeBase] = None
_global_lock = threading.Lock()

def get_timebase() -> TimeBase:
    global _global_tb
    if _global_tb is None:
        with _global_lock:
            if _global_tb is None:
                _global_tb = TimeBase()
    return _global_tb

def resync_epoch(new_real_ns: int) -> None:
    get_timebase().resync(new_real_ns)

def stamp() -> Tuple[int, int]:
    return get_timebase().stamp()

def stamp_s() -> Tuple[float, float]:
    return get_timebase().stamp_s()

def mono_ns() -> int:
    return get_timebase().mono_ns()

def est_ns() -> int:
    return get_timebase().est_ns()

def to_sec(ns: int) -> float:
    return TimeBase.to_sec(ns)

def to_iso8601(est_ns_: int) -> str:
    return TimeBase.to_iso8601(est_ns_)
