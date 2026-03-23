from __future__ import annotations

import re
import threading
from dataclasses import dataclass
from typing import List, Optional


_CHANNEL_RE = re.compile(r"^\s*(CH(?P<index>\d+))\s*:\s*(?P<value>.+?)\s*$", re.IGNORECASE)
_VALUE_RE = re.compile(r"^\s*(?P<number>[+\-]?(?:\d+(?:\.\d*)?|\.\d+))(?:\s*(?P<unit>[A-Za-z%]+))?\s*$")


@dataclass(frozen=True)
class ChannelSample:
    channel: str
    index: int
    raw_value: str
    numeric_value: Optional[float]
    unit: str


def parse_channel_line(line: str) -> Optional[ChannelSample]:
    if not line:
        return None

    match = _CHANNEL_RE.match(line.strip())
    if match is None:
        return None

    channel = match.group(1).upper()
    index = int(match.group("index"))
    raw_value = match.group("value").strip()

    value_match = _VALUE_RE.match(raw_value)
    if value_match is None:
        return ChannelSample(channel=channel, index=index, raw_value=raw_value, numeric_value=None, unit="")

    number = float(value_match.group("number"))
    unit = (value_match.group("unit") or "").strip()
    return ChannelSample(channel=channel, index=index, raw_value=raw_value, numeric_value=number, unit=unit)


class ChannelFrameBuffer:
    """
    聚合 CH0..CH{N-1} 的一帧文本值。

    这里刻意只负责“按通道收齐一帧”，不替脚本决定单位语义；
    单位漂移、数值是否合法由上层结合具体传感器场景判断。
    """

    def __init__(self, n_channels: int):
        self.n_channels = int(n_channels)
        self._buf = {f"CH{i}": None for i in range(self.n_channels)}
        self._lock = threading.Lock()

    def update(self, sample: ChannelSample) -> Optional[List[str]]:
        if sample.channel not in self._buf:
            return None

        with self._lock:
            self._buf[sample.channel] = sample.raw_value
            if not all(self._buf[f"CH{i}"] is not None for i in range(self.n_channels)):
                return None

            row = [self._buf[f"CH{i}"] for i in range(self.n_channels)]
            for i in range(self.n_channels):
                self._buf[f"CH{i}"] = None
            return row
