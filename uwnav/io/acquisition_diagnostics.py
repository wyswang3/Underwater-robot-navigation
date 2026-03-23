from __future__ import annotations

import csv
import json
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional


def wall_time_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _json_detail(details: Dict[str, Any]) -> str:
    if not details:
        return ""
    return json.dumps(details, ensure_ascii=False, sort_keys=True)


class SensorRunDiagnostics:
    """
    为 Python 采集/校验脚本提供统一的事件落盘与 session summary。

    设计目标：
    1. 不侵入 authority 主链，只服务于工具链与部署验证。
    2. 让“空采集 / 坏样本 / 错字段 / 超时”有统一落点，避免现场只剩终端滚屏。
    3. 保持输出稳定简单：一个 events.csv + 一个 session_summary.json。
    """

    def __init__(self, out_dir: Path | str, prefix: str, sensor_id: str):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = prefix
        self.sensor_id = sensor_id
        self.run_ts = time.strftime("%Y%m%d_%H%M%S")
        self.events_path = self.out_dir / f"{prefix}_events_{self.run_ts}.csv"
        self.summary_path = self.out_dir / f"{prefix}_session_summary_{self.run_ts}.json"

        self._counter: Counter[str] = Counter()
        self._meta: Dict[str, Any] = {"sensor_id": sensor_id, "prefix": prefix}
        self._files: Dict[str, str] = {}
        self._warned_codes: set[str] = set()
        self._closed = False

        self.start_wall_time = wall_time_now()
        self.start_mono_ns = time.monotonic_ns()

        with self.events_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "wall_time",
                "mono_ns",
                "sensor_id",
                "level",
                "code",
                "message",
                "detail_json",
            ])

        self.event("info", "session_started", "sensor session started")

    def bump(self, key: str, delta: int = 1) -> int:
        self._counter[key] += int(delta)
        return self._counter[key]

    def set_meta(self, key: str, value: Any) -> None:
        self._meta[key] = value

    def note_file(self, key: str, path: Path | str) -> None:
        self._files[key] = str(path)

    def event(self, level: str, code: str, message: str, **details: Any) -> None:
        row = [
            wall_time_now(),
            str(time.monotonic_ns()),
            self.sensor_id,
            level,
            code,
            message,
            _json_detail(details),
        ]
        with self.events_path.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow(row)

    def info(self, code: str, message: str, **details: Any) -> None:
        self.event("info", code, message, **details)

    def warn(self, code: str, message: str, **details: Any) -> None:
        self.event("warn", code, message, **details)

    def error(self, code: str, message: str, **details: Any) -> None:
        self.event("error", code, message, **details)

    def warn_once(self, code: str, message: str, **details: Any) -> None:
        if code in self._warned_codes:
            return
        self._warned_codes.add(code)
        self.warn(code, message, **details)

    def finalize(self, status: str, message: str = "", **extra_meta: Any) -> None:
        if self._closed:
            return
        self._closed = True

        for key, value in extra_meta.items():
            self._meta[key] = value

        self.event(
            "info" if status == "ok" else "warn",
            "session_finished",
            message or f"sensor session finished with status={status}",
            status=status,
        )

        summary = {
            "sensor_id": self.sensor_id,
            "prefix": self.prefix,
            "status": status,
            "message": message,
            "start_wall_time": self.start_wall_time,
            "finish_wall_time": wall_time_now(),
            "start_mono_ns": self.start_mono_ns,
            "finish_mono_ns": time.monotonic_ns(),
            "counters": dict(sorted(self._counter.items())),
            "files": self._files,
            "meta": self._meta,
            "events_path": str(self.events_path),
        }
        self.summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
