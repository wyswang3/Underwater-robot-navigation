from __future__ import annotations

import csv
import json
import os
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional


EVENT_HEADER = [
    "mono_ns",
    "wall_time",
    "component",
    "event",
    "level",
    "run_id",
    "message",
    "sensor_id",
    "state",
    "fault_code",
    "device_path",
    "sample_seq",
    "detail_json",
]


def wall_time_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _json_detail(details: Dict[str, Any]) -> str:
    if not details:
        return ""
    return json.dumps(details, ensure_ascii=False, sort_keys=True, default=str)


class SensorRunDiagnostics:
    """
    为 Python 采集/校验脚本提供统一的事件落盘与 session summary。

    设计目标：
    1. 不侵入 authority 主链，只服务于工具链与部署验证。
    2. 让“空采集 / 坏样本 / 错字段 / 超时”有统一落点，避免现场只剩终端滚屏。
    3. 保持输出稳定简单：一个 events.csv + 一个 session_summary.json。
    """

    def __init__(self, out_dir: Path | str, prefix: str, sensor_id: str, component: Optional[str] = None):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = prefix
        self.sensor_id = sensor_id
        self.component = component or prefix
        self.run_ts = time.strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{self.prefix}_{self.run_ts}_{os.getpid()}"
        self.events_path = self.out_dir / f"{prefix}_events_{self.run_ts}.csv"
        self.summary_path = self.out_dir / f"{prefix}_session_summary_{self.run_ts}.json"

        self._counter: Counter[str] = Counter()
        self._level_counter: Counter[str] = Counter()
        self._event_counter: Counter[str] = Counter()
        self._meta: Dict[str, Any] = {
            "sensor_id": sensor_id,
            "prefix": prefix,
            "component": self.component,
            "run_id": self.run_id,
            "pid": os.getpid(),
        }
        self._files: Dict[str, str] = {}
        self._warned_codes: set[str] = set()
        self._closed = False
        self._last_event: Optional[Dict[str, Any]] = None
        self._last_problem_event: Optional[Dict[str, Any]] = None

        self.start_wall_time = wall_time_now()
        self.start_mono_ns = time.monotonic_ns()

        with self.events_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(EVENT_HEADER)

        self.event("info", "session_started", "sensor session started", state="starting")

    def bump(self, key: str, delta: int = 1) -> int:
        self._counter[key] += int(delta)
        return self._counter[key]

    def set_meta(self, key: str, value: Any) -> None:
        self._meta[key] = value

    def note_file(self, key: str, path: Path | str) -> None:
        self._files[key] = str(path)

    def event(self, level: str, code: str, message: str, **details: Any) -> None:
        normalized_level = level.lower()
        payload = dict(details)
        mono_ns = time.monotonic_ns()
        state = payload.pop("state", "")
        fault_code = payload.pop("fault_code", "")
        device_path = payload.pop("device_path", None)
        if device_path in (None, ""):
            device_path = payload.get("port") or self._meta.get("port") or ""
        sample_seq = payload.pop("sample_seq", payload.pop("seq", ""))

        row = [
            str(mono_ns),
            wall_time_now(),
            self.component,
            code,
            normalized_level,
            self.run_id,
            message,
            self.sensor_id,
            "" if state in (None, "") else str(state),
            "" if fault_code in (None, "") else str(fault_code),
            "" if device_path in (None, "") else str(device_path),
            "" if sample_seq in (None, "") else str(sample_seq),
            _json_detail(payload),
        ]
        with self.events_path.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow(row)

        snapshot = {
            "mono_ns": mono_ns,
            "wall_time": row[1],
            "component": self.component,
            "event": code,
            "level": normalized_level,
            "run_id": self.run_id,
            "message": message,
            "sensor_id": self.sensor_id,
            "state": row[8],
            "fault_code": row[9],
            "device_path": row[10],
            "sample_seq": row[11],
            "detail": payload,
        }
        self._level_counter[normalized_level] += 1
        self._event_counter[code] += 1
        self._last_event = snapshot
        if normalized_level in {"warn", "error"}:
            self._last_problem_event = snapshot

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

        finish_level = "info" if status in {"ok", "stopped"} else "warn"
        self.event(
            finish_level,
            "session_finished",
            message or f"sensor session finished with status={status}",
            status=status,
            state=status,
        )

        finish_mono_ns = time.monotonic_ns()
        summary = {
            "run_id": self.run_id,
            "sensor_id": self.sensor_id,
            "prefix": self.prefix,
            "component": self.component,
            "status": status,
            "message": message,
            "start_wall_time": self.start_wall_time,
            "finish_wall_time": wall_time_now(),
            "start_mono_ns": self.start_mono_ns,
            "finish_mono_ns": finish_mono_ns,
            "duration_s": round(max(0, finish_mono_ns - self.start_mono_ns) / 1e9, 3),
            "counters": dict(sorted(self._counter.items())),
            "level_counters": dict(sorted(self._level_counter.items())),
            "event_counters": dict(sorted(self._event_counter.items())),
            "files": self._files,
            "meta": self._meta,
            "events_path": str(self.events_path),
            "summary_path": str(self.summary_path),
            "last_event": self._last_event,
            "last_problem_event": self._last_problem_event,
        }
        self.summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
