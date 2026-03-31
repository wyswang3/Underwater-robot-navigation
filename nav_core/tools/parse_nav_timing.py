#!/usr/bin/env python3
"""Parse nav_timing.bin TimingTracePacketV1 records.

This parser is intentionally minimal and P0-oriented:
- verifies the fixed 48-byte ABI used by TimingTracePacketV1
- summarizes consumed/rejected/device-state records
- reports duplicate and out-of-order sample timestamps
- estimates sensor->recv and recv->consume delays
- prints a short event timeline for stale/rejected/device transitions
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import pathlib
import statistics
import struct
import sys
from typing import Dict, Iterable, List, Tuple


RECORD = struct.Struct("<IHHqqqqII")
UNKNOWN_AGE_MS = 0xFFFFFFFF

KIND_NAMES = {
    1: "imu_consumed",
    2: "dvl_consumed",
    3: "nav_published",
    4: "imu_rejected",
    5: "dvl_rejected",
    6: "imu_device_state",
    7: "dvl_device_state",
}

FLAG_NAMES = {
    1 << 0: "fresh",
    1 << 1: "accepted",
    1 << 2: "valid",
    1 << 3: "stale",
    1 << 4: "degraded",
    1 << 5: "rejected",
    1 << 6: "out_of_order",
    1 << 7: "device_online",
    1 << 8: "device_mismatch",
    1 << 9: "device_reconnecting",
}

DEVICE_STATES = {
    0: "DISCONNECTED",
    1: "PROBING",
    2: "CONNECTING",
    3: "ONLINE",
    4: "MISMATCH",
    5: "ERROR_BACKOFF",
    6: "RECONNECTING",
}


def decode_flags(flags: int) -> List[str]:
    out: List[str] = []
    for bit, name in FLAG_NAMES.items():
        if flags & bit:
            out.append(name)
    return out


def sample_family(kind: int) -> str | None:
    if kind in (1, 4):
        return "imu"
    if kind in (2, 5):
        return "dvl"
    return None


def percentile(values: List[float], pct: float) -> float:
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    values = sorted(values)
    pos = (len(values) - 1) * pct
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return values[lo]
    frac = pos - lo
    return values[lo] * (1.0 - frac) + values[hi] * frac


def read_records(path: pathlib.Path) -> List[dict]:
    raw = path.read_bytes()
    if len(raw) % RECORD.size != 0:
        raise ValueError(
            f"{path} size {len(raw)} is not a multiple of {RECORD.size} bytes"
        )

    records: List[dict] = []
    for idx in range(0, len(raw), RECORD.size):
        version, kind, flags, sensor, recv, consume, publish, age_ms, fault = RECORD.unpack_from(
            raw, idx
        )
        records.append(
            {
                "index": idx // RECORD.size,
                "version": version,
                "kind": kind,
                "kind_name": KIND_NAMES.get(kind, f"unknown_{kind}"),
                "flags": flags,
                "flag_names": decode_flags(flags),
                "sensor_time_ns": sensor,
                "recv_mono_ns": recv,
                "consume_mono_ns": consume,
                "publish_mono_ns": publish,
                "age_ms": None if age_ms == UNKNOWN_AGE_MS else age_ms,
                "fault_code": fault,
            }
        )
    return records


def summarize(records: Iterable[dict]) -> dict:
    by_kind: Dict[str, int] = collections.Counter()
    duplicates = collections.Counter()
    out_of_order = collections.Counter()
    last_sensor_by_family: Dict[str, int] = {}

    sensor_to_recv_ms: List[float] = []
    recv_to_consume_ms: List[float] = []
    consume_to_publish_ms: List[float] = []
    event_timeline: List[str] = []
    device_events = collections.Counter()

    total_stale = 0
    total_rejected = 0
    total_accepted = 0

    for rec in records:
        by_kind[rec["kind_name"]] += 1

        if "stale" in rec["flag_names"]:
            total_stale += 1
        if "rejected" in rec["flag_names"]:
            total_rejected += 1
        if "accepted" in rec["flag_names"]:
            total_accepted += 1

        family = sample_family(rec["kind"])
        sensor_ns = rec["sensor_time_ns"]
        if family and sensor_ns > 0:
            prev = last_sensor_by_family.get(family)
            if prev is not None:
                if sensor_ns == prev:
                    duplicates[family] += 1
                elif sensor_ns < prev:
                    out_of_order[family] += 1
            last_sensor_by_family[family] = max(sensor_ns, prev or sensor_ns)

        recv_ns = rec["recv_mono_ns"]
        consume_ns = rec["consume_mono_ns"]
        publish_ns = rec["publish_mono_ns"]
        if sensor_ns > 0 and recv_ns >= sensor_ns:
            sensor_to_recv_ms.append((recv_ns - sensor_ns) / 1e6)
        if recv_ns > 0 and consume_ns >= recv_ns:
            recv_to_consume_ms.append((consume_ns - recv_ns) / 1e6)
        if consume_ns > 0 and publish_ns >= consume_ns:
            consume_to_publish_ms.append((publish_ns - consume_ns) / 1e6)

        if rec["kind"] in (6, 7):
            label = "imu" if rec["kind"] == 6 else "dvl"
            state_name = DEVICE_STATES.get(rec["fault_code"], f"STATE_{rec['fault_code']}")
            device_events[f"{label}:{state_name}"] += 1
            event_timeline.append(
                f"{rec['index']:04d} {label}_device state={state_name} "
                f"publish={publish_ns} flags={','.join(rec['flag_names']) or '-'}"
            )
            continue

        significant = (
            rec["kind"] in (4, 5)
            or rec["kind"] == 3
            and (
                "stale" in rec["flag_names"]
                or "degraded" in rec["flag_names"]
                or "valid" not in rec["flag_names"]
            )
        )
        if significant:
            event_timeline.append(
                f"{rec['index']:04d} {rec['kind_name']} sensor={sensor_ns} recv={recv_ns} "
                f"consume={consume_ns} publish={publish_ns} age_ms={rec['age_ms']} "
                f"flags={','.join(rec['flag_names']) or '-'} fault={rec['fault_code']}"
            )

    return {
        "total_records": sum(by_kind.values()),
        "counts_by_kind": dict(by_kind),
        "duplicates": dict(duplicates),
        "out_of_order": dict(out_of_order),
        "accepted_records": total_accepted,
        "rejected_records": total_rejected,
        "stale_flag_records": total_stale,
        "device_events": dict(device_events),
        "latency_ms": {
            "sensor_to_recv": summarize_latency(sensor_to_recv_ms),
            "recv_to_consume": summarize_latency(recv_to_consume_ms),
            "consume_to_publish": summarize_latency(consume_to_publish_ms),
        },
        "timeline": event_timeline,
    }


def summarize_latency(values: List[float]) -> dict:
    if not values:
        return {}
    return {
        "count": len(values),
        "min": round(min(values), 3),
        "p50": round(percentile(values, 0.50), 3),
        "p95": round(percentile(values, 0.95), 3),
        "max": round(max(values), 3),
        "mean": round(statistics.fmean(values), 3),
    }


def print_human(summary: dict, timeline_limit: int) -> None:
    print(f"total_records: {summary['total_records']}")
    print("counts_by_kind:")
    for kind, count in sorted(summary["counts_by_kind"].items()):
        print(f"  {kind}: {count}")

    print(
        "sample_ordering:"
        f" duplicates={summary['duplicates'] or '{}'}"
        f" out_of_order={summary['out_of_order'] or '{}'}"
    )
    print(
        "record_flags:"
        f" accepted={summary['accepted_records']}"
        f" rejected={summary['rejected_records']}"
        f" stale={summary['stale_flag_records']}"
    )

    print("latency_ms:")
    for name, stats in summary["latency_ms"].items():
        if not stats:
            print(f"  {name}: <none>")
            continue
        print(
            f"  {name}: count={stats['count']} min={stats['min']}"
            f" p50={stats['p50']} p95={stats['p95']}"
            f" max={stats['max']} mean={stats['mean']}"
        )

    print("device_events:")
    if not summary["device_events"]:
        print("  <none>")
    else:
        for name, count in sorted(summary["device_events"].items()):
            print(f"  {name}: {count}")

    print("timeline:")
    if not summary["timeline"]:
        print("  <none>")
    else:
        for line in summary["timeline"][:timeline_limit]:
            print(f"  {line}")


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=pathlib.Path, help="nav_timing.bin file path")
    parser.add_argument("--json", action="store_true", help="emit JSON summary")
    parser.add_argument(
        "--timeline-limit", type=int, default=20, help="max number of timeline rows to print"
    )
    args = parser.parse_args(argv)

    records = read_records(args.path)
    summary = summarize(records)

    if args.json:
        json.dump(summary, sys.stdout, indent=2, sort_keys=True)
        print()
    else:
        print_human(summary, args.timeline_limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
