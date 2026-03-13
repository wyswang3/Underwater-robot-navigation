#!/usr/bin/env python3
"""Merge minimal nav/control/telemetry logs into one monotonic timeline.

Inputs are intentionally small and P1-oriented:
- nav_timing.bin: sensor timing / device-state trace
- nav.bin: processed DVL sample log
- nav_state.bin: published NavState snapshots
- control_loop_*.csv: control state log
- telemetry_*_timeline.csv / telemetry_*_events.csv: telemetry/UI-facing snapshots
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import pathlib
import sys
from typing import Iterable, List

import parse_nav_timing


class NavState(ctypes.Structure):
    _fields_ = [
        ("t_ns", ctypes.c_uint64),
        ("pos", ctypes.c_double * 3),
        ("vel", ctypes.c_double * 3),
        ("rpy", ctypes.c_double * 3),
        ("depth", ctypes.c_double),
        ("omega_b", ctypes.c_double * 3),
        ("acc_b", ctypes.c_double * 3),
        ("age_ms", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("stale", ctypes.c_uint8),
        ("degraded", ctypes.c_uint8),
        ("nav_state", ctypes.c_uint8),
        ("health", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("fault_code", ctypes.c_uint16),
        ("sensor_mask", ctypes.c_uint16),
        ("status_flags", ctypes.c_uint16),
        ("reserved1", ctypes.c_uint16),
    ]


class DvlFrame(ctypes.Structure):
    _fields_ = [
        ("sensor_time_ns", ctypes.c_int64),
        ("recv_mono_ns", ctypes.c_int64),
        ("consume_mono_ns", ctypes.c_int64),
        ("mono_ns", ctypes.c_int64),
        ("est_ns", ctypes.c_int64),
        ("bottom_lock", ctypes.c_bool),
        ("track_type", ctypes.c_uint8),
        ("vel_body_mps", ctypes.c_float * 3),
        ("has_body_vel", ctypes.c_bool),
        ("vel_enu_mps", ctypes.c_float * 3),
        ("has_enu_vel", ctypes.c_bool),
        ("dist_enu_m", ctypes.c_float * 3),
        ("has_dist_enu", ctypes.c_bool),
        ("altitude_m", ctypes.c_float),
        ("has_altitude", ctypes.c_bool),
        ("fom", ctypes.c_float),
        ("valid", ctypes.c_bool),
        ("quality", ctypes.c_int32),
    ]


def read_binary_records(path: pathlib.Path, ctype: type[ctypes.Structure]) -> List[ctypes.Structure]:
    raw = path.read_bytes()
    size = ctypes.sizeof(ctype)
    if len(raw) % size != 0:
        raise ValueError(f"{path} size {len(raw)} is not a multiple of {size}")
    records = []
    for off in range(0, len(raw), size):
        records.append(ctype.from_buffer_copy(raw[off : off + size]))
    return records


def csv_rows(path: pathlib.Path) -> Iterable[dict]:
    with path.open(newline="") as f:
        yield from csv.DictReader(f)


def make_event(ts_ns: int, source: str, summary: str) -> dict:
    return {"ts_ns": int(ts_ns), "source": source, "summary": summary}


def merge_events(args: argparse.Namespace) -> List[dict]:
    events: List[dict] = []

    if args.nav_timing:
        for rec in parse_nav_timing.read_records(args.nav_timing):
            ts_ns = rec["publish_mono_ns"] or rec["consume_mono_ns"] or rec["recv_mono_ns"]
            flags = ",".join(rec["flag_names"]) or "-"
            events.append(
                make_event(
                    ts_ns,
                    "nav_timing",
                    f"{rec['kind_name']} age_ms={rec['age_ms']} fault={rec['fault_code']} flags={flags}",
                )
            )

    if args.nav_bin:
        for rec in read_binary_records(args.nav_bin, DvlFrame):
            events.append(
                make_event(
                    rec.mono_ns,
                    "nav_bin",
                    "dvl_sample "
                    f"valid={int(rec.valid)} bottom_lock={int(rec.bottom_lock)} "
                    f"body={int(rec.has_body_vel)} enu={int(rec.has_enu_vel)} "
                    f"alt={int(rec.has_altitude)}",
                )
            )

    if args.nav_state:
        for rec in read_binary_records(args.nav_state, NavState):
            events.append(
                make_event(
                    rec.t_ns,
                    "nav_state",
                    f"valid={rec.valid} stale={rec.stale} degraded={rec.degraded} "
                    f"state={rec.nav_state} health={rec.health} fault={rec.fault_code} "
                    f"age_ms={rec.age_ms} sensor_mask=0x{rec.sensor_mask:04x} "
                    f"status_flags=0x{rec.status_flags:04x}",
                )
            )

    if args.control_log:
        for row in csv_rows(args.control_log):
            ts_ns = int(row["MonoNS"])
            events.append(
                make_event(
                    ts_ns,
                    "control",
                    f"mode={row['effective_mode']} armed={row['armed']} failsafe={row['failsafe']} "
                    f"nav_valid={row['nav_valid']} nav_stale={row['nav_stale']} "
                    f"nav_fault={row['nav_fault_code']} nav_age_ms={row['nav_age_ms']}",
                )
            )

    if args.telemetry_timeline:
        for row in csv_rows(args.telemetry_timeline):
            ts_ns = int(row["telemetry_stamp_ns"])
            events.append(
                make_event(
                    ts_ns,
                    "telemetry_timeline",
                    f"mode={row['active_mode']} armed={row['armed']} failsafe={row['failsafe_active']} "
                    f"nav_valid={row['nav_valid']} nav_stale={row['nav_stale']} "
                    f"nav_age_ms={row['nav_age_ms']} health={row['health_state']} "
                    f"cmd_status={row['cmd_status']} event_code={row['event_code']}",
                )
            )

    if args.telemetry_events:
        for row in csv_rows(args.telemetry_events):
            ts_ns = int(row["stamp_ns"])
            events.append(
                make_event(
                    ts_ns,
                    "telemetry_events",
                    f"{row['kind']} seq={row['seq_or_cmd']} code={row['code']} "
                    f"fault={row['fault_code']} intent={row['intent_id']} status={row['status']}",
                )
            )

    events.sort(key=lambda item: (item["ts_ns"], item["source"], item["summary"]))
    return events


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-timing", type=pathlib.Path)
    parser.add_argument("--nav-bin", type=pathlib.Path)
    parser.add_argument("--nav-state", type=pathlib.Path)
    parser.add_argument("--control-log", type=pathlib.Path)
    parser.add_argument("--telemetry-timeline", type=pathlib.Path)
    parser.add_argument("--telemetry-events", type=pathlib.Path)
    parser.add_argument("--limit", type=int, default=50)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    events = merge_events(args)
    summary = {
        "total_events": len(events),
        "sources": sorted({item["source"] for item in events}),
        "timeline": events[: args.limit],
    }

    if args.json:
        json.dump(summary, sys.stdout, indent=2)
        print()
    else:
        print(f"total_events: {summary['total_events']}")
        print("sources:")
        for source in summary["sources"]:
            print(f"  {source}")
        print("timeline:")
        if not summary["timeline"]:
            print("  <none>")
        for item in summary["timeline"]:
            print(f"  {item['ts_ns']} [{item['source']}] {item['summary']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
