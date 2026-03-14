#!/usr/bin/env python3
"""Merge nav/control/telemetry logs into one monotonic incident timeline.

Inputs remain intentionally small and P1-oriented:
- nav_timing.bin: sensor timing / device-state trace
- nav.bin: processed DVL sample log
- nav_state.bin: published NavState snapshots
- control_loop_*.csv: control state log
- telemetry_*_timeline.csv / telemetry_*_events.csv: telemetry/UI-facing snapshots

The tool now supports:
- source filtering
- time-range filtering
- event-type anchors (stale/reconnecting/mismatch/failsafe/command_failed...)
- event windows around anchors
- CSV export for incident bundles
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import pathlib
import sys
from typing import Iterable, List, Optional, Sequence

import parse_nav_timing


NAV_FLAG_IMU_DEVICE_ONLINE = 1 << 6
NAV_FLAG_DVL_DEVICE_ONLINE = 1 << 7
NAV_FLAG_IMU_BIND_MISMATCH = 1 << 8
NAV_FLAG_DVL_BIND_MISMATCH = 1 << 9
NAV_FLAG_IMU_RECONNECTING = 1 << 10
NAV_FLAG_DVL_RECONNECTING = 1 << 11

EVENT_CHOICES = (
    "reconnecting",
    "mismatch",
    "stale",
    "invalid",
    "degraded",
    "failsafe",
    "command_rejected",
    "command_failed",
    "command_expired",
    "nav_fault",
)


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
    return [ctype.from_buffer_copy(raw[off : off + size]) for off in range(0, len(raw), size)]


def csv_rows(path: pathlib.Path) -> Iterable[dict]:
    with path.open(newline="") as f:
        yield from csv.DictReader(f)


def int_field(row: dict, key: str, default: int = 0) -> int:
    raw = row.get(key, "")
    if raw in ("", None):
        return default
    return int(raw)


def flag_has(flags: int, flag: int) -> bool:
    return (int(flags) & int(flag)) != 0


def nav_tags_from_state(
    *,
    valid: int,
    stale: int,
    degraded: int,
    fault_code: int,
    status_flags: int,
) -> List[str]:
    """Normalize nav validity/fault/flags into the tag vocabulary used by incident filters."""
    tags: List[str] = []
    flags = int(status_flags)

    if int(stale):
        tags.append("stale")
    if not int(valid):
        tags.append("invalid")
    elif int(degraded):
        tags.append("degraded")

    if flag_has(flags, NAV_FLAG_IMU_BIND_MISMATCH) or flag_has(flags, NAV_FLAG_DVL_BIND_MISMATCH):
        tags.append("mismatch")
    if flag_has(flags, NAV_FLAG_IMU_RECONNECTING) or flag_has(flags, NAV_FLAG_DVL_RECONNECTING):
        tags.append("reconnecting")
    if int(fault_code) != 0:
        tags.append("nav_fault")
    return sorted(set(tags))


def command_tags(status: Optional[int]) -> List[str]:
    if status == 2:
        return ["command_rejected"]
    if status == 4:
        return ["command_expired"]
    if status == 5:
        return ["command_failed"]
    return []


def make_event(
    ts_ns: int,
    source: str,
    summary: str,
    *,
    tags: Optional[Sequence[str]] = None,
    fault_code: Optional[int] = None,
    command_status: Optional[int] = None,
    event_code: Optional[int] = None,
) -> dict:
    return {
        "ts_ns": int(ts_ns),
        "source": source,
        "summary": summary,
        "tags": sorted(set(tags or [])),
        "fault_code": None if fault_code is None else int(fault_code),
        "command_status": None if command_status is None else int(command_status),
        "event_code": None if event_code is None else int(event_code),
        "highlight": False,
        "anchor_ids": [],
    }


def merge_events(args: argparse.Namespace) -> List[dict]:
    events: List[dict] = []

    if args.nav_timing:
        for rec in parse_nav_timing.read_records(args.nav_timing):
            ts_ns = rec["publish_mono_ns"] or rec["consume_mono_ns"] or rec["recv_mono_ns"]
            flags = ",".join(rec["flag_names"]) or "-"
            if rec["kind"] in (6, 7):
                label = "imu" if rec["kind"] == 6 else "dvl"
                state_name = parse_nav_timing.DEVICE_STATES.get(
                    rec["fault_code"], f"STATE_{rec['fault_code']}"
                )
                tags: List[str] = []
                if state_name in ("PROBING", "CONNECTING", "ERROR_BACKOFF", "RECONNECTING"):
                    tags.append("reconnecting")
                if state_name == "MISMATCH":
                    tags.append("mismatch")
                events.append(
                    make_event(
                        ts_ns,
                        "nav_timing",
                        f"{label}_device state={state_name} flags={flags}",
                        tags=tags,
                        event_code=rec["kind"],
                    )
                )
                continue

            tags = []
            if "stale" in rec["flag_names"]:
                tags.append("stale")
            if "degraded" in rec["flag_names"]:
                tags.append("degraded")
            if rec["kind"] in (4, 5):
                tags.append("invalid")
            if rec["fault_code"] != 0:
                tags.append("nav_fault")
            events.append(
                make_event(
                    ts_ns,
                    "nav_timing",
                    f"{rec['kind_name']} age_ms={rec['age_ms']} fault={rec['fault_code']} flags={flags}",
                    tags=tags,
                    fault_code=rec["fault_code"],
                    event_code=rec["kind"],
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
            tags = nav_tags_from_state(
                valid=rec.valid,
                stale=rec.stale,
                degraded=rec.degraded,
                fault_code=rec.fault_code,
                status_flags=rec.status_flags,
            )
            events.append(
                make_event(
                    rec.t_ns,
                    "nav_state",
                    f"valid={rec.valid} stale={rec.stale} degraded={rec.degraded} "
                    f"state={rec.nav_state} health={rec.health} fault={rec.fault_code} "
                    f"age_ms={rec.age_ms} sensor_mask=0x{rec.sensor_mask:04x} "
                    f"status_flags=0x{rec.status_flags:04x}",
                    tags=tags,
                    fault_code=rec.fault_code,
                )
            )

    if args.control_log:
        for row in csv_rows(args.control_log):
            ts_ns = int_field(row, "MonoNS")
            nav_fault_code = int_field(row, "nav_fault_code")
            nav_status_flags = int_field(row, "nav_status_flags")
            tags = nav_tags_from_state(
                valid=int_field(row, "nav_valid"),
                stale=int_field(row, "nav_stale"),
                degraded=int_field(row, "nav_degraded"),
                fault_code=nav_fault_code,
                status_flags=nav_status_flags,
            )
            if int_field(row, "failsafe"):
                tags.append("failsafe")
            events.append(
                make_event(
                    ts_ns,
                    "control",
                    f"mode={row['effective_mode']} armed={row['armed']} failsafe={row['failsafe']} "
                    f"nav_valid={row['nav_valid']} nav_stale={row['nav_stale']} "
                    f"nav_fault={nav_fault_code} nav_age_ms={row['nav_age_ms']}",
                    tags=tags,
                    fault_code=nav_fault_code,
                )
            )

    if args.telemetry_timeline:
        for row in csv_rows(args.telemetry_timeline):
            ts_ns = int_field(row, "telemetry_stamp_ns")
            nav_fault_code = int_field(row, "nav_fault_code")
            nav_status_flags = int_field(row, "nav_status_flags")
            cmd_status = int_field(row, "cmd_status")
            tags = nav_tags_from_state(
                valid=int_field(row, "nav_valid"),
                stale=int_field(row, "nav_stale"),
                degraded=int_field(row, "nav_degraded"),
                fault_code=nav_fault_code,
                status_flags=nav_status_flags,
            )
            if int_field(row, "failsafe_active"):
                tags.append("failsafe")
            tags.extend(command_tags(cmd_status))
            events.append(
                make_event(
                    ts_ns,
                    "telemetry_timeline",
                    f"mode={row['active_mode']} armed={row['armed']} failsafe={row['failsafe_active']} "
                    f"nav_valid={row['nav_valid']} nav_stale={row['nav_stale']} "
                    f"nav_fault={nav_fault_code} nav_age_ms={row['nav_age_ms']} "
                    f"health={row['health_state']} cmd_status={cmd_status} event_code={row['event_code']}",
                    tags=tags,
                    fault_code=nav_fault_code,
                    command_status=cmd_status,
                    event_code=int_field(row, "event_code"),
                )
            )

    if args.telemetry_events:
        for row in csv_rows(args.telemetry_events):
            ts_ns = int_field(row, "stamp_ns")
            cmd_status = int_field(row, "status", default=-1)
            tags: List[str] = []
            if row.get("kind") == "command_result":
                tags.extend(command_tags(cmd_status))
            if int_field(row, "fault_code") != 0:
                tags.append("nav_fault")
            events.append(
                make_event(
                    ts_ns,
                    "telemetry_events",
                    f"{row['kind']} seq={row['seq_or_cmd']} code={row['code']} "
                    f"fault={row['fault_code']} intent={row['intent_id']} status={row['status']}",
                    tags=tags,
                    fault_code=int_field(row, "fault_code"),
                    command_status=None if cmd_status < 0 else cmd_status,
                    event_code=int_field(row, "code"),
                )
            )

    events.sort(key=lambda item: (item["ts_ns"], item["source"], item["summary"]))
    return events


def scalar_filters_match(event: dict, args: argparse.Namespace) -> bool:
    if args.sources and event["source"] not in args.sources:
        return False
    if args.from_ns is not None and event["ts_ns"] < args.from_ns:
        return False
    if args.to_ns is not None and event["ts_ns"] > args.to_ns:
        return False
    if args.fault_code is not None and event["fault_code"] != args.fault_code:
        return False
    if args.command_status is not None and event["command_status"] != args.command_status:
        return False
    return True


def tag_filters_match(event: dict, event_filters: Sequence[str]) -> bool:
    if not event_filters:
        return True
    return any(tag in event["tags"] for tag in event_filters)


def event_key(event: dict) -> tuple[int, str, str]:
    return (event["ts_ns"], event["source"], event["summary"])


def anchor_key_map(anchors: Sequence[dict], event_filters: Sequence[str]) -> dict[tuple[int, str, str], int]:
    """Collapse near-simultaneous source echoes into one incident anchor id."""
    relevant = set(event_filters)
    anchor_to_id: dict[tuple[int, str, str], int] = {}
    current_id = 0
    last_ts_ns: Optional[int] = None
    last_tags: set[str] = set()
    merge_gap_ns = 20_000_000

    for anchor in anchors:
        match_tags = set(anchor["tags"]) & relevant
        if (last_ts_ns is None or
                anchor["ts_ns"] - last_ts_ns > merge_gap_ns or
                not (match_tags & last_tags)):
            current_id += 1
            last_tags = set(match_tags)
        else:
            last_tags |= match_tags
        anchor_to_id[event_key(anchor)] = current_id
        last_ts_ns = anchor["ts_ns"]
    return anchor_to_id


def select_events(events: Sequence[dict], args: argparse.Namespace) -> List[dict]:
    """Apply scalar filters first, then expand event anchors into a review window."""
    anchor_id = getattr(args, "anchor_id", None)
    base = [dict(event) for event in events if scalar_filters_match(event, args)]
    for event in base:
        event["highlight"] = False
        event["anchor_ids"] = []
    if not args.events:
        return base

    before_ns = max(0, int(args.window_before_ms * 1_000_000))
    after_ns = max(0, int(args.window_after_ms * 1_000_000))
    anchors = [event for event in base if tag_filters_match(event, args.events)]
    anchor_keys = anchor_key_map(anchors, args.events)

    if before_ns == 0 and after_ns == 0:
        for event in anchors:
            event["highlight"] = True
            event["anchor_ids"] = [anchor_keys[event_key(event)]]
        return [
            event for event in anchors
            if anchor_id is None or anchor_id in event["anchor_ids"]
        ]

    selected: List[dict] = []
    for event in base:
        anchor_ids: List[int] = []
        for anchor in anchors:
            if anchor["ts_ns"] - before_ns <= event["ts_ns"] <= anchor["ts_ns"] + after_ns:
                anchor_ids.append(anchor_keys[event_key(anchor)])
        if anchor_ids:
            event["anchor_ids"] = sorted(set(anchor_ids))
            event["highlight"] = event_key(event) in anchor_keys
            selected.append(event)
    if anchor_id is not None:
        return [event for event in selected if anchor_id in event["anchor_ids"]]
    return selected


def write_csv(path: pathlib.Path, events: Sequence[dict]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "ts_ns",
                "source",
                "tags",
                "fault_code",
                "command_status",
                "event_code",
                "highlight",
                "anchor_ids",
                "summary",
            ],
        )
        writer.writeheader()
        for event in events:
            writer.writerow(
                {
                    "ts_ns": event["ts_ns"],
                    "source": event["source"],
                    "tags": "|".join(event["tags"]),
                    "fault_code": "" if event["fault_code"] is None else event["fault_code"],
                    "command_status": "" if event["command_status"] is None else event["command_status"],
                    "event_code": "" if event["event_code"] is None else event["event_code"],
                    "highlight": int(bool(event["highlight"])),
                    "anchor_ids": "|".join(str(anchor_id) for anchor_id in event["anchor_ids"]),
                    "summary": event["summary"],
                }
            )


def print_human(events: Sequence[dict], limit: int) -> None:
    print(f"selected_events: {len(events)}")
    print("timeline:")
    if not events:
        print("  <none>")
        return
    for event in events[:limit]:
        marker = "*" if event["highlight"] else " "
        tag_str = ",".join(event["tags"]) or "-"
        anchor_str = ",".join(str(anchor_id) for anchor_id in event["anchor_ids"]) or "-"
        print(
            f"{marker} {event['ts_ns']} [{event['source']}] "
            f"anchors={anchor_str} tags={tag_str} {event['summary']}"
        )


def summarize_anchors(events: Sequence[dict]) -> List[dict]:
    anchors: List[dict] = []
    seen: set[int] = set()
    for event in events:
        if not event["highlight"]:
            continue
        for anchor_id in event["anchor_ids"]:
            if anchor_id in seen:
                continue
            seen.add(anchor_id)
            anchors.append(
                {
                    "anchor_id": anchor_id,
                    "ts_ns": event["ts_ns"],
                    "source": event["source"],
                    "tags": list(event["tags"]),
                    "summary": event["summary"],
                }
            )
    anchors.sort(key=lambda item: item["anchor_id"])
    return anchors


def selected_range(events: Sequence[dict]) -> tuple[Optional[int], Optional[int]]:
    if not events:
        return (None, None)
    return (min(event["ts_ns"] for event in events), max(event["ts_ns"] for event in events))


def write_binary_slice(
    src_path: pathlib.Path,
    ctype: type[ctypes.Structure],
    out_path: pathlib.Path,
    start_ns: int,
    end_ns: int,
    timestamp_of,
) -> int:
    records = [
        record for record in read_binary_records(src_path, ctype)
        if start_ns <= int(timestamp_of(record)) <= end_ns
    ]
    if not records:
        return 0
    out_path.write_bytes(b"".join(bytes(record) for record in records))
    return len(records)


def write_csv_slice(
    src_path: pathlib.Path,
    out_path: pathlib.Path,
    start_ns: int,
    end_ns: int,
    timestamp_key: str,
) -> int:
    rows = [
        row for row in csv_rows(src_path)
        if start_ns <= int_field(row, timestamp_key) <= end_ns
    ]
    if not rows:
        return 0
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def write_nav_timing_slice(
    src_path: pathlib.Path,
    out_path: pathlib.Path,
    start_ns: int,
    end_ns: int,
) -> int:
    rows: List[dict] = []
    for record in parse_nav_timing.read_records(src_path):
        ts_ns = record["publish_mono_ns"] or record["consume_mono_ns"] or record["recv_mono_ns"]
        if not (start_ns <= ts_ns <= end_ns):
            continue
        rows.append(
            {
                "ts_ns": ts_ns,
                "kind": record["kind"],
                "kind_name": record["kind_name"],
                "sensor_time_ns": record["sensor_time_ns"],
                "recv_mono_ns": record["recv_mono_ns"],
                "consume_mono_ns": record["consume_mono_ns"],
                "publish_mono_ns": record["publish_mono_ns"],
                "age_ms": record["age_ms"],
                "fault_code": record["fault_code"],
                "flags": "|".join(record["flag_names"]),
            }
        )
    if not rows:
        return 0
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def nav_state_semantic_signature(record: NavState) -> tuple:
    """Project NavState to replay-relevant fields, ignoring padding bytes."""

    return (
        int(record.t_ns),
        tuple(float(value) for value in record.pos),
        tuple(float(value) for value in record.vel),
        tuple(float(value) for value in record.rpy),
        float(record.depth),
        tuple(float(value) for value in record.omega_b),
        tuple(float(value) for value in record.acc_b),
        int(record.age_ms),
        int(record.valid),
        int(record.stale),
        int(record.degraded),
        int(record.nav_state),
        int(record.health),
        int(record.fault_code),
        int(record.sensor_mask),
        int(record.status_flags),
    )


def write_nav_state_replay_slice(
    src_path: pathlib.Path,
    out_path: pathlib.Path,
    start_ns: int,
    end_ns: int,
) -> Optional[dict[str, int | str]]:
    """Export a replayable NavState slice for the selected incident window.

    Normal incident windows use `NavState::t_ns` as the selection key. Real bench
    captures on the fully invalid/no-device path can keep `t_ns == 0` forever
    because the estimator never produced a valid state stamp. In that case replay
    timing cannot be reconstructed from the log itself, but downstream
    propagation/failsafe verification still benefits from one representative
    snapshot. The fallback therefore exports a single constant frame only when:
      - the normal time-window slice is empty,
      - every record has `t_ns == 0`, and
      - every record is byte-identical.
    """

    records = read_binary_records(src_path, NavState)
    in_window = [record for record in records if start_ns <= int(record.t_ns) <= end_ns]
    if in_window:
        out_path.write_bytes(b"".join(bytes(record) for record in in_window))
        return {
            "records": len(in_window),
            "selection_mode": "time_window",
        }

    if not records:
        return None

    first_record = records[0]
    if any(int(record.t_ns) != 0 for record in records):
        return None
    first_signature = nav_state_semantic_signature(first_record)
    if any(nav_state_semantic_signature(record) != first_signature for record in records[1:]):
        return None

    out_path.write_bytes(bytes(first_record))
    return {
        "records": 1,
        "selection_mode": "constant_zero_t_ns_fallback",
        "source_records": len(records),
    }


def export_incident_bundle(
    bundle_dir: pathlib.Path,
    args: argparse.Namespace,
    selected: Sequence[dict],
    summary: dict,
) -> None:
    start_ns, end_ns = selected_range(selected)
    if start_ns is None or end_ns is None:
        raise ValueError("cannot export incident bundle without selected events")

    bundle_dir.mkdir(parents=True, exist_ok=True)
    write_csv(bundle_dir / "incident_timeline.csv", selected)

    bundle_files: dict[str, dict[str, object]] = {
        "incident_timeline.csv": {
            "kind": "merged_timeline",
            "records": len(selected),
        }
    }

    if args.nav_timing:
        count = write_nav_timing_slice(
            args.nav_timing,
            bundle_dir / "nav_timing_window.csv",
            start_ns,
            end_ns,
        )
        if count:
            bundle_files["nav_timing_window.csv"] = {"kind": "nav_timing_csv", "records": count}

    if args.nav_bin:
        count = write_binary_slice(
            args.nav_bin,
            DvlFrame,
            bundle_dir / "nav_bin_window.bin",
            start_ns,
            end_ns,
            lambda record: record.mono_ns,
        )
        if count:
            bundle_files["nav_bin_window.bin"] = {"kind": "nav_bin_binary", "records": count}

    if args.nav_state:
        nav_state_meta = write_nav_state_replay_slice(
            args.nav_state,
            bundle_dir / "nav_state_window.bin",
            start_ns,
            end_ns,
        )
        if nav_state_meta:
            bundle_files["nav_state_window.bin"] = {
                "kind": "nav_state_binary",
                "replay_entrypoint": True,
                **nav_state_meta,
            }

    if args.control_log:
        count = write_csv_slice(
            args.control_log,
            bundle_dir / "control_window.csv",
            start_ns,
            end_ns,
            "MonoNS",
        )
        if count:
            bundle_files["control_window.csv"] = {"kind": "control_csv", "records": count}

    if args.telemetry_timeline:
        count = write_csv_slice(
            args.telemetry_timeline,
            bundle_dir / "telemetry_timeline_window.csv",
            start_ns,
            end_ns,
            "telemetry_stamp_ns",
        )
        if count:
            bundle_files["telemetry_timeline_window.csv"] = {
                "kind": "telemetry_timeline_csv",
                "records": count,
            }

    if args.telemetry_events:
        count = write_csv_slice(
            args.telemetry_events,
            bundle_dir / "telemetry_events_window.csv",
            start_ns,
            end_ns,
            "stamp_ns",
        )
        if count:
            bundle_files["telemetry_events_window.csv"] = {
                "kind": "telemetry_events_csv",
                "records": count,
            }

    bundle_summary = dict(summary)
    bundle_summary["selected_window"] = {"from_ns": start_ns, "to_ns": end_ns}
    bundle_summary["bundle_files"] = bundle_files
    bundle_summary["replay"] = {
        "inject_layer": "nav_state_shm_upstream_of_nav_viewd",
        "input_file": "nav_state_window.bin" if "nav_state_window.bin" in bundle_files else None,
        "tool_hint": "uwnav_nav_replay --incident-bundle <bundle_dir>",
    }
    with (bundle_dir / "incident_summary.json").open("w", encoding="utf-8") as f:
        json.dump(bundle_summary, f, indent=2)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-timing", type=pathlib.Path)
    parser.add_argument("--nav-bin", type=pathlib.Path)
    parser.add_argument("--nav-state", type=pathlib.Path)
    parser.add_argument("--control-log", type=pathlib.Path)
    parser.add_argument("--telemetry-timeline", type=pathlib.Path)
    parser.add_argument("--telemetry-events", type=pathlib.Path)
    parser.add_argument("--source", dest="sources", action="append",
                        choices=["nav_timing", "nav_bin", "nav_state", "control",
                                 "telemetry_timeline", "telemetry_events"])
    parser.add_argument("--event", dest="events", action="append", choices=EVENT_CHOICES)
    parser.add_argument("--anchor-id", type=int)
    parser.add_argument("--window-before-ms", type=int, default=0)
    parser.add_argument("--window-after-ms", type=int, default=0)
    parser.add_argument("--from-ns", type=int)
    parser.add_argument("--to-ns", type=int)
    parser.add_argument("--fault-code", type=int)
    parser.add_argument("--command-status", type=int)
    parser.add_argument("--csv-out", type=pathlib.Path)
    parser.add_argument("--bundle-dir", type=pathlib.Path)
    parser.add_argument("--limit", type=int, default=50)
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: List[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.anchor_id is not None and args.anchor_id <= 0:
        parser.error("--anchor-id must be >= 1")
    if args.anchor_id is not None and not args.events:
        parser.error("--anchor-id requires at least one --event filter")

    events = merge_events(args)
    selected = select_events(events, args)
    anchors = summarize_anchors(selected)

    summary = {
        "total_events": len(events),
        "selected_events": len(selected),
        "sources": sorted({item["source"] for item in selected}),
        "anchors": anchors,
        "filters": {
            "sources": args.sources or [],
            "events": args.events or [],
            "anchor_id": args.anchor_id,
            "from_ns": args.from_ns,
            "to_ns": args.to_ns,
            "fault_code": args.fault_code,
            "command_status": args.command_status,
            "window_before_ms": args.window_before_ms,
            "window_after_ms": args.window_after_ms,
        },
        "timeline": selected[: args.limit],
    }

    if args.csv_out:
        write_csv(args.csv_out, selected)
    if args.bundle_dir:
        export_incident_bundle(args.bundle_dir, args, selected, summary)

    if args.json:
        json.dump(summary, sys.stdout, indent=2)
        print()
    else:
        print(f"total_events: {summary['total_events']}")
        print(f"selected_events: {summary['selected_events']}")
        print("sources:")
        for source in summary["sources"]:
            print(f"  {source}")
        print_human(selected, args.limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
