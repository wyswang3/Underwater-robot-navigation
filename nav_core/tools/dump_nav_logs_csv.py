#!/usr/bin/env python3
"""Dump nav binary logs into CSV for analysis/ML workflows.

Supports:
- nav_state.bin -> nav_state.csv
- nav.bin       -> nav_bin.csv
- nav_timing.bin-> nav_timing.csv
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys
from typing import Iterable, List, Optional

import merge_robot_timeline as mrt
import parse_nav_timing


def ensure_out_path(path: pathlib.Path, overwrite: bool) -> pathlib.Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not overwrite:
        raise RuntimeError(f"output already exists: {path} (use --overwrite to replace)")
    return path


def write_nav_state_csv(records: List[mrt.NavState], path: pathlib.Path) -> int:
    fields = [
        "t_ns",
        "pos_x",
        "pos_y",
        "pos_z",
        "vel_x",
        "vel_y",
        "vel_z",
        "roll",
        "pitch",
        "yaw",
        "depth",
        "omega_b_x",
        "omega_b_y",
        "omega_b_z",
        "acc_b_x",
        "acc_b_y",
        "acc_b_z",
        "age_ms",
        "valid",
        "stale",
        "degraded",
        "nav_state",
        "health",
        "fault_code",
        "sensor_mask",
        "status_flags",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            writer.writerow(
                {
                    "t_ns": int(rec.t_ns),
                    "pos_x": float(rec.pos[0]),
                    "pos_y": float(rec.pos[1]),
                    "pos_z": float(rec.pos[2]),
                    "vel_x": float(rec.vel[0]),
                    "vel_y": float(rec.vel[1]),
                    "vel_z": float(rec.vel[2]),
                    "roll": float(rec.rpy[0]),
                    "pitch": float(rec.rpy[1]),
                    "yaw": float(rec.rpy[2]),
                    "depth": float(rec.depth),
                    "omega_b_x": float(rec.omega_b[0]),
                    "omega_b_y": float(rec.omega_b[1]),
                    "omega_b_z": float(rec.omega_b[2]),
                    "acc_b_x": float(rec.acc_b[0]),
                    "acc_b_y": float(rec.acc_b[1]),
                    "acc_b_z": float(rec.acc_b[2]),
                    "age_ms": int(rec.age_ms),
                    "valid": int(rec.valid),
                    "stale": int(rec.stale),
                    "degraded": int(rec.degraded),
                    "nav_state": int(rec.nav_state),
                    "health": int(rec.health),
                    "fault_code": int(rec.fault_code),
                    "sensor_mask": int(rec.sensor_mask),
                    "status_flags": int(rec.status_flags),
                }
            )
    return len(records)


def write_nav_bin_csv(records: List[mrt.DvlFrame], path: pathlib.Path) -> int:
    fields = [
        "sensor_time_ns",
        "recv_mono_ns",
        "consume_mono_ns",
        "mono_ns",
        "est_ns",
        "bottom_lock",
        "track_type",
        "vel_body_x",
        "vel_body_y",
        "vel_body_z",
        "has_body_vel",
        "vel_enu_x",
        "vel_enu_y",
        "vel_enu_z",
        "has_enu_vel",
        "dist_enu_x",
        "dist_enu_y",
        "dist_enu_z",
        "has_dist_enu",
        "altitude_m",
        "has_altitude",
        "fom",
        "valid",
        "quality",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            writer.writerow(
                {
                    "sensor_time_ns": int(rec.sensor_time_ns),
                    "recv_mono_ns": int(rec.recv_mono_ns),
                    "consume_mono_ns": int(rec.consume_mono_ns),
                    "mono_ns": int(rec.mono_ns),
                    "est_ns": int(rec.est_ns),
                    "bottom_lock": int(bool(rec.bottom_lock)),
                    "track_type": int(rec.track_type),
                    "vel_body_x": float(rec.vel_body_mps[0]),
                    "vel_body_y": float(rec.vel_body_mps[1]),
                    "vel_body_z": float(rec.vel_body_mps[2]),
                    "has_body_vel": int(bool(rec.has_body_vel)),
                    "vel_enu_x": float(rec.vel_enu_mps[0]),
                    "vel_enu_y": float(rec.vel_enu_mps[1]),
                    "vel_enu_z": float(rec.vel_enu_mps[2]),
                    "has_enu_vel": int(bool(rec.has_enu_vel)),
                    "dist_enu_x": float(rec.dist_enu_m[0]),
                    "dist_enu_y": float(rec.dist_enu_m[1]),
                    "dist_enu_z": float(rec.dist_enu_m[2]),
                    "has_dist_enu": int(bool(rec.has_dist_enu)),
                    "altitude_m": float(rec.altitude_m),
                    "has_altitude": int(bool(rec.has_altitude)),
                    "fom": float(rec.fom),
                    "valid": int(bool(rec.valid)),
                    "quality": int(rec.quality),
                }
            )
    return len(records)


def write_nav_timing_csv(records: Iterable[dict], path: pathlib.Path) -> int:
    fields = [
        "index",
        "version",
        "kind",
        "kind_name",
        "flags",
        "flag_names",
        "sensor_time_ns",
        "recv_mono_ns",
        "consume_mono_ns",
        "publish_mono_ns",
        "age_ms",
        "fault_code",
    ]
    count = 0
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            writer.writerow(
                {
                    "index": int(rec["index"]),
                    "version": int(rec["version"]),
                    "kind": int(rec["kind"]),
                    "kind_name": rec["kind_name"],
                    "flags": int(rec["flags"]),
                    "flag_names": "|".join(rec["flag_names"]),
                    "sensor_time_ns": int(rec["sensor_time_ns"]),
                    "recv_mono_ns": int(rec["recv_mono_ns"]),
                    "consume_mono_ns": int(rec["consume_mono_ns"]),
                    "publish_mono_ns": int(rec["publish_mono_ns"]),
                    "age_ms": "" if rec["age_ms"] is None else int(rec["age_ms"]),
                    "fault_code": int(rec["fault_code"]),
                }
            )
            count += 1
    return count


def nav_published_records(records: Iterable[dict]) -> List[dict]:
    return [rec for rec in records if rec.get("kind_name") == "nav_published"]


def write_nav_state_timeline_csv(
    nav_states: List[mrt.NavState],
    timing_records: List[dict],
    path: pathlib.Path,
) -> int:
    fields = [
        "row_index",
        "t_ns",
        "pos_x",
        "pos_y",
        "pos_z",
        "vel_x",
        "vel_y",
        "vel_z",
        "roll",
        "pitch",
        "yaw",
        "depth",
        "omega_b_x",
        "omega_b_y",
        "omega_b_z",
        "acc_b_x",
        "acc_b_y",
        "acc_b_z",
        "age_ms",
        "valid",
        "stale",
        "degraded",
        "nav_state",
        "health",
        "fault_code",
        "sensor_mask",
        "status_flags",
        "timing_index",
        "timing_flags",
        "timing_flag_names",
        "sensor_time_ns",
        "recv_mono_ns",
        "consume_mono_ns",
        "publish_mono_ns",
        "timing_age_ms",
        "timing_fault_code",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        count = min(len(nav_states), len(timing_records))
        for idx in range(count):
            nav = nav_states[idx]
            timing = timing_records[idx]
            writer.writerow(
                {
                    "row_index": idx,
                    "t_ns": int(nav.t_ns),
                    "pos_x": float(nav.pos[0]),
                    "pos_y": float(nav.pos[1]),
                    "pos_z": float(nav.pos[2]),
                    "vel_x": float(nav.vel[0]),
                    "vel_y": float(nav.vel[1]),
                    "vel_z": float(nav.vel[2]),
                    "roll": float(nav.rpy[0]),
                    "pitch": float(nav.rpy[1]),
                    "yaw": float(nav.rpy[2]),
                    "depth": float(nav.depth),
                    "omega_b_x": float(nav.omega_b[0]),
                    "omega_b_y": float(nav.omega_b[1]),
                    "omega_b_z": float(nav.omega_b[2]),
                    "acc_b_x": float(nav.acc_b[0]),
                    "acc_b_y": float(nav.acc_b[1]),
                    "acc_b_z": float(nav.acc_b[2]),
                    "age_ms": int(nav.age_ms),
                    "valid": int(nav.valid),
                    "stale": int(nav.stale),
                    "degraded": int(nav.degraded),
                    "nav_state": int(nav.nav_state),
                    "health": int(nav.health),
                    "fault_code": int(nav.fault_code),
                    "sensor_mask": int(nav.sensor_mask),
                    "status_flags": int(nav.status_flags),
                    "timing_index": int(timing["index"]),
                    "timing_flags": int(timing["flags"]),
                    "timing_flag_names": "|".join(timing["flag_names"]),
                    "sensor_time_ns": int(timing["sensor_time_ns"]),
                    "recv_mono_ns": int(timing["recv_mono_ns"]),
                    "consume_mono_ns": int(timing["consume_mono_ns"]),
                    "publish_mono_ns": int(timing["publish_mono_ns"]),
                    "timing_age_ms": "" if timing["age_ms"] is None else int(timing["age_ms"]),
                    "timing_fault_code": int(timing["fault_code"]),
                }
            )
    return count


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-state", type=pathlib.Path, help="nav_state.bin path")
    parser.add_argument("--nav-bin", type=pathlib.Path, help="nav.bin path")
    parser.add_argument("--nav-timing", type=pathlib.Path, help="nav_timing.bin path")
    parser.add_argument("--out-dir", type=pathlib.Path, default=None)
    parser.add_argument("--overwrite", action="store_true", help="overwrite existing CSVs")
    parser.add_argument(
        "--merge-nav-state-timing",
        action="store_true",
        help="emit nav_state_timeline.csv by aligning nav_state.bin with nav_timing.bin",
    )
    return parser


def resolve_out_dir(inputs: List[pathlib.Path], out_dir: Optional[pathlib.Path]) -> pathlib.Path:
    if out_dir is not None:
        return out_dir
    if inputs:
        return inputs[0].parent
    raise RuntimeError("no inputs provided")


def main(argv: List[str]) -> int:
    args = build_parser().parse_args(argv)
    inputs = [p for p in [args.nav_state, args.nav_bin, args.nav_timing] if p is not None]
    if not inputs:
        print("[dump_nav_logs_csv][ERR] specify at least one input", file=sys.stderr)
        return 2

    out_dir = resolve_out_dir(inputs, args.out_dir)

    if args.nav_state:
        records = mrt.read_binary_records(args.nav_state, mrt.NavState)
        out_path = ensure_out_path(out_dir / "nav_state.csv", args.overwrite)
        count = write_nav_state_csv(records, out_path)
        print(f"[dump_nav_logs_csv] nav_state: {count} rows -> {out_path}")

    if args.nav_bin:
        records = mrt.read_binary_records(args.nav_bin, mrt.DvlFrame)
        out_path = ensure_out_path(out_dir / "nav_bin.csv", args.overwrite)
        count = write_nav_bin_csv(records, out_path)
        print(f"[dump_nav_logs_csv] nav_bin: {count} rows -> {out_path}")

    if args.nav_timing:
        records = parse_nav_timing.read_records(args.nav_timing)
        out_path = ensure_out_path(out_dir / "nav_timing.csv", args.overwrite)
        count = write_nav_timing_csv(records, out_path)
        print(f"[dump_nav_logs_csv] nav_timing: {count} rows -> {out_path}")

    if args.merge_nav_state_timing:
        if not args.nav_state or not args.nav_timing:
            print(
                "[dump_nav_logs_csv][ERR] --merge-nav-state-timing requires --nav-state and --nav-timing",
                file=sys.stderr,
            )
            return 3
        nav_states = mrt.read_binary_records(args.nav_state, mrt.NavState)
        timing = parse_nav_timing.read_records(args.nav_timing)
        published = nav_published_records(timing)
        if len(nav_states) != len(published):
            print(
                f"[dump_nav_logs_csv][WARN] nav_state rows={len(nav_states)} "
                f"nav_published rows={len(published)}; writing min length",
                file=sys.stderr,
            )
        out_path = ensure_out_path(out_dir / "nav_state_timeline.csv", args.overwrite)
        count = write_nav_state_timeline_csv(nav_states, published, out_path)
        print(f"[dump_nav_logs_csv] nav_state_timeline: {count} rows -> {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
