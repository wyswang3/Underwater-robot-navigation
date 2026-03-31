#!/usr/bin/env python3
"""Capture the current USB serial identity view used by DeviceBinder.

This tool intentionally mirrors the binding scan order in `device_binding.cpp`:
- prefer `/dev/serial/by-id/*`
- also include `/dev/ttyUSB*` and `/dev/ttyACM*`
- resolve VID/PID/serial from `/sys/class/tty/<tty>/device` parents

It is designed for bench bring-up, not for permanent hardware automation.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
from typing import Iterable


def trim_text(path: pathlib.Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def lowercase(value: str) -> str:
    return value.lower()


def resolve_tty_name(path: pathlib.Path) -> str:
    try:
        resolved = path.resolve(strict=False)
    except OSError:
        resolved = path
    return resolved.name or path.name


def load_identity(path: pathlib.Path, sys_root: pathlib.Path) -> dict:
    tty_name = resolve_tty_name(path)
    identity = {
        "path": str(path),
        "canonical_path": str(path.resolve(strict=False)),
        "vendor_id": "",
        "product_id": "",
        "serial": "",
    }
    if not tty_name:
        return identity

    current = (sys_root / tty_name / "device").resolve(strict=False)
    visited: set[pathlib.Path] = set()
    while current not in visited and current != current.parent:
        visited.add(current)
        vendor_id = trim_text(current / "idVendor")
        product_id = trim_text(current / "idProduct")
        serial = trim_text(current / "serial")
        if vendor_id or product_id or serial:
            identity["vendor_id"] = lowercase(vendor_id)
            identity["product_id"] = lowercase(product_id)
            identity["serial"] = serial
            break
        current = current.parent
    return identity


def discover_paths(dev_root: pathlib.Path) -> Iterable[pathlib.Path]:
    by_id = dev_root / "serial" / "by-id"
    if by_id.exists():
        for entry in sorted(by_id.iterdir()):
            if entry.is_symlink() or entry.is_char_device():
                yield entry

    for entry in sorted(dev_root.iterdir()):
        name = entry.name
        if name.startswith("ttyUSB") or name.startswith("ttyACM"):
            yield entry


def scan_serial_snapshot(dev_root: pathlib.Path, sys_root: pathlib.Path) -> list[dict]:
    devices: list[dict] = []
    seen: set[str] = set()
    for path in discover_paths(dev_root):
        identity = load_identity(path, sys_root)
        key = identity["canonical_path"] or identity["path"]
        if key in seen:
            continue
        seen.add(key)
        devices.append(identity)
    return devices


def print_table(devices: list[dict]) -> None:
    if not devices:
        print("no serial devices discovered")
        return
    print("path,canonical_path,vendor_id,product_id,serial")
    for device in devices:
        print(
            ",".join(
                [
                    device["path"],
                    device["canonical_path"],
                    device["vendor_id"],
                    device["product_id"],
                    device["serial"],
                ]
            )
        )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dev-root", type=pathlib.Path, default=pathlib.Path("/dev"))
    parser.add_argument(
        "--sys-root",
        type=pathlib.Path,
        default=pathlib.Path("/sys/class/tty"),
        help="directory that contains <tty>/device",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if not args.dev_root.exists():
        raise SystemExit(f"dev root not found: {args.dev_root}")
    if not args.sys_root.exists():
        raise SystemExit(f"sys root not found: {args.sys_root}")

    devices = scan_serial_snapshot(args.dev_root, args.sys_root)
    if args.json:
        print(json.dumps(devices, indent=2))
    else:
        print_table(devices)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(os.sys.argv[1:]))
