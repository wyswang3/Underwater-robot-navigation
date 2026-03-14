#!/usr/bin/env python3
"""Compare an incident bundle with replay outputs using collapsed state signatures.

The comparison is intentionally minimal and P1-oriented:
- Use the incident bundle's exported control / telemetry CSVs as the original
  authority for the selected window.
- Ignore exact sample cadence and timestamps.
- Collapse consecutive duplicate rows into state signatures.
- Search the original signature sequence inside replay outputs so replay startup
  prefixes (for example, `no_nav` before the first injected frame) do not create
  false mismatches.
"""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import sys
from typing import Callable, Iterable, Optional, Sequence


CONTROL_FIELDS = (
    "has_nav",
    "nav_present",
    "nav_valid",
    "nav_stale",
    "nav_degraded",
    "failsafe",
    "nav_state",
    "nav_health",
    "nav_fault_code",
    "nav_status_flags",
)

TELEMETRY_FIELDS = (
    "failsafe_active",
    "nav_valid",
    "nav_stale",
    "nav_degraded",
    "nav_state",
    "nav_health",
    "nav_fault_code",
    "nav_status_flags",
    "health_state",
    "fault_state",
    "last_fault_code",
    "cmd_status",
    "cmd_fault_code",
)

EVENT_FIELDS = (
    "kind",
    "code",
    "fault_code",
    "status",
)


def csv_rows(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def collapse_signatures(
    rows: Iterable[dict[str, str]],
    fields: Sequence[str],
    row_filter: Optional[Callable[[dict[str, str]], bool]] = None,
) -> list[dict[str, str]]:
    signatures: list[dict[str, str]] = []
    for row in rows:
        if row_filter and not row_filter(row):
            continue
        signature = {field: row.get(field, "") for field in fields}
        if not signatures or signature != signatures[-1]:
            signatures.append(signature)
    return signatures


def find_subsequence(
    expected: Sequence[dict[str, str]],
    actual: Sequence[dict[str, str]],
) -> Optional[int]:
    if not expected:
        return 0 if not actual else None
    if len(expected) > len(actual):
        return None
    for start in range(len(actual) - len(expected) + 1):
        if list(actual[start : start + len(expected)]) == list(expected):
            return start
    return None


def compare_channel(
    *,
    name: str,
    expected_path: pathlib.Path,
    actual_path: Optional[pathlib.Path],
    fields: Sequence[str],
    row_filter: Optional[Callable[[dict[str, str]], bool]] = None,
) -> dict[str, object]:
    if not expected_path.exists():
        return {
            "channel": name,
            "available": False,
            "match": True,
            "reason": "original channel not present in incident bundle",
        }

    expected_rows = csv_rows(expected_path)
    expected_signatures = collapse_signatures(expected_rows, fields, row_filter=row_filter)

    if actual_path is None or not actual_path.exists():
        return {
            "channel": name,
            "available": True,
            "match": False,
            "reason": "replay output file missing",
            "expected_signatures": expected_signatures,
            "actual_signatures": [],
        }

    actual_rows = csv_rows(actual_path)
    actual_signatures = collapse_signatures(actual_rows, fields, row_filter=row_filter)
    match_start = find_subsequence(expected_signatures, actual_signatures)

    return {
        "channel": name,
        "available": True,
        "match": match_start is not None,
        "match_start_index": match_start,
        "expected_row_count": len(expected_rows),
        "actual_row_count": len(actual_rows),
        "expected_signature_count": len(expected_signatures),
        "actual_signature_count": len(actual_signatures),
        "expected_signatures": expected_signatures,
        "actual_signatures": actual_signatures,
    }


def compare_bundle(
    *,
    incident_bundle: pathlib.Path,
    replay_control_log: pathlib.Path,
    replay_telemetry_timeline: pathlib.Path,
    replay_telemetry_events: Optional[pathlib.Path] = None,
) -> dict[str, object]:
    channels = {
        "control": compare_channel(
            name="control",
            expected_path=incident_bundle / "control_window.csv",
            actual_path=replay_control_log,
            fields=CONTROL_FIELDS,
        ),
        "telemetry_timeline": compare_channel(
            name="telemetry_timeline",
            expected_path=incident_bundle / "telemetry_timeline_window.csv",
            actual_path=replay_telemetry_timeline,
            fields=TELEMETRY_FIELDS,
        ),
        "telemetry_events": compare_channel(
            name="telemetry_events",
            expected_path=incident_bundle / "telemetry_events_window.csv",
            actual_path=replay_telemetry_events,
            fields=EVENT_FIELDS,
            row_filter=lambda row: row.get("kind") == "command_result",
        ),
    }

    overall_match = all(result["match"] for result in channels.values() if result["available"])
    return {
        "incident_bundle": str(incident_bundle),
        "compare_mode": "collapsed_signature_subsequence",
        "notes": [
            "timestamps and sample counts are informational only",
            "startup prefixes are allowed when the original signature sequence appears intact later",
            "this does not compare raw IMU/DVL frames or exact replay cadence",
        ],
        "channels": channels,
        "overall_match": overall_match,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--incident-bundle", type=pathlib.Path, required=True)
    parser.add_argument("--replay-control-log", type=pathlib.Path, required=True)
    parser.add_argument("--replay-telemetry-timeline", type=pathlib.Path, required=True)
    parser.add_argument("--replay-telemetry-events", type=pathlib.Path)
    parser.add_argument("--json-out", type=pathlib.Path)
    parser.add_argument("--json", action="store_true")
    return parser


def print_text_report(report: dict[str, object]) -> None:
    print(f"incident_bundle: {report['incident_bundle']}")
    print(f"compare_mode: {report['compare_mode']}")
    print(f"overall_match: {int(bool(report['overall_match']))}")
    for channel, result in report["channels"].items():
        if not result["available"]:
            print(f"{channel}: skipped ({result['reason']})")
            continue
        print(
            f"{channel}: match={int(bool(result['match']))} "
            f"expected_signatures={result.get('expected_signature_count', 0)} "
            f"actual_signatures={result.get('actual_signature_count', 0)} "
            f"start_index={result.get('match_start_index')}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    report = compare_bundle(
        incident_bundle=args.incident_bundle,
        replay_control_log=args.replay_control_log,
        replay_telemetry_timeline=args.replay_telemetry_timeline,
        replay_telemetry_events=args.replay_telemetry_events,
    )

    if args.json_out:
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print_text_report(report)

    return 0 if report["overall_match"] else 1


if __name__ == "__main__":
    sys.exit(main())
