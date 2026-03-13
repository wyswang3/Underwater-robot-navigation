from __future__ import annotations

import argparse
import csv
import pathlib
import tempfile
import unittest
import sys


TOOLS_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import merge_robot_timeline  # noqa: E402
import parse_nav_timing  # noqa: E402


def write_text(path: pathlib.Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


class MergeRobotTimelineTests(unittest.TestCase):
    def test_event_window_selects_surrounding_events_and_csv_export(self) -> None:
        with tempfile.TemporaryDirectory(prefix="merge_robot_timeline_") as td:
            root = pathlib.Path(td)
            nav_timing = root / "nav_timing.bin"
            control_log = root / "control.csv"
            telemetry_events = root / "telemetry_events.csv"
            csv_out = root / "window.csv"

            nav_timing.write_bytes(
                parse_nav_timing.RECORD.pack(
                    1,
                    6,  # imu_device_state
                    parse_nav_timing.kTimingTraceDeviceReconnecting
                    if hasattr(parse_nav_timing, "kTimingTraceDeviceReconnecting")
                    else (1 << 9),
                    0,
                    0,
                    0,
                    1_000_000_000,
                    0xFFFFFFFF,
                    6,  # RECONNECTING
                )
            )

            write_text(
                control_log,
                "MonoNS,effective_mode,armed,failsafe,nav_valid,nav_stale,nav_degraded,"
                "nav_age_ms,nav_fault_code,nav_status_flags\n"
                "1020000000,Failsafe,1,1,0,0,0,10,12,1024\n",
            )
            write_text(
                telemetry_events,
                "kind,stamp_ns,seq_or_cmd,code,fault_code,arg0,arg1,intent_id,source,status\n"
                "command_result,1030000000,9,4,12,0,0,77,1,5\n",
            )

            args = argparse.Namespace(
                nav_timing=nav_timing,
                nav_bin=None,
                nav_state=None,
                control_log=control_log,
                telemetry_timeline=None,
                telemetry_events=telemetry_events,
                sources=None,
                events=["reconnecting"],
                window_before_ms=5,
                window_after_ms=40,
                from_ns=None,
                to_ns=None,
                fault_code=None,
                command_status=None,
                csv_out=csv_out,
                limit=10,
                json=False,
            )

            merged = merge_robot_timeline.merge_events(args)
            selected = merge_robot_timeline.select_events(merged, args)
            merge_robot_timeline.write_csv(csv_out, selected)

            self.assertEqual([item["source"] for item in selected],
                             ["nav_timing", "control", "telemetry_events"])
            self.assertTrue(selected[0]["highlight"])
            self.assertTrue(selected[1]["highlight"])
            self.assertFalse(selected[2]["highlight"])

            with csv_out.open(newline="") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]["highlight"], "1")

    def test_command_failed_filter_targets_runtime_result_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="merge_robot_timeline_cmd_") as td:
            root = pathlib.Path(td)
            telemetry_events = root / "telemetry_events.csv"
            write_text(
                telemetry_events,
                "kind,stamp_ns,seq_or_cmd,code,fault_code,arg0,arg1,intent_id,source,status\n"
                "command_result,100,1,1,0,0,0,10,1,1\n"
                "command_result,200,2,4,12,0,0,11,1,5\n",
            )

            args = argparse.Namespace(
                nav_timing=None,
                nav_bin=None,
                nav_state=None,
                control_log=None,
                telemetry_timeline=None,
                telemetry_events=telemetry_events,
                sources=None,
                events=["command_failed"],
                window_before_ms=0,
                window_after_ms=0,
                from_ns=None,
                to_ns=None,
                fault_code=None,
                command_status=None,
                csv_out=None,
                limit=10,
                json=False,
            )

            merged = merge_robot_timeline.merge_events(args)
            selected = merge_robot_timeline.select_events(merged, args)
            self.assertEqual(len(selected), 1)
            self.assertEqual(selected[0]["command_status"], 5)
            self.assertIn("command_failed", selected[0]["tags"])


if __name__ == "__main__":
    unittest.main()
