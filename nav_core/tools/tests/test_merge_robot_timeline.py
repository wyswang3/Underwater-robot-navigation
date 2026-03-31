from __future__ import annotations

import argparse
import csv
import ctypes
import json
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


def write_nav_state_file(path: pathlib.Path, records: list[merge_robot_timeline.NavState]) -> None:
    path.write_bytes(b"".join(bytes(record) for record in records))


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

    def test_bundle_export_keeps_anchor_ids_and_replay_input(self) -> None:
        with tempfile.TemporaryDirectory(prefix="merge_robot_timeline_bundle_") as td:
            root = pathlib.Path(td)
            nav_state = root / "nav_state.bin"
            control_log = root / "control.csv"
            telemetry_timeline = root / "telemetry_timeline.csv"
            bundle_dir = root / "bundle"

            ok = merge_robot_timeline.NavState()
            ok.t_ns = 1_000_000_000
            ok.valid = 1
            ok.stale = 0
            ok.degraded = 0
            ok.nav_state = 2
            ok.health = 1
            ok.age_ms = 10

            reconnecting = merge_robot_timeline.NavState()
            reconnecting.t_ns = 1_040_000_000
            reconnecting.valid = 0
            reconnecting.stale = 0
            reconnecting.degraded = 0
            reconnecting.nav_state = 4
            reconnecting.health = 3
            reconnecting.age_ms = 20
            reconnecting.fault_code = 12
            reconnecting.status_flags = merge_robot_timeline.NAV_FLAG_IMU_RECONNECTING

            mismatch = merge_robot_timeline.NavState()
            mismatch.t_ns = 1_090_000_000
            mismatch.valid = 0
            mismatch.stale = 0
            mismatch.degraded = 0
            mismatch.nav_state = 4
            mismatch.health = 3
            mismatch.age_ms = 30
            mismatch.fault_code = 14
            mismatch.status_flags = merge_robot_timeline.NAV_FLAG_DVL_BIND_MISMATCH

            write_nav_state_file(nav_state, [ok, reconnecting, mismatch])
            write_text(
                control_log,
                "MonoNS,effective_mode,armed,failsafe,nav_valid,nav_stale,nav_degraded,"
                "nav_age_ms,nav_fault_code,nav_status_flags\n"
                "1045000000,Failsafe,1,1,0,0,0,20,12,1024\n"
                "1095000000,Failsafe,1,1,0,0,0,30,14,512\n",
            )
            write_text(
                telemetry_timeline,
                "telemetry_stamp_ns,telemetry_seq,active_mode,armed,estop_latched,"
                "failsafe_active,controller_name,desired_controller,nav_valid,nav_state,"
                "nav_health,nav_stale,nav_degraded,nav_age_ms,nav_fault_code,"
                "nav_status_flags,health_state,degraded,fault_state,last_fault_code,"
                "stm32_link_state,pwm_link_state,heartbeat_age_ms,cmd_status,"
                "cmd_fault_code,event_seq,event_code,event_fault_code\n"
                "1046000000,1,4,1,0,1,pid,pid,0,1,3,0,0,20,12,1024,3,0,1,4,2,2,0,2,4,1,4,4\n"
                "1096000000,2,4,1,0,1,pid,pid,0,1,3,0,0,30,14,512,3,0,1,4,2,2,0,2,4,2,4,4\n",
            )

            args = argparse.Namespace(
                nav_timing=None,
                nav_bin=None,
                nav_state=nav_state,
                control_log=control_log,
                telemetry_timeline=telemetry_timeline,
                telemetry_events=None,
                sources=None,
                events=["reconnecting", "mismatch"],
                anchor_id=2,
                window_before_ms=10,
                window_after_ms=10,
                from_ns=None,
                to_ns=None,
                fault_code=None,
                command_status=None,
                csv_out=None,
                bundle_dir=bundle_dir,
                limit=20,
                json=False,
            )

            merged = merge_robot_timeline.merge_events(args)
            selected = merge_robot_timeline.select_events(merged, args)
            summary = {
                "total_events": len(merged),
                "selected_events": len(selected),
                "sources": sorted({item["source"] for item in selected}),
                "anchors": merge_robot_timeline.summarize_anchors(selected),
                "filters": {},
                "timeline": selected,
            }
            merge_robot_timeline.export_incident_bundle(bundle_dir, args, selected, summary)

            self.assertEqual(len(selected), 3)
            self.assertTrue(all(item["anchor_ids"] == [2] for item in selected))
            self.assertTrue((bundle_dir / "incident_timeline.csv").exists())
            self.assertTrue((bundle_dir / "nav_state_window.bin").exists())
            self.assertTrue((bundle_dir / "control_window.csv").exists())
            self.assertTrue((bundle_dir / "telemetry_timeline_window.csv").exists())

            with (bundle_dir / "incident_summary.json").open(encoding="utf-8") as f:
                bundle_summary = json.load(f)

            self.assertEqual(bundle_summary["selected_window"]["from_ns"], 1_090_000_000)
            self.assertEqual(bundle_summary["selected_window"]["to_ns"], 1_096_000_000)
            self.assertEqual(bundle_summary["replay"]["input_file"], "nav_state_window.bin")
            self.assertEqual(bundle_summary["anchors"][0]["anchor_id"], 2)
            self.assertEqual(
                bundle_summary["bundle_files"]["nav_state_window.bin"]["replay_entrypoint"],
                True,
            )

    def test_bundle_export_falls_back_to_constant_zero_t_ns_nav_state(self) -> None:
        with tempfile.TemporaryDirectory(prefix="merge_robot_timeline_constant_nav_") as td:
            root = pathlib.Path(td)
            nav_state = root / "nav_state.bin"
            control_log = root / "control.csv"
            bundle_dir = root / "bundle"

            invalid = merge_robot_timeline.NavState()
            invalid.t_ns = 0
            invalid.valid = 0
            invalid.stale = 0
            invalid.degraded = 0
            invalid.nav_state = 0
            invalid.health = 0
            invalid.age_ms = 0xFFFFFFFF
            invalid.fault_code = 10

            write_nav_state_file(nav_state, [invalid, invalid, invalid])
            write_text(
                control_log,
                "MonoNS,effective_mode,armed,failsafe,nav_valid,nav_stale,nav_degraded,"
                "nav_age_ms,nav_fault_code,nav_status_flags\n"
                "2000000000,Failsafe,0,0,0,1,1,4294967295,10,0\n",
            )

            args = argparse.Namespace(
                nav_timing=None,
                nav_bin=None,
                nav_state=nav_state,
                control_log=control_log,
                telemetry_timeline=None,
                telemetry_events=None,
                sources=None,
                events=None,
                anchor_id=None,
                window_before_ms=0,
                window_after_ms=0,
                from_ns=None,
                to_ns=None,
                fault_code=None,
                command_status=None,
                csv_out=None,
                bundle_dir=bundle_dir,
                limit=20,
                json=False,
            )

            selected = [
                merge_robot_timeline.make_event(
                    2_000_000_000,
                    "control",
                    "mode=Failsafe armed=0 failsafe=0 nav_valid=0 nav_stale=1 nav_fault=10",
                    tags=["invalid", "stale", "nav_fault"],
                    fault_code=10,
                )
            ]
            summary = {
                "total_events": 1,
                "selected_events": 1,
                "sources": ["control"],
                "anchors": [],
                "filters": {},
                "timeline": selected,
            }

            merge_robot_timeline.export_incident_bundle(bundle_dir, args, selected, summary)

            nav_state_window = bundle_dir / "nav_state_window.bin"
            self.assertTrue(nav_state_window.exists())
            self.assertEqual(nav_state_window.stat().st_size, ctypes.sizeof(merge_robot_timeline.NavState))

            with (bundle_dir / "incident_summary.json").open(encoding="utf-8") as f:
                bundle_summary = json.load(f)

            self.assertEqual(
                bundle_summary["bundle_files"]["nav_state_window.bin"]["selection_mode"],
                "constant_zero_t_ns_fallback",
            )
            self.assertEqual(
                bundle_summary["bundle_files"]["nav_state_window.bin"]["source_records"],
                3,
            )


if __name__ == "__main__":
    unittest.main()
