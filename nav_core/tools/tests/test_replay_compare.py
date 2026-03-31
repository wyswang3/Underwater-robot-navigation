from __future__ import annotations

import pathlib
import tempfile
import unittest
import sys


TOOLS_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import replay_compare  # noqa: E402


def write_text(path: pathlib.Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


class ReplayCompareTests(unittest.TestCase):
    def test_compare_matches_after_startup_prefix(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replay_compare_match_") as td:
            root = pathlib.Path(td)
            bundle = root / "bundle"
            bundle.mkdir()

            write_text(
                bundle / "control_window.csv",
                "MonoNS,has_nav,nav_present,nav_valid,nav_stale,nav_degraded,failsafe,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags\n"
                "100,1,1,0,1,1,0,4,3,10,0\n"
                "200,1,1,0,1,1,0,4,3,10,0\n",
            )
            write_text(
                bundle / "telemetry_timeline_window.csv",
                "telemetry_stamp_ns,failsafe_active,nav_valid,nav_stale,nav_degraded,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags,health_state,fault_state,"
                "last_fault_code,cmd_status,cmd_fault_code\n"
                "100,0,0,1,1,1,3,10,0,3,1,0,0,0\n"
                "200,0,0,1,1,1,3,10,0,3,1,0,0,0\n",
            )
            write_text(
                root / "replay_control.csv",
                "MonoNS,has_nav,nav_present,nav_valid,nav_stale,nav_degraded,failsafe,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags\n"
                "10,0,0,0,1,0,0,0,0,8,0\n"
                "20,1,1,0,1,1,0,4,3,10,0\n"
                "30,1,1,0,1,1,0,4,3,10,0\n",
            )
            write_text(
                root / "replay_telemetry_timeline.csv",
                "telemetry_stamp_ns,failsafe_active,nav_valid,nav_stale,nav_degraded,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags,health_state,fault_state,"
                "last_fault_code,cmd_status,cmd_fault_code\n"
                "10,0,0,1,1,0,3,8,0,3,1,0,0,0\n"
                "20,0,0,1,1,1,3,10,0,3,1,0,0,0\n"
                "30,0,0,1,1,1,3,10,0,3,1,0,0,0\n",
            )

            report = replay_compare.compare_bundle(
                incident_bundle=bundle,
                replay_control_log=root / "replay_control.csv",
                replay_telemetry_timeline=root / "replay_telemetry_timeline.csv",
            )

            self.assertTrue(report["overall_match"])
            self.assertEqual(report["channels"]["control"]["match_start_index"], 1)
            self.assertEqual(report["channels"]["telemetry_timeline"]["match_start_index"], 1)
            self.assertTrue(report["channels"]["telemetry_events"]["match"])

    def test_compare_reports_mismatch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replay_compare_mismatch_") as td:
            root = pathlib.Path(td)
            bundle = root / "bundle"
            bundle.mkdir()

            write_text(
                bundle / "control_window.csv",
                "MonoNS,has_nav,nav_present,nav_valid,nav_stale,nav_degraded,failsafe,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags\n"
                "100,1,1,0,1,1,0,4,3,10,0\n",
            )
            write_text(
                bundle / "telemetry_timeline_window.csv",
                "telemetry_stamp_ns,failsafe_active,nav_valid,nav_stale,nav_degraded,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags,health_state,fault_state,"
                "last_fault_code,cmd_status,cmd_fault_code\n"
                "100,0,0,1,1,1,3,10,0,3,1,0,0,0\n",
            )
            write_text(
                root / "replay_control.csv",
                "MonoNS,has_nav,nav_present,nav_valid,nav_stale,nav_degraded,failsafe,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags\n"
                "20,1,1,0,1,1,0,4,3,12,0\n",
            )
            write_text(
                root / "replay_telemetry_timeline.csv",
                "telemetry_stamp_ns,failsafe_active,nav_valid,nav_stale,nav_degraded,nav_state,"
                "nav_health,nav_fault_code,nav_status_flags,health_state,fault_state,"
                "last_fault_code,cmd_status,cmd_fault_code\n"
                "20,0,0,1,1,1,3,12,0,3,1,0,0,0\n",
            )

            report = replay_compare.compare_bundle(
                incident_bundle=bundle,
                replay_control_log=root / "replay_control.csv",
                replay_telemetry_timeline=root / "replay_telemetry_timeline.csv",
            )

            self.assertFalse(report["overall_match"])
            self.assertFalse(report["channels"]["control"]["match"])
            self.assertFalse(report["channels"]["telemetry_timeline"]["match"])


if __name__ == "__main__":
    unittest.main()
