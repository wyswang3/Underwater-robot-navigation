from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from uwnav.io.acquisition_diagnostics import SensorRunDiagnostics


class AcquisitionDiagnosticsTests(unittest.TestCase):
    def test_event_and_summary_are_written(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            diag = SensorRunDiagnostics(Path(tmpdir), "imu_capture", "IMU")
            diag.bump("raw_samples")
            diag.note_file("raw_csv", Path(tmpdir) / "imu_raw.csv")
            diag.warn("idle_timeout", "imu idle", idle_s=3.5)
            diag.finalize("ok", "done")

            self.assertTrue(diag.events_path.exists())
            self.assertTrue(diag.summary_path.exists())

            with diag.events_path.open("r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

            self.assertGreaterEqual(len(rows), 3)
            self.assertEqual(diag.run_id, rows[0]["run_id"])
            self.assertEqual("imu_capture", rows[0]["component"])
            self.assertEqual("session_started", rows[0]["event"])
            self.assertEqual("idle_timeout", rows[1]["event"])
            self.assertEqual("warn", rows[1]["level"])
            self.assertEqual("IMU", rows[1]["sensor_id"])

            summary = json.loads(diag.summary_path.read_text(encoding="utf-8"))
            self.assertEqual("ok", summary["status"])
            self.assertEqual(diag.run_id, summary["run_id"])
            self.assertEqual(1, summary["counters"]["raw_samples"])
            self.assertEqual(str(Path(tmpdir) / "imu_raw.csv"), summary["files"]["raw_csv"])
            self.assertEqual(2, summary["level_counters"]["info"])
            self.assertEqual(1, summary["level_counters"]["warn"])
            self.assertEqual("idle_timeout", summary["last_problem_event"]["event"])


if __name__ == "__main__":
    unittest.main()
