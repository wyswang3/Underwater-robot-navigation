from __future__ import annotations

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

            summary = json.loads(diag.summary_path.read_text(encoding="utf-8"))
            self.assertEqual("ok", summary["status"])
            self.assertEqual(1, summary["counters"]["raw_samples"])
            self.assertEqual(str(Path(tmpdir) / "imu_raw.csv"), summary["files"]["raw_csv"])


if __name__ == "__main__":
    unittest.main()
