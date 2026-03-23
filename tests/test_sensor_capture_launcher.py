from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "apps" / "acquire" / "sensor_capture_launcher.py"
SPEC = importlib.util.spec_from_file_location("sensor_capture_launcher", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class SensorCaptureLauncherTests(unittest.TestCase):
    def _make_args(self):
        return SimpleNamespace(
            sensors=["imu", "dvl", "volt"],
            data_root=None,
            child_log_level="INFO",
            child_stat_every=3.0,
            imu_port="/dev/ttyIMU",
            imu_baud=230400,
            imu_addr=0x50,
            dvl_port="/dev/ttyDVL",
            dvl_baud=115200,
            dvl_ping_rate=10,
            dvl_avg_count=8,
            dvl_raw_only=False,
            volt_port="/dev/ttyVOLT",
            volt_baud=115200,
            volt_channels=16,
        )

    def test_parse_sensor_list_deduplicates(self) -> None:
        sensors = MODULE._parse_sensor_list("imu,dvl,imu,volt")
        self.assertEqual(["imu", "dvl", "volt"], sensors)

    def test_build_child_specs_contains_all_default_scripts(self) -> None:
        args = self._make_args()
        specs = MODULE.build_child_specs(args, Path("/tmp/uwnav_data"))
        self.assertEqual(["imu", "dvl", "volt"], [spec.name for spec in specs])
        self.assertTrue(any("imu_logger.py" in part for part in specs[0].command))
        self.assertTrue(any("DVL_logger.py" in part for part in specs[1].command))
        self.assertTrue(any("Volt32_logger.py" in part for part in specs[2].command))

    def test_write_manifest_records_selected_sensors(self) -> None:
        args = self._make_args()
        with tempfile.TemporaryDirectory() as tmpdir:
            specs = MODULE.build_child_specs(args, Path(tmpdir))
            manifest = MODULE._write_manifest(Path(tmpdir), specs, args)
            payload = manifest.read_text(encoding="utf-8")
            self.assertIn('"selected_sensors"', payload)
            self.assertIn('"imu"', payload)
            self.assertIn('"dvl"', payload)
            self.assertIn('"volt"', payload)


if __name__ == "__main__":
    unittest.main()
