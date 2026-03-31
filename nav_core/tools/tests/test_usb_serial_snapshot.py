from __future__ import annotations

import pathlib
import tempfile
import unittest
import sys


TOOLS_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import usb_serial_snapshot  # noqa: E402


class UsbSerialSnapshotTests(unittest.TestCase):
    def test_scan_prefers_by_id_and_reads_sysfs_identity(self) -> None:
        with tempfile.TemporaryDirectory(prefix="usb_serial_snapshot_") as td:
            root = pathlib.Path(td)
            dev_root = root / "dev"
            sys_root = root / "sys" / "class" / "tty"
            (dev_root / "serial" / "by-id").mkdir(parents=True)
            (sys_root / "ttyUSB9" / "device").mkdir(parents=True)
            (dev_root / "ttyUSB9").touch()
            (dev_root / "serial" / "by-id" / "usb-imu").symlink_to(dev_root / "ttyUSB9")
            (sys_root / "ttyUSB9" / "device" / "idVendor").write_text("10C4\n", encoding="utf-8")
            (sys_root / "ttyUSB9" / "device" / "idProduct").write_text("EA60\n", encoding="utf-8")
            (sys_root / "ttyUSB9" / "device" / "serial").write_text("imu-001\n", encoding="utf-8")

            devices = usb_serial_snapshot.scan_serial_snapshot(dev_root, sys_root)
            self.assertEqual(len(devices), 1)
            self.assertEqual(devices[0]["path"], str(dev_root / "serial" / "by-id" / "usb-imu"))
            self.assertEqual(devices[0]["vendor_id"], "10c4")
            self.assertEqual(devices[0]["product_id"], "ea60")
            self.assertEqual(devices[0]["serial"], "imu-001")


if __name__ == "__main__":
    unittest.main()
