from __future__ import annotations

import unittest

from uwnav.drivers.dvl.hover_h1000.io import DVLSerialInterface
from uwnav.drivers.dvl.hover_h1000.protocol import parse_lines


class DVLProtocolTests(unittest.TestCase):
    def test_parse_lines_ignores_command_echo_without_data_frame_marker(self) -> None:
        self.assertEqual([], parse_lines("CZ\r0000000000000"))

    def test_parse_lines_extracts_known_frames_from_mixed_raw_line(self) -> None:
        raw = ":TS,00010100000186,35.0,+25.0,   0.0,1500.0,003:BI,   -30,   +37,    +0,    +0,A"
        pkts = parse_lines(raw)
        self.assertEqual(["TS", "BI"], [pkt["src"] for pkt in pkts])

    def test_parse_lines_keeps_multi_frame_boundary_on_realistic_line(self) -> None:
        raw = ":BD,       +0.00,       -0.00,       -0.00,  +5.84,  0.09:SA, +4.76,0.89, .34"
        pkts = parse_lines(raw)
        self.assertEqual(["BD", "SA"], [pkt["src"] for pkt in pkts])
        self.assertEqual(0.0, pkts[0]["e"])

    def test_pkt_to_dvldata_drops_non_motion_frames(self) -> None:
        pkt = {"src": "SA", "pitch": 4.8, "roll": 1.0, "heading": 42.0}
        self.assertIsNone(DVLSerialInterface._pkt_to_dvldata(pkt, 0.0))

    def test_pkt_to_dvldata_accepts_motion_frame(self) -> None:
        pkt = {"src": "BE", "ve": 8.0, "vn": -47.0, "vu": 0.0, "valid": True}
        dvldata = DVLSerialInterface._pkt_to_dvldata(pkt, 0.0)
        self.assertIsNotNone(dvldata)
        assert dvldata is not None
        self.assertEqual("BE", dvldata.src)
        self.assertEqual(0.008, dvldata.ve_enu)
        self.assertEqual(-0.047, dvldata.vn_enu)
        self.assertEqual(0.0, dvldata.vu_enu)


if __name__ == "__main__":
    unittest.main()
