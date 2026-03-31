from __future__ import annotations

import unittest

from uwnav.io.channel_frames import ChannelFrameBuffer, parse_channel_line


class ChannelFrameTests(unittest.TestCase):
    def test_parse_channel_line_with_unit(self) -> None:
        sample = parse_channel_line("CH5: 1.433V")
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual("CH5", sample.channel)
        self.assertEqual(5, sample.index)
        self.assertEqual(1.433, sample.numeric_value)
        self.assertEqual("V", sample.unit)

    def test_parse_channel_line_with_bad_value(self) -> None:
        sample = parse_channel_line("CH1: bad")
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertIsNone(sample.numeric_value)
        self.assertEqual("", sample.unit)

    def test_channel_frame_buffer_collects_complete_frame(self) -> None:
        buf = ChannelFrameBuffer(2)
        self.assertIsNone(buf.update(parse_channel_line("CH0: 1.000V")))
        row = buf.update(parse_channel_line("CH1: 0.002A"))
        self.assertEqual(["1.000V", "0.002A"], row)


if __name__ == "__main__":
    unittest.main()
