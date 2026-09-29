import math
import unittest

from blind_navigation.navigation.models import compress_path


class PathModelTest(unittest.TestCase):
    def test_compress_path_merges_straight_segments(self) -> None:
        segments = compress_path(
            [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)],
            sample_distance=0.5,
            angle_threshold_deg=10.0,
        )
        self.assertEqual(len(segments), 1)
        self.assertTrue(math.isclose(segments[0].distance, 1.0))
        self.assertTrue(segments[0].is_last_segment)

    def test_compress_path_keeps_turn(self) -> None:
        segments = compress_path(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            sample_distance=0.5,
            angle_threshold_deg=10.0,
        )
        self.assertEqual(len(segments), 2)


if __name__ == "__main__":
    unittest.main()
