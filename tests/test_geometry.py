import math
import unittest

from blind_navigation.navigation.geometry import (
    distance_to_segment,
    normalize_angle,
    transform_point,
)


class GeometryTest(unittest.TestCase):
    def test_normalize_angle(self) -> None:
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi)

    def test_transform_point(self) -> None:
        x, y = transform_point((1.0, 0.0), (2.0, 3.0), math.pi / 2)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 4.0)

    def test_distance_to_segment(self) -> None:
        self.assertAlmostEqual(
            distance_to_segment((0.5, 1.0), (0.0, 0.0), (1.0, 0.0)), 1.0
        )


if __name__ == "__main__":
    unittest.main()
