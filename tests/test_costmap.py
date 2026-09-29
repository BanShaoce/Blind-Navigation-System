import unittest
from types import SimpleNamespace

from blind_navigation.navigation.costmap import LocalCostmap


def make_costmap(data: list[int], *, sec: int = 1) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)),
        metadata=SimpleNamespace(
            resolution=1.0,
            size_x=3,
            size_y=3,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=data,
    )


class LocalCostmapTest(unittest.TestCase):
    def test_cost_query_and_bounds(self) -> None:
        costmap = LocalCostmap()
        costmap.update(make_costmap(list(range(9))))
        self.assertEqual(costmap.cost_at(1.2, 1.2), 4)
        self.assertTrue(costmap.contains(2.9, 2.9))
        self.assertEqual(costmap.cost_at(-0.1, 0.0), LocalCostmap.UNKNOWN)
        self.assertFalse(costmap.contains(-0.1, 0.0))
        self.assertEqual(costmap.cost_at(3.0, 0.0), LocalCostmap.UNKNOWN)

    def test_older_update_is_ignored(self) -> None:
        costmap = LocalCostmap()
        costmap.update(make_costmap([5] * 9, sec=2))
        costmap.update(make_costmap([9] * 9, sec=1))
        self.assertEqual(costmap.cost_at(0.0, 0.0), 5)

    def test_unknown_area_is_not_safe(self) -> None:
        costmap = LocalCostmap()
        self.assertFalse(costmap.ready)
        self.assertFalse(costmap.is_safe(0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
