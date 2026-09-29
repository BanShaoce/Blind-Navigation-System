"""线程安全的局部代价地图查询与临时绕行点搜索。"""

from __future__ import annotations

import math
import threading
from typing import Any


class LocalCostmap:
    UNKNOWN = -1
    SAFE_COST = 30
    OBSTACLE_COST = 80

    def __init__(self) -> None:
        self._message: Any | None = None
        self._lock = threading.Lock()

    def update(self, message: Any) -> None:
        with self._lock:
            if self._message is not None and self._stamp(message) < self._stamp(
                self._message
            ):
                return
            self._message = message

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._message is not None

    @staticmethod
    def _stamp(message: Any) -> tuple[int, int]:
        stamp = message.header.stamp
        return stamp.sec, stamp.nanosec

    def cost_at(self, world_x: float, world_y: float) -> int:
        with self._lock:
            if self._message is None:
                return self.UNKNOWN
            index = self._grid_index_unlocked(world_x, world_y)
            if index is None or index >= len(self._message.data):
                return self.UNKNOWN
            return int(self._message.data[index])

    def contains(self, world_x: float, world_y: float) -> bool:
        with self._lock:
            return (
                self._message is not None
                and self._grid_index_unlocked(world_x, world_y) is not None
            )

    def _grid_index_unlocked(self, world_x: float, world_y: float) -> int | None:
        if self._message is None:
            return None
        metadata = self._message.metadata
        grid_x = math.floor(
            (world_x - metadata.origin.position.x) / metadata.resolution
        )
        grid_y = math.floor(
            (world_y - metadata.origin.position.y) / metadata.resolution
        )
        if not (0 <= grid_x < metadata.size_x and 0 <= grid_y < metadata.size_y):
            return None
        return grid_y * metadata.size_x + grid_x

    def is_safe(self, x: float, y: float, *, margin: float = 0.2) -> bool:
        center_cost = self.cost_at(x, y)
        if center_cost == self.UNKNOWN or center_cost >= self.SAFE_COST:
            return False
        for angle in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2):
            cost = self.cost_at(
                x + margin * math.cos(angle), y + margin * math.sin(angle)
            )
            if cost == self.UNKNOWN or cost >= self.SAFE_COST:
                return False
        return True

    def can_move_forward(self, x: float, y: float, heading: float) -> bool:
        for distance in (0.2, 0.4, 0.6):
            cost = self.cost_at(
                x + distance * math.cos(heading),
                y + distance * math.sin(heading),
            )
            if cost == self.UNKNOWN or cost >= self.SAFE_COST:
                return False
        return True

    def find_avoid_point(
        self,
        obstacle: tuple[float, float] | None,
        path_heading: float,
        side: str,
        *,
        step: float = 0.1,
    ) -> tuple[float, float] | None:
        if obstacle is None:
            return None
        if side not in {"left", "right"}:
            raise ValueError(f"未知绕行方向: {side}")
        obstacle_x, obstacle_y = obstacle
        perpendicular = path_heading + (math.pi / 2 if side == "left" else -math.pi / 2)
        lateral = 0.3
        while lateral <= 2.0 + 1e-9:
            world_x = obstacle_x + lateral * math.cos(perpendicular)
            world_y = obstacle_y + lateral * math.sin(perpendicular)
            if self.is_safe(world_x, world_y) and self.can_move_forward(
                world_x, world_y, path_heading
            ):
                return world_x, world_y
            lateral += step
        return None
