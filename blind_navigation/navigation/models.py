"""导航领域模型与纯计算逻辑。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


Point = tuple[float, float]


class NavigationState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    PERCEPTION_ACTIVE = "PERCEPTION_ACTIVE"


@dataclass(slots=True)
class PathSegmentInfo:
    distance: float = 0.0
    heading: float = 0.0
    start_point: Point | None = None
    end_point: Point | None = None
    is_last_segment: bool = False


def compress_path(
    points: list[Point], *, sample_distance: float, angle_threshold_deg: float
) -> list[PathSegmentInfo]:
    """按距离采样路径，并合并方向接近的相邻线段。"""
    if len(points) < 2:
        return []

    key_points = [points[0]]
    accumulated = 0.0
    for index in range(1, len(points)):
        previous = points[index - 1]
        current = points[index]
        accumulated += math.dist(previous, current)
        if index == len(points) - 1 or accumulated >= sample_distance:
            key_points.append(current)
            accumulated = 0.0

    segments: list[PathSegmentInfo] = []
    for start, end in zip(key_points, key_points[1:], strict=False):
        distance = math.dist(start, end)
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        if segments:
            previous = segments[-1]
            heading_diff = math.atan2(
                math.sin(heading - previous.heading),
                math.cos(heading - previous.heading),
            )
            if abs(math.degrees(heading_diff)) <= angle_threshold_deg:
                previous.distance += distance
                previous.end_point = end
                previous.heading = math.atan2(
                    end[1] - previous.start_point[1],
                    end[0] - previous.start_point[0],
                )
                continue
        segments.append(
            PathSegmentInfo(
                distance=distance,
                heading=heading,
                start_point=start,
                end_point=end,
            )
        )

    if segments:
        segments[-1].is_last_segment = True
    return segments
