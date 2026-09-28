"""二维导航几何计算。"""

from __future__ import annotations

import math

from blind_navigation.navigation.models import Point


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_point(point: Point, translation: Point, yaw: float) -> Point:
    x, y = point
    translate_x, translate_y = translation
    return (
        x * math.cos(yaw) - y * math.sin(yaw) + translate_x,
        x * math.sin(yaw) + y * math.cos(yaw) + translate_y,
    )


def distance_to_segment(point: Point, start: Point, end: Point) -> float:
    point_x, point_y = point
    start_x, start_y = start
    end_x, end_y = end
    vector_x, vector_y = end_x - start_x, end_y - start_y
    length_squared = vector_x * vector_x + vector_y * vector_y
    if length_squared == 0.0:
        return math.dist(point, start)
    ratio = max(
        0.0,
        min(
            1.0,
            ((point_x - start_x) * vector_x + (point_y - start_y) * vector_y)
            / length_squared,
        ),
    )
    projection = (start_x + ratio * vector_x, start_y + ratio * vector_y)
    return math.dist(point, projection)
