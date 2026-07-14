from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def distance_to(self, other: "Vec2") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def unit(dx: float, dy: float) -> tuple[float, float]:
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 1.0, 0.0
    return dx / length, dy / length


def point_line_distance(point: Vec2, a: Vec2, b: Vec2) -> float:
    vx, vy = b.x - a.x, b.y - a.y
    wx, wy = point.x - a.x, point.y - a.y
    denom = vx * vx + vy * vy
    if denom <= 1e-9:
        return point.distance_to(a)
    t = clamp((wx * vx + wy * vy) / denom, 0.0, 1.0)
    proj = Vec2(a.x + t * vx, a.y + t * vy)
    return point.distance_to(proj)

