from __future__ import annotations

import math

from mujoco_soccer.control.planar_base_controller import BaseTarget


def apply_local_avoidance(robot: str, target: BaseTarget, positions: dict[str, tuple[float, float]]) -> BaseTarget:
    x, y = positions[robot]
    ax = ay = 0.0
    for other, (ox, oy) in positions.items():
        if other == robot:
            continue
        dx, dy = x - ox, y - oy
        dist = math.hypot(dx, dy)
        if 1e-6 < dist < 0.50:
            scale = (0.50 - dist) * 0.25 / dist
            ax += dx * scale
            ay += dy * scale
    return BaseTarget(
        max(-3.25, min(3.25, target.x + ax)),
        max(-2.25, min(2.25, target.y + ay)),
        target.yaw,
        target.max_speed,
        target.max_yaw_rate,
        target.acceleration_limit,
    )

