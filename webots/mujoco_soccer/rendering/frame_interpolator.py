from __future__ import annotations

import math
from dataclasses import dataclass

from mujoco_soccer.physics.geometry import clamp, wrap_to_pi


@dataclass(frozen=True)
class RenderPose:
    t: float
    x: float
    y: float
    yaw: float


def interpolate_pose(previous: RenderPose, current: RenderPose, render_time: float) -> RenderPose:
    span = max(1e-9, current.t - previous.t)
    alpha = clamp((render_time - previous.t) / span, 0.0, 1.0)
    yaw_delta = wrap_to_pi(current.yaw - previous.yaw)
    return RenderPose(
        t=render_time,
        x=previous.x + (current.x - previous.x) * alpha,
        y=previous.y + (current.y - previous.y) * alpha,
        yaw=wrap_to_pi(previous.yaw + yaw_delta * alpha),
    )


def smoothstep(value: float) -> float:
    x = clamp(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def shortest_angle_lerp(a: float, b: float, alpha: float) -> float:
    return wrap_to_pi(a + wrap_to_pi(b - a) * clamp(alpha, 0.0, 1.0))
