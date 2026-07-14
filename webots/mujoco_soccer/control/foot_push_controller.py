from __future__ import annotations

import math
from dataclasses import dataclass

from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.physics.geometry import unit


@dataclass
class PushPlan:
    robot: str
    target_x: float
    target_y: float
    direction_x: float
    direction_y: float
    speed: float
    min_displacement: float
    action: str


def plan_push(robot: str, ball_xy: tuple[float, float], target_xy: tuple[float, float], speed: float, min_displacement: float, action: str) -> tuple[BaseTarget, PushPlan]:
    dx, dy = unit(target_xy[0] - ball_xy[0], target_xy[1] - ball_xy[1])
    behind = 0.305
    if action == "pass":
        behind = 0.345
    yaw = math.atan2(dy, dx)
    right_foot_offset_x = dy * 0.072
    right_foot_offset_y = -dx * 0.072
    start = BaseTarget(
        ball_xy[0] - dx * behind - right_foot_offset_x,
        ball_xy[1] - dy * behind - right_foot_offset_y,
        yaw,
        max_speed=0.28,
    )
    final_backoff = 0.17
    if action == "pass":
        final_backoff = 0.105
    elif action == "shoot":
        final_backoff = -0.08
    elif action == "red1_clear":
        final_backoff = -0.03
    elif action == "red2_counter":
        final_backoff = 0.08
    elif action == "receive":
        final_backoff = 0.13
    push = PushPlan(
        robot=robot,
        target_x=ball_xy[0] - dx * final_backoff - right_foot_offset_x,
        target_y=ball_xy[1] - dy * final_backoff - right_foot_offset_y,
        direction_x=dx,
        direction_y=dy,
        speed=speed,
        min_displacement=min_displacement,
        action=action,
    )
    return start, push
