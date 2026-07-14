from __future__ import annotations

import math
from dataclasses import dataclass


ROBOT_NAMES = ["BLUE_1", "BLUE_2", "RED_1", "RED_2"]


@dataclass(frozen=True)
class MoveTarget:
    robot: str
    xy: tuple[float, float]
    yaw: float
    speed: float
    phase: str


@dataclass(frozen=True)
class PushPlan:
    robot: str
    direction: tuple[float, float]
    distance: float
    min_ball_displacement: float
    strategy: str
    stage: str


OPENING_TARGETS = [
    MoveTarget("BLUE_1", (-1.05, -0.43), 0.45, 0.18, "RUN_POSITION"),
    MoveTarget("BLUE_2", (-0.95, 1.10), -0.55, 0.24, "SUPPORT_RUN"),
    MoveTarget("RED_1", (-0.05, -1.05), 2.55, 0.18, "INTERCEPT_RUN"),
    MoveTarget("RED_2", (-0.15, 0.28), -2.45, 0.18, "BLOCK_PASS_LINE"),
]

SCENARIO_PUSHES = [
    PushPlan("BLUE_1", (1.0, 0.05), 0.24, 0.08, "DRIBBLE", "BLUE_1_DRIBBLE_1"),
    PushPlan("BLUE_1", (1.0, 0.02), 0.24, 0.08, "DRIBBLE", "BLUE_1_DRIBBLE_2"),
    PushPlan("BLUE_1", (0.96, 0.28), 0.46, 0.20, "PASS", "BLUE_1_PASS_TO_BLUE_2"),
    PushPlan("BLUE_2", (1.0, -0.08), 0.54, 0.25, "SHOOT", "BLUE_2_SHOOT"),
    PushPlan("RED_1", (-0.25, 0.97), 0.42, 0.15, "BLOCK", "RED_1_CLEAR"),
    PushPlan("RED_2", (-1.0, -0.12), 0.42, 0.15, "BLOCK", "RED_2_COUNTER"),
]


def yaw_from_direction(direction: tuple[float, float]) -> float:
    return math.atan2(direction[1], direction[0])


def unit(direction: tuple[float, float]) -> tuple[float, float]:
    length = math.hypot(direction[0], direction[1])
    if length <= 1e-9:
        return (1.0, 0.0)
    return (direction[0] / length, direction[1] / length)


def prepare_pose(ball_xy: list[float], direction: tuple[float, float]) -> tuple[list[float], float]:
    dx, dy = unit(direction)
    yaw = yaw_from_direction((dx, dy))
    # Empirical T1 right-foot offset from root in the fixed stance.
    root_x = ball_xy[0] - dx * 0.225 - (-dy) * 0.055
    root_y = ball_xy[1] - dy * 0.225 - (dx) * 0.055
    return [root_x, root_y], yaw
