"""
定点球站位脚本 (开球 / 任意球)
================================
参考 Booster GoToReadyPosition / GoToFreekickPosition，按 kid 9×6 缩放。
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

from common.config import (
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_LINE_DIST,
    GOAL_X,
    OUR_GOAL_X,
)
from common.world_state import Team

Pos = Tuple[float, float, float]  # x, y, theta


def _clamp(x: float, y: float) -> Tuple[float, float]:
    return (
        max(-FIELD_WIDTH / 2 + 0.35, min(FIELD_WIDTH / 2 - 0.35, x)),
        max(-FIELD_HEIGHT / 2 + 0.35, min(FIELD_HEIGHT / 2 - 0.35, y)),
    )


def goalkeeper_home(team: Team) -> Pos:
    if team == Team.BLUE:
        return (OUR_GOAL_X + GOAL_LINE_DIST, 0.0, 0.0)
    return (GOAL_X - GOAL_LINE_DIST, 0.0, math.pi)


def kickoff_formation(
    team: Team,
    *,
    robot_ids: List[int],
    goalkeeper_id: int,
    is_kicking_team: bool,
) -> Dict[int, Pos]:
    out: Dict[int, Pos] = {}
    out[goalkeeper_id] = goalkeeper_home(team)
    field = [rid for rid in robot_ids if rid != goalkeeper_id]
    attack = 1.0 if team == Team.BLUE else -1.0
    face = 0.0 if team == Team.BLUE else math.pi
    if not field:
        return out

    if is_kicking_team:
        slots = [(-0.55 * attack, 0.05), (-1.6 * attack, 1.2), (-1.6 * attack, -1.2)]
    else:
        slots = [(-2.0 * attack, 0.0), (-2.4 * attack, 1.5), (-2.4 * attack, -1.5)]

    for i, rid in enumerate(field):
        sx, sy = slots[min(i, len(slots) - 1)]
        x, y = _clamp(sx, sy)
        out[rid] = (x, y, face)
    return out


def freekick_formation(
    team: Team,
    *,
    robot_ids: List[int],
    goalkeeper_id: int,
    ball_x: float,
    ball_y: float,
    is_attacking: bool,
) -> Dict[int, Pos]:
    out: Dict[int, Pos] = {}
    out[goalkeeper_id] = goalkeeper_home(team)
    field = [rid for rid in robot_ids if rid != goalkeeper_id]
    if not field:
        return out

    our_goal_x = OUR_GOAL_X if team == Team.BLUE else GOAL_X
    if team == Team.BLUE:
        kick_dir = math.atan2(-ball_y, GOAL_X - ball_x)
        def_dir = math.atan2(ball_y, ball_x - OUR_GOAL_X)
    else:
        kick_dir = math.atan2(-ball_y, OUR_GOAL_X - ball_x)
        def_dir = math.atan2(ball_y, ball_x - GOAL_X)

    if is_attacking:
        slots = [
            (ball_x - 0.55 * math.cos(kick_dir), ball_y - 0.55 * math.sin(kick_dir)),
        ]
        side = math.atan2(ball_y, ball_x - our_goal_x)
        slots.append((ball_x - 2.0 * math.cos(side), ball_y - 2.0 * math.sin(side) + 0.8))
        slots.append((ball_x - 2.2 * math.cos(side), ball_y - 2.2 * math.sin(side) - 0.8))
        faces = [kick_dir, kick_dir, kick_dir]
    else:
        slots, faces = [], []
        for k, ang_off in enumerate((0.0, 0.35, -0.35)):
            ang = def_dir + ang_off
            dist = 2.9 + 0.15 * k
            sx = ball_x - dist * math.cos(ang)
            sy = ball_y - dist * math.sin(ang)
            slots.append((sx, sy))
            faces.append(math.atan2(ball_y - sy, ball_x - sx))

    for i, rid in enumerate(field):
        sx, sy = slots[min(i, len(slots) - 1)]
        x, y = _clamp(sx, sy)
        out[rid] = (x, y, faces[min(i, len(faces) - 1)])
    return out
