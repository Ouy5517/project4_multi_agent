"""
简化足球规则工具
================
- 越位线 (相对第二名对方后卫 + 球)
- 禁止进入对方球门底抢球
- 合法移动目标裁剪
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

from common.config import (
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_MOUTH_DEPTH,
    GOAL_WIDTH,
    GOAL_X,
    OFFSIDE_BUFFER,
    OUR_GOAL_X,
)
from common.world_state import Robot, Team


def offside_line_x(
    attackers_attack_plus_x: bool,
    ball_x: float,
    defenders: Iterable[Robot],
) -> float:
    """
    返回攻击方不得越过的 x (含缓冲)。
    越位: 比球和第二名对方后卫都更靠近对方门线。
    故合法推进上限 ≈ max(球, 第二后卫) (蓝攻右) / min(...) (黄攻左)。
    """
    defs = list(defenders)
    if attackers_attack_plus_x:
        xs = sorted((r.x for r in defs), reverse=True)
        second = xs[1] if len(xs) >= 2 else (xs[0] if xs else 0.0)
        return max(second, ball_x) - OFFSIDE_BUFFER
    xs = sorted(r.x for r in defs)
    second = xs[1] if len(xs) >= 2 else (xs[0] if xs else 0.0)
    return min(second, ball_x) + OFFSIDE_BUFFER


def is_offside_position(
    x: float,
    ball_x: float,
    defenders: Iterable[Robot],
    team: Team,
) -> bool:
    attack_plus = team == Team.BLUE
    defs = list(defenders)
    if attack_plus:
        xs = sorted((r.x for r in defs), reverse=True)
        second = xs[1] if len(xs) >= 2 else (xs[0] if xs else 0.0)
        return x > ball_x and x > second
    xs = sorted(r.x for r in defs)
    second = xs[1] if len(xs) >= 2 else (xs[0] if xs else 0.0)
    return x < ball_x and x < second


def clamp_out_of_enemy_goal_mouth(
    x: float,
    y: float,
    team: Team,
) -> Tuple[float, float]:
    """
    禁止站在对方球门底下 (门线内侧 GOAL_MOUTH_DEPTH、门宽内)。
    进攻方只能在禁抢区外处理球。
    """
    half_w = GOAL_WIDTH / 2 + 0.15
    if team == Team.BLUE:
        # 右门内侧
        edge = GOAL_X - GOAL_MOUTH_DEPTH
        if x > edge and abs(y) <= half_w:
            x = edge
    else:
        edge = OUR_GOAL_X + GOAL_MOUTH_DEPTH
        if x < edge and abs(y) <= half_w:
            x = edge
    return x, y


def clamp_onside(
    x: float,
    y: float,
    ball_x: float,
    defenders: Iterable[Robot],
    team: Team,
) -> Tuple[float, float]:
    """把目标压到越位线合法侧。"""
    attack_plus = team == Team.BLUE
    line = offside_line_x(attack_plus, ball_x, defenders)
    if attack_plus and x > line:
        x = line
    elif (not attack_plus) and x < line:
        x = line
    return x, y


def clamp_field(x: float, y: float) -> Tuple[float, float]:
    margin = 0.25
    return (
        max(-FIELD_WIDTH / 2 + margin, min(FIELD_WIDTH / 2 - margin, x)),
        max(-FIELD_HEIGHT / 2 + margin, min(FIELD_HEIGHT / 2 - margin, y)),
    )


def legalize_move_target(
    x: float,
    y: float,
    *,
    team: Team,
    ball_x: float,
    defenders: Iterable[Robot],
    apply_offside: bool = True,
) -> Tuple[float, float]:
    """组合: 场地裁剪 → 门底禁区 → 越位线。"""
    x, y = clamp_field(x, y)
    x, y = clamp_out_of_enemy_goal_mouth(x, y, team)
    if apply_offside:
        x, y = clamp_onside(x, y, ball_x, defenders, team)
    return x, y
