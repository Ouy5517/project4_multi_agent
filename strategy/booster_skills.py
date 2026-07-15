"""
Booster robocup_demo 几何技能移植 (适用于上帝视角 2D/MuJoCo)
================================================================
从官方 brain_tree / brain.cpp 提炼、按 kid 9×6 场缩放:

已移植 (本仓库演示可用):
- CalcKickDir (shoot / cross / clear)
- Chase: direct 逼近 + circle-back 绕后
- GoToGoalBlockingPosition 门线卡位
- Assist 支援站位
- updateCostToKick 控球 cost 竞选
- isAngleGood 射击窗 (射门门控)
- 定点球站位见 set_piece.py

未移植 (依赖视觉/ROS/实机, 本演示收益低):
- Cam*/RLVisionKick/自定位/深度避障/GameController/队友 UDP
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from common.config import (
    ADJUST_RANGE,
    ASSIST_BACK_PRIMARY,
    ASSIST_BACK_SECONDARY,
    ANGLE_GOOD_MARGIN,
    BALL_KEEP_CLEAR,
    CHASE_BEHIND_DIST,
    CHASE_SAFE_DIST,
    CROSS_ANGLE_THRESHOLD,
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_LINE_DIST,
    GOAL_WIDTH,
    KICK_ALIGN_TOLERANCE,
    PENALTY_BOX_DEPTH,
    PRESS_FLANK_BACK,
    PRESS_FLANK_SIDE,
    ROBOT_KICK_RANGE,
)
from common.world_state import Goal, Robot, WorldState


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def goal_post_angles(
    ball_x: float,
    ball_y: float,
    goal_x: float,
    y_min: float,
    y_max: float,
    margin: float = 0.0,
) -> Tuple[float, float]:
    """返回 (θ_left_post, θ_right_post) — 自球看两门柱。"""
    theta_l = math.atan2(y_max - margin - ball_y, goal_x - ball_x)
    theta_r = math.atan2(y_min + margin - ball_y, goal_x - ball_x)
    return theta_l, theta_r


def calc_kick_dir(ws: WorldState, *, defending_clear: bool = False) -> Tuple[float, str]:
    """
    对应 CalcKickDir。
    返回 (kick_dir_rad, mode) mode ∈ {shoot, cross, clear}。
    """
    ball = ws.ball
    goal = ws.opponent_goal
    our = ws.our_goal

    if defending_clear:
        kick_dir = math.atan2(ball.y - our.center[1], ball.x - our.x)
        if abs(ball.x - our.x) < abs(goal.x - our.x) * 0.45:
            kick_dir = math.atan2(-ball.y * 0.2, goal.x - ball.x)
        return wrap_to_pi(kick_dir), "clear"

    theta_l, theta_r = goal_post_angles(
        ball.x, ball.y, goal.x, goal.y_min, goal.y_max, margin=0.05
    )
    aperture = wrap_to_pi(theta_l - theta_r)
    if abs(aperture) < CROSS_ANGLE_THRESHOLD and abs(ball.x) < abs(goal.x) * 0.85:
        pen_x = goal.x - math.copysign(PENALTY_BOX_DEPTH * 0.55, goal.x)
        kick_dir = math.atan2(-ball.y, pen_x - ball.x)
        return wrap_to_pi(kick_dir), "cross"

    kick_dir = math.atan2(goal.center[1] - ball.y, goal.x - ball.x)
    return wrap_to_pi(kick_dir), "shoot"


def is_angle_good(
    robot: Robot,
    ball_x: float,
    ball_y: float,
    goal: Goal,
    *,
    margin: float = ANGLE_GOOD_MARGIN,
    kind: str = "kick",
) -> bool:
    """人→球 (或机器人朝向) 是否落入两门柱夹角。"""
    if kind == "shoot":
        angle = robot.theta
    else:
        angle = math.atan2(ball_y - robot.y, ball_x - robot.x)
    theta_l, theta_r = goal_post_angles(
        ball_x, ball_y, goal.x, goal.y_min, goal.y_max, margin=margin,
    )
    if (theta_l - theta_r) < (2.0 * math.pi / 3.0):
        theta_l, theta_r = goal_post_angles(
            ball_x, ball_y, goal.x, goal.y_min, goal.y_max, margin=0.5,
        )
    return theta_l > angle > theta_r


def is_kick_aligned(
    robot: Robot,
    ball_x: float,
    ball_y: float,
    kick_dir: float,
    tol: float = KICK_ALIGN_TOLERANCE,
) -> bool:
    theta_rb = math.atan2(ball_y - robot.y, ball_x - robot.x)
    return abs(wrap_to_pi(kick_dir - theta_rb)) < tol


def chase_approach_point(
    robot: Robot,
    ball_x: float,
    ball_y: float,
    kick_dir: float,
    *,
    behind: float = CHASE_BEHIND_DIST,
    safe_dist: float = CHASE_SAFE_DIST,
) -> Tuple[float, float, str]:
    """Chase: 夹角小时直接绕到球后; 否则 circle-back。"""
    theta_rb = math.atan2(ball_y - robot.y, ball_x - robot.x)
    delta = abs(wrap_to_pi(kick_dir - theta_rb))
    if delta < math.pi / 2:
        tx = ball_x - behind * math.cos(kick_dir)
        ty = ball_y - behind * math.sin(kick_dir)
        return tx, ty, "direct"

    theta_br = math.atan2(robot.y - ball_y, robot.x - ball_x)
    err = wrap_to_pi(kick_dir + math.pi - theta_br)
    sign = 1.0 if err >= 0 else -1.0
    dist = math.hypot(robot.x - ball_x, robot.y - ball_y)
    ratio = min(1.0, safe_dist / max(dist, 1e-3))
    ang = math.acos(max(-1.0, min(1.0, ratio))) if dist > 1e-3 else 0.0
    theta_tan = theta_br + sign * ang
    s = min(dist, safe_dist)
    tx = ball_x + s * math.cos(theta_tan)
    ty = ball_y + s * math.sin(theta_tan)
    return tx, ty, "circle_back"


def adjust_behind_ball(
    ball_x: float,
    ball_y: float,
    kick_dir: float,
    range_m: float = ADJUST_RANGE * 0.55,
) -> Tuple[float, float]:
    """站在球正后方 range_m (简化 Adjust 目标点)。"""
    return (
        ball_x - range_m * math.cos(kick_dir),
        ball_y - range_m * math.sin(kick_dir),
    )


def goal_line_block_position(
    ws: WorldState,
    *,
    dist_to_goalline: float = GOAL_LINE_DIST,
    as_goalkeeper: bool = False,
) -> Tuple[float, float]:
    """GoToGoalBlockingPosition。"""
    ball = ws.ball
    gx, gy = ws.our_goal.center
    inward = 1.0 if gx < 0 else -1.0
    line_x = gx + inward * dist_to_goalline

    if as_goalkeeper:
        x = line_x
    else:
        x = line_x
        if inward > 0:
            x = max(line_x, min(ball.x - 1.2, 0.0))
        else:
            x = min(line_x, max(ball.x + 1.2, 0.0))

    if inward > 0:
        x = max(gx + 0.35, min(x, ball.x - 0.12))
    else:
        x = min(gx - 0.35, max(x, ball.x + 0.12))

    depth = ball.x - gx
    if abs(depth) < 0.15:
        y = max(-GOAL_WIDTH / 2, min(GOAL_WIDTH / 2, ball.y))
    else:
        y = ball.y * (dist_to_goalline / abs(depth))
        half = (GOAL_WIDTH / 2) if as_goalkeeper else (FIELD_HEIGHT / 2 * 0.45)
        y = max(-half, min(half, y))

    x = max(-FIELD_WIDTH / 2 + 0.3, min(FIELD_WIDTH / 2 - 0.3, x))
    y = max(-FIELD_HEIGHT / 2 + 0.3, min(FIELD_HEIGHT / 2 - 0.3, y))
    return x, y


def _clamp_field(x: float, y: float) -> Tuple[float, float]:
    x = max(-FIELD_WIDTH / 2 + 0.3, min(FIELD_WIDTH / 2 - 0.3, x))
    y = max(-FIELD_HEIGHT / 2 + 0.3, min(FIELD_HEIGHT / 2 - 0.3, y))
    return x, y


def keep_clear_of_ball(
    x: float,
    y: float,
    ball_x: float,
    ball_y: float,
    min_dist: float = BALL_KEEP_CLEAR,
) -> Tuple[float, float]:
    """把目标点推出球周禁区, 避免非持球人挤到球上。"""
    dx, dy = x - ball_x, y - ball_y
    dist = math.hypot(dx, dy)
    if dist >= min_dist:
        return _clamp_field(x, y)
    if dist < 1e-4:
        dx, dy = 1.0, 0.0
        dist = 1.0
    scale = min_dist / dist
    return _clamp_field(ball_x + dx * scale, ball_y + dy * scale)


def press_flank_position(ws: WorldState, robot_id: int) -> Tuple[float, float]:
    """
    争球逼抢位: 落在球侧后方, 绝不踩球。
    用于支援位在未控球时接替「冲向球」的错误行为。
    """
    ball = ws.ball
    gx = ws.our_goal.x
    attack = 1.0 if ws.opponent_goal.x > gx else -1.0
    side = PRESS_FLANK_SIDE if (robot_id % 2 == 0) else -PRESS_FLANK_SIDE
    x = ball.x - attack * PRESS_FLANK_BACK
    y = ball.y + side
    return keep_clear_of_ball(x, y, ball.x, ball.y)


def assist_support_position(
    ws: WorldState,
    robot_id: int,
    *,
    secondary: bool = False,
) -> Tuple[float, float]:
    """非 lead 支援站位。"""
    ball = ws.ball
    gx, gy = ws.our_goal.center
    back = ASSIST_BACK_SECONDARY if secondary else ASSIST_BACK_PRIMARY
    attack = 1.0 if ws.opponent_goal.x > gx else -1.0
    x = ball.x - attack * back
    if attack > 0:
        x = max(x, gx + GOAL_LINE_DIST * 0.6)
    else:
        x = min(x, gx - GOAL_LINE_DIST * 0.6)

    depth = ball.x - gx
    if abs(depth) < 0.2:
        y = ball.y
    else:
        y = ball.y * ((x - gx) / depth)
    y += 0.55 if (robot_id % 2 == 0) else -0.55

    return keep_clear_of_ball(x, y, ball.x, ball.y)


def count_crowd_near_ball(
    ws: WorldState,
    radius: float,
    *,
    exclude_id: Optional[int] = None,
) -> int:
    ball = ws.ball
    n = 0
    for r in list(ws.teammates) + list(ws.opponents):
        if exclude_id is not None and r.id == exclude_id:
            continue
        if math.hypot(r.x - ball.x, r.y - ball.y) <= radius:
            n += 1
    return n


def ball_control_cost(
    ws: WorldState, robot: Robot, kick_dir: Optional[float] = None
) -> float:
    """updateCostToKick 简化版。"""
    ball = ws.ball
    dist = math.hypot(robot.x - ball.x, robot.y - ball.y)
    cost = dist
    face = abs(
        wrap_to_pi(math.atan2(ball.y - robot.y, ball.x - robot.x) - robot.theta)
    )
    cost += 0.35 * face
    if kick_dir is not None:
        theta_rb = math.atan2(ball.y - robot.y, ball.x - robot.x)
        cost += abs(wrap_to_pi(kick_dir - theta_rb)) * (0.4 / 0.3)
    for opp in ws.opponents:
        d = math.hypot(opp.x - robot.x, opp.y - robot.y)
        if d < 1.5:
            cost += 0.5
        if d < 0.6:
            cost += 1.5
    return cost


def rank_by_ball_cost(ws: WorldState) -> List[Tuple[int, float]]:
    kick_dir, _ = calc_kick_dir(ws)
    ranked = [(r.id, ball_control_cost(ws, r, kick_dir)) for r in ws.teammates]
    ranked.sort(key=lambda t: t[1])
    return ranked


def should_enter_kick(
    robot: Robot,
    ball_x: float,
    ball_y: float,
    kick_dir: float,
) -> bool:
    dist = math.hypot(robot.x - ball_x, robot.y - ball_y)
    if dist > ADJUST_RANGE * 1.15:
        return False
    return is_kick_aligned(robot, ball_x, ball_y, kick_dir) or dist <= ROBOT_KICK_RANGE
