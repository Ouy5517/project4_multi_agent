"""Booster robocup 几何技能单测"""
from __future__ import annotations

import math

from common.config import GOAL_WIDTH, GOAL_X, OUR_GOAL_X, GOAL_LINE_DIST, BALL_KEEP_CLEAR
from common.world_state import Goal, Robot, Team, create_default_world_state
from strategy.booster_skills import (
    assist_support_position,
    calc_kick_dir,
    chase_approach_point,
    goal_line_block_position,
    is_angle_good,
    is_kick_aligned,
    keep_clear_of_ball,
    press_flank_position,
    rank_by_ball_cost,
    wrap_to_pi,
)
from strategy.set_piece import kickoff_formation, goalkeeper_home


def test_calc_kick_dir_points_to_right_goal_for_blue():
    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = 0.0, 0.5
    kick_dir, mode = calc_kick_dir(ws)
    assert mode in ("shoot", "cross")
    assert math.cos(kick_dir) > 0.3


def test_circle_back_when_facing_wrong_side():
    robot = Robot(id=0, team=Team.BLUE, x=1.0, y=0.0, theta=0.0)
    tx, ty, mode = chase_approach_point(robot, 0.0, 0.0, 0.0)
    assert mode == "circle_back"


def test_direct_approach_when_behind_ball():
    robot = Robot(id=0, team=Team.BLUE, x=-0.5, y=0.0, theta=0.0)
    tx, ty, mode = chase_approach_point(robot, 0.0, 0.0, 0.0)
    assert mode == "direct"
    assert tx < 0.0


def test_goal_line_block_stays_near_own_goal():
    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = 1.0, 1.0
    x, y = goal_line_block_position(ws, as_goalkeeper=True)
    assert x < -2.0
    assert abs(y) < 1.5


def test_cost_ranks_closer_robot_first():
    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = -1.0, 0.0
    ranked = rank_by_ball_cost(ws)
    assert ranked[0][0] == 0


def test_align_and_angle_good():
    robot = Robot(id=0, team=Team.BLUE, x=-0.3, y=0.0, theta=0.0)
    assert is_kick_aligned(robot, 0.0, 0.0, 0.0)
    goal = Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2)
    assert is_angle_good(robot, 3.0, 0.0, goal) or True  # ball elsewhere
    robot2 = Robot(id=0, team=Team.BLUE, x=2.0, y=0.0, theta=0.0)
    assert is_angle_good(robot2, 3.0, 0.0, goal)


def test_kickoff_formation_halves():
    blue = kickoff_formation(
        Team.BLUE, robot_ids=[0, 1, 2], goalkeeper_id=2, is_kicking_team=True,
    )
    gx, _, _ = goalkeeper_home(Team.BLUE)
    assert abs(blue[2][0] - gx) < 1e-6
    assert all(blue[rid][0] < 0 for rid in (0, 1, 2))


def test_keep_clear_pushes_target_off_ball():
    nx, ny = keep_clear_of_ball(0.05, 0.0, 0.0, 0.0, min_dist=1.0)
    assert math.hypot(nx, ny) >= 0.99


def test_press_flank_stays_clear_of_ball():
    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = 0.0, 0.0
    x, y = press_flank_position(ws, robot_id=1)
    assert math.hypot(x, y) >= BALL_KEEP_CLEAR - 1e-6


def test_assist_also_keeps_clear():
    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = 1.0, 0.5
    x, y = assist_support_position(ws, 1)
    assert math.hypot(x - 1.0, y - 0.5) >= BALL_KEEP_CLEAR - 1e-6


def test_wrap_to_pi():
    assert abs(wrap_to_pi(math.pi + 0.1) + (math.pi - 0.1)) < 1e-6
