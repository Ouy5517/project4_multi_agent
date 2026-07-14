"""碰撞、越位、门底禁抢、进球回合测试"""
from __future__ import annotations

import math

from common.config import GOAL_MOUTH_DEPTH, GOAL_X, MIN_ROBOT_SEPARATION, OUR_GOAL_X
from common.world_state import Robot, RobotRole, Team
from decision.decision_fsm import DecisionFSM, DecisionState
from decision.match_controller import MatchController
from simulation.field_simulator import Simulator
from strategy.soccer_rules import (
    clamp_out_of_enemy_goal_mouth,
    is_offside_position,
    legalize_move_target,
    offside_line_x,
)


def test_robot_collision_separates_overlap():
    sim = Simulator(num_blue=2, num_yellow=0)
    a, b = sim.blue_robots[0], sim.blue_robots[1]
    a.x, a.y = 0.0, 0.0
    b.x, b.y = 0.05, 0.0  # 严重重叠
    sim._resolve_robot_collisions()
    assert math.hypot(a.x - b.x, a.y - b.y) >= MIN_ROBOT_SEPARATION - 1e-3


def test_goal_mouth_ban_blue_cannot_stand_in_yellow_goal():
    x, y = clamp_out_of_enemy_goal_mouth(GOAL_X - 0.2, 0.0, Team.BLUE)
    assert x <= GOAL_X - GOAL_MOUTH_DEPTH + 1e-6


def test_offside_line_and_clamp_for_blue():
    defs = [
        Robot(id=10, team=Team.YELLOW, x=1.0, y=0.0, role=RobotRole.IDLE),
        Robot(id=11, team=Team.YELLOW, x=2.0, y=0.0, role=RobotRole.IDLE),
        Robot(id=12, team=Team.YELLOW, x=0.5, y=1.0, role=RobotRole.IDLE),
    ]
    ball_x = 0.0
    line = offside_line_x(True, ball_x, defs)
    # second=1.0, line = max(1,0)-buffer ≈ 0.85
    assert 0.7 < line < 1.05
    assert is_offside_position(2.5, ball_x, defs, Team.BLUE)
    assert not is_offside_position(0.5, ball_x, defs, Team.BLUE)
    nx, _ = legalize_move_target(3.0, 0.0, team=Team.BLUE, ball_x=ball_x, defenders=defs)
    assert nx <= line + 1e-6


def test_move_target_filtered_by_sim_rules():
    sim = Simulator()
    # 黄队后卫很靠后
    sim.yellow_robots[10].x = 1.0
    sim.yellow_robots[11].x = 1.2
    sim.yellow_robots[12].x = 0.8
    sim.ball.x = 0.0
    sim.set_move_target(0, GOAL_X - 0.1, 0.0)  # 蓝队想冲门底
    tx, ty = sim._move_targets[0]
    assert tx <= GOAL_X - GOAL_MOUTH_DEPTH + 1e-6


def test_match_goal_scores_and_kickoff_resets():
    sim = Simulator()
    action_ws = __import__("common.robot_action", fromlist=["MockRobotAction"]).MockRobotAction(sim)
    blue = DecisionFSM(__import__("common.world_state", fromlist=["create_default_world_state"]).create_default_world_state(), action_ws)
    yellow = DecisionFSM(
        __import__("common.world_state", fromlist=["create_default_world_state"]).create_default_world_state().perspective_for(Team.YELLOW),
        action_ws,
        robot_ids=[10, 11, 12],
        team=Team.YELLOW,
    )
    # 弄乱位置
    sim.blue_robots[0].x = 3.0
    sim.ball.x = GOAL_X + 0.05
    sim.ball.y = 0.0
    match = MatchController()
    scorer = match.detect_goal(sim.ball.x, sim.ball.y)
    assert scorer == Team.BLUE
    match.handle_goal(scorer, sim, blue, yellow)
    assert match.blue_score == 1
    assert abs(sim.ball.x) < 1e-6
    assert abs(sim.blue_robots[0].x - sim._init_positions[0][0]) < 1e-6
    assert blue.get_state(0) == DecisionState.IDLE
    assert match.frozen
