"""碰撞、越位、门底禁抢、进球/定点球测试"""
from __future__ import annotations

import math

from common.config import (
    GOAL_LINE_DIST,
    GOAL_MOUTH_DEPTH,
    GOAL_X,
    MIN_ROBOT_SEPARATION,
    OUR_GOAL_X,
    SET_PIECE_HOLD,
)
from common.robot_action import MockRobotAction
from common.world_state import Robot, RobotRole, Team, create_default_world_state
from decision.decision_fsm import DecisionFSM, DecisionState
from decision.match_controller import MatchController, MatchPhase
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
    b.x, b.y = 0.05, 0.0
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
    assert 0.7 < line < 1.05
    assert is_offside_position(2.5, ball_x, defs, Team.BLUE)
    nx, _ = legalize_move_target(3.0, 0.0, team=Team.BLUE, ball_x=ball_x, defenders=defs)
    assert nx <= line + 1e-6


def test_match_goal_scores_and_kickoff_formation():
    sim = Simulator()
    action = MockRobotAction(sim)
    blue = DecisionFSM(create_default_world_state(), action, goalkeeper_id=2)
    yellow = DecisionFSM(
        create_default_world_state().perspective_for(Team.YELLOW),
        action, robot_ids=[10, 11, 12], team=Team.YELLOW, goalkeeper_id=12,
    )
    sim.ball.x = GOAL_X + 0.05
    sim.ball.y = 0.0
    match = MatchController()
    scorer = match.detect_goal(sim.ball.x, sim.ball.y)
    assert scorer == Team.BLUE
    match.handle_goal(scorer, sim, blue, yellow)
    assert match.blue_score == 1
    assert match.phase == MatchPhase.KICKOFF
    assert abs(sim.ball.x) < 1e-6
    assert abs(sim.blue_robots[2].x - (OUR_GOAL_X + GOAL_LINE_DIST)) < 1e-5
    assert match.frozen


def test_goalkeeper_role_sticky():
    sim = Simulator()
    action = MockRobotAction(sim)
    ws = create_default_world_state()
    ws.teammates[2].x, ws.teammates[2].y = OUR_GOAL_X + 1.5, 0.0
    ws.ball.x, ws.ball.y = OUR_GOAL_X + 1.8, 0.1
    fsm = DecisionFSM(ws, action, goalkeeper_id=2)
    for _ in range(8):
        fsm.update(ws)
    assert ws.teammates[2].role == RobotRole.GOALKEEPER
    assert fsm.get_state(2) == DecisionState.BLOCK
    assert fsm._ball_carrier_id != 2


def test_freekick_releases_after_hold():
    sim = Simulator()
    action = MockRobotAction(sim)
    blue = DecisionFSM(create_default_world_state(), action, goalkeeper_id=2)
    yellow = DecisionFSM(
        create_default_world_state().perspective_for(Team.YELLOW),
        action, robot_ids=[10, 11, 12], team=Team.YELLOW, goalkeeper_id=12,
    )
    match = MatchController()
    match.begin_freekick(sim, blue, yellow, Team.BLUE, ball_x=2.0, ball_y=1.0)
    assert match.phase == MatchPhase.FREE_KICK
    match.update_cooldown(SET_PIECE_HOLD + 0.1)
    match.tick_set_piece(sim)
    assert match.phase == MatchPhase.PLAY


def test_supporter_does_not_target_ball_when_contested():
    """争球时支援应站侧翼, 不当第二冲球人。"""
    from common.config import BALL_KEEP_CLEAR
    sim = Simulator()
    action = MockRobotAction(sim)
    # 两队前锋贴球争球 → 任一方 team_has_possession 可能 False
    sim.ball.x, sim.ball.y = 0.0, 0.0
    sim.blue_robots[0].x, sim.blue_robots[0].y = -0.25, 0.0
    sim.blue_robots[1].x, sim.blue_robots[1].y = -0.4, 0.4
    sim.yellow_robots[10].x, sim.yellow_robots[10].y = 0.25, 0.0

    ws = create_default_world_state()
    ws.ball.x, ws.ball.y = 0.0, 0.0
    for r in ws.teammates:
        br = sim.blue_robots[r.id]
        r.x, r.y = br.x, br.y
    ws.opponents = [
        Robot(id=10, team=Team.YELLOW, x=0.25, y=0.0, role=RobotRole.IDLE),
    ]
    fsm = DecisionFSM(ws, action, goalkeeper_id=2)
    fsm.update(ws)
    # 支援不应以球心为目标 (keep_clear)
    target = sim._move_targets.get(fsm._supporter_id)
    assert target is not None
    assert math.hypot(target[0] - 0.0, target[1] - 0.0) >= BALL_KEEP_CLEAR - 0.05
    # 持球人可以靠近球
    assert fsm._ball_carrier_id is not None
    assert fsm._supporter_id != fsm._ball_carrier_id
