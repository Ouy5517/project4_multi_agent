"""转向速度与黄队对抗控制器单测"""
from __future__ import annotations

import math

from common.config import DT, ROBOT_MAX_SPEED, ROBOT_TURN_SPEED
from common.robot_action import MockRobotAction
from common.world_state import Ball, Goal, Robot, RobotRole, Team, WorldState
from decision.opponent_controller import YellowOpponentController
from simulation.field_simulator import Simulator


def test_turn_uses_robot_turn_speed_not_linear_speed():
    sim = Simulator(num_blue=1, num_yellow=0)
    robot = sim.blue_robots[0]
    robot.theta = 0.0
    sim.set_turn_target(0, math.pi)  # 转 180°
    # 旧 bug: max_turn = 0.5*dt → 需 ~6s；新: π*dt → ~1s
    elapsed = 0.0
    while 0 in sim._turn_targets and elapsed < 2.0:
        sim._update_robots(DT)
        elapsed += DT
    assert 0 not in sim._turn_targets
    assert elapsed < 1.2  # 明显快于旧的 ~6s
    assert abs(ROBOT_TURN_SPEED - math.pi) < 1e-3
    assert ROBOT_TURN_SPEED > ROBOT_MAX_SPEED * 2


def test_move_auto_faces_travel_direction():
    sim = Simulator(num_blue=1, num_yellow=0)
    robot = sim.blue_robots[0]
    robot.x, robot.y, robot.theta = 0.0, 0.0, 0.0
    sim.set_move_target(0, 0.0, 2.0)  # 向上走 → 应朝 +Y (π/2)
    for _ in range(5):
        sim._update_robots(DT)
    # 已开始转向 +Y
    assert abs(robot.theta - math.pi / 2) < math.pi / 2


def _make_ws(opponents: list[Robot], ball: Ball) -> WorldState:
    return WorldState(
        timestamp=0.0,
        ball=ball,
        teammates=[],
        opponents=opponents,
        our_goal=Goal(x=-4.5, y_min=-1.0, y_max=1.0),
        opponent_goal=Goal(x=4.5, y_min=-1.0, y_max=1.0),
    )


def test_yellow_assigns_chase_intercept_defend():
    sim = Simulator(num_blue=0, num_yellow=3)
    action = MockRobotAction(sim)
    ai = YellowOpponentController(action)
    opps = [
        Robot(id=10, team=Team.YELLOW, x=2.0, y=0.0, role=RobotRole.IDLE),
        Robot(id=11, team=Team.YELLOW, x=3.0, y=1.0, role=RobotRole.IDLE),
        Robot(id=12, team=Team.YELLOW, x=3.5, y=-1.0, role=RobotRole.IDLE),
    ]
    ball = Ball(x=0.0, y=0.0)
    ws = _make_ws(opps, ball)
    ai.update(ws)
    roles = ai.last_roles
    assert roles[10] == "CHASE"  # nearest to ball
    assert "INTERCEPT" in roles.values()
    assert "DEFEND" in roles.values()
    assert 10 in sim._move_targets


def test_yellow_chaser_kicks_when_close():
    sim = Simulator(num_blue=0, num_yellow=1)
    # place ball next to yellow 10
    sim.yellow_robots[10].x = 0.0
    sim.yellow_robots[10].y = 0.0
    sim.ball.x = 0.15
    sim.ball.y = 0.0
    action = MockRobotAction(sim)
    ai = YellowOpponentController(action)
    opp = Robot(id=10, team=Team.YELLOW, x=0.0, y=0.0, role=RobotRole.IDLE)
    ws = _make_ws([opp], Ball(x=0.15, y=0.0))
    ai.update(ws)
    assert len(sim._kick_queue) == 1
