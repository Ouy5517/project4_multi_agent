import math

import pytest

from common.config import (
    BALL_DAMPING_PER_SECOND,
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_WIDTH,
    ROBOT_RADIUS,
    ROBOT_TURN_SPEED,
)
from common.world_state import Ball, Robot, Team
from simulation.field_simulator import InvalidSimulationState, Simulator


def test_turning_uses_robot_turn_speed_not_move_speed():
    sim = Simulator()
    robot = sim.get_robot_by_id(0)
    sim.set_turn_target(0, math.pi)

    sim.update(0.1)

    assert robot.theta == pytest.approx(ROBOT_TURN_SPEED * 0.1)


def test_ball_motion_is_stable_across_30hz_and_60hz():
    def run(dt):
        sim = Simulator()
        sim.ball.x = 0.0
        sim.ball.y = 0.0
        sim.ball.vx = 2.0
        steps = round(1.0 / dt)
        for _ in range(steps):
            sim.update(dt)
        return sim.ball.x

    assert abs(run(1 / 30) - run(1 / 60)) < 0.03


def test_goal_opening_crossing_scores_without_bounce():
    sim = Simulator()
    sim.ball.x = FIELD_WIDTH / 2 - 0.02
    sim.ball.y = 0.0
    sim.ball.vx = 2.0

    events = sim.update(0.1)

    assert "GOAL_BLUE" in events
    assert sim.ball.vx == 0.0
    assert sim.ball.x >= FIELD_WIDTH / 2


def test_endline_outside_goal_is_out_of_bounds_not_goal():
    sim = Simulator()
    sim.ball.x = FIELD_WIDTH / 2 - 0.02
    sim.ball.y = GOAL_WIDTH
    sim.ball.vx = 2.0

    events = sim.update(0.1)

    assert "OUT_OF_BOUNDS" in events
    assert "GOAL_BLUE" not in events
    assert sim.ball.vx == 0.0


def test_reset_restores_loaded_scenario_state():
    sim = Simulator()
    ball = Ball(x=-2.0, y=0.0)
    blue = [Robot(id=0, team=Team.BLUE, x=-2.0, y=0.2)]
    yellow = [Robot(id=10, team=Team.YELLOW, x=3.0, y=0.0)]
    sim.load_state(ball, blue, yellow)
    sim.ball.x = 1.0
    sim.get_robot_by_id(0).x = 1.0

    sim.reset()

    assert sim.ball.position == (-2.0, 0.0)
    assert sim.get_robot_by_id(0).position == (-2.0, 0.2)


def test_nan_state_raises_invalid_simulation_state():
    sim = Simulator()
    sim.ball.x = math.nan

    with pytest.raises(InvalidSimulationState):
        sim.update(1 / 30)


def test_robot_overlap_is_resolved_as_collision_event():
    sim = Simulator()
    r0 = sim.get_robot_by_id(0)
    r1 = sim.get_robot_by_id(1)
    r0.x = r1.x = 0.0
    r0.y = r1.y = 0.0

    events = sim.update(1 / 30)

    assert "COLLISION" in events
    distance = math.hypot(r0.x - r1.x, r0.y - r1.y)
    assert distance >= 2 * ROBOT_RADIUS - 1e-6
