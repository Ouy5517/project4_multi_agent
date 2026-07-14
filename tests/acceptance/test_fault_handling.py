import math

import pytest

from common.world_state import Ball, Robot, Team
from simulation.field_simulator import InvalidSimulationState, Simulator


def test_invalid_ball_nan_fails_stably():
    sim = Simulator()
    sim.ball.y = math.nan
    with pytest.raises(InvalidSimulationState):
        sim.update()


def test_missing_action_robot_is_rejected():
    sim = Simulator()
    assert sim.get_robot_by_id(999) is None
    sim.queue_kick(999, 50, 0)
    assert sim.update() == []


def test_loaded_duplicate_robot_ids_collapse_to_single_authority():
    sim = Simulator()
    sim.load_state(
        Ball(x=0, y=0),
        [Robot(id=0, team=Team.BLUE, x=-1, y=0), Robot(id=0, team=Team.BLUE, x=-2, y=0)],
        [],
    )
    assert len(sim.get_robots(Team.BLUE)) == 1
