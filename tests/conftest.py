"""
测试夹具 - 提供共享的 WorldState 和 Simulator 实例
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH,
    GOAL_X, OUR_GOAL_X, NUM_ROBOTS_PER_TEAM
)
from common.world_state import (
    WorldState, Ball, Robot, Goal, Team, RobotRole,
    create_default_world_state, create_pass_scenario,
    create_shoot_scenario, create_threat_scenario
)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator


@pytest.fixture
def empty_ws():
    """空地 WorldState (无机器人)"""
    return WorldState(
        ball=Ball(x=0, y=0),
        teammates=[],
        opponents=[],
        our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH/2, y_max=GOAL_WIDTH/2),
        opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH/2, y_max=GOAL_WIDTH/2),
    )


@pytest.fixture
def default_ws():
    """默认开球 WorldState"""
    return create_default_world_state()


@pytest.fixture
def pass_ws():
    """传球场景"""
    return create_pass_scenario()


@pytest.fixture
def shoot_ws():
    """射门场景"""
    return create_shoot_scenario()


@pytest.fixture
def threat_ws():
    """防守威胁场景"""
    return create_threat_scenario()


@pytest.fixture
def simulator():
    """新鲜仿真器实例"""
    return Simulator()


@pytest.fixture
def mock_action(simulator):
    """MockRobotAction (连接到仿真器)"""
    return MockRobotAction(simulator)
