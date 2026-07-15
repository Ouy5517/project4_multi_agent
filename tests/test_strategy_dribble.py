"""
带球策略测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.world_state import create_default_world_state, create_shoot_scenario
from common.config import ROBOT_KICK_RANGE
from strategy.strategy_dribble import DribbleStrategy


class TestApproachBall:
    def test_approach_ball_not_controlled(self, default_ws):
        """远离球时需要接近"""
        ds = DribbleStrategy(default_ws)
        # 0号离球约1m
        arrived, tx, ty = ds.approach_ball(0)
        assert arrived is False
        # 目标应该靠近球

    def test_is_ball_controlled(self, shoot_ws):
        """在范围内时控制球"""
        ds = DribbleStrategy(shoot_ws)
        assert ds.is_ball_controlled(0) is True

    def test_is_ball_not_controlled(self, default_ws):
        """0号离球约1m, 不应视为控制"""
        ds = DribbleStrategy(default_ws)
        assert ds.is_ball_controlled(0) is False


class TestDribbleToward:
    def test_dribble_toward_goal(self, shoot_ws):
        """射门场景中向球门带球"""
        ds = DribbleStrategy(shoot_ws)
        # 向对方球门带球
        should_dribble, direction, power, dist = ds.dribble_toward(0, 4.5, 0)
        # 应该返回有效方向
        if should_dribble:
            # 方向应该朝右 (对方球门)
            assert -0.5 < direction < 0.5

    def test_dribble_plan_path(self, shoot_ws):
        """路径规划"""
        ds = DribbleStrategy(shoot_ws)
        waypoints = ds.plan_dribble_path(0, 4.5, 0)
        assert len(waypoints) >= 1
        # 最后一个点是目标
        assert waypoints[-1] == (4.5, 0)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
