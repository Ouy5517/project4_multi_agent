"""
WorldState 数据结构和 Provider 测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import math
from common.world_state import (
    WorldState, Ball, Robot, Goal, Team, RobotRole,
    create_default_world_state, create_pass_scenario,
    create_shoot_scenario, create_threat_scenario, WorldStateProvider
)
from common.config import FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH, GOAL_X, OUR_GOAL_X


class TestBall:
    def test_ball_initialization(self):
        b = Ball(x=1.0, y=2.0, vx=0.5, vy=-0.3)
        assert b.x == 1.0
        assert b.y == 2.0
        assert b.vx == 0.5
        assert b.speed == math.sqrt(0.5**2 + 0.3**2)

    def test_ball_not_moving(self):
        b = Ball()
        assert not b.is_moving


class TestWorldState:
    def test_distance_calculation(self, default_ws):
        """测试距离计算"""
        r0 = default_ws.teammates[0]
        d = default_ws.distance(r0, default_ws.ball)
        assert d > 0
        # 机器人0离球大约1m
        assert 0.5 < d < 2.0

    def test_is_in_field(self, default_ws):
        """测试场地边界检查"""
        assert default_ws.is_in_field(0, 0) is True
        assert default_ws.is_in_field(10, 0) is False
        assert default_ws.is_in_field(0, 10) is False

    def test_closest_teammate_to_ball(self, default_ws):
        """测试最近队友查找"""
        closest = default_ws.closest_teammate_to_ball()
        assert closest is not None
        assert closest.id == 0  # 0号离球最近

    def test_closest_opponent_to_ball(self, default_ws):
        """测试最近对手查找"""
        closest = default_ws.closest_opponent_to_ball()
        assert closest is not None
        assert closest.id == 10  # 10号离球最近

    def test_has_possession(self, shoot_ws):
        """测试控球判断"""
        # 在射门场景中，0号离球最近
        assert shoot_ws.has_possession(0) is True

    def test_robot_query(self, default_ws):
        """测试机器人查询"""
        r = default_ws.get_robot_by_id(0)
        assert r is not None
        assert r.team == Team.BLUE

        r = default_ws.get_robot_by_id(999)
        assert r is None

    def test_all_robots(self, default_ws):
        """测试所有机器人汇总"""
        all_r = default_ws.all_robots()
        assert len(all_r) == 6  # 3 blue + 3 yellow


class TestMockScenarios:
    def test_default_scenario(self):
        ws = create_default_world_state()
        assert ws.ball.x == 0.0
        assert len(ws.teammates) == 3
        assert len(ws.opponents) == 3

    def test_pass_scenario(self):
        ws = create_pass_scenario()
        # 0号持球
        assert ws.ball.x == -2.0
        assert ws.get_robot_by_id(0).x == -2.0
        # 1号在有利位置
        assert ws.get_robot_by_id(1).y > 1.0

    def test_shoot_scenario(self):
        ws = create_shoot_scenario()
        # 球在对方半场
        assert ws.ball.x > 2.0
        assert ws.get_robot_by_id(0).x > 2.0

    def test_threat_scenario(self):
        ws = create_threat_scenario()
        # 球在己方半场
        assert ws.ball.x < 0
        # 对手10号接近球
        assert ws.get_robot_by_id(10).x < 0


class TestWorldStateProvider:
    def test_provider_without_simulator(self):
        """无仿真器时使用默认 WorldState"""
        provider = WorldStateProvider()
        ws = provider.get()
        assert isinstance(ws, WorldState)
        assert len(ws.teammates) == 3

    def test_provider_with_mock(self):
        """设置 Mock 世界状态"""
        provider = WorldStateProvider()
        ws = create_pass_scenario()
        provider.set_mock(ws)
        result = provider.get()
        assert result.ball.x == ws.ball.x


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
