"""
射门策略测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.world_state import create_shoot_scenario, create_default_world_state
from common.config import SHOOT_RANGE
from strategy.strategy_shoot import ShootStrategy, ShootEvaluation


class TestShootEvaluation:
    def test_shoot_viable_when_close(self, shoot_ws):
        """近距离射门可行"""
        ss = ShootStrategy(shoot_ws)
        evaluation = ss.evaluate_shoot_opportunity(0)
        assert evaluation.is_viable is True
        assert evaluation.score > 0.3

    def test_shoot_not_viable_when_far(self, default_ws):
        """远距离射门不可行"""
        ss = ShootStrategy(default_ws)
        evaluation = ss.evaluate_shoot_opportunity(0)
        # 距离太远
        assert evaluation.is_viable is False

    def test_shoot_not_viable_without_ball(self, shoot_ws):
        """没控制球时不可射门"""
        ss = ShootStrategy(shoot_ws)
        # 机器人2离球很远
        evaluation = ss.evaluate_shoot_opportunity(2)
        assert evaluation.is_viable is False


class TestShootParams:
    def test_calculate_direction(self, shoot_ws):
        """射门方向应指向球门"""
        ss = ShootStrategy(shoot_ws)
        direction = ss.calculate_shoot_direction(0)
        # 射门方向应该朝右 (对方球门在右边)
        assert -0.5 < direction < 0.5

    def test_calculate_power(self, shoot_ws):
        """力度计算"""
        ss = ShootStrategy(shoot_ws)
        power_short = ss.calculate_shoot_power(0.5)
        power_long = ss.calculate_shoot_power(3.0)
        assert power_short > 0
        assert power_long > power_short  # 远距离力度更大
        assert power_long <= 100

    def test_execute_shoot(self, shoot_ws):
        """执行射门"""
        ss = ShootStrategy(shoot_ws)
        success, direction, power = ss.execute_shoot(0)
        assert success is True
        assert power > 0


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
