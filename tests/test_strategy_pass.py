"""
传球策略测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import math
import pytest
from common.world_state import (
    create_default_world_state, create_pass_scenario,
    Robot, Ball, Goal, Team
)
from common.config import OUR_GOAL_X, GOAL_X, GOAL_WIDTH
from strategy.strategy_pass import PassStrategy, PassOption


class TestPassPathClear:
    """传球路径检测"""

    def test_path_clear_no_opponents(self, pass_ws):
        """无对手阻挡时路径畅通"""
        ps = PassStrategy(pass_ws)
        r0 = pass_ws.get_robot_by_id(0)
        r1 = pass_ws.get_robot_by_id(1)
        assert ps.is_pass_path_clear(r0, r1) is True

    def test_can_pass(self, pass_ws):
        """综合传球判断"""
        ps = PassStrategy(pass_ws)
        r0 = pass_ws.get_robot_by_id(0)
        r1 = pass_ws.get_robot_by_id(1)
        assert ps.can_pass(r0, r1) is True


class TestPassParams:
    """传球参数计算"""

    def test_calculate_direction(self, pass_ws):
        ps = PassStrategy(pass_ws)
        # 从 (-2, 0) 到 (0, 1.5)
        direction = ps.calculate_pass_direction((-2, 0), (0, 1.5))
        # 应该向右上方
        assert 0 < direction < math.pi / 2

    def test_calculate_power_range(self, pass_ws):
        ps = PassStrategy(pass_ws)
        # 短距离传球力度
        power_short = ps.calculate_pass_power(1.5)
        assert 25 <= power_short <= 45

        # 长距离传球力度
        power_long = ps.calculate_pass_power(4.5)
        assert power_long > power_short


class TestPassOptions:
    """传球选项评估"""

    def test_evaluate_pass_options(self, pass_ws):
        ps = PassStrategy(pass_ws)
        options = ps.evaluate_pass_options(0)
        # 至少有一个选项 (1号队友)
        assert len(options) >= 1
        # 选项按评分排序
        assert options[0].score >= options[-1].score if len(options) > 1 else True

    def test_find_best_receiver(self, pass_ws):
        ps = PassStrategy(pass_ws)
        best = ps.find_best_receiver(0)
        assert best is not None
        # 1号是更好的传球选择 (位置更靠前)
        assert best.receiver_id == 1

    def test_execute_pass(self, pass_ws):
        ps = PassStrategy(pass_ws)
        success, direction, power = ps.execute_pass(0, 1)
        assert success is True
        assert power > 0
        # 方向朝右上方
        assert direction > 0


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
