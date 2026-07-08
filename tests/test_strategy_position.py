"""
跑位策略测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.world_state import create_default_world_state, create_pass_scenario
from common.config import FIELD_WIDTH, FIELD_HEIGHT
from strategy.strategy_position import PositionStrategy


class TestSupportPosition:
    def test_support_position(self, pass_ws):
        """支援站位在持球者附近"""
        ps = PositionStrategy(pass_ws)
        target = ps.calculate_support_position(0, 1)
        assert len(target) == 2
        # 在场地内
        assert -FIELD_WIDTH/2 <= target[0] <= FIELD_WIDTH/2
        assert -FIELD_HEIGHT/2 <= target[1] <= FIELD_HEIGHT/2


class TestOpenSpace:
    def test_open_space_in_field(self, default_ws):
        """空当位置应在场地内"""
        ps = PositionStrategy(default_ws)
        target = ps.calculate_open_space(1)
        assert len(target) == 2
        assert -FIELD_WIDTH/2 <= target[0] <= FIELD_WIDTH/2
        assert -FIELD_HEIGHT/2 <= target[1] <= FIELD_HEIGHT/2

    def test_default_position(self, default_ws):
        """默认站位"""
        ps = PositionStrategy(default_ws)
        target = ps.calculate_default_position(1)
        assert len(target) == 2


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
