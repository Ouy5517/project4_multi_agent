"""
卡位策略测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.world_state import create_threat_scenario, create_default_world_state
from common.config import FIELD_WIDTH, FIELD_HEIGHT, OUR_GOAL_X
from strategy.strategy_block import BlockStrategy


class TestDefensivePosition:
    def test_between_ball_and_goal(self, threat_ws):
        """防守站位应在球和球门之间"""
        bs = BlockStrategy(threat_ws)
        target = bs.calculate_defensive_position(2)
        assert len(target) == 2
        # x 应在球门和球之间
        assert OUR_GOAL_X <= target[0] <= threat_ws.ball.x

    def test_block_position(self, threat_ws):
        """针对特定对手的卡位"""
        bs = BlockStrategy(threat_ws)
        target = bs.calculate_block_position(2, 10)
        assert len(target) == 2
        assert -FIELD_WIDTH/2 <= target[0] <= FIELD_WIDTH/2


class TestThreatDetection:
    def test_is_goal_threatened(self, threat_ws):
        """威胁场景检测"""
        bs = BlockStrategy(threat_ws)
        # 在威胁场景中, 对手在己方半场控球
        # 注意: 球在(-3, 0), 对手10号也在(-3, 0.2)
        # 但对手10号的 team 是 YELLOW, 不是 BLUE
        assert bs.is_goal_threatened() is True

    def test_no_threat_default(self, default_ws):
        """默认场景无直接威胁"""
        bs = BlockStrategy(default_ws)
        assert bs.get_threat_level() == 0.0


class TestPredict:
    def test_predict_path(self, default_ws):
        """对手路径预判"""
        bs = BlockStrategy(default_ws)
        path = bs.predict_opponent_path(10, steps=3)
        assert len(path) == 3


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
