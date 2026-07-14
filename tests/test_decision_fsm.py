"""
决策状态机测试
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from common.world_state import (
    create_default_world_state, create_pass_scenario,
    create_shoot_scenario, create_threat_scenario,
    WorldStateProvider, RobotRole
)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import (
    DecisionFSM, DecisionState, RobotFSM, FSMTransition
)


class TestRobotFSM:
    def test_initial_state(self):
        fsm = RobotFSM(0)
        assert fsm.state == DecisionState.IDLE

    def test_transition(self):
        fsm = RobotFSM(0)
        t = fsm.transition(DecisionState.CHASE, "test")
        assert fsm.state == DecisionState.CHASE
        assert t.from_state == DecisionState.IDLE
        assert t.to_state == DecisionState.CHASE
        assert t.robot_id == 0

    def test_can_reevaluate(self):
        fsm = RobotFSM(0)
        assert fsm.can_reevaluate() is False  # 刚创建, timer=0
        fsm.state_timer = 2.0  # 模拟时间推移
        assert fsm.can_reevaluate() is True


class TestDecisionFSM:
    def test_init(self, default_ws, mock_action):
        fsm = DecisionFSM(default_ws, mock_action)
        assert len(fsm._fsms) == 3
        for i in range(3):
            assert fsm.get_state(i) == DecisionState.IDLE

    def test_first_update_assigns_roles(self, default_ws, mock_action):
        """第一次更新后分配角色"""
        fsm = DecisionFSM(default_ws, mock_action)
        # 更新仿真器使其产生状态
        fsm.update(default_ws)

        # BALL_CARRIER 应该被分配 (离球最近的)
        assert fsm._ball_carrier_id is not None

    def test_ball_carrier_goes_to_chase(self, default_ws, mock_action):
        """持球者进入 CHASE 状态"""
        fsm = DecisionFSM(default_ws, mock_action)
        # 多次更新
        for _ in range(5):
            fsm.update(default_ws)

        # 最近的机器人(0号)应该进入 CHASE
        state = fsm.get_state(0)
        assert state in [DecisionState.CHASE, DecisionState.DRIBBLE]

    def test_simulation_with_pass_scenario(self, pass_ws, simulator, mock_action):
        """传球场景的仿真测试"""
        # 初始化仿真器位置匹配传球场景
        for r in pass_ws.teammates:
            sim_r = simulator.get_robot_by_id(r.id)
            if sim_r:
                sim_r.x, sim_r.y = r.x, r.y
        for r in pass_ws.opponents:
            sim_r = simulator.get_robot_by_id(r.id)
            if sim_r:
                sim_r.x, sim_r.y = r.x, r.y
        simulator.ball.x = pass_ws.ball.x
        simulator.ball.y = pass_ws.ball.y

        provider = WorldStateProvider(simulator)
        fsm = DecisionFSM(provider.get(), mock_action)

        # 运行几帧
        for _ in range(10):
            simulator.update()
            ws = provider.get()
            fsm.update(ws)

        # 0号应该进入某个活跃状态
        state = fsm.get_state(0)
        assert state != DecisionState.IDLE

    def test_simulation_with_threat_scenario(self, threat_ws, simulator, mock_action):
        """防守威胁场景"""
        for r in threat_ws.teammates:
            sim_r = simulator.get_robot_by_id(r.id)
            if sim_r:
                sim_r.x, sim_r.y = r.x, r.y
        for r in threat_ws.opponents:
            sim_r = simulator.get_robot_by_id(r.id)
            if sim_r:
                sim_r.x, sim_r.y = r.x, r.y
        simulator.ball.x = threat_ws.ball.x
        simulator.ball.y = threat_ws.ball.y

        provider = WorldStateProvider(simulator)
        fsm = DecisionFSM(provider.get(), mock_action)

        for _ in range(10):
            simulator.update()
            ws = provider.get()
            fsm.update(ws)

        # 应该有角色分配
        assert fsm._ball_carrier_id is not None

    def test_decision_summary(self, default_ws, mock_action):
        """决策摘要统计"""
        fsm = DecisionFSM(default_ws, mock_action)
        for _ in range(30):
            fsm.update(default_ws)

        summary = fsm.get_decision_summary()
        assert summary["total_ticks"] == 30
        assert summary["total_decisions"] > 0


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
