import pytest

from common.config import DT
from common.robot_action import MockRobotAction
from common.world_state import WorldStateProvider
from decision.decision_fsm import DecisionFSM, DecisionState
from evaluation.scenario_evaluator import run_scenario
from simulation.field_simulator import Simulator
from simulation.scenarios import load_scenario_into_simulator


@pytest.mark.acceptance
def test_position_block_scenario_outputs_block_or_position_result():
    result = run_scenario("position_block", seed=1003, fast=True)

    assert result.success, result.failure_code
    assert result.outcome == "position_or_block"
    assert result.count_event("POSITION_OR_BLOCK") == 1


@pytest.mark.acceptance
def test_pass_fixed_strategy_switching_records_reason_codes():
    sim = Simulator()
    load_scenario_into_simulator(sim, "pass_fixed")
    provider = WorldStateProvider(sim)
    action = MockRobotAction(sim)
    fsm = DecisionFSM(provider.get(), action)

    reason_codes = set()
    states = set()
    for _ in range(90):
        sim.update(DT)
        ws = provider.get()
        fsm.update(ws, DT)
        for event in fsm.drain_decision_events():
            reason_codes.add(event.reason_code)
            states.add(event.state)

    assert "IDLE_TO_CHASE" in reason_codes
    assert "MAINTAIN_STATE" in reason_codes
    assert DecisionState.CHASE.value in states
    assert DecisionState.PASS.value in states
    assert DecisionState.RECEIVE.value in states
