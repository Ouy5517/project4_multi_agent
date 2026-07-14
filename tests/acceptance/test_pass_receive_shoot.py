import pytest

from evaluation.scenario_evaluator import run_scenario


@pytest.mark.acceptance
def test_pass_receive_shoot_flow_scores_goal():
    result = run_scenario("pass_receive_shoot", seed=1101, fast=True)

    assert result.success, result.failure_code
    assert result.outcome == "pass_receive_shoot_success"
    assert result.metrics["time_to_goal_s"] <= 15.0
    assert result.count_event("PASS_KICKED") == 1
    assert result.count_event("PASS_RECEIVED") == 1
    assert result.count_event("SHOT_KICKED") == 1
    assert result.count_event("GOAL_BLUE") == 1
    assert result.events.index("PASS_KICKED") < result.events.index("PASS_RECEIVED")
    assert result.events.index("PASS_RECEIVED") < result.events.index("SHOT_KICKED")
    assert result.events.index("SHOT_KICKED") < result.events.index("GOAL_BLUE")
