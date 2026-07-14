import pytest

from evaluation.scenario_evaluator import run_scenario


@pytest.mark.acceptance
def test_fixed_pass_is_received():
    result = run_scenario("pass_fixed", seed=1001, fast=True)
    assert result.success, result.failure_code
    assert result.outcome == "pass_received"
    assert result.metrics["ball_progress_m"] >= 0.8
    assert result.metrics["receive_distance_m"] <= 0.30
    assert result.metrics["time_to_receive_s"] <= 6.0
    assert result.count_event("PASS_KICKED") == 1
    assert result.count_event("PASS_RECEIVED") == 1
