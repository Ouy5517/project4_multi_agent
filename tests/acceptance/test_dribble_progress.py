import pytest

from evaluation.scenario_evaluator import run_scenario


@pytest.mark.acceptance
def test_dribble_open_progresses_ball_under_control():
    result = run_scenario("dribble_open", seed=1002, fast=True)

    assert result.success, result.failure_code
    assert result.outcome == "dribble_progress"
    assert result.metrics["ball_progress_m"] >= 1.5
    assert result.metrics["control_rate"] >= 0.80
    assert result.metrics["out_of_bounds"] == 0
    assert result.metrics["kick_cooldown_violations"] == 0
