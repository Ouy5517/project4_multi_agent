from __future__ import annotations

import re
from pathlib import Path

from common.final_submission import (
    RUN_ID_RE,
    compute_dribble_success,
    contains_supervisor_ball_move,
    make_run_id,
)
from common.robot_action import ActionType
from demos.final_mock_2v2_demo import build_scenarios
from strategy.team_strategy import TeamStrategy


PROJECT = Path(__file__).resolve().parents[1]
RUNNER_CONFIGS = Path("/home/plon/Workspace/booster_t1_webots/runner_extracted/configs")


def test_new_run_id_format_and_uniqueness() -> None:
    ids = {make_run_id() for _ in range(8)}
    assert len(ids) == 8
    assert all(RUN_ID_RE.match(item) for item in ids)


def test_cleanup_scripts_do_not_use_wide_kill() -> None:
    text = (PROJECT / "scripts" / "stop_final_submission_demo.sh").read_text()
    start = (PROJECT / "scripts" / "start_final_submission_demo.sh").read_text()
    assert "killall" not in text + start
    assert "pkill" not in text + start
    assert "trap cleanup EXIT INT TERM HUP" in start


def test_no_record_config_fields_are_disabled() -> None:
    text = (RUNNER_CONFIGS / "common_module_options_final_no_record.lua").read_text()
    assert "record_backends = {}" in text
    assert not re.search(r"^\s*record_data\s*=\s*true", text, re.M)
    assert not re.search(r"^\s*record_data_\s*=\s*true", text, re.M)
    assert not re.search(r"^\s*record_traj_data_\s*=\s*true", text, re.M)
    cfg = (RUNNER_CONFIGS / "config_final_no_record.lua").read_text()
    assert "common_module_options_final_no_record" in cfg


def test_real_summary_missing_ball_coordinates_cannot_succeed() -> None:
    summary = compute_dribble_success({"ball_initial_position": [0.0, 0.0]})
    assert summary["dribble_success"] is False
    assert summary["failure_reason"] == "missing ball coordinates"


def test_real_summary_small_ball_displacement_cannot_succeed() -> None:
    summary = compute_dribble_success(
        {"ball_initial_position": [0.0, 0.0], "ball_final_position": [0.05, 0.0]}
    )
    assert summary["dribble_success"] is False


def _first_strategy(name: str) -> str:
    strategy = TeamStrategy()
    actions = strategy.decide(build_scenarios()[name])
    return actions[0].action_type.value if actions else "HOLD"


def test_mock_scenario_a_returns_pass() -> None:
    assert _first_strategy("A_PASS") == ActionType.PASS.value


def test_mock_scenario_b_returns_dribble() -> None:
    assert _first_strategy("B_DRIBBLE") == ActionType.DRIBBLE.value


def test_mock_scenario_c_returns_shoot() -> None:
    assert _first_strategy("C_SHOOT") == ActionType.SHOOT.value


def test_mock_scenario_d_returns_block_or_hold() -> None:
    assert _first_strategy("D_BLOCK") in {ActionType.BLOCK.value, ActionType.HOLD.value}


def test_mock_summary_flags_are_true_by_construction() -> None:
    text = (PROJECT / "demos" / "final_mock_2v2_demo.py").read_text()
    assert '"mock": True' in text
    assert '"real_strategy_engine": True' in text
    assert '"mock_action_adapter": True' in text


def test_real_mode_does_not_move_ball_with_supervisor() -> None:
    assert not contains_supervisor_ball_move(PROJECT / "demos" / "final_real_dribble_demo.py")
