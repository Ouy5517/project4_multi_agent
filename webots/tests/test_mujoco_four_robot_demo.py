from __future__ import annotations

import ast
import json
import math
from pathlib import Path

import mujoco

from common.robot_action import ActionType
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.logging.summary_builder import missing_summary_fields
from mujoco_soccer.orchestration.stage_machine import STAGES, default_stage_statuses
from mujoco_soccer.physics.ball_guard import BallGuard
from mujoco_soccer.run_demo import DemoRunner
from mujoco_soccer.strategy.defensive_strategy import DefensiveStrategy
from mujoco_soccer.strategy.world_state_adapter import ROBOTS
from mujoco_soccer.tools_generate_proxy_model import main as generate_model


MODEL = Path("mujoco_soccer/models/t1_2v2_soccer.xml")


def load_model() -> mujoco.MjModel:
    generate_model()
    return mujoco.MjModel.from_xml_path(str(MODEL))


def names(model: mujoco.MjModel, obj: mujoco.mjtObj, count: int) -> set[str]:
    return {mujoco.mj_id2name(model, obj, idx) or "" for idx in range(count)}


def test_mujoco_model_loads_and_has_four_unique_robots() -> None:
    model = load_model()
    body_names = names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)
    for robot in ROBOTS:
        assert f"{robot}_base" in body_names
    assert len({f"{robot}_base" for robot in ROBOTS}) == 4


def test_base_joints_actuators_foot_proxies_and_ball_contract() -> None:
    model = load_model()
    joint_names = names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuator_names = names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    geom_names = names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    for robot in ROBOTS:
        for axis in ("x", "y", "yaw"):
            assert f"{robot}_base_{axis}" in joint_names
            assert f"{robot}_base_{axis}_act" in actuator_names
    for prefix in ("BLUE1", "BLUE2", "RED1", "RED2"):
        assert f"{prefix}_LEFT_FOOT_BALL_PROXY" in geom_names
        assert f"{prefix}_RIGHT_FOOT_BALL_PROXY" in geom_names
    assert "soccer_ball_free" in joint_names
    assert "soccer_ball_geom" in geom_names
    assert not [name for name in actuator_names if "ball" in name.lower()]


def test_ball_guard_source_scan_is_clean() -> None:
    model = load_model()
    guard = BallGuard(model, Path.cwd())
    assert guard.ball_qposadr >= 0
    assert guard.ball_dofadr >= 0
    assert guard.scan_sources() == []


def test_stage_machine_is_non_blocking_metadata() -> None:
    tree = ast.parse(Path("mujoco_soccer/orchestration/stage_machine.py").read_text())
    assert not [node for node in ast.walk(tree) if isinstance(node, (ast.While, ast.For))]
    statuses = default_stage_statuses()
    assert set(statuses) == set(STAGES)
    assert all(item.timeout > 0 for item in statuses.values())
    assert all(item.max_retries <= 1 for item in statuses.values())


def test_strategy_actions_include_required_defense_values() -> None:
    assert ActionType.DRIBBLE.value == "DRIBBLE"
    assert ActionType.PASS.value == "PASS"
    assert ActionType.SHOOT.value == "SHOOT"
    assert ActionType.CLEAR.value == "CLEAR"
    assert DefensiveStrategy()


def test_blue1_path_budget_shortens_over_budget_target() -> None:
    runner = DemoRunner("model-check", "unit_path_budget", render=False)
    controller = runner.controllers["T1_BLUE_1"].base
    controller.path_length = 3.50
    target = BaseTarget(1.5, 1.5, 0.0, 0.3)
    planned = runner.plan_path_aware_target("T1_BLUE_1", target, "UNIT")
    x, y, _ = controller.pose(runner.data)
    assert math.hypot(planned.x - x, planned.y - y) <= 0.36
    assert math.hypot(planned.x - target.x, planned.y - target.y) > 0.1


def test_blue1_hard_limit_freezes_translation_without_log_clipping() -> None:
    runner = DemoRunner("model-check", "unit_path_hard_limit", render=False)
    controller = runner.controllers["T1_BLUE_1"].base
    controller.path_length = 3.91
    x, y, _ = controller.pose(runner.data)
    planned = runner.plan_path_aware_target("T1_BLUE_1", BaseTarget(2.0, 2.0, 1.0), "UNIT")
    assert planned.x == x
    assert planned.y == y
    assert controller.path_length == 3.91


def test_pass_only_mode_and_pass_success_threshold_are_declared() -> None:
    source = Path("mujoco_soccer/run_demo.py").read_text()
    assert '"pass-only"' in source
    assert 'displacement >= 0.35' in source
    assert 'stage + "_FREE_ROLL"' in source
    assert 'stable_ball' in source


def test_dribble2_flows_directly_to_pass_without_blue1_parking() -> None:
    source = Path("mujoco_soccer/run_demo.py").read_text()
    start = source.index('def stage_pass_receive_shoot')
    end = source.index('def stage_defense')
    block = source[start:end]
    assert "position_blue2_for_pass" in block
    assert "run_pass_push" in block
    assert "parking" not in block.lower()


def test_pass_receiver_distance_target_is_not_too_close() -> None:
    runner = DemoRunner("model-check", "unit_pass_distance", render=False)
    ball = runner.ball_xy()
    receive = runner.position_blue2_for_pass("UNIT_PASS")
    assert math.hypot(receive[0] - ball[0], receive[1] - ball[1]) >= 0.65


def test_latest_candidate_summary_has_required_fields() -> None:
    summary_path = Path("results/mujoco_four_robot_demo/full_no_render_final_candidate/summary.json")
    if not summary_path.exists():
        return
    summary = json.loads(summary_path.read_text())
    assert missing_summary_fields(summary) == []
    assert summary["simulation_time"] <= 120.01
    assert summary["ball_freejoint"] is True
