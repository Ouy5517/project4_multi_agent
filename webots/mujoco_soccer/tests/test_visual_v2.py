from __future__ import annotations

from pathlib import Path

import mujoco

from mujoco_soccer.control.robot_controller import RobotController
from mujoco_soccer.strategy.world_state_adapter import ROBOTS
from mujoco_soccer.tools_generate_proxy_model import main as generate_model


VISUAL_V2_MODEL = Path("mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml")
BASELINE_MODEL = Path("mujoco_soccer/models/t1_2v2_soccer.xml")


def _load_visual_v2() -> mujoco.MjModel:
    generate_model()
    return mujoco.MjModel.from_xml_path(str(VISUAL_V2_MODEL))


def _names(model: mujoco.MjModel, obj: mujoco.mjtObj, count: int) -> set[str]:
    return {mujoco.mj_id2name(model, obj, idx) or "" for idx in range(count)}


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def test_visual_v2_model_is_independent_and_baseline_still_exists() -> None:
    model = _load_visual_v2()
    assert VISUAL_V2_MODEL.exists()
    assert BASELINE_MODEL.exists()
    assert model.nbody > 0


def test_visual_v2_robot_shell_parts_exist_and_do_not_collide() -> None:
    model = _load_visual_v2()
    geom_names = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    required_suffixes = [
        "visual_head_shell",
        "visual_visor",
        "visual_torso_shell",
        "visual_left_shoulder_shell",
        "visual_right_shoulder_shell",
        "visual_left_thigh_shell",
        "visual_right_thigh_shell",
        "visual_left_knee_shell",
        "visual_right_knee_shell",
        "visual_left_shin_shell",
        "visual_right_shin_shell",
        "visual_left_foot_shell",
        "visual_right_foot_shell",
    ]
    for robot in ROBOTS:
        for suffix in required_suffixes:
            name = f"{robot}_{suffix}"
            assert name in geom_names
            gid = _geom_id(model, name)
            assert model.geom_contype[gid] == 0
            assert model.geom_conaffinity[gid] == 0
            assert model.geom_group[gid] == 2


def test_visual_v2_keeps_physical_ball_and_foot_proxy_contract() -> None:
    model = _load_visual_v2()
    joint_names = _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    geom_names = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    assert "soccer_ball_free" in joint_names
    ball = _geom_id(model, "soccer_ball_geom")
    assert model.geom_contype[ball] == 2
    for prefix in ("BLUE1", "BLUE2", "RED1", "RED2"):
        for side in ("LEFT", "RIGHT"):
            name = f"{prefix}_{side}_FOOT_BALL_PROXY"
            assert name in geom_names
            gid = _geom_id(model, name)
            assert model.geom_contype[gid] == 4
            assert model.geom_conaffinity[gid] == 2


def test_visual_v2_field_goals_lines_and_cameras_exist() -> None:
    model = _load_visual_v2()
    geom_names = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    camera_names = _names(model, mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)
    for name in [
        "line_mid",
        "center_circle_0",
        "visual_v2_blue_penalty_back",
        "visual_v2_red_penalty_back",
        "visual_v2_blue_goal_left_post",
        "visual_v2_blue_goal_right_post",
        "visual_v2_blue_goal_crossbar",
        "visual_v2_red_goal_left_post",
        "visual_v2_red_goal_right_post",
        "visual_v2_red_goal_crossbar",
    ]:
        assert name in geom_names
        gid = _geom_id(model, name)
        assert model.geom_contype[gid] == 0
        assert model.geom_conaffinity[gid] == 0
    assert {"broadcast", "follow_ball", "overview"} <= camera_names


def test_visual_v2_viewer_recorder_config_and_motion_mode_exist() -> None:
    assert Path("mujoco_soccer/rendering/clean_visual_viewer.py").exists()
    assert Path("mujoco_soccer/rendering/visual_v2_recorder.py").exists()
    assert Path("mujoco_soccer/config/visual_v2.yaml").exists()
    assert Path("scripts/start_mujoco_visual_soccer_demo_v2.sh").exists()
    model = _load_visual_v2()
    controller = RobotController.create(model, "T1_BLUE_1", visual_v2=True)
    assert controller.base.turn_first is True
    assert controller.gait.path_coupled is True


def test_visual_v2_run_demo_keeps_ball_guard_contract_declared() -> None:
    source = Path("mujoco_soccer/run_demo.py").read_text()
    assert '"visual-check-v2"' in source
    assert '"visual-v2-demo"' in source
    assert "VisualV2Recorder" in source
    assert "direct_ball_qpos_write" in source
    assert "direct_ball_qvel_write" in source
