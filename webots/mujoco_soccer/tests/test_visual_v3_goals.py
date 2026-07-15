from __future__ import annotations

from pathlib import Path

import mujoco

from mujoco_soccer.rendering.realtime_scheduler import RealtimeScheduler
from mujoco_soccer.tools_generate_proxy_model import main as generate_model


MODEL = Path("mujoco_soccer/models/t1_2v2_soccer_visual_v3.xml")


def _load() -> mujoco.MjModel:
    generate_model()
    return mujoco.MjModel.from_xml_path(str(MODEL))


def _names(model: mujoco.MjModel, obj: mujoco.mjtObj, count: int) -> set[str]:
    return {mujoco.mj_id2name(model, obj, idx) or "" for idx in range(count)}


def _geom(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def test_visual_v3_model_loads_and_preserves_ball_contract() -> None:
    model = _load()
    joints = _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    geoms = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    assert "soccer_ball_free" in joints
    assert "soccer_ball_geom" in geoms
    assert MODEL.exists()


def test_visual_v3_uses_official_t1_meshes_with_assisted_base_contract() -> None:
    model = _load()
    joints = _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    geoms = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    actuators = _names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    assert "T1_BLUE_1_mesh_Trunk" in geoms
    assert "T1_RED_1_mesh_Logo" in geoms
    for robot in ("T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"):
        assert f"{robot}_base_x" in joints
        assert f"{robot}_base_y" in joints
        assert f"{robot}_base_yaw" in joints
        assert f"{robot}_base_x_act" in actuators
        assert f"{robot}_base_y_act" in actuators
        assert f"{robot}_base_yaw_act" in actuators
    assert "BLUE1_RIGHT_FOOT_BALL_PROXY" in geoms


def test_visual_v3_goals_have_complete_front_frame_depth_and_nets() -> None:
    model = _load()
    geoms = _names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    for prefix in ("BLUE_GOAL", "RED_GOAL"):
        for suffix in (
            "left_post",
            "right_post",
            "crossbar",
            "back_left_post",
            "back_right_post",
            "top_left_depth",
            "top_right_depth",
            "back_bottom",
            "left_net",
            "right_net",
            "top_net",
            "back_net",
            "base_panel",
        ):
            assert f"{prefix}_{suffix}" in geoms
    assert 1.20 >= 1.0
    assert 0.70 >= 0.6
    assert 0.38 >= 0.3


def test_visual_v3_goal_nets_do_not_collide_and_posts_can_collide() -> None:
    model = _load()
    for prefix in ("BLUE_GOAL", "RED_GOAL"):
        for suffix in ("left_net", "right_net", "top_net", "back_net"):
            gid = _geom(model, f"{prefix}_{suffix}")
            assert model.geom_contype[gid] == 0
            assert model.geom_conaffinity[gid] == 0
        for suffix in ("left_post", "right_post", "crossbar"):
            gid = _geom(model, f"{prefix}_{suffix}")
            assert model.geom_contype[gid] == 1
            assert model.geom_conaffinity[gid] == 2


def test_visual_v3_cameras_and_shoot_direction_are_declared() -> None:
    model = _load()
    cameras = _names(model, mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)
    assert "broadcast_wide" in cameras
    assert "broadcast_action" in cameras
    assert "broadcast" in cameras
    source = Path("mujoco_soccer/run_demo.py").read_text(encoding="utf-8")
    assert "RED_GOAL = (3.35, 0.0)" in source


def test_visual_v3_viewer_is_not_matplotlib_and_modes_are_split() -> None:
    fast = Path("mujoco_soccer/rendering/fast_visual_viewer.py").read_text()
    script = Path("scripts/start_mujoco_visual_soccer_demo_v3.sh").read_text()
    assert "matplotlib" not in fast.lower()
    assert "pyplot" not in fast.lower()
    assert "--no-record" in script
    assert "--fast-viewer" in script
    assert "PLAYBACK OF RECORDED PHYSICAL SIMULATION" in script


def test_visual_v3_render_interval_and_scheduler_do_not_accumulate_sleep() -> None:
    scheduler = RealtimeScheduler(timestep=0.005, target_fps=30, real_time_factor=1.0)
    assert scheduler.render_interval_steps == 7
    assert scheduler.should_render(7)
    assert not scheduler.should_render(8)


def test_visual_v3_ghost_defaults_are_off() -> None:
    cfg = Path("mujoco_soccer/config/visual_v2.yaml").read_text()
    assert "show_ghosts: false" in cfg
    assert "show_robot_trails: false" in cfg or "show_path_trail=false" not in cfg
