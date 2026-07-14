import mujoco
import pytest

from common.world_state import create_pass_scenario
from common.world_state import WorldStateProvider
from simulation.field_simulator import Simulator
from simulation.scenarios import load_scenario_into_simulator
from visualization import mujoco_viewer
from visualization.mujoco_viewer import MujocoSoccerViewer, build_scene_xml


def test_wsl_defaults_to_software_opengl_without_overriding_user_choice():
    automatic_env = {}
    mujoco_viewer.configure_wsl_opengl(automatic_env, "5.15.0-microsoft-standard-WSL2")
    assert automatic_env["LIBGL_ALWAYS_SOFTWARE"] == "1"

    explicit_env = {"LIBGL_ALWAYS_SOFTWARE": "0"}
    mujoco_viewer.configure_wsl_opengl(explicit_env, "5.15.0-microsoft-standard-WSL2")
    assert explicit_env["LIBGL_ALWAYS_SOFTWARE"] == "0"


def test_scene_xml_loads_lightweight_soccer_entities():
    xml = build_scene_xml(blue_ids=[0, 1, 2], yellow_ids=[10, 11, 12])

    model = mujoco.MjModel.from_xml_string(xml)

    for body_name in ("ball", "blue_0", "blue_1", "blue_2", "yellow_10", "yellow_11", "yellow_12"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name) >= 0
    assert "builtin=\"checker\"" in xml
    assert model.ngeom < 40


def test_render_syncs_world_state_into_mujoco_without_window():
    world_state = create_pass_scenario()
    viewer = MujocoSoccerViewer(world_state, launch_window=False)

    viewer.render(world_state, fsm=None)

    ball_id = mujoco.mj_name2id(viewer.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    robot_id = mujoco.mj_name2id(viewer.model, mujoco.mjtObj.mjOBJ_BODY, "blue_0")
    assert viewer.data.xpos[ball_id][0] == pytest.approx(world_state.ball.x)
    assert viewer.data.xpos[ball_id][1] == pytest.approx(world_state.ball.y)
    assert viewer.data.xpos[robot_id][0] == pytest.approx(world_state.teammates[0].x)
    assert viewer.data.xpos[robot_id][1] == pytest.approx(world_state.teammates[0].y)
    assert viewer.is_running()

    viewer.close()
    assert not viewer.is_running()


def test_viewer_accepts_showcase_phase_label():
    viewer = MujocoSoccerViewer(create_pass_scenario(), launch_window=False)

    viewer.set_phase("4/5 卡位防守")

    assert viewer.phase_label == "4/5 卡位防守"
    viewer.close()


def test_viewer_hides_robots_missing_from_later_showcase_phase():
    viewer = MujocoSoccerViewer(create_pass_scenario(), launch_window=False)
    simulator = Simulator()
    load_scenario_into_simulator(simulator, "2v2_attack_defense")

    viewer.render(WorldStateProvider(simulator).get(), fsm=None)

    hidden_body_id = mujoco.mj_name2id(
        viewer.model, mujoco.mjtObj.mjOBJ_BODY, "blue_2"
    )
    assert viewer.data.xpos[hidden_body_id][2] < -1.0
    viewer.close()
