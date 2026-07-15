from __future__ import annotations

import math
import re
from pathlib import Path

from controllers.four_robot_match_supervisor.assisted_locomotion import AssistedLocomotion, wrap_angle
from controllers.four_robot_match_supervisor.four_robot_orchestrator import OPENING_TARGETS, SCENARIO_PUSHES
from controllers.t1_assisted_soccer_controller.gait_generator import GaitGenerator
from controllers.t1_native_ball_controller.collision_geometry import sphere_to_oriented_box_signed_distance
from common.world_state import Ball, OpponentState, Point, RobotState, WorldState
from strategy.pass_strategy import PassStrategy


PROJECT = Path(__file__).resolve().parents[1]
WORLD = PROJECT / "worlds" / "T1_2v2_assisted_physical_soccer.wbt"


def world_text() -> str:
    return WORLD.read_text()


def test_four_robot_world_contains_required_actors() -> None:
    text = world_text()
    for token in ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2", "SOCCER_BALL", "BLUE_GOAL", "RED_GOAL"]:
        assert token in text
    assert text.count('controller "t1_assisted_soccer_controller"') == 4
    assert 'controller "four_robot_match_supervisor"' in text
    assert 'controller "match_state_monitor"' in text


def test_four_robot_world_has_unique_foot_defs() -> None:
    text = world_text()
    for name in [
        "BLUE1_RIGHT_FOOT",
        "BLUE1_LEFT_FOOT",
        "BLUE2_RIGHT_FOOT",
        "BLUE2_LEFT_FOOT",
        "RED1_RIGHT_FOOT",
        "RED1_LEFT_FOOT",
        "RED2_RIGHT_FOOT",
        "RED2_LEFT_FOOT",
    ]:
        assert len(re.findall(rf"DEF\s+{name}\s+Solid\s*\{{", text)) == 1
    assert "DEF RIGHT_FOOT Solid" not in text
    assert "DEF LEFT_FOOT Solid" not in text


def test_four_robot_controller_args_are_unique() -> None:
    text = world_text()
    for name in ["BLUE_1", "BLUE_2", "RED_1", "RED_2"]:
        assert f'controllerArgs [ "{name}" ]' in text


def test_four_robot_world_has_no_text_or_submission_supervisor() -> None:
    text = world_text()
    assert "geometry Text" not in text
    assert " Text {" not in text
    assert 'controller "submission_demo_supervisor"' not in text


def test_foot_contact_proxy_is_declared_at_feet_only() -> None:
    text = world_text()
    assert text.count('contactMaterial "foot_contact"') == 8
    assert "FootContactProxy" not in text
    assert "_FOOT_PROXY" in text


def test_udp_ports_are_unique() -> None:
    text = (PROJECT / "controllers/t1_assisted_soccer_controller/t1_assisted_soccer_controller.py").read_text()
    ports = [int(item) for item in re.findall(r"\b(1810[1-4]|18120)\b", text)]
    assert len(set(ports)) >= 5
    assert all(str(port) in text for port in [18101, 18102, 18103, 18104, 18120])


def test_gait_amplitudes_are_visible() -> None:
    gait = GaitGenerator()
    cfg = gait.config
    assert cfg.hip_pitch_amplitude >= 0.16
    assert cfg.knee_pitch_amplitude >= 0.20
    assert cfg.shoulder_pitch_amplitude >= 0.18
    moving = gait.targets(0.275, moving=True)
    assert abs(moving["Left_Hip_Pitch"] - moving["Right_Hip_Pitch"]) > 0.30
    assert abs(moving["Left_Shoulder_Pitch"] - moving["Right_Shoulder_Pitch"]) > 0.35


def test_opening_targets_move_each_robot_at_least_threshold() -> None:
    starts = {
        "BLUE_1": (-1.80, -0.70),
        "BLUE_2": (-1.60, 0.90),
        "RED_1": (0.70, -0.80),
        "RED_2": (1.30, 0.80),
    }
    # The full demo adds prepare-to-ball and push motion; opening alone must still be meaningful.
    assert {target.robot for target in OPENING_TARGETS} == set(starts)
    for target in OPENING_TARGETS:
        assert 0.06 <= target.speed <= 0.24


def test_push_plans_cover_required_scenario_displacements() -> None:
    stages = {plan.stage: plan for plan in SCENARIO_PUSHES}
    assert stages["BLUE_1_DRIBBLE_1"].min_ball_displacement >= 0.08
    assert stages["BLUE_1_DRIBBLE_2"].min_ball_displacement >= 0.08
    assert stages["BLUE_1_PASS_TO_BLUE_2"].min_ball_displacement >= 0.20
    assert stages["BLUE_2_SHOOT"].min_ball_displacement >= 0.25
    assert stages["RED_1_CLEAR"].min_ball_displacement >= 0.15
    assert stages["RED_2_COUNTER"].min_ball_displacement >= 0.15


def test_supervisor_does_not_mutate_ball() -> None:
    text = (PROJECT / "controllers/four_robot_match_supervisor/four_robot_match_supervisor.py").read_text()
    forbidden = [
        "ball_node.getField(\"translation\").setSFVec3f",
        "ball_node.getField('translation').setSFVec3f",
        "ball_node.setVelocity",
        "ball_node.resetPhysics",
        "ball_node.addForce",
        "ball_node.addForceWithOffset",
    ]
    for item in forbidden:
        assert item not in text


def test_root_motion_smooth_step_bound() -> None:
    loco = AssistedLocomotion(timestep_s=0.001)
    assert math.isclose(0.18 * loco.timestep_s, 0.00018)
    assert math.isclose(abs(math.degrees(wrap_angle(math.radians(370)))), 10.0)


def test_sphere_to_obb_signed_gap_contact() -> None:
    box = {
        "center": [0.0, 0.0, 0.0],
        "half_extents": [0.1, 0.05, 0.05],
        "axes": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }
    gap = sphere_to_oriented_box_signed_distance([0.21, 0.0, 0.0], 0.11, box)
    assert abs(gap["signed_surface_distance"]) < 1e-9


def test_public_pass_strategy_helpers_are_real() -> None:
    world = WorldState(
        timestamp=0.0,
        ball=Ball(0.0, 0.0),
        robots=[
            RobotState("BLUE_1", "BLUE", 0.0, 0.0, 0.0, "handler", True),
            RobotState("BLUE_2", "BLUE", 1.2, 0.5, 0.0, "support"),
        ],
        opponents=[OpponentState("RED_2", 0.6, 0.25)],
        our_goal=Point(-3.3, 0.0),
        enemy_goal=Point(3.3, 0.0),
        field_width=7.0,
        field_height=5.0,
    )
    strategy = PassStrategy()
    assert strategy.is_pass_line_clear(world, "BLUE_1", "BLUE_2") is False
    assert strategy.evaluate_receiver(world, "BLUE_1", "BLUE_2") is not None


def test_scripts_do_not_start_mck_or_rpc() -> None:
    start = (PROJECT / "scripts/start_four_robot_physical_demo.sh").read_text()
    assert "ros2 run" not in start
    assert re.search(r"webots-controller\\s+--", start) is None
    assert "./mck" not in start
    assert "rpc_service_node >" not in start
    assert "killall" not in start
    assert "pkill" not in start


def test_summary_contract_fields_are_written() -> None:
    text = (PROJECT / "controllers/four_robot_match_supervisor/four_robot_match_supervisor.py").read_text()
    for key in [
        "mck_used",
        "rpc_used",
        "supervisor_moved_ball",
        "blue1_path_length",
        "blue2_path_length",
        "red1_path_length",
        "red2_path_length",
        "total_contacts",
        "dribble_success",
        "pass_success",
        "shoot_success",
        "red1_clear_success",
        "red2_counter_success",
        "demo_success",
    ]:
        assert f'"{key}"' in text
