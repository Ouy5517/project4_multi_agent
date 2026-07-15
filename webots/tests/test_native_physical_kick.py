from __future__ import annotations

import ast
import math
import re
from pathlib import Path

from controllers.t1_native_ball_controller.coordinate_frames import (
    horizontal_distance as frame_horizontal_distance,
    inverse_transform,
    normalize_xy,
    robot_to_world,
    transform_point,
    world_to_robot,
)
from controllers.t1_native_ball_controller.collision_geometry import (
    sphere_to_axis_aligned_box_signed_distance,
    sphere_to_oriented_box_signed_distance,
)
from common.native_joint_utils import classify_motor_name, missing_required_leg_joints
from common.robot_action import ActionType, RobotAction
from integration.native_robot_action_adapter import (
    NATIVE_KICK_SEQUENCE,
    MotionState,
    NativeRobotActionAdapter,
    clip_target,
    horizontal_displacement,
    smoothstep,
    success_from_ball_motion,
)


PROJECT = Path(__file__).resolve().parents[1]
WORLD = PROJECT / "worlds" / "T1_native_physical_kick.wbt"


def motor_names_from_world() -> list[str]:
    text = WORLD.read_text()
    return re.findall(r'RotationalMotor\s*\{\s*name\s+"([^"]+)"', text)


def test_native_world_uses_controller_not_extern() -> None:
    text = WORLD.read_text()
    assert 'controller "t1_native_ball_controller"' in text
    assert 'controller "<extern>"' not in text
    assert "DEF SOCCER_BALL" in text


def test_motor_device_detection_from_world_names() -> None:
    names = motor_names_from_world()
    assert len(names) >= 20
    assert any("Right" in name and "Hip" in name for name in names)


def test_joint_map_complete_from_real_motor_names() -> None:
    classes: dict[str, list[str]] = {}
    for name in motor_names_from_world():
        classes.setdefault(classify_motor_name(name), []).append(name)
    assert missing_required_leg_joints(classes) == set()


def test_target_angle_clipping() -> None:
    assert clip_target(2.0, -1.0, 1.0) == 1.0
    assert clip_target(-2.0, -1.0, 1.0) == -1.0
    assert clip_target(0.25, -1.0, 1.0) == 0.25


def test_smoothstep_interpolation_bounds() -> None:
    assert smoothstep(-1.0) == 0.0
    assert smoothstep(0.0) == 0.0
    assert smoothstep(1.0) == 1.0
    assert smoothstep(2.0) == 1.0
    assert 0.45 < smoothstep(0.5) < 0.55


def test_state_machine_order_contains_recover_after_swing() -> None:
    assert NATIVE_KICK_SEQUENCE.index(MotionState.SWING_FORWARD) < NATIVE_KICK_SEQUENCE.index(MotionState.RECOVER)
    assert NATIVE_KICK_SEQUENCE[-1] == MotionState.DONE


def test_ball_displacement_threshold() -> None:
    assert horizontal_displacement([0.0, 0.0], [0.03, 0.04]) == 0.05
    assert not success_from_ball_motion([0.0, 0.0], [0.05, 0.0])
    assert success_from_ball_motion([0.0, 0.0], [0.051, 0.0])


def test_supervisor_moved_ball_flag_false_in_controller() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert '"supervisor_moved_ball": False' in text
    assert "setSFVec3f" not in text
    assert ".setSFVec3f" not in text
    forbidden_ball_mutations = [
        "translation.setSFVec3f",
        "rotation.setSFRotation",
        ".resetPhysics(",
        ".addForce(",
        ".addForceWithOffset(",
        "ball_node.setVelocity",
        "ball.setVelocity",
    ]
    for forbidden in forbidden_ball_mutations:
        assert forbidden not in text


def test_read_only_world_observer_exposes_only_getters() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    tree = ast.parse(text)
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ReadOnlyWorldObserver")
    public_methods = {
        node.name
        for node in cls.body
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_") and node.name != "__init__"
    }
    assert public_methods == {"get_robot_pose", "get_foot_position", "get_ball_position", "get_ball_velocity"}
    assert not any(name.startswith("set") for name in public_methods)


def test_dribble_calls_native_adapter() -> None:
    adapter = NativeRobotActionAdapter(max_pushes=3)
    action = RobotAction("T1_BLUE_1", ActionType.DRIBBLE, {"x": 1.0, "y": 0.0})
    row = adapter.execute(action)
    assert row["command"] == "dribble"
    assert row["max_pushes"] == 3


def test_shoot_calls_native_adapter() -> None:
    adapter = NativeRobotActionAdapter(max_pushes=3)
    action = RobotAction("T1_BLUE_1", ActionType.SHOOT, {"x": 3.3, "y": 0.0})
    row = adapter.execute(action)
    assert row["command"] == "shoot"
    assert row["max_pushes"] == 1


def test_assisted_mode_is_explicit() -> None:
    adapter = NativeRobotActionAdapter(assisted_mode=True)
    assert adapter.get_status()["assisted_mode"] is True


def test_max_three_light_pushes_in_config() -> None:
    text = (PROJECT / "config" / "native_kick.yaml").read_text()
    assert "max_pushes: 3" in text
    assert "hold_stand_min_seconds: 2.0" in text
    assert "hold_stand_timeout_seconds: 8.0" in text
    assert "max_joint_error_rad: 0.08" in text
    assert "max_joint_velocity_rad_s: 0.20" in text
    assert "skip_weight_shift: true" in text


def test_each_push_has_recover_stop_phase() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "MotionState.CALIBRATE" in text
    assert "MotionState.PREPARE_KICK" in text
    assert "assisted_hold_stand" in text
    assert "MotionState.RECOVER" in text
    assert "MotionState.VERIFY_BALL" in text


def test_assisted_world_label_and_single_robot_supervisor() -> None:
    text = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    assert "geometry Text" not in text
    assert 'controller "submission_demo_supervisor"' not in text
    start = text.index("DEF T1_BLUE_1 Robot {")
    end = text.index("# === Simplified WorldState markers")
    assert text[start:end].count("supervisor TRUE") == 1


def test_assisted_world_has_unique_real_foot_defs() -> None:
    text = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    assert len(re.findall(r"DEF\s+RIGHT_FOOT\s+Solid\s*\{", text)) == 1
    assert len(re.findall(r"DEF\s+LEFT_FOOT\s+Solid\s*\{", text)) == 1
    assert len(re.findall(r"DEF\s+SOCCER_BALL\s+Solid\s*\{", text)) == 1
    right_body = text[text.index("DEF RIGHT_FOOT Solid {"): text.index('name "right_foot_link"') + 200]
    left_body = text[text.index("DEF LEFT_FOOT Solid {"): text.index('name "left_foot_link"') + 200]
    assert 'name "right_foot_link"' in right_body and "boundingObject Pose" in right_body
    assert 'name "left_foot_link"' in left_body and "boundingObject Pose" in left_body


def test_assisted_observer_resolution_order_and_preconditions() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert 'self._resolve_foot("right")' in text
    assert 'upper = "RIGHT_FOOT"' in text
    assert 'title = "Right_Foot"' in text
    assert "getFromDef({upper})" in text
    assert "getFromProtoDef" not in text
    assert "recursive name search" in text
    assert "right_foot_link" in text
    assert "MotionState.RESOLVE_NODES" in text
    assert "MotionState.FAILED_PRECONDITION" in text
    assert "return self.finish()" in text


def test_assisted_geometry_and_calibration_are_gated() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "if not self.geometry_ok" in text
    assert "return self.finish()" in text
    assert "if not self.calibrate_right_leg()" in text
    assert "if not self.write_predicted_trajectories" in text
    assert "right foot geometry unavailable for calibration" in text


def test_foot_geometry_config_matches_world_box() -> None:
    text = (PROJECT / "config" / "native_kick.yaml").read_text()
    assert "forward_half_length_m: 0.112435" in text
    assert "lateral_half_width_m: 0.05" in text
    assert "vertical_half_height_m: 0.0155" in text
    assert "max_surface_gap_m: 0.07" in text
    assert "max_lateral_offset_m: 0.04" in text


def test_assisted_contact_requires_distance_and_ball_motion() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "dist <= limit and (ball_changed or moving)" in text
    assert "FOOT_BALL_CONTACT_ESTIMATED" in text
    assert "ball_speed_after" in text
    assert "right_foot_front" in text


def test_assisted_kick_skips_precheck_non_contact_levels() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "def predicted_contact_levels" in text
    assert "TRAJECTORY_LEVEL_FILTER" in text
    assert "TRAJECTORY_LEVEL" in text
    assert 'if self.mode == "assisted-kick":' in text
    assert "levels = self.predicted_contact_levels(levels)" in text
    assert "predicted_contact" in text
    assert "min_distance <= threshold" in text
    assert "selected_level" in text
    assert "FAILED_TRAJECTORY_PRECHECK" in text
    assert "trajectory precheck found no predicted-contact level to execute" in text


def test_submission_demo_labels_use_supervisor_not_node() -> None:
    text = (PROJECT / "controllers" / "submission_demo_supervisor" / "submission_demo_supervisor.py").read_text()
    assert ".setLabel(" in text
    assert "node.setLabel" not in text
    assert "sup.setLabel" in text


def test_trajectory_consistency_and_direction_guard_outputs() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "trajectory_execution_consistency.json" in text
    assert "predicted_vs_actual_trajectory.json" in text
    assert "predicted_joint_deltas" in text
    assert "FAILED_TRAJECTORY_DIRECTION" in text
    assert "progress_to_ball" in text
    assert "foot_ball_obb_signed_distance" in text


def test_node_test_script_mode() -> None:
    text = (PROJECT / "scripts" / "start_native_physical_kick.sh").read_text()
    assert "assisted-node-test" in text
    assert "node_resolution_success" in text
    assert "[[ \"$MODE\" == assisted-* ]]" in text


def test_ground_support_config_matches_field_collision_box() -> None:
    config = (PROJECT / "config" / "native_kick.yaml").read_text()
    world = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    assert "ground_def: green_field" in config
    assert "x_min: -3.5" in config and "x_max: 3.5" in config
    assert "y_min: -2.5" in config and "y_max: 2.5" in config
    assert "top_z: 0.0" in config
    assert 'name "green_field"' in world
    assert "boundingObject Box { size 7 5 0.01 }" in world


def test_scene_support_state_gates_hold_stand() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "SCENE_SUPPORT_CHECK" in text
    assert "FAILED_SCENE_SUPPORT" in text
    assert "def finish_support_check" in text
    assert "scene_support_check failed" in text or "scene support check failed" in text
    assert text.index("MotionState.SCENE_SUPPORT_CHECK") < text.index("stable = self.assisted_hold_stand()")


def test_support_check_script_mode() -> None:
    text = (PROJECT / "scripts" / "start_native_physical_kick.sh").read_text()
    assert "support-check" in text
    assert "support_check_success" in text


def test_ball_physics_analysis_preserves_dynamic_ball() -> None:
    world = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    ball = world[world.index("DEF SOCCER_BALL Solid {"):world.index("# === ASSISTED PHYSICAL KICK")]
    assert "boundingObject Sphere { radius 0.11 }" in ball
    assert "physics Physics { density -1 mass 0.43 }" in ball
    assert "locked TRUE" not in ball


def test_native_scripts_do_not_start_mck_or_rpc() -> None:
    text = (PROJECT / "scripts" / "start_native_physical_kick.sh").read_text()
    assert "ros2 run booster_rpc_service" not in text
    assert '"$WEBOTS_HOME/webots-controller"' not in text
    assert "./mck" not in text


def test_coordinate_frame_module_round_trips_points() -> None:
    pose = [
        0.0, -1.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 3.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    local = [0.25, -0.5, 0.75]
    world = robot_to_world(local, pose)
    assert world == [1.5, 2.25, 3.75]
    assert all(abs(a - b) < 1e-9 for a, b in zip(world_to_robot(world, pose), local))
    assert transform_point(inverse_transform(pose), world) == world_to_robot(world, pose)
    assert normalize_xy([3.0, 4.0]) == [0.6, 0.8]
    assert frame_horizontal_distance([0.0, 0.0], [3.0, 4.0]) == 5.0


def test_coordinate_frame_gate_precedes_support_and_can_fail() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "COORDINATE_FRAME_CHECK" in text
    assert "FAILED_COORDINATE_FRAME" in text
    assert text.index("MotionState.COORDINATE_FRAME_CHECK") < text.index("MotionState.SCENE_SUPPORT_CHECK")
    assert "robot_to_right_foot_distance_out_of_range" in text
    assert "world_hash_mismatch" in text
    assert "return self.finish()" in text


def test_frame_and_geometry_script_modes_are_assisted_without_mck() -> None:
    text = (PROJECT / "scripts" / "start_native_physical_kick.sh").read_text()
    assert "frame-check" in text and "geometry-check" in text
    assert "coordinate_frame_valid" in text
    assert "geometry_check_success" in text
    assert "WORLD_REALPATH" in text and "WORLD_SHA256" in text
    assert "rpc_service_node" in text
    assert "./mck" not in text


def test_foot_node_ownership_and_hash_diagnosis_are_recorded() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "foot_node_ownership.json" in text
    assert "duplicate_def_count" in text
    assert "source_world_lines" in text
    assert "project_world" in text and "runtime_world" in text
    assert "sha256_file" in text
    assert "world_hashes_match" in text


def test_foot_axis_selection_uses_goal_direction_and_writes_file() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "def select_foot_axis" in text
    assert "RED_GOAL" in text
    assert "for axis_index, axis_name in ((0, \"X\"), (1, \"Y\"))" in text
    assert "angle_to_kick_direction_deg" in text
    assert "foot_axis_selection.json" in text
    assert "vector = [orientation[axis_index], orientation[axis_index + 3], orientation[axis_index + 6]]" in text


def test_scene_and_ball_adjustment_v3_match_world() -> None:
    world = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    scene = (PROJECT / "results" / "native_physical_kick" / "scene_translation_adjustment_v3.json").read_text()
    ball = (PROJECT / "results" / "native_physical_kick" / "ball_placement_adjustment_v4.json").read_text()
    assert "translation 0.013387494410705969 0.024827210968314528 0.666" in world
    assert "translation 0.35182914436305174 0.014820036366521698 0.112" in world
    assert '"target_feet_midpoint_xy": [' in scene
    assert '"expected_left_to_right_foot_distance_unchanged_m": 0.21955916107907464' in scene
    assert '"right_foot_surface_gap_m": 0.06821202081493598' in ball
    assert '"supervisor_moved_ball": false' in ball


def test_geometry_check_exits_before_calibration_or_swing() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "def finish_geometry_check" in text
    assert "geometry_check_summary.json" in text
    assert "if self.mode == \"geometry-check\":" in text
    assert text.index("return self.finish_geometry_check()") < text.index("if not self.geometry_ok")
    assert "support_check_success" in text
    assert "ball_in_front" in text and "feet_on_ground" in text and "ball_on_ground" in text


def test_assisted_world_internal_t1_translations_are_relative() -> None:
    text = (PROJECT / "worlds" / "T1_native_assisted_kick.wbt").read_text()
    robot = text[text.index("DEF T1_BLUE_1 Robot {"): text.index("# === Simplified WorldState markers")]
    assert "translation -0.1 0.0 0.666" not in robot
    assert "translation 0 -0.00025 -0.012" in robot
    assert "translation 0 0.00025 -0.012" in robot


def test_sphere_to_obb_signed_distance_and_overlap() -> None:
    box = {
        "center": [0.0, 0.0, 0.0],
        "half_extents": [0.5, 0.25, 0.1],
        "axes": [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    }
    clear = sphere_to_oriented_box_signed_distance([0.0, 0.8, 0.0], 0.1, box)
    assert math.isclose(clear["signed_surface_distance"], 0.2, abs_tol=1e-9)
    overlap = sphere_to_oriented_box_signed_distance([0.0, 0.55, 0.0], 0.1, box)
    assert overlap["overlapping"]
    assert math.isclose(overlap["overlap_depth"], 0.05, abs_tol=1e-9)


def test_sphere_to_axis_aligned_box_distance() -> None:
    row = sphere_to_axis_aligned_box_signed_distance([0.0, 0.0, 0.3], 0.1, [-1.0, -1.0, -0.1], [1.0, 1.0, 0.0])
    assert math.isclose(row["signed_surface_distance"], 0.2, abs_tol=1e-9)
    assert not row["overlapping"]


def test_collision_audit_and_settle_modes_are_gated_before_motion() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "collision-audit" in text and "settle-check" in text
    assert "INITIAL_COLLISION_AUDIT" in text
    assert "initial_contact_audit.json" in text
    assert "initial_collision_geometry.json" in text
    assert "settle_check.json" in text
    assert 'self.mode not in {"support-check", "collision-audit", "settle-check"}' in text
    assert text.index('if self.mode == "collision-audit"') < text.index("MotionState.SCENE_SUPPORT_CHECK")
    assert text.index('if self.mode == "settle-check"') < text.index("MotionState.SCENE_SUPPORT_CHECK")


def test_settle_failure_prevents_geometry_and_calibration() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "settle_check_success" in text
    assert "return self.finish_settle_check()" in text
    assert "self.finish_settle_check(write_summary=False)" in text
    assert "SETTLE_CHECK_PREFLIGHT_DONE" in text
    assert text.index("return self.finish_settle_check()") < text.index("MotionState.SCENE_SUPPORT_CHECK")
    assert text.index("if not self.geometry_ok") < text.index("MotionState.CALIBRATE")


def test_initial_ejection_is_not_kick_success() -> None:
    text = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert '"initial_ejection"' in text
    assert '"kick_success": False' in text
    assert "first_ball_motion_event" in text


def test_collision_audit_script_mode_and_no_supervisor_ball_motion() -> None:
    script = (PROJECT / "scripts" / "start_native_physical_kick.sh").read_text()
    controller = (PROJECT / "controllers" / "t1_native_ball_controller" / "t1_native_ball_controller.py").read_text()
    assert "collision-audit" in script and "settle-check" in script
    assert "collision_audit_success" in script
    assert "settle_check_success" in script
    for forbidden in ("setSFVec3f", "ball_node.setVelocity", ".addForce(", ".resetPhysics("):
        assert forbidden not in controller
