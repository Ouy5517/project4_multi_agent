from __future__ import annotations

from typing import Any


REQUIRED_SUMMARY_FIELDS = [
    "run_id",
    "engine",
    "model_path",
    "model_type",
    "official_t1_model_used",
    "assisted_planar_locomotion",
    "native_joint_gait",
    "direct_ball_state_write",
    "ball_physics",
    "completed_stages",
    "failed_stages",
    "timed_out_stages",
    "simulation_time",
    "wall_time",
    "per_robot_path_length",
    "per_robot_continuous_motion",
    "per_robot_max_turn",
    "per_robot_actual_joint_amplitude",
    "per_robot_contact_count",
    "total_contacts",
    "dribble_strategy",
    "pass_strategy",
    "shoot_strategy",
    "defensive_strategy",
    "dribble_displacement",
    "pass_displacement",
    "receive_displacement",
    "shoot_displacement",
    "red1_clear_displacement",
    "red2_counter_displacement",
    "ball_total_path",
    "ball_mutation_detected",
    "joint_limit_violation",
    "nan_detected",
    "visible_motion_success",
    "physical_contact_success",
    "strategy_success",
    "demo_success",
    "failure_reason",
]


def missing_summary_fields(summary: dict[str, Any]) -> list[str]:
    return [field for field in REQUIRED_SUMMARY_FIELDS if field not in summary]

