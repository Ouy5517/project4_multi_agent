from __future__ import annotations

from mujoco_soccer.control.joint_trajectory import JointTrajectoryState, trajectory_offsets


def test_kick_trajectory_has_leg_offsets() -> None:
    offsets = trajectory_offsets("shoot", 0.45)
    assert offsets["Right_Hip_Pitch"] > 0.3
    assert offsets["Right_Knee_Pitch"] > 0.1
    assert "Waist" in offsets


def test_block_trajectory_has_defensive_pose_offsets() -> None:
    offsets = trajectory_offsets("block", 0.5)
    assert offsets["Left_Shoulder_Roll"] > 0.5
    assert offsets["Right_Shoulder_Roll"] < -0.5
    assert offsets["Left_Knee_Pitch"] > 0.1


def test_joint_trajectory_state_lifecycle() -> None:
    state = JointTrajectoryState()
    state.start("pass")
    assert state.active
    assert state.joint_offsets()
    while state.active:
        state.step(0.1)
    assert not state.active
