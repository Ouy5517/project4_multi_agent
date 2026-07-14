"""mujoco_soccer 踢球肢体摆动单测"""
from mujoco_soccer.control.kick_swing import (
    KickSwingState,
    joint_offsets,
    style_from_action,
)


def test_style_from_action():
    assert style_from_action("dribble_1") == "dribble"
    assert style_from_action("receive") == "dribble"
    assert style_from_action("pass") == "kick"
    assert style_from_action("shoot") == "kick"


def test_kick_offsets_peak_forward_hip():
    mid = joint_offsets(0.48, "kick")
    assert mid["Right_Hip_Pitch"] > 0.4  # 前踢


def test_dribble_offsets_shallower():
    kick = joint_offsets(0.48, "kick")
    dribble = joint_offsets(0.42, "dribble")
    assert abs(dribble["Right_Hip_Pitch"]) < abs(kick["Right_Hip_Pitch"])


def test_kick_swing_lifecycle():
    state = KickSwingState()
    assert not state.active
    state.start("kick")
    assert state.active
    dt = 0.05
    steps = 0
    while state.active and steps < 40:
        state.step(dt)
        steps += 1
        _ = state.joint_offsets()
        _ = state.push_pose_equivalent()
    assert not state.active
    assert steps > 5
