"""手臂下垂姿势单测 (mujoco_soccer 代理模型)"""
from mujoco_soccer.control.arm_pose import (
    LEFT_SHOULDER_ROLL_HANG,
    RIGHT_SHOULDER_ROLL_HANG,
    ELBOW_HANG,
    hanging_arm_targets,
    kick_arm_overlay,
)


def test_hanging_arms_are_down():
    """代理零位已下垂: roll 近零, 肘微屈, pitch 静立为 0"""
    arms = hanging_arm_targets(0.0, moving=False)
    assert abs(arms["Left_Shoulder_Roll"]) < 0.2
    assert abs(arms["Right_Shoulder_Roll"]) < 0.2
    assert arms["Left_Shoulder_Roll"] == LEFT_SHOULDER_ROLL_HANG
    assert arms["Right_Shoulder_Roll"] == RIGHT_SHOULDER_ROLL_HANG
    assert arms["Left_Elbow_Pitch"] == ELBOW_HANG
    assert abs(arms["Left_Shoulder_Pitch"]) < 1e-6
    assert abs(arms["Right_Shoulder_Pitch"]) < 1e-6


def test_walking_swing_keeps_hang_roll():
    import math
    arms = hanging_arm_targets(math.pi / 2, moving=True, swing_amp=0.45)
    assert abs(arms["Left_Shoulder_Roll"]) < 0.2
    assert abs(arms["Right_Shoulder_Roll"]) < 0.2
    assert arms["Left_Elbow_Pitch"] < -0.2
    assert abs(arms["Left_Shoulder_Pitch"]) > 0.2 or abs(arms["Right_Shoulder_Pitch"]) > 0.2


def test_kick_overlay_keeps_hang():
    arms = kick_arm_overlay(0.5, "kick")
    assert abs(arms["Left_Shoulder_Roll"]) < 0.2
    assert abs(arms["Right_Shoulder_Roll"]) < 0.2
    assert arms["Left_Elbow_Pitch"] < -0.2
