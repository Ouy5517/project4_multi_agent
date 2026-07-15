"""
T1 代理模型手臂姿势 (mujoco_soccer 四机)
=========================================
代理骨架: 上臂 geom 已沿 -Z 伸展，零位就是自然下垂。
- Shoulder_Roll ≈ 0 保持下垂 (勿套官方 T1 的 ±1.25，那是横伸零位专用)
- Shoulder_Pitch 前后摆臂
- Elbow_Pitch 略负 = 自然微屈

对比: 官方 Booster T1 零位是横伸 T 字，需 Shoulder_Roll 下垂；
见根目录 simulation/limb_animator.py。
"""

from __future__ import annotations

import math

# 代理模型下垂: roll 近零; 微收一点贴身侧
LEFT_SHOULDER_ROLL_HANG = 0.08
RIGHT_SHOULDER_ROLL_HANG = -0.08
ELBOW_HANG = -0.45


def hanging_arm_targets(
    phase: float = 0.0,
    *,
    moving: bool = False,
    swing_amp: float = 0.45,
    turn_bias: float = 0.0,
    brake: float = 0.0,
    kick_blend: float = 0.0,
) -> dict[str, float]:
    """
    返回代理模型手臂关节目标 (绝对角)。
    始终保持下垂; moving 时在下垂基础上前后摆动。
    """
    # 刹车时略张开, 仍接近身侧下垂
    roll_open = 0.12 * brake
    l_roll = LEFT_SHOULDER_ROLL_HANG + roll_open
    r_roll = RIGHT_SHOULDER_ROLL_HANG - roll_open

    if moving and kick_blend < 0.55:
        left = math.sin(phase)
        right = -left
        l_pitch = swing_amp * right - 0.08 * turn_bias
        r_pitch = swing_amp * left - 0.08 * turn_bias
        # 屈肘随摆臂轻微泵动 (更负=更屈, 不抬臂)
        l_elbow = ELBOW_HANG - 0.20 * max(0.0, right)
        r_elbow = ELBOW_HANG - 0.20 * max(0.0, left)
    else:
        l_pitch = 0.0
        r_pitch = 0.0
        l_elbow = ELBOW_HANG
        r_elbow = ELBOW_HANG

    if brake > 1e-3:
        l_pitch += 0.18 * brake
        r_pitch += 0.18 * brake
        l_elbow += 0.10 * brake
        r_elbow += 0.10 * brake

    return {
        "Left_Shoulder_Roll": l_roll,
        "Right_Shoulder_Roll": r_roll,
        "Left_Shoulder_Pitch": l_pitch,
        "Right_Shoulder_Pitch": r_pitch,
        "Left_Elbow_Pitch": l_elbow,
        "Right_Elbow_Pitch": r_elbow,
    }


def kick_arm_overlay(progress: float, style: str = "kick") -> dict[str, float]:
    """踢球时在下垂基姿上叠加前后臂摆 (绝对角)。"""
    base = hanging_arm_targets(0.0, moving=False)
    p = max(0.0, min(1.0, progress))
    amp = 0.35 if style == "dribble" else 0.55

    if p < 0.25:
        t = p / 0.25
        l_pitch = amp * t
        r_pitch = -0.40 * amp * t
    elif p < 0.48:
        t = (p - 0.25) / 0.23
        l_pitch = amp * (1.0 - 1.6 * t)
        r_pitch = -0.40 * amp + amp * t
    elif p < 0.70:
        t = (p - 0.48) / 0.22
        l_pitch = -0.5 * amp * (1.0 - t)
        r_pitch = 0.45 * amp * (1.0 - 0.5 * t)
    else:
        t = (p - 0.70) / 0.30
        l_pitch = -0.25 * amp * (1.0 - t)
        r_pitch = 0.22 * amp * (1.0 - t)

    base["Left_Shoulder_Pitch"] = l_pitch
    base["Right_Shoulder_Pitch"] = r_pitch
    base["Left_Elbow_Pitch"] = ELBOW_HANG - 0.20 * amp
    base["Right_Elbow_Pitch"] = ELBOW_HANG - 0.15 * amp
    return base
