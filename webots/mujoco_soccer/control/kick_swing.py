"""
踢球肢体摆动相位 (mujoco_soccer)
================================
在 VisibleGait 上叠加真正的踢腿时间轴, 替代单纯的 push_pose 常值偏置。

style:
  kick    — 传球/射门大力摆腿
  dribble — 带球短促轻触
"""

from __future__ import annotations

from dataclasses import dataclass


KICK_DURATION = 0.55
DRIBBLE_DURATION = 0.32


def style_from_action(action: str | None) -> str:
    if action is None:
        return "kick"
    key = str(action).lower()
    if key.startswith("dribble") or key in {"receive", "press_ball"}:
        return "dribble"
    return "kick"


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _kick_offsets(progress: float) -> dict[str, float]:
    """大力踢球腿部关节偏移 (手臂由 arm_pose 负责下垂摆动)。"""
    if progress < 0.25:
        t = _smoothstep(progress / 0.25)
        r_hip_back = _lerp(0.0, 0.55, t)
        r_knee = _lerp(0.0, 1.20, t)
        forward = 0.0
    elif progress < 0.48:
        t = _smoothstep((progress - 0.25) / 0.23)
        r_hip_back = _lerp(0.55, -1.05, t)
        r_knee = _lerp(1.20, 0.22, t)
        forward = max(0.0, -r_hip_back)
    elif progress < 0.70:
        t = _smoothstep((progress - 0.48) / 0.22)
        r_hip_back = _lerp(-1.05, -0.80, t)
        r_knee = _lerp(0.22, 0.30, t)
        forward = max(0.0, -r_hip_back)
    else:
        t = _smoothstep((progress - 0.70) / 0.30)
        r_hip_back = _lerp(-0.80, 0.0, t)
        r_knee = _lerp(0.30, 0.0, t)
        forward = max(0.0, -r_hip_back)

    # T1: Hip_Pitch 正 = 前抬; 取 -r_hip_back 使前踢为正
    # 手臂由 VisibleGaitController + arm_pose 下垂摆动, 此处只驱动腿
    scale = 0.85
    return {
        "Right_Hip_Pitch": -r_hip_back * scale,
        "Right_Knee_Pitch": r_knee * scale * 0.7,
        "Right_Ankle_Pitch": r_hip_back * 0.22 * scale,
        "Left_Hip_Pitch": -0.06 * scale,
        "Left_Knee_Pitch": 0.12 * scale,
        "Waist": forward * 0.07 * scale,
        "Head_pitch": 0.06,
    }


def _dribble_offsets(progress: float) -> dict[str, float]:
    """带球轻触腿部关节偏移。"""
    if progress < 0.30:
        t = _smoothstep(progress / 0.30)
        r_hip = _lerp(0.0, 0.18, t)
        r_knee = _lerp(0.12, 0.50, t)
    elif progress < 0.55:
        t = _smoothstep((progress - 0.30) / 0.25)
        r_hip = _lerp(0.18, -0.45, t)
        r_knee = _lerp(0.50, 0.18, t)
    else:
        t = _smoothstep((progress - 0.55) / 0.45)
        r_hip = _lerp(-0.45, 0.0, t)
        r_knee = _lerp(0.18, 0.08, t)

    scale = 0.55
    return {
        "Right_Hip_Pitch": -r_hip * scale,
        "Right_Knee_Pitch": r_knee * scale * 0.7,
        "Right_Ankle_Pitch": r_hip * 0.20 * scale,
        "Left_Hip_Pitch": -0.03 * scale,
        "Left_Knee_Pitch": 0.08 * scale,
        "Waist": max(0.0, -r_hip) * 0.05 * scale,
        "Head_pitch": 0.03,
    }


def joint_offsets(progress: float, style: str = "kick") -> dict[str, float]:
    p = max(0.0, min(1.0, progress))
    if style == "dribble":
        return _dribble_offsets(p)
    return _kick_offsets(p)


@dataclass
class KickSwingState:
    """单机踢球摆动状态机"""

    progress: float = -1.0  # <0 表示未激活
    style: str = "kick"
    duration: float = KICK_DURATION

    @property
    def active(self) -> bool:
        return self.progress >= 0.0

    def start(self, style: str = "kick") -> None:
        self.style = style if style in {"kick", "dribble"} else "kick"
        self.duration = DRIBBLE_DURATION if self.style == "dribble" else KICK_DURATION
        self.progress = 0.0

    def stop(self) -> None:
        self.progress = -1.0

    def step(self, dt: float) -> bool:
        """推进相位; 返回 True 表示仍在播放"""
        if self.progress < 0.0:
            return False
        self.progress += dt / max(1e-6, self.duration)
        if self.progress >= 1.0:
            self.progress = -1.0
            return False
        return True

    def joint_offsets(self) -> dict[str, float]:
        if self.progress < 0.0:
            return {}
        return joint_offsets(min(1.0, self.progress), self.style)

    def push_pose_equivalent(self) -> float:
        """兼容旧逻辑: 触球附近给出接近 1 的 push_pose"""
        if self.progress < 0.0:
            return 0.0
        p = self.progress
        peak = 0.42 if self.style == "dribble" else 0.48
        width = 0.22
        dist = abs(p - peak) / width
        return max(0.0, 1.0 - dist * dist)
