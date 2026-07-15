from __future__ import annotations

from dataclasses import dataclass


TRAJECTORY_DURATION = {
    "pass": 0.50,
    "shoot": 0.62,
    "clear": 0.55,
    "intercept": 0.38,
    "dribble": 0.34,
    "block": 0.70,
}


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _smoothstep(value: float) -> float:
    t = _clamp01(value)
    return t * t * (3.0 - 2.0 * t)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _kick_like(progress: float, power: float) -> dict[str, float]:
    p = _clamp01(progress)
    if p < 0.28:
        t = _smoothstep(p / 0.28)
        hip = _lerp(0.0, 0.52 * power, t)
        knee = _lerp(0.10, 1.05 * power, t)
    elif p < 0.52:
        t = _smoothstep((p - 0.28) / 0.24)
        hip = _lerp(0.52 * power, -1.00 * power, t)
        knee = _lerp(1.05 * power, 0.20, t)
    else:
        t = _smoothstep((p - 0.52) / 0.48)
        hip = _lerp(-0.80 * power, 0.0, t)
        knee = _lerp(0.25, 0.08, t)
    return {
        "Right_Hip_Pitch": -hip,
        "Right_Knee_Pitch": knee * 0.70,
        "Right_Ankle_Pitch": hip * 0.18,
        "Left_Hip_Pitch": -0.05 * power,
        "Left_Knee_Pitch": 0.10 * power,
        "Waist": max(0.0, -hip) * 0.06,
        "Head_pitch": 0.04,
    }


def trajectory_offsets(action: str, progress: float) -> dict[str, float]:
    key = action.lower()
    p = _clamp01(progress)
    if key == "shoot":
        return _kick_like(p, 1.0)
    if key in {"pass", "clear"}:
        return _kick_like(p, 0.78 if key == "pass" else 0.86)
    if key in {"dribble", "intercept"}:
        return _kick_like(p, 0.48)
    if key == "block":
        hold = _smoothstep(min(p, 1.0 - p) / 0.28)
        return {
            "Left_Hip_Pitch": -0.08 * hold,
            "Right_Hip_Pitch": -0.08 * hold,
            "Left_Knee_Pitch": 0.26 * hold,
            "Right_Knee_Pitch": 0.26 * hold,
            "Left_Hip_Roll": 0.10 * hold,
            "Right_Hip_Roll": -0.10 * hold,
            "Left_Shoulder_Roll": 0.70 * hold,
            "Right_Shoulder_Roll": -0.70 * hold,
            "Left_Elbow_Pitch": -0.25 * hold,
            "Right_Elbow_Pitch": -0.25 * hold,
            "Waist": 0.04 * hold,
            "Head_pitch": 0.10 * hold,
        }
    return {}


@dataclass
class JointTrajectoryState:
    action: str | None = None
    progress: float = -1.0
    duration: float = 0.0

    @property
    def active(self) -> bool:
        return self.progress >= 0.0 and self.action is not None

    def start(self, action: str) -> None:
        key = action.lower()
        self.action = key
        self.duration = TRAJECTORY_DURATION.get(key, 0.40)
        self.progress = 0.0

    def step(self, dt: float) -> bool:
        if not self.active:
            return False
        self.progress += dt / max(1e-6, self.duration)
        if self.progress >= 1.0:
            self.action = None
            self.progress = -1.0
            self.duration = 0.0
            return False
        return True

    def joint_offsets(self) -> dict[str, float]:
        if not self.active or self.action is None:
            return {}
        return trajectory_offsets(self.action, self.progress)

    def push_pose_equivalent(self) -> float:
        if not self.active or self.action is None or self.action == "block":
            return 0.0
        peak = 0.42
        width = 0.24
        dist = abs(self.progress - peak) / width
        return max(0.0, 1.0 - dist * dist)
