from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GaitConfig:
    cycle_seconds: float = 1.1
    hip_pitch_amplitude: float = 0.20
    knee_pitch_amplitude: float = 0.25
    ankle_amplitude: float = 0.08
    hip_roll_amplitude: float = 0.045
    shoulder_pitch_amplitude: float = 0.23
    elbow_amplitude: float = 0.12
    return_gain: float = 0.06


class GaitGenerator:
    """Smooth visible gait targets for assisted root locomotion demos."""

    def __init__(self, config: GaitConfig | None = None) -> None:
        self.config = config or GaitConfig()

    def targets(self, sim_time: float, moving: bool, turn_rate: float = 0.0) -> dict[str, float]:
        cfg = self.config
        if not moving:
            return self._stand_targets()

        phase = (sim_time % cfg.cycle_seconds) / cfg.cycle_seconds
        s = math.sin(2.0 * math.pi * phase)
        c = math.cos(2.0 * math.pi * phase)
        turn_bias = max(-0.05, min(0.05, turn_rate * 0.18))
        return {
            "Left_Hip_Pitch": cfg.hip_pitch_amplitude * s,
            "Right_Hip_Pitch": -cfg.hip_pitch_amplitude * s,
            "Left_Knee_Pitch": cfg.knee_pitch_amplitude * max(0.0, -s),
            "Right_Knee_Pitch": cfg.knee_pitch_amplitude * max(0.0, s),
            "Crank_Up_Left": -cfg.ankle_amplitude * s,
            "Crank_Down_Left": cfg.ankle_amplitude * s,
            "Crank_Up_Right": cfg.ankle_amplitude * s,
            "Crank_Down_Right": -cfg.ankle_amplitude * s,
            "Left_Hip_Roll": cfg.hip_roll_amplitude * c + turn_bias,
            "Right_Hip_Roll": -cfg.hip_roll_amplitude * c + turn_bias,
            "Left_Shoulder_Pitch": -cfg.shoulder_pitch_amplitude * s,
            "Right_Shoulder_Pitch": cfg.shoulder_pitch_amplitude * s,
            "Left_Elbow_Pitch": cfg.elbow_amplitude * max(0.0, s),
            "Right_Elbow_Pitch": cfg.elbow_amplitude * max(0.0, -s),
        }

    def push_targets(self, foot: str, fraction: float) -> dict[str, float]:
        t = max(0.0, min(1.0, fraction))
        smooth = t * t * (3.0 - 2.0 * t)
        sign = -1.0 if foot.upper() == "RIGHT" else 1.0
        prefix = "Right" if foot.upper() == "RIGHT" else "Left"
        crank_suffix = "Right" if foot.upper() == "RIGHT" else "Left"
        return {
            f"{prefix}_Hip_Pitch": sign * 0.18 * smooth,
            f"{prefix}_Knee_Pitch": sign * 0.10 * smooth,
            f"Crank_Up_{crank_suffix}": -sign * 0.06 * smooth,
            f"Crank_Down_{crank_suffix}": sign * 0.06 * smooth,
            "Left_Shoulder_Pitch": 0.12,
            "Right_Shoulder_Pitch": -0.12,
        }

    def _stand_targets(self) -> dict[str, float]:
        return {
            "Left_Hip_Pitch": 0.0,
            "Right_Hip_Pitch": 0.0,
            "Left_Knee_Pitch": 0.04,
            "Right_Knee_Pitch": 0.04,
            "Crank_Up_Left": 0.0,
            "Crank_Down_Left": 0.0,
            "Crank_Up_Right": 0.0,
            "Crank_Down_Right": 0.0,
            "Left_Hip_Roll": 0.0,
            "Right_Hip_Roll": 0.0,
            "Left_Shoulder_Pitch": 0.0,
            "Right_Shoulder_Pitch": 0.0,
            "Left_Elbow_Pitch": 0.04,
            "Right_Elbow_Pitch": 0.04,
        }
