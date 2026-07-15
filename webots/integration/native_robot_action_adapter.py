from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from common.robot_action import ActionType, RobotAction


class MotionState(str, Enum):
    INITIALIZE = "INITIALIZE"
    HOLD_STAND = "HOLD_STAND"
    PREPARE = "PREPARE"
    SHIFT_WEIGHT = "SHIFT_WEIGHT"
    LIFT_FOOT = "LIFT_FOOT"
    SWING_FORWARD = "SWING_FORWARD"
    CONTACT_HOLD = "CONTACT_HOLD"
    RETRACT = "RETRACT"
    RECOVER = "RECOVER"
    VERIFY_BALL = "VERIFY_BALL"
    DRIBBLE_REPOSITION = "DRIBBLE_REPOSITION"
    DONE = "DONE"
    FAILED = "FAILED"


NATIVE_KICK_SEQUENCE = [
    MotionState.INITIALIZE,
    MotionState.HOLD_STAND,
    MotionState.PREPARE,
    MotionState.SHIFT_WEIGHT,
    MotionState.LIFT_FOOT,
    MotionState.SWING_FORWARD,
    MotionState.CONTACT_HOLD,
    MotionState.RETRACT,
    MotionState.RECOVER,
    MotionState.VERIFY_BALL,
    MotionState.DONE,
]


def smoothstep(value: float) -> float:
    x = max(0.0, min(1.0, value))
    return x * x * (3.0 - 2.0 * x)


def interpolate(start: float, target: float, fraction: float) -> float:
    return start + smoothstep(fraction) * (target - start)


def clip_target(value: float, lower: float | None, upper: float | None) -> float:
    if lower is not None and math.isfinite(lower):
        value = max(lower, value)
    if upper is not None and math.isfinite(upper):
        value = min(upper, value)
    return value


def horizontal_displacement(initial: list[float] | tuple[float, float] | None,
                            final: list[float] | tuple[float, float] | None) -> float:
    if not initial or not final:
        return 0.0
    return math.hypot(float(final[0]) - float(initial[0]), float(final[1]) - float(initial[1]))


def success_from_ball_motion(initial: list[float] | None, final: list[float] | None,
                             threshold: float = 0.05) -> bool:
    return horizontal_displacement(initial, final) > threshold


@dataclass
class NativeRobotActionAdapter:
    """Thin native action adapter; Webots Motor execution lives in the controller."""

    assisted_mode: bool = False
    max_pushes: int = 3
    calls: list[dict[str, Any]] = field(default_factory=list)

    def _record(self, command: str, **kwargs: Any) -> dict[str, Any]:
        entry = {
            "time": time.time(),
            "adapter": "NativeRobotActionAdapter",
            "command": command,
            "assisted_mode": self.assisted_mode,
        }
        entry.update(kwargs)
        self.calls.append(entry)
        return entry

    def prepare(self) -> dict[str, Any]:
        return self._record("prepare")

    def walk_or_align(self) -> dict[str, Any]:
        return self._record("walk_or_align")

    def dribble(self, target: dict[str, Any] | None = None) -> dict[str, Any]:
        return self._record("dribble", target=target or {}, max_pushes=self.max_pushes)

    def shoot(self, target: dict[str, Any] | None = None) -> dict[str, Any]:
        return self._record("shoot", target=target or {}, max_pushes=1)

    def stop(self) -> dict[str, Any]:
        return self._record("stop")

    def get_status(self) -> dict[str, Any]:
        return {
            "assisted_mode": self.assisted_mode,
            "max_pushes": self.max_pushes,
            "call_count": len(self.calls),
        }

    def execute(self, action: RobotAction) -> dict[str, Any]:
        if action.action_type == ActionType.DRIBBLE:
            return self.dribble(action.target)
        if action.action_type == ActionType.SHOOT:
            return self.shoot(action.target)
        return self._record("unsupported", action_type=action.action_type.value)
