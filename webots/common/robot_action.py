from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class ActionType(str, Enum):
    PASS = "PASS"
    MOVE_TO_RECEIVE = "MOVE_TO_RECEIVE"
    MOVE_TO_SUPPORT = "MOVE_TO_SUPPORT"
    DRIBBLE = "DRIBBLE"
    SHOOT = "SHOOT"
    BLOCK = "BLOCK"
    INTERCEPT = "INTERCEPT"
    CLEAR = "CLEAR"
    COUNTER = "COUNTER"
    HOLD = "HOLD"
    MARK_OPPONENT = "MARK_OPPONENT"
    CHASE_BALL = "CHASE_BALL"
    STOP = "STOP"


@dataclass(frozen=True)
class RobotAction:
    robot_id: str
    action_type: ActionType
    target: dict[str, Any]
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    reason: str = ""
    confidence: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["action_type"] = self.action_type.value
        return data
