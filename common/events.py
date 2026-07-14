from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class ActionEvent:
    event_id: str
    tick: int
    timestamp: float
    robot_id: int
    action: str
    params: Dict[str, Any] = field(default_factory=dict)
    accepted: bool = True
    reject_code: Optional[str] = None


@dataclass(frozen=True)
class DecisionEvent:
    tick: int
    timestamp: float
    robot_id: int
    state: str
    role: str
    reason_code: str
    reason: str


@dataclass(frozen=True)
class OutcomeEvent:
    tick: int
    timestamp: float
    outcome: str
    success: bool
    metrics: Dict[str, float] = field(default_factory=dict)
    failure_code: Optional[str] = None
