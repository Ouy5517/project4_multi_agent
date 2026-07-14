from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Optional, Tuple

from common.config import ROLE_HOLD_SECONDS, ROLE_SWITCH_MARGIN
from common.world_state import RobotRole, WorldState


class PassPhase(str, Enum):
    PLANNED = "planned"
    KICKED = "kicked"
    IN_FLIGHT = "in_flight"
    RECEIVED = "received"
    FAILED = "failed"


@dataclass
class PassIntent:
    passer_id: int
    receiver_id: int
    target: Tuple[float, float]
    created_at: float
    expires_at: float
    phase: PassPhase = PassPhase.PLANNED
    kick_event_id: Optional[str] = None
    failure_code: Optional[str] = None


@dataclass(frozen=True)
class TeamPlan:
    roles: Dict[int, RobotRole]
    tasks: Dict[int, str]


class TeamCoordinator:
    """Assign stable team roles and single high-level tasks per robot."""

    def __init__(self):
        self.pass_intent: Optional[PassIntent] = None
        self._carrier_id: Optional[int] = None
        self._carrier_since: float = 0.0

    def plan(self, world_state: WorldState) -> TeamPlan:
        if not world_state.teammates:
            return TeamPlan(roles={}, tasks={})

        roles = self._assign_roles(world_state)
        tasks = {
            robot_id: self._task_for_role(role)
            for robot_id, role in roles.items()
        }
        return TeamPlan(roles=roles, tasks=tasks)

    def _assign_roles(self, world_state: WorldState) -> Dict[int, RobotRole]:
        distances = sorted(
            (
                (robot.id, math.hypot(robot.x - world_state.ball.x, robot.y - world_state.ball.y))
                for robot in world_state.teammates
            ),
            key=lambda item: item[1],
        )
        nearest_id, nearest_dist = distances[0]

        if self._carrier_id is None or self._carrier_id not in {item[0] for item in distances}:
            self._carrier_id = nearest_id
            self._carrier_since = world_state.timestamp
        elif nearest_id != self._carrier_id:
            current_dist = dict(distances)[self._carrier_id]
            hold_elapsed = world_state.timestamp - self._carrier_since
            challenger_advantage = current_dist - nearest_dist
            if hold_elapsed >= ROLE_HOLD_SECONDS and challenger_advantage > ROLE_SWITCH_MARGIN:
                self._carrier_id = nearest_id
                self._carrier_since = world_state.timestamp

        carrier_id = self._carrier_id
        remaining = [item for item in distances if item[0] != carrier_id]
        roles: Dict[int, RobotRole] = {carrier_id: RobotRole.BALL_CARRIER}
        if remaining:
            roles[remaining[0][0]] = RobotRole.SUPPORTER
        for robot_id, _ in remaining[1:]:
            roles[robot_id] = RobotRole.DEFENDER
        return roles

    @staticmethod
    def _task_for_role(role: RobotRole) -> str:
        if role == RobotRole.BALL_CARRIER:
            return "CHASE"
        if role == RobotRole.SUPPORTER:
            return "POSITION"
        if role == RobotRole.DEFENDER:
            return "BLOCK"
        return "STOP"
