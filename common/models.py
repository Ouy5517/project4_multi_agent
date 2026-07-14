from __future__ import annotations

from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def distance_to(self, other: "Vec2") -> float:
        return hypot(other.x - self.x, other.y - self.y)

    def move_towards(self, target: "Vec2", max_distance: float) -> "Vec2":
        distance = self.distance_to(target)
        if distance == 0 or distance <= max_distance:
            return target
        ratio = max_distance / distance
        return Vec2(
            self.x + (target.x - self.x) * ratio,
            self.y + (target.y - self.y) * ratio,
        )


@dataclass
class RobotState:
    robot_id: str
    position: Vec2
    role: str
    has_ball: bool = False


@dataclass
class BallState:
    position: Vec2
    owner_id: str | None = None


@dataclass
class WorldState:
    time_s: float
    field_width: float
    field_height: float
    robots: dict[str, RobotState]
    ball: BallState

    def is_inside_field(self, point: Vec2) -> bool:
        return 0.0 <= point.x <= self.field_width and 0.0 <= point.y <= self.field_height

