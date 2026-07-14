from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Point":
        return cls(x=float(data["x"]), y=float(data["y"]))

    def distance_to(self, other: "Point") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class Ball(Point):
    vx: float = 0.0
    vy: float = 0.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Ball":
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
        )


@dataclass(frozen=True)
class RobotState:
    robot_id: str
    team: str
    x: float
    y: float
    theta: float
    role: str
    has_ball: bool = False
    vx: float = 0.0
    vy: float = 0.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotState":
        return cls(
            robot_id=str(data["robot_id"]),
            team=str(data["team"]),
            x=float(data["x"]),
            y=float(data["y"]),
            theta=float(data.get("theta", 0.0)),
            role=str(data.get("role", "support")),
            has_ball=bool(data.get("has_ball", False)),
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
        )

    @property
    def point(self) -> Point:
        return Point(self.x, self.y)


@dataclass(frozen=True)
class OpponentState:
    opponent_id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "OpponentState":
        return cls(
            opponent_id=str(data["opponent_id"]),
            x=float(data["x"]),
            y=float(data["y"]),
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
        )

    @property
    def point(self) -> Point:
        return Point(self.x, self.y)


@dataclass(frozen=True)
class WorldState:
    timestamp: float
    ball: Ball
    robots: list[RobotState]
    opponents: list[OpponentState]
    our_goal: Point
    enemy_goal: Point
    field_width: float
    field_height: float
    scenario_name: str = "unknown"
    next_states: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorldState":
        return cls(
            timestamp=float(data.get("timestamp", 0.0)),
            ball=Ball.from_dict(data["ball"]),
            robots=[RobotState.from_dict(item) for item in data.get("robots", [])],
            opponents=[
                OpponentState.from_dict(item) for item in data.get("opponents", [])
            ],
            our_goal=Point.from_dict(data["our_goal"]),
            enemy_goal=Point.from_dict(data["enemy_goal"]),
            field_width=float(data["field_width"]),
            field_height=float(data["field_height"]),
            scenario_name=str(data.get("scenario_name", "unknown")),
            next_states=list(data.get("next_states", [])),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def ball_carrier(self) -> RobotState | None:
        return next((robot for robot in self.robots if robot.has_ball), None)

    def nearest_robot_to_ball(self) -> RobotState | None:
        if not self.robots:
            return None
        ball_point = Point(self.ball.x, self.ball.y)
        return min(self.robots, key=lambda robot: robot.point.distance_to(ball_point))

    def teammates_except(self, robot_id: str) -> list[RobotState]:
        return [robot for robot in self.robots if robot.robot_id != robot_id]

    def nearest_opponent_to_our_goal(self) -> OpponentState | None:
        if not self.opponents:
            return None
        return min(
            self.opponents,
            key=lambda opponent: opponent.point.distance_to(self.our_goal),
        )

    def summary(self) -> str:
        carrier = self.ball_carrier()
        carrier_text = carrier.robot_id if carrier else "无"
        return (
            f"球=({self.ball.x:.2f}, {self.ball.y:.2f}), "
            f"持球={carrier_text}, 我方机器人={len(self.robots)}, "
            f"对手={len(self.opponents)}, 敌方球门=({self.enemy_goal.x:.2f}, "
            f"{self.enemy_goal.y:.2f})"
        )


def load_world_state(path: Path) -> WorldState:
    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)
    data.setdefault("scenario_name", path.stem)
    return WorldState.from_dict(data)
