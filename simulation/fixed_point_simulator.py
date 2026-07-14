from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path

from common.models import BallState, RobotState, Vec2, WorldState
from communication.mock_team_bus import MockTeamBus
from decision.pass_fsm import FixedPointPassFSM, PassConfig, PassState


@dataclass(frozen=True)
class Event:
    time_s: float
    actor: str
    action: str
    result: str
    detail: str


@dataclass(frozen=True)
class SimulationResult:
    success: bool
    final_state: str
    elapsed_s: float
    ball_owner_id: str | None
    receiver_position: Vec2
    events: tuple[Event, ...]
    message_count: int


class FixedPointSimulator:
    def __init__(self, config: PassConfig | None = None) -> None:
        self.config = config or PassConfig()
        self.events: list[Event] = []
        self.bus = MockTeamBus()
        self.world = WorldState(
            time_s=0.0,
            field_width=12.0,
            field_height=8.0,
            robots={
                self.config.passer_id: RobotState(
                    robot_id=self.config.passer_id,
                    position=Vec2(2.0, 4.0),
                    role="passer",
                    has_ball=True,
                ),
                self.config.receiver_id: RobotState(
                    robot_id=self.config.receiver_id,
                    position=Vec2(5.0, 1.5),
                    role="receiver",
                ),
            },
            ball=BallState(position=Vec2(2.2, 4.0), owner_id=self.config.passer_id),
        )
        self.fsm = FixedPointPassFSM(self.config, self.bus, self.record)

    def record(
        self, time_s: float, actor: str, action: str, result: str, detail: str
    ) -> None:
        self.events.append(
            Event(round(time_s, 3), actor, action, result, detail)
        )

    def run(self, duration_s: float = 15.0, dt: float = 0.1) -> SimulationResult:
        if duration_s <= 0 or dt <= 0:
            raise ValueError("duration_s and dt must be positive")

        while self.world.time_s <= duration_s:
            self.fsm.update(self.world, dt)
            if self.fsm.state in (PassState.DONE, PassState.FAILED):
                break
            self.world.time_s = round(self.world.time_s + dt, 10)

        if self.fsm.state not in (PassState.DONE, PassState.FAILED):
            self.fsm._transition(self.world, PassState.FAILED, "run duration exhausted")

        receiver = self.world.robots[self.config.receiver_id]
        return SimulationResult(
            success=self.fsm.state == PassState.DONE,
            final_state=self.fsm.state.value,
            elapsed_s=self.world.time_s,
            ball_owner_id=self.world.ball.owner_id,
            receiver_position=receiver.position,
            events=tuple(self.events),
            message_count=len(self.bus.history),
        )


def export_events_csv(events: tuple[Event, ...], output_path: str | Path) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(
            file, fieldnames=["time_s", "actor", "action", "result", "detail"]
        )
        writer.writeheader()
        writer.writerows(asdict(event) for event in events)
    return path

