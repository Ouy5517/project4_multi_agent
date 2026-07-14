from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable

from common.models import Vec2, WorldState
from communication.mock_team_bus import MessageType, MockTeamBus, TeamMessage


class PassState(str, Enum):
    INIT = "INIT"
    WAIT_RECEIVER = "WAIT_RECEIVER"
    PASSING = "PASSING"
    DONE = "DONE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class PassConfig:
    passer_id: str = "R1"
    receiver_id: str = "R2"
    fixed_target: Vec2 = Vec2(7.0, 4.0)
    receiver_speed_mps: float = 1.5
    ball_speed_mps: float = 4.0
    arrival_tolerance_m: float = 0.12
    timeout_s: float = 15.0


EventRecorder = Callable[[float, str, str, str, str], None]


class FixedPointPassFSM:
    def __init__(
        self,
        config: PassConfig,
        bus: MockTeamBus,
        record: EventRecorder,
    ) -> None:
        self.config = config
        self.bus = bus
        self.record = record
        self.state = PassState.INIT
        self.receiver_target: Vec2 | None = None
        self.ready_sent = False

    def _transition(self, world: WorldState, new_state: PassState, reason: str) -> None:
        old_state = self.state
        self.state = new_state
        self.record(
            world.time_s,
            "FSM",
            "STATE_TRANSITION",
            new_state.value,
            f"{old_state.value} -> {new_state.value}; {reason}",
        )

    def update(self, world: WorldState, dt: float) -> None:
        if self.state in (PassState.DONE, PassState.FAILED):
            return

        if world.time_s > self.config.timeout_s:
            self._transition(world, PassState.FAILED, "scenario timeout")
            return

        if self.state == PassState.INIT:
            if not world.is_inside_field(self.config.fixed_target):
                self._transition(world, PassState.FAILED, "fixed target outside field")
                return

            message = TeamMessage(
                message_type=MessageType.PASS_TARGET,
                sender_id=self.config.passer_id,
                receiver_id=self.config.receiver_id,
                created_at_s=world.time_s,
                target=self.config.fixed_target,
            )
            self.bus.publish(message)
            self.record(
                world.time_s,
                self.config.passer_id,
                "SEND_MESSAGE",
                MessageType.PASS_TARGET.value,
                f"target=({message.target.x:.2f}, {message.target.y:.2f})",
            )
            self._transition(world, PassState.WAIT_RECEIVER, "pass target published")
            return

        if self.state == PassState.WAIT_RECEIVER:
            self._update_receiver(world, dt)
            ready = self.bus.consume(
                self.config.passer_id, MessageType.RECEIVER_READY
            )
            if ready is not None:
                passer = world.robots[self.config.passer_id]
                passer.has_ball = False
                world.ball.owner_id = None
                self.record(
                    world.time_s,
                    self.config.passer_id,
                    "RECEIVE_MESSAGE",
                    MessageType.RECEIVER_READY.value,
                    "receiver is at fixed target; kick starts",
                )
                self._transition(world, PassState.PASSING, "receiver ready")
            return

        if self.state == PassState.PASSING:
            target = self.config.fixed_target
            world.ball.position = world.ball.position.move_towards(
                target, self.config.ball_speed_mps * dt
            )
            if world.ball.position.distance_to(target) <= self.config.arrival_tolerance_m:
                receiver = world.robots[self.config.receiver_id]
                world.ball.position = target
                world.ball.owner_id = receiver.robot_id
                receiver.has_ball = True
                self.record(
                    world.time_s,
                    self.config.receiver_id,
                    "RECEIVE_BALL",
                    "SUCCESS",
                    f"ball received at ({target.x:.2f}, {target.y:.2f})",
                )
                self._transition(world, PassState.DONE, "fixed-point pass completed")

    def _update_receiver(self, world: WorldState, dt: float) -> None:
        receiver = world.robots[self.config.receiver_id]

        if self.receiver_target is None:
            target_message = self.bus.consume(
                receiver.robot_id, MessageType.PASS_TARGET
            )
            if target_message is None or target_message.target is None:
                return
            self.receiver_target = target_message.target
            self.record(
                world.time_s,
                receiver.robot_id,
                "RECEIVE_MESSAGE",
                MessageType.PASS_TARGET.value,
                f"move_to=({self.receiver_target.x:.2f}, {self.receiver_target.y:.2f})",
            )

        receiver.position = receiver.position.move_towards(
            self.receiver_target, self.config.receiver_speed_mps * dt
        )

        arrived = (
            receiver.position.distance_to(self.receiver_target)
            <= self.config.arrival_tolerance_m
        )
        if arrived and not self.ready_sent:
            receiver.position = self.receiver_target
            ready_message = TeamMessage(
                message_type=MessageType.RECEIVER_READY,
                sender_id=receiver.robot_id,
                receiver_id=self.config.passer_id,
                created_at_s=world.time_s,
                target=self.receiver_target,
            )
            self.bus.publish(ready_message)
            self.ready_sent = True
            self.record(
                world.time_s,
                receiver.robot_id,
                "SEND_MESSAGE",
                MessageType.RECEIVER_READY.value,
                "receiver reached fixed target",
            )

