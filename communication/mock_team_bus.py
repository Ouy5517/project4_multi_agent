from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from common.models import Vec2


class MessageType(str, Enum):
    PASS_TARGET = "PASS_TARGET"
    RECEIVER_READY = "RECEIVER_READY"


@dataclass(frozen=True)
class TeamMessage:
    message_type: MessageType
    sender_id: str
    receiver_id: str
    created_at_s: float
    target: Vec2 | None = None


class MockTeamBus:
    """An in-memory FIFO queue used to simulate team communication.

    It deliberately contains no socket, DDS, ROS 2, or robot SDK dependency.
    The interface can later be replaced by a real publisher/subscriber adapter.
    """

    def __init__(self) -> None:
        self._pending: list[TeamMessage] = []
        self.history: list[TeamMessage] = []

    def publish(self, message: TeamMessage) -> None:
        self._pending.append(message)
        self.history.append(message)

    def consume(
        self, receiver_id: str, message_type: MessageType
    ) -> TeamMessage | None:
        for index, message in enumerate(self._pending):
            if (
                message.receiver_id == receiver_id
                and message.message_type == message_type
            ):
                return self._pending.pop(index)
        return None

