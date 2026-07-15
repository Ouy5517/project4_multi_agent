"""GameController state ROS provider.

Wraps parsing, QoS, and staleness concerns for the ``/soccer/game_controller``
topic. Runtime uses it to feed :class:`GameControlState` back into the truth provider.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String as RosString

from .config import SoccerConfig
from .types import GameControlState
from .game_state import game_control_state_from_json


__all__ = ["GameControllerRosProvider", "GameControllerRosBridge"]


class GameControllerRosProvider:
    """ROS topic provider for GameController state."""

    def __init__(
        self,
        node: Node,
        config: SoccerConfig,
        logger: Any,
        on_state: Callable[[GameControlState], None],
    ):
        self._node = node
        self._config = config
        self._logger = logger
        self._on_state = on_state
        self._subscriptions: list[Any] = []
        self._last_state_log_key: tuple[object, ...] | None = None
        self.last_topic_at = 0.0

    def start(self) -> None:
        if self._subscriptions:
            return

        qos = self._qos(depth=10)
        self._subscriptions.append(
            self._node.create_subscription(
                RosString,
                self._config.game_controller_topic,
                self._topic_callback,
                qos,
            )
        )
        self._log_subscription()

    def stop(self) -> None:
        for subscription in self._subscriptions:
            try:
                self._node.destroy_subscription(subscription)
            except Exception as exc:
                self._logger.warn(
                    f"destroy GameController subscription failed: {exc}",
                    event="game_controller_subscription_destroy_failed",
                    team_id=self._config.team_id,
                    topic=self._config.game_controller_topic,
                    error=str(exc),
                )
        self._subscriptions.clear()

    def _log_subscription(self) -> None:
        self._logger.info(
            f"Subscribed GameController state topic: "
            f"{self._config.game_controller_topic}",
            event="game_controller_topic_subscribed",
            team_id=self._config.team_id,
            topic=self._config.game_controller_topic,
        )

    def _topic_callback(self, msg: Any) -> None:
        try:
            game_state = game_control_state_from_json(str(msg.data))
        except ValueError as exc:
            self._logger.warn(
                f"Ignore invalid GameController topic payload: {exc}",
                event="game_controller_invalid_topic_payload",
                team_id=self._config.team_id,
                topic=self._config.game_controller_topic,
                error=str(exc),
            )
            return
        self.last_topic_at = time.monotonic()
        game_state.last_seen_at = self.last_topic_at
        self._log_state_change(game_state)
        self._on_state(game_state)

    def _log_state_change(self, game_state: GameControlState) -> None:
        key = (
            game_state.state.value,
            game_state.game_phase.value,
            game_state.set_play.value,
            game_state.stopped,
            game_state.kicking_team,
            game_state.secs_remaining,
            game_state.secondary_time,
        )
        if key == self._last_state_log_key:
            return
        self._last_state_log_key = key
        self._logger.info(
            "GameController state changed",
            event="game_controller_state_changed",
            console=False,
            team_id=self._config.team_id,
            packet_number=game_state.packet_number,
            state=game_state.state.value,
            game_phase=game_state.game_phase.value,
            set_play=game_state.set_play.value,
            stopped=game_state.stopped,
            kicking_team=game_state.kicking_team,
            secs_remaining=game_state.secs_remaining,
            secondary_time=game_state.secondary_time,
        )

    def _qos(self, depth: int) -> Any:
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )


# Backward-compatible name for existing imports outside this package.
GameControllerRosBridge = GameControllerRosProvider
