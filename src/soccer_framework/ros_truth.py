"""ROS ground-truth adapter that subscribes to simulator topics for the team.

Encapsulates ROS topic names, QoS options, and callback thread safety. Strategy
code only reads a ``player_id``-keyed context snapshot through :class:`PlayContextProvider`.
"""

from __future__ import annotations

import copy
import threading
import time
from typing import Any

from geometry_msgs.msg import Pose2D as RosPose2D
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from .config import SoccerConfig
from .types import (
    BallState,
    GameControlState,
    Pose2D,
    RobotState,
    PlayContext,
    PlayContextProvider,
)


__all__ = ["RosTruthProvider"]


class RosTruthProvider(PlayContextProvider):
    """V1 simulator truth adapter.

    Topic names are isolated here. Strategy and robot control code only consume
    PlayContext keyed by player_id.
    Topic names are isolated here; strategy and robot-control code only consume
    ``PlayContext`` keyed by player_id.
    """

    def __init__(
        self,
        node: Node,
        config: SoccerConfig,
        logger: Any,
    ):
        self._node = node
        self._config = config
        self._logger = logger
        self._lock = threading.RLock()
        self._subscriptions: list[Any] = []
        self._game_state: GameControlState | None = None
        self._robots: dict[int, RobotState] = {
            player_id: RobotState(player_id=player_id)
            for player_id in config.player_ids
        }
        self._opponents: dict[int, RobotState] = {
            player_id: RobotState(player_id=player_id)
            for player_id in range(1, len(config.opponent_robot_names) + 1)
        }
        self._ball: BallState | None = None

    def start(self) -> None:
        if self._subscriptions:
            return
        truth_qos = self._qos(depth=1)
        pose_topics: list[tuple[int, str, str, str, str]] = []
        for player_id, robot_name in enumerate(self._config.robot_names, start=1):
            pose_topic = self._topic_for_robot(
                robot_name,
                "soccer/sim/ground_truth/robot_pose",
            )
            self._subscriptions.append(
                self._node.create_subscription(
                    RosPose2D,
                    pose_topic,
                    self._make_pose_callback(player_id),
                    truth_qos,
                )
            )
            pose_topics.append(
                (
                    player_id,
                    robot_name or "<default>",
                    self._config.display_name_for_player(player_id),
                    self._config.ready_slot_for_player(player_id).value,
                    pose_topic,
                )
            )

        opponent_pose_topics: list[tuple[int, str, str, str]] = []
        for player_id, robot_name in enumerate(
            self._config.opponent_robot_names,
            start=1,
        ):
            pose_topic = self._topic_for_robot(
                robot_name,
                "soccer/sim/ground_truth/robot_pose",
            )
            self._subscriptions.append(
                self._node.create_subscription(
                    RosPose2D,
                    pose_topic,
                    self._make_opponent_pose_callback(player_id),
                    truth_qos,
                )
            )
            opponent_pose_topics.append(
                (
                    player_id,
                    robot_name,
                    self._config.opponent_display_name_for_player(player_id),
                    pose_topic,
                )
            )

        ball_topic = self._topic_for_team("soccer/sim/ground_truth/ball")
        self._subscriptions.append(
            self._node.create_subscription(
                RosPose2D,
                ball_topic,
                self._ball_callback,
                truth_qos,
            )
        )
        self._log_truth_subscriptions(pose_topics, opponent_pose_topics, ball_topic)

    def _log_truth_subscriptions(
        self,
        pose_topics: list[tuple[int, str, str, str, str]],
        opponent_pose_topics: list[tuple[int, str, str, str]],
        ball_topic: str,
    ) -> None:
        topic_prefix = f"/team{self._config.team_id}"
        members = ", ".join(
            f"p{player_id}:{display_name}({robot_name})/{ready_slot}"
            for player_id, robot_name, display_name, ready_slot, _topic in pose_topics
        )
        self._logger.info(
            f"Truth provider team_id={self._config.team_id} "
            f"topic_prefix={topic_prefix} members=[{members}]",
            event="truth_provider_started",
            team_id=self._config.team_id,
            topic_prefix=topic_prefix,
            members=[
                {
                    "player_id": player_id,
                    "robot_name": robot_name,
                    "display_name": display_name,
                    "ready_slot": ready_slot,
                    "pose_topic": pose_topic,
                }
                for player_id, robot_name, display_name, ready_slot, pose_topic in pose_topics
            ],
            ball_topic=ball_topic,
        )
        for player_id, robot_name, display_name, ready_slot, pose_topic in pose_topics:
            self._logger.info(
                f"Truth subscription p{player_id} robot={display_name}({robot_name}) "
                f"ready_slot={ready_slot} pose_topic={pose_topic}",
                event="truth_subscription",
                team_id=self._config.team_id,
                player_id=player_id,
                robot_name=robot_name,
                display_name=display_name,
                ready_slot=ready_slot,
                pose_topic=pose_topic,
            )
        for player_id, robot_name, display_name, pose_topic in opponent_pose_topics:
            self._logger.info(
                f"Opponent truth subscription p{player_id} robot={display_name}({robot_name}) "
                f"pose_topic={pose_topic}",
                event="truth_opponent_subscription",
                team_id=self._config.team_id,
                player_id=player_id,
                robot_name=robot_name,
                display_name=display_name,
                pose_topic=pose_topic,
            )
        self._logger.info(
            f"Truth subscription team_id={self._config.team_id} "
            f"ball_topic={ball_topic}",
            event="truth_ball_subscription",
            team_id=self._config.team_id,
            ball_topic=ball_topic,
        )

    def _topic_for_robot(self, robot_name: str, suffix: str) -> str:
        if robot_name:
            return self._join_topic(f"team{self._config.team_id}", robot_name, suffix)
        return self._topic_for_team(suffix)

    def _topic_for_team(self, suffix: str) -> str:
        return self._join_topic(f"team{self._config.team_id}", suffix)

    def _join_topic(self, *parts: str) -> str:
        clean_parts = [part.strip("/") for part in parts if part.strip("/")]
        return "/" + "/".join(clean_parts)

    def stop(self) -> None:
        for subscription in self._subscriptions:
            try:
                self._node.destroy_subscription(subscription)
            except Exception as exc:
                self._logger.warn(
                    f"destroy subscription failed: {exc}",
                    event="truth_subscription_destroy_failed",
                    team_id=self._config.team_id,
                    error=str(exc),
                )
        self._subscriptions.clear()

    def set_game_state(self, game_state: GameControlState) -> None:
        with self._lock:
            self._game_state = game_state

    def get_snapshot(self) -> PlayContext:
        with self._lock:
            return PlayContext(
                game_state=copy.deepcopy(self._game_state),
                teammates=copy.deepcopy(self._robots),
                opponents=copy.deepcopy(self._opponents),
                ball=copy.deepcopy(self._ball),
            )

    def _qos(self, depth: int) -> Any:
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

    def _make_pose_callback(self, player_id: int):
        def callback(msg: Any) -> None:
            pose = Pose2D(x=float(msg.x), y=float(msg.y), theta=float(msg.theta))
            with self._lock:
                robot = self._robots.setdefault(
                    player_id,
                    RobotState(player_id=player_id),
                )
                robot.pose = pose
                robot.last_seen_at = time.monotonic()

        return callback

    def _make_opponent_pose_callback(self, player_id: int):
        def callback(msg: Any) -> None:
            pose = Pose2D(x=float(msg.x), y=float(msg.y), theta=float(msg.theta))
            with self._lock:
                robot = self._opponents.setdefault(
                    player_id,
                    RobotState(player_id=player_id),
                )
                robot.pose = pose
                robot.last_seen_at = time.monotonic()

        return callback

    def _ball_callback(self, msg: Any) -> None:
        now = time.monotonic()
        ball = BallState(
            x=float(msg.x),
            y=float(msg.y),
            last_seen_at=now,
            confidence=1.0,
        )
        with self._lock:
            self._ball = ball
