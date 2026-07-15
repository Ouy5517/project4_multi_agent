"""Project assembly layer that wires framework adapters, strategy, and BT into the control loop.

This layer is outside ``soccer_framework`` because it builds a runnable soccer
system from framework pieces. It depends on :mod:`soccer_framework` and :mod:`play`,
while the framework does not import it back.

This module holds two kinds of objects, which can be split later by responsibility:

``kit.ready_stance``
:class:`SoccerKit`: team-level toolkit that owns playbook-agnostic tools such
as field frame, rule checks, avoidance, kick hysteresis, READY stance, and
movement/kick output. It only holds tools and binds services; leaves and Playbooks access attributes directly.
:class:`SoccerTeamRuntime`: control loop that connects framework adapters and
the behavior tree, with :class:`TeamStrategyTree` as the top-level strategy facade.

Competitors usually do not need this file. Change :mod:`src.play` for tactics and
the adapters in :mod:`soccer_framework` for hardware backend changes.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Any, Protocol

from .behavior_tree import TeamCommandExecutor, TeamStrategyTree, create_team_tree
from .soccer_framework import (
    BallState,
    GameControlState,
    GameState,
    KickIntent,
    MoveIntent,
    NoopIntent,
    Pose2D,
    RobotCommand,
    RobotRuntimeStatus,
    RobotState,
    SetPlay,
    SoccerConfig,
    PlayContext,
)
from .soccer_framework.ros_adapter import SoccerRosAdapter
from .tactics import (
    KickHysteresis,
    MotionController,
    ObstacleCollector,
    ReadyStance,
    Targeting,
    TeamFieldFrame,
)

if TYPE_CHECKING:
    from .play.playbook import Playbook


__all__ = [
    "RobotServices",
    "SoccerTeamRuntime",
    "SoccerKit",
    "TeamCommandExecutor",
    "TeamStrategyTree",
    "create_team_tree",
]


# ----------------------------------------------------------------------
# Team toolkit
# ----------------------------------------------------------------------


class RobotServices(Protocol):
    """Protocol used by BT leaves to access hardware services.

    Extracts the BT-facing surface of :class:`TeamRobotManager` so strategy code
    does not directly depend on runtime, avoiding cycles and allowing fakes in tests.
    """

    def poll_runtime_status(
        self,
        player_id: int,
        now: float,
    ) -> RobotRuntimeStatus: ...

    def trigger_get_up(self, player_id: int, now: float) -> bool: ...

    def ensure_walk_mode(self, player_id: int, reason: str) -> None: ...


class SoccerKit:
    """Soccer toolkit used by BT leaves and :class:`Playbook` for shared capabilities.

    ``kit.ready_stance`` /
    Owns playbook-agnostic tools: field frame, rule checks, avoidance, kick
    hysteresis, READY positioning, and movement/kick output. It only holds objects and
    binds services; leaves and Playbooks access ``kit.targeting``, ``kit.ready_stance``,
    and ``kit.motion`` directly. Role selection and kick targets belong to :class:`Playbook`.
    """

    def __init__(self, config: SoccerConfig):
        self.config = config
        self.field = TeamFieldFrame(config)
        self.obstacles = ObstacleCollector(config, self.field)
        self.targeting = Targeting(config, self.field, self.obstacles)
        self.ready_stance = ReadyStance(config, self.field)
        self.kicker = KickHysteresis(
            enter=config.strategy.soccer_kick_enter_distance,
            exit=config.strategy.soccer_kick_exit_distance,
            exit_delay=config.strategy.soccer_kick_exit_delay_sec,
        )
        self.motion = MotionController(config, self.field, self.kicker, self.obstacles)
        self._robot_services: RobotServices | None = None

    # Service binding

    def bind_robot_services(self, services: RobotServices) -> None:
        """Bind :class:`RobotServices` for BT leaves; usually called once during runtime startup."""

        self._robot_services = services

    def robot_services(self) -> RobotServices | None:
        """Return currently bound hardware services, or None for tests and dry-run."""

        return self._robot_services

    # Common checks

    def is_player_allowed(
        self,
        game: GameControlState,
        player_id: int,
    ) -> bool:
        """Whether this player is currently allowed to receive commands, not penalized or substituted.

        Same logic as :meth:`GameControlState.is_active_player`, with
        ``self.config.team_id`` bound so it can be passed as a targeting callback.
        """

        return game.is_active_player(self.config.team_id, player_id)

    def is_opponent_restart(self, game: GameControlState) -> bool:
        return (
            game.state == GameState.PLAYING
            and not game.stopped
            and game.set_play != SetPlay.NONE
            and game.has_kicking_team()
            and game.kicking_team != self.config.team_id
        )


# ----------------------------------------------------------------------
# Control loop
# ----------------------------------------------------------------------


def _pose_record(pose: Pose2D | None) -> dict[str, object] | None:
    if pose is None:
        return None
    return {
        "x": round(pose.x, 3),
        "y": round(pose.y, 3),
        "theta": round(pose.theta, 3),
    }


def _ball_record(ball: BallState | None, now: float) -> dict[str, object] | None:
    if ball is None:
        return None
    return {
        "x": round(ball.x, 3),
        "y": round(ball.y, 3),
        "age_sec": round(max(0.0, now - ball.last_seen_at), 3),
        "confidence": round(ball.confidence, 3),
    }


def _command_record(command: RobotCommand) -> dict[str, object]:
    record: dict[str, object] = {"reason": command.reason}
    intent = command.intent
    if isinstance(intent, MoveIntent):
        record["intent"] = "move"
        record["vx"] = round(intent.vx, 3)
        record["vy"] = round(intent.vy, 3)
        record["vyaw"] = round(intent.vyaw, 3)
    elif isinstance(intent, KickIntent):
        record["intent"] = "kick"
        record["kick_direction"] = round(intent.direction, 3)
        record["kick_power"] = round(intent.power, 3)
        record["kick_ball_x"] = round(intent.ball_x, 3)
        record["kick_ball_y"] = round(intent.ball_y, 3)
    elif isinstance(intent, NoopIntent):
        record["intent"] = "noop"
    else:
        record["intent"] = "stop"
    return record


def _robot_record(
    robot: RobotState,
    game: GameControlState | None,
    config: SoccerConfig,
    command: RobotCommand | None,
    now: float,
) -> dict[str, object]:
    player = (
        game.get_player_state(config.team_id, robot.player_id) if game is not None else None
    )
    record: dict[str, object] = {
        "player_id": robot.player_id,
        "display_name": config.display_name_for_player(robot.player_id),
        "ready_slot": config.ready_slot_for_player(robot.player_id).value,
        "penalty": player.penalty.value if player is not None else "NONE",
        "pose": _pose_record(robot.pose),
        "pose_age_sec": (
            round(max(0.0, now - robot.last_seen_at), 3)
            if robot.last_seen_at > 0.0
            else None
        ),
    }
    if command is not None:
        record["command"] = _command_record(command)
    return record


class SoccerTeamRuntime(TeamCommandExecutor):
    """Control loop that connects framework adapters and the behavior tree.

    Entry points such as main.py or ros_debug nodes use only ``start`` and
    ``stop``. ``tree``, ``robot_manager``, and ``ros_adapter`` remain visible
    for focused tests and debugging.
    """

    def __init__(
        self,
        logger: Any | None = None,
        config: SoccerConfig | None = None,
    ):
        if logger is None:
            raise TypeError("SoccerTeamRuntime requires a logger")
        self.config = SoccerConfig.from_env() if config is None else config
        self._logger = logger
        # Create the default Playbook through the play registry; to change tactics,
        # update register(..., default=True) in play/__init__.py or call PLAYBOOKS.create("xxx", ...) here.
        from .play import PLAYBOOKS

        self.kit = SoccerKit(self.config)
        self.playbook: Playbook = PLAYBOOKS.create_default(self.kit)
        from .soccer_framework.robot import TeamRobotManager

        self.robot_manager = TeamRobotManager(
            self.config,
            logger,
        )
        self.kit.bind_robot_services(self.robot_manager)
        self.ros_adapter = SoccerRosAdapter(
            config=self.config,
            logger=logger,
        )
        self.tree = TeamStrategyTree(
            self.kit,
            self.playbook,
            self.ros_adapter.context_provider,
            logger=logger,
        )
        self._control_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_command_log_at = 0.0
        self._started = False

    def start(self) -> None:
        if self._started:
            self._logger.info(
                "SoccerTeamRuntime is already active",
                event="runtime_already_active",
                team_id=self.config.team_id,
            )
            return
        self._started = True
        self._logger.info(
            "SoccerTeamRuntime is starting",
            event="runtime_starting",
            team_id=self.config.team_id,
            control_hz=self.config.control_hz,
        )
        self._log_config()
        self.robot_manager.start()
        self.ros_adapter.start()
        self._stop_event.clear()
        self._control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self._control_thread.start()

    def stop(self) -> None:
        was_started = self._started
        if not was_started:
            self.ros_adapter.stop()
            return
        self._started = False
        self._logger.info(
            "SoccerTeamRuntime is stopping",
            event="runtime_stopping",
            team_id=self.config.team_id,
        )
        self._stop_event.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        self.ros_adapter.stop()
        self.robot_manager.close()
        self._logger.info(
            "SoccerTeamRuntime stopped",
            event="runtime_stopped",
            console=False,
            team_id=self.config.team_id,
        )

    def control_loop(self) -> None:
        period = 1.0 / max(self.config.control_hz, 1.0)
        while not self._stop_event.is_set():
            started_at = time.monotonic()
            try:
                self.tree.tick(now=started_at, executor=self)
                self._log_commands(
                    started_at, self.tree.last_context, self.tree.last_executed_commands,
                )
            except Exception as exc:
                self._logger.warn(
                    f"control loop failed: {exc.__class__.__name__}: {exc}",
                    event="control_loop_failed",
                    team_id=self.config.team_id,
                    error_type=exc.__class__.__name__,
                    error=str(exc),
                )
                self.robot_manager.stop_all("control loop error")

            elapsed = time.monotonic() - started_at
            self._stop_event.wait(max(0.0, period - elapsed))

    def execute_team_commands(
        self,
        commands: dict[int, RobotCommand],
    ) -> dict[int, RobotCommand]:
        for player_id, command in commands.items():
            self.robot_manager.apply_command(player_id, command)
        return commands

    def _log_commands(
        self,
        now: float,
        context: PlayContext,
        commands: dict[int, RobotCommand],
    ) -> None:
        if now - self._last_command_log_at < 2.0:
            return
        self._last_command_log_at = now
        game_state = context.game_state
        state = game_state.state.value if game_state else "NONE"
        reasons = ", ".join(
            f"p{player_id}:{command.reason}" for player_id, command in commands.items()
        )
        self._logger.info(
            f"Soccer control state={state} {reasons}",
            event="control_summary",
            team_id=self.config.team_id,
            state=state,
            game_phase=game_state.game_phase.value if game_state else None,
            set_play=game_state.set_play.value if game_state else None,
            stopped=game_state.stopped if game_state else None,
            kicking_team=game_state.kicking_team if game_state else None,
            ball=_ball_record(context.ball, now),
            players=[
                _robot_record(robot, game_state, self.config, commands.get(player_id), now)
                for player_id, robot in sorted(context.teammates.items())
            ],
        )

    def _log_config(self) -> None:
        mapping = ", ".join(
            f"p{player_id}:{self.config.display_name_for_player(player_id)}"
            f"({robot_name or '<default>'})/"
            f"{self.config.ready_slot_for_player(player_id).value}"
            for player_id, robot_name in enumerate(self.config.robot_names, start=1)
        )
        self._logger.info(
            f"Soccer config team_id={self.config.team_id} "
            f"control_hz={self.config.control_hz} "
            f"gc_topic={self.config.game_controller_topic} robots=[{mapping}]",
        )
        self._logger.info(
            f"Field adultsize length={self.config.field_length} "
            f"width={self.config.field_width} "
            f"circle={self.config.center_circle_radius} "
            f"penalty_area={self.config.penalty_area_length}x"
            f"{self.config.penalty_area_width} "
            f"goal_area={self.config.goal_area_length}x"
            f"{self.config.goal_area_width} "
            f"markers={len(self.config.field_dimensions().markers())}",
        )
        self._logger.info(
            "Soccer runtime config",
            event="runtime_config",
            console=False,
            team_id=self.config.team_id,
            control_hz=self.config.control_hz,
            game_controller_topic=self.config.game_controller_topic,
            robots=[
                {
                    "player_id": player_id,
                    "robot_name": robot_name or "<default>",
                    "display_name": self.config.display_name_for_player(player_id),
                    "ready_slot": self.config.ready_slot_for_player(player_id).value,
                }
                for player_id, robot_name in enumerate(self.config.robot_names, start=1)
            ],
            field={
                "length": self.config.field_length,
                "width": self.config.field_width,
                "center_circle_radius": self.config.center_circle_radius,
                "penalty_area_length": self.config.penalty_area_length,
                "penalty_area_width": self.config.penalty_area_width,
                "goal_area_length": self.config.goal_area_length,
                "goal_area_width": self.config.goal_area_width,
                "marker_count": len(self.config.field_dimensions().markers()),
            },
        )
