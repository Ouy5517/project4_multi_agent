"""Top-level TeamStrategyTree assembly with one-way dependency from tree to kit and playbook.

Overall shape:

Sequence("TeamRoot")
|-- DataLayer
|-- MatchControl
|   |-- SafetyGuards
|   |-- ReadyPhase
|   |-- PlayingPhase
|   `-- StopAll("unsupported state")
|-- SafetyOverrides
`-- CommitTeamCommands

DataLayer is a Sequence because nodes depend on earlier writes. MatchControl is a
Selector because only the first successful state branch should run. SafetyOverrides
runs after MatchControl so it can inspect this tick's ``/cmd/{player_id}`` slots,
switching mode only for commands that need walk and independently overriding players
that are disallowed or fallen.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import time
from typing import TYPE_CHECKING, Any, Protocol

import py_trees

from ..soccer_framework import (
    BallState,
    GameControlState,
    KickIntent,
    MoveIntent,
    NoopIntent,
    RobotCommand,
    RobotRuntimeStatus,
    PlayContext,
    PlayContextProvider,
)
from .blackboard import (
    BlackboardKeys,
    BlackboardClient,
    cmd_key,
    robot_status_key,
)
from .nodes.actions import CommitTeamCommands, StopAll
from .nodes.data import (
    UpdateClock,
    UpdateGameState,
    UpdateRecentBall,
    UpdateRobotPoses,
    UpdateRobotStatus,
    UpdatePlayContext,
)
from .ready_subtree import create_ready_subtree
from .safety_subtree import (
    create_safety_overrides_subtree,
    create_safety_subtree,
)

if TYPE_CHECKING:
    from ..play.playbook import Playbook
    from ..runtime import RobotServices, SoccerKit


_TRACE_OFF_VALUES = {"", "0", "false", "off", "none", "disabled", "no"}
_TRACE_ON_VALUES = {"1", "true", "on", "yes", "full"}


class TeamCommandExecutor(Protocol):
    """Runtime-side command executor protocol."""

    def execute_team_commands(
        self,
        commands: dict[int, RobotCommand],
    ) -> dict[int, RobotCommand]:
        """Execute or record a complete team-command set."""
        ...


def create_team_tree(
    kit: "SoccerKit",
    playbook: "Playbook",
    get_now: Callable[[], float],
    context_provider: PlayContextProvider,
) -> tuple[py_trees.behaviour.Behaviour, CommitTeamCommands]:
    """Build the complete team behavior tree and return (root, commit_node)."""

    # Delayed import because play.subtree -> play.role -> kit.blackboard
    # can otherwise loop back to kit while team/__init__.py is loading.
    from ..play.play_subtree import create_play_subtree

    data_layer = py_trees.composites.Sequence(
        name="DataLayer",
        memory=False,
        children=[
            UpdateClock(get_now),
            UpdatePlayContext(context_provider),
            UpdateGameState(),
            UpdateRecentBall(),
            UpdateRobotPoses(),
            *[
                UpdateRobotStatus(kit, player_id)
                for player_id in kit.config.player_ids
            ],
        ],
    )
    match_control = py_trees.composites.Selector(
        name="MatchControl",
        memory=False,
        children=[
            create_safety_subtree(kit),
            create_ready_subtree(kit),
            create_play_subtree(kit, playbook),
            StopAll(kit, "unsupported state"),
        ],
    )
    safety_overrides = create_safety_overrides_subtree(kit)
    committer = CommitTeamCommands(kit)
    root = py_trees.composites.Sequence(
        name="TeamRoot",
        memory=False,
        children=[data_layer, match_control, safety_overrides, committer],
    )
    return root, committer


class TeamStrategyTree:
    """Template top-level assembly that wires :class:`SoccerKit`, :class:`Playbook`, and the BT.

    Each :meth:`tick` receives a timestamp, defaulting to :func:`time.monotonic`,
    and the tree pulls a context snapshot from the provider. The last snapshot is cached
    in :attr:`last_context` for runtime logs and traces without repeated snapshots.

    .. code-block:: python

    from src.behavior_tree import TeamStrategyTree
    from src.play import PLAYBOOKS
    from src.runtime import SoccerKit

    kit = SoccerKit(config)
    tree = TeamStrategyTree(kit, PLAYBOOKS.create_default(kit), context_provider)
    """

    def __init__(
        self,
        kit: "SoccerKit",
        playbook: "Playbook",
        context_provider: PlayContextProvider,
        logger: Any | None = None,
    ):
        self.kit = kit
        self.playbook = playbook
        self.context_provider = context_provider
        self._logger = logger
        self._now: float = 0.0
        self._tick_id = 0
        self._last_trace_at = 0.0
        self._trace_mode = _trace_mode_from_config(kit.config.debug.bt_trace_ticks)
        self._trace_sample_sec = _trace_sample_sec_from_config(
            kit.config.debug.bt_trace_sample_sec
        )
        self.root, self._committer = create_team_tree(
            kit,
            playbook,
            self._current_now,
            context_provider,
        )
        self._tree = py_trees.trees.BehaviourTree(self.root)
        # The root-level client writes the EXECUTOR slot once and refreshes it each tick.
        self._tick_blackboard = BlackboardClient(name="TeamStrategyTree.tick")
        self._context_reader = BlackboardClient(name="TeamStrategyTree.last_context")

    def bind_robot_services(self, services: "RobotServices") -> None:
        """Forward to :meth:`SoccerKit.bind_robot_services` for nearby test and caller access."""

        self.kit.bind_robot_services(services)

    def tick(
        self,
        now: float | None = None,
        executor: TeamCommandExecutor | None = None,
    ) -> None:
        if now is None:
            now = time.monotonic()
        self._now = now
        self._tick_id += 1
        self._tick_blackboard.write(BlackboardKeys.EXECUTOR, executor)
        if not self._should_trace(now):
            self.root.tick_once()
            return

        visitor = _TickTraceVisitor()
        self._tree.visitors.append(visitor)
        started_at = time.perf_counter()
        try:
            self._tree.tick()
        finally:
            try:
                self._tree.visitors.remove(visitor)
            except ValueError:
                pass
        duration_ms = round((time.perf_counter() - started_at) * 1000.0, 3)
        self._last_trace_at = now
        self._log_tick_trace(visitor, duration_ms)

    @property
    def last_context(self) -> PlayContext:
        """Snapshot written by this tick's ``UpdatePlayContext``; empty before the first ``tick``."""

        context = self._context_reader.read(BlackboardKeys.PLAY_CONTEXT)
        return context if isinstance(context, PlayContext) else PlayContext()

    @property
    def last_commands(self) -> dict[int, RobotCommand]:
        return dict(self._committer.last_committed)

    @property
    def last_executed_commands(self) -> dict[int, RobotCommand]:
        return dict(self._committer.last_executed)

    def ascii_tree(self, show_status: bool = False) -> str:
        return py_trees.display.ascii_tree(self.root, show_status=show_status)

    def _current_now(self) -> float:
        return self._now

    def _should_trace(self, now: float) -> bool:
        if self._logger is None:
            return False
        if self._trace_mode == "off":
            return False
        if self._trace_mode == "sampled":
            return now - self._last_trace_at >= self._trace_sample_sec
        return True

    def _log_tick_trace(
        self,
        visitor: "_TickTraceVisitor",
        duration_ms: float,
    ) -> None:
        info = getattr(self._logger, "info", None)
        if not callable(info):
            return
        root_status = _status_name(self.root.status)
        info(
            "Soccer BT tick trace",
            event="bt_tick",
            console=False,
            tick_id=self._tick_id,
            now=round(self._now, 6),
            duration_ms=duration_ms,
            root_status=root_status,
            nodes=visitor.nodes,
            blackboard=_blackboard_snapshot(self.kit, self.last_context, self._now),
            commands_committed={
                str(player_id): _command_record(command)
                for player_id, command in sorted(self.last_commands.items())
            },
            commands_executed={
                str(player_id): _command_record(command)
                for player_id, command in sorted(self.last_executed_commands.items())
            },
        )


class _TickTraceVisitor(py_trees.visitors.VisitorBase):
    """Collect visited node statuses for one tick."""

    def __init__(self) -> None:
        super().__init__(full=False)
        self.nodes: list[dict[str, object]] = []

    def initialise(self) -> None:
        self.nodes = []

    def run(self, behaviour: py_trees.behaviour.Behaviour) -> None:
        self.nodes.append(
            {
                "name": behaviour.name,
                "type": type(behaviour).__name__,
                "status": _status_name(behaviour.status),
                "path": _node_path(behaviour),
                "depth": _node_depth(behaviour),
            }
        )


def _trace_mode_from_config(value: str) -> str:
    value = value.strip().lower()
    if value in _TRACE_OFF_VALUES:
        return "off"
    if value in _TRACE_ON_VALUES:
        return "on"
    if value == "sampled":
        return "sampled"
    return "off"


def _trace_sample_sec_from_config(value: float) -> float:
    return max(0.0, value)


def _node_path(behaviour: py_trees.behaviour.Behaviour) -> str:
    names = [behaviour.name]
    parent = behaviour.parent
    while parent is not None:
        names.append(parent.name)
        parent = parent.parent
    return "/".join(reversed(names))


def _node_depth(behaviour: py_trees.behaviour.Behaviour) -> int:
    depth = 0
    parent = behaviour.parent
    while parent is not None:
        depth += 1
        parent = parent.parent
    return depth


def _status_name(status: object) -> str:
    if isinstance(status, py_trees.common.Status):
        return status.name
    return str(status)


def _blackboard_snapshot(
    kit: "SoccerKit",
    context: PlayContext,
    now: float,
) -> dict[str, object]:
    game = context.game_state
    snapshot: dict[str, object] = {
        BlackboardKeys.NOW: round(now, 6),
        "/game/state": _game_record(game),
        "/ball/state": _ball_record(context.ball, now),
        BlackboardKeys.ROLES: _roles_record(
            py_trees.blackboard.Blackboard.storage.get(BlackboardKeys.ROLES)
        ),
        BlackboardKeys.SAFETY_ACTIVE: bool(
            py_trees.blackboard.Blackboard.storage.get(BlackboardKeys.SAFETY_ACTIVE)
        ),
    }
    for player_id in kit.config.player_ids:
        status = py_trees.blackboard.Blackboard.storage.get(
            robot_status_key(player_id)
        )
        if isinstance(status, RobotRuntimeStatus):
            snapshot[robot_status_key(player_id)] = _robot_status_record(status, now)
        command = py_trees.blackboard.Blackboard.storage.get(cmd_key(player_id))
        if isinstance(command, RobotCommand):
            snapshot[cmd_key(player_id)] = _command_record(command)
    return snapshot


def _game_record(game: GameControlState | None) -> dict[str, object] | None:
    if game is None:
        return None
    teams: list[dict[str, object]] = []
    for team in game.teams:
        teams.append(
            {
                "team_number": team.team_number,
                "score": team.score,
                "players": [
                    {
                        "player_id": player_id,
                        "penalty": player.penalty.value,
                        "secs_till_unpenalised": player.secs_till_unpenalised,
                        "warnings": player.warnings,
                        "cautions": player.cautions,
                    }
                    for player_id, player in enumerate(team.players, start=1)
                    if player.penalty.value != "NONE"
                    or player.secs_till_unpenalised > 0
                    or player.warnings > 0
                    or player.cautions > 0
                ],
            }
        )
    return {
        "packet_number": game.packet_number,
        "state": game.state.value,
        "game_phase": game.game_phase.value,
        "set_play": game.set_play.value,
        "stopped": game.stopped,
        "kicking_team": game.kicking_team,
        "secs_remaining": game.secs_remaining,
        "secondary_time": game.secondary_time,
        "teams": teams,
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


def _roles_record(value: object) -> dict[str, object] | None:
    by_player = getattr(value, "by_player", None)
    if isinstance(by_player, dict):
        return {str(player_id): str(role) for player_id, role in by_player.items()}
    if isinstance(by_player, Mapping):
        return {str(player_id): str(role) for player_id, role in by_player.items()}
    return None


def _robot_status_record(
    status: RobotRuntimeStatus,
    now: float,
) -> dict[str, object]:
    return {
        "mode": status.mode,
        "fall_down_state": status.fall_down_state,
        "fall_down_recoverable": status.fall_down_recoverable,
        "age_sec": (
            round(max(0.0, now - status.updated_at), 3)
            if status.updated_at > 0.0
            else None
        ),
    }


def _command_record(command: RobotCommand) -> dict[str, object]:
    intent = command.intent
    record: dict[str, object] = {"reason": command.reason}
    if isinstance(intent, MoveIntent):
        record.update(
            {
                "intent": "move",
                "vx": round(intent.vx, 3),
                "vy": round(intent.vy, 3),
                "vyaw": round(intent.vyaw, 3),
            }
        )
    elif isinstance(intent, KickIntent):
        record.update(
            {
                "intent": "kick",
                "direction": round(intent.direction, 3),
                "power": round(intent.power, 3),
                "ball_x": round(intent.ball_x, 3),
                "ball_y": round(intent.ball_y, 3),
            }
        )
    elif isinstance(intent, NoopIntent):
        record["intent"] = "noop"
    else:
        record["intent"] = "stop"
    return record
