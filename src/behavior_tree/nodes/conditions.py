"""Condition leaves that read the blackboard and answer yes/no.

PLAY-stage dynamic-role condition leaves (IsRole) live in :mod:`src.play.nodes`; this module keeps only playbook-agnostic conditions:

IsGameInState / IsGameStopped / IsNonPlayingState / Match state: HasGameState / IsGameInState / IsGameStopped / IsNonPlayingState
Global safety: IsGlobalSafetyActive
IsPlayerAllowed / IsPlayerDisallowed / Player eligibility: IsAllPlayersInactive / IsPlayerAllowed / IsPlayerDisallowed
Ball state: IsBallKnown
IsOpponentKickoffActive / Rules: IsOpponentRestartActive / IsOpponentKickoffActive
Kicking: IsInKickRange
IsNotWalkMode / IsWalkModeRequired / For SafetyOverrides: IsRobotFallen / IsNotWalkMode / IsWalkModeRequired
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ...soccer_framework import (
    BallState,
    GameControlState,
    GamePhase,
    GameState,
    SetPlay,
    KickIntent,
    MoveIntent,
    RobotCommand,
    RobotRuntimeStatus,
    PlayContext,
)
from ..blackboard import BlackboardKeys, BlackboardClient, cmd_key, robot_status_key
from ...tactics import KickHysteresis

if TYPE_CHECKING:
    from ...runtime import SoccerKit


class _ReadOnlyLeaf(py_trees.behaviour.Behaviour):
    """Shared base for condition leaves: read-only blackboard access and never RUNNING."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = BlackboardClient(name=name)

    def _read_context(self) -> PlayContext | None:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        return context if isinstance(context, PlayContext) else None

    def _read_game(self) -> GameControlState | None:
        context = self._read_context()
        return context.game_state if context is not None else None

    def _read_ball(self) -> BallState | None:
        context = self._read_context()
        return context.ball if context is not None else None


# GameController state


class HasGameState(_ReadOnlyLeaf):
    """Whether the referee topic is valid; FAILURE until first connection."""

    def __init__(self):
        super().__init__("HasGameState")

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if self._read_game() is not None
            else py_trees.common.Status.FAILURE
        )


class IsGameInState(_ReadOnlyLeaf):
    """SUCCESS when current GameState equals ``state``; otherwise FAILURE."""

    def __init__(self, state: GameState):
        super().__init__(f"IsGameInState({state.value})")
        self._state = state

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None or game.state != self._state:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsGameStopped(_ReadOnlyLeaf):
    """Mirrors the GameController v19 ``stopped`` flag."""

    def __init__(self):
        super().__init__("IsGameStopped")

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None or not game.stopped:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsNonPlayingState(_ReadOnlyLeaf):
    """INITIAL"""

    def __init__(self):
        super().__init__("IsNonPlayingState")

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None:
            return py_trees.common.Status.FAILURE
        non_playing = game.game_phase == GamePhase.TIMEOUT or game.state in {
            GameState.INITIAL,
            GameState.SET,
            GameState.FINISHED,
        }
        return (
            py_trees.common.Status.SUCCESS
            if non_playing
            else py_trees.common.Status.FAILURE
        )


class IsGlobalSafetyActive(_ReadOnlyLeaf):
    """Global SafetyGuards already wrote a team stop, so this tick keeps per-player reasons untouched."""

    def __init__(self):
        super().__init__("IsGlobalSafetyActive")

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if bool(self.blackboard.read(BlackboardKeys.SAFETY_ACTIVE))
            else py_trees.common.Status.FAILURE
        )


# Player eligibility


class IsAllPlayersInactive(_ReadOnlyLeaf):
    """SUCCESS when every team player is unavailable, penalized, injured, or not on field."""

    def __init__(self, kit: "SoccerKit"):
        super().__init__("IsAllPlayersInactive")
        self._kit = kit

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        context = self._read_context()
        if game is None or context is None:
            return py_trees.common.Status.FAILURE
        for player_id in self._kit.config.player_ids:
            if self._kit.is_player_allowed(game, player_id):
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsPlayerAllowed(_ReadOnlyLeaf):
    """Whether one player is currently allowed to play."""

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"IsPlayerAllowed({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None:
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if self._kit.is_player_allowed(game, self._player_id)
            else py_trees.common.Status.FAILURE
        )


class IsPlayerDisallowed(_ReadOnlyLeaf):
    """Whether one player is currently not allowed to play."""

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"IsPlayerDisallowed({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None:
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if not self._kit.is_player_allowed(game, self._player_id)
            else py_trees.common.Status.FAILURE
        )


# Ball state


class IsBallKnown(_ReadOnlyLeaf):
    """The data layer has accepted a fresh, trusted ball."""

    def __init__(self):
        super().__init__("IsBallKnown")

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if self._read_ball() is not None
            else py_trees.common.Status.FAILURE
        )


class IsOpponentRestartActive(_ReadOnlyLeaf):
    """Whether we still need to keep restart avoidance during an opponent restart."""

    def __init__(self, kit: "SoccerKit"):
        super().__init__("IsOpponentRestartActive")
        self._kit = kit

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None:
            return py_trees.common.Status.FAILURE
        if self._kit.is_opponent_restart(game):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsOpponentKickoffActive(_ReadOnlyLeaf):
    """Whether we should hold ready positions while the opponent kickoff is still active."""

    def __init__(self, kit: "SoccerKit"):
        super().__init__("IsOpponentKickoffActive")
        self._kit = kit

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None:
            return py_trees.common.Status.FAILURE
        kickoff_active = (
            game.state == GameState.PLAYING
            and game.set_play == SetPlay.NONE
            and game.secondary_time > 0
            and game.has_kicking_team()
            and game.kicking_team != self._kit.config.team_id
        )
        return (
            py_trees.common.Status.SUCCESS
            if kickoff_active
            else py_trees.common.Status.FAILURE
        )


# Kicking


class IsInKickRange(_ReadOnlyLeaf):
    """Whether the player is inside the kick-distance hysteresis window.

    The hysteresis model owns enter/exit thresholds and delay; this leaf only queries it.
    """

    def __init__(self, player_id: int, kicker: KickHysteresis):
        super().__init__(f"IsInKickRange({player_id})")
        self._player_id = player_id
        self._kicker = kicker

    def update(self) -> py_trees.common.Status:
        context = self._read_context()
        ball = self._read_ball()
        now = self.blackboard.read(BlackboardKeys.NOW)
        if context is None or ball is None or now is None:
            return py_trees.common.Status.FAILURE

        robot = context.teammates.get(self._player_id)
        if robot is None or robot.pose is None:
            return py_trees.common.Status.FAILURE

        distance = math.hypot(ball.x - robot.pose.x, ball.y - robot.pose.y)
        if self._kicker.in_kick_range(self._player_id, distance, now):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# SafetyOverrides / Hardware status and SafetyOverrides


def _read_robot_status(
    blackboard: BlackboardClient,
    player_id: int,
) -> RobotRuntimeStatus | None:
    value = blackboard.read(robot_status_key(player_id))
    return value if isinstance(value, RobotRuntimeStatus) else None


class IsRobotFallen(_ReadOnlyLeaf):
    """SUCCESS when robot ``fall_down_state`` is not ``normal``.

    ``getting_up`` also counts as non-normal, so action commands stay disabled during recovery.
    """

    def __init__(self, player_id: int):
        super().__init__(f"IsRobotFallen({player_id})")
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        game = self._read_game()
        if game is None or game.state not in {GameState.READY, GameState.PLAYING}:
            return py_trees.common.Status.FAILURE
        status = _read_robot_status(self.blackboard, self._player_id)
        if status is None or status.is_fall_down_normal:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsNotWalkMode(_ReadOnlyLeaf):
    """SUCCESS when ``status.mode`` has been observed and is not ``walk``.

    ``mode is None`` means ``get_mode`` has not succeeded yet, so this tick avoids an early mode switch.
    """

    def __init__(self, player_id: int):
        super().__init__(f"IsNotWalkMode({player_id})")
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        status = _read_robot_status(self.blackboard, self._player_id)
        if status is None or status.mode is None or status.mode == "walk":
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsWalkModeRequired(_ReadOnlyLeaf):
    """Whether this tick's BT command for the player requires walk mode.

    This only works after BT writes ``/cmd/{player_id}``; stop/no-op commands do
    not need walk mode, avoiding repeated set_mode while penalized or after the match.
    """

    def __init__(self, player_id: int):
        super().__init__(f"IsWalkModeRequired({player_id})")
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        command = self.blackboard.read(cmd_key(self._player_id))
        if not isinstance(command, RobotCommand):
            return py_trees.common.Status.FAILURE
        intent = command.intent
        if isinstance(intent, KickIntent):
            return py_trees.common.Status.SUCCESS
        if isinstance(intent, MoveIntent) and (
            abs(intent.vx) > 1e-9 or abs(intent.vy) > 1e-9 or abs(intent.vyaw) > 1e-9
        ):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
