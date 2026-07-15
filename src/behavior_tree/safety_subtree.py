"""Factory functions for SafetyGuards and SafetyOverrides subtrees.

Both subtrees are independent of playbook tactics:

Selector(SafetyGuards)
|-- NoGameStop:        not HasGameState -> StopAll "no game controller state"
|-- AllInactiveStop:   IsAllPlayersInactive -> StopAll "all players inactive"
|-- StoppedPlayStop:   IsGameStopped -> StopAll "game stopped"
|-- NonPlayingStop:    IsNonPlayingState -> StopAll "non playing state"
`-- NoPlayingBallStop: IsGameInState(PLAYING), not IsBallKnown -> StopAll "waiting for ball"

Parallel(SafetyOverrides)
`-- Sequence(PlayerSafety(player_id)) x N
    |-- AllowedGuard: penalized or absent players stop unless SafetyGuards already stopped the team
    |-- FallDownGuard: fallen players get up and stop
    `-- WalkModeGuard: commands that need walking trigger ensure_walk_mode
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import GameState
from .nodes.actions import (
    StopAll,
    StopPlayer,
    TriggerEnterWalkMode,
    TriggerGetUp,
)
from .nodes.conditions import (
    HasGameState,
    IsAllPlayersInactive,
    IsBallKnown,
    IsGameInState,
    IsGameStopped,
    IsGlobalSafetyActive,
    IsNonPlayingState,
    IsNotWalkMode,
    IsPlayerDisallowed,
    IsRobotFallen,
    IsWalkModeRequired,
)

if TYPE_CHECKING:
    from ..runtime import SoccerKit


# ----------------------------------------------------------------------
# Global safety guards
# ----------------------------------------------------------------------


def create_safety_subtree(kit: "SoccerKit") -> py_trees.behaviour.Behaviour:
    """Selector of global guards; any hit stops the whole team."""

    no_game = py_trees.composites.Sequence(
        name="NoGameStop",
        memory=False,
        children=[
            py_trees.decorators.Inverter(name="¬HasGameState", child=HasGameState()),
            StopAll(kit, "no game controller state"),
        ],
    )
    all_inactive = py_trees.composites.Sequence(
        name="AllInactiveStop",
        memory=False,
        children=[
            IsAllPlayersInactive(kit),
            StopAll(kit, "all players inactive"),
        ],
    )
    stopped = py_trees.composites.Sequence(
        name="StoppedPlayStop",
        memory=False,
        children=[
            IsGameStopped(),
            StopAll(kit, "game stopped"),
        ],
    )
    non_playing = py_trees.composites.Sequence(
        name="NonPlayingStop",
        memory=False,
        children=[
            IsNonPlayingState(),
            StopAll(kit, "non playing state"),
        ],
    )
    no_playing_ball = py_trees.composites.Sequence(
        name="NoPlayingBallStop",
        memory=False,
        children=[
            IsGameInState(GameState.PLAYING),
            py_trees.decorators.Inverter(name="¬IsBallKnown", child=IsBallKnown()),
            StopAll(kit, "waiting for ball"),
        ],
    )
    return py_trees.composites.Selector(
        name="SafetyGuards",
        memory=False,
        children=[no_game, all_inactive, stopped, non_playing, no_playing_ball],
    )


# ----------------------------------------------------------------------
# SafetyOverrides: per-player hardware-readiness overlays
# ----------------------------------------------------------------------


def _create_player_safety_subtree(
    kit: "SoccerKit",
    player_id: int,
) -> py_trees.behaviour.Behaviour:
    """One player's safety overlay: get up when fallen and switch to walk when needed.

    Sequence(PlayerSafety(player_id))
    |-- Selector(FallDownGuard)
    |   |-- Sequence(IsRobotFallen,
    |   |            TriggerGetUp,
    |   |            StopPlayer "fall down recovery")
    |   `-- AlwaysSuccess
    `-- Selector(WalkModeGuard)
        |-- Sequence(IsNotWalkMode,
        |            IsWalkModeRequired,
        |            TriggerEnterWalkMode)
        `-- AlwaysSuccess

    Each Selector has an ``AlwaysSuccess`` fallback so PlayerSafety always
    succeeds. Its purpose is to patch this tick's ``/cmd/{player_id}``, not to
    affect MatchControl semantics. ``IsWalkModeRequired`` prevents repeated set_mode
    for penalized players or stop commands.
    """

    fall_down_guard = py_trees.composites.Selector(
        name=f"FallDownGuard({player_id})",
        memory=False,
        children=[
            py_trees.composites.Sequence(
                name=f"FallDownRecovery({player_id})",
                memory=False,
                children=[
                    IsRobotFallen(player_id),
                    TriggerGetUp(kit, player_id),
                    StopPlayer(kit, player_id, "fall down recovery"),
                ],
            ),
            py_trees.behaviours.Success(name=f"FallDownOk({player_id})"),
        ],
    )
    allowed_guard = py_trees.composites.Selector(
        name=f"AllowedGuard({player_id})",
        memory=False,
        children=[
            IsGlobalSafetyActive(),
            py_trees.composites.Sequence(
                name=f"AllowedStop({player_id})",
                memory=False,
                children=[
                    IsPlayerDisallowed(kit, player_id),
                    StopPlayer(kit, player_id, "inactive or penalized"),
                ],
            ),
            py_trees.behaviours.Success(name=f"AllowedOk({player_id})"),
        ],
    )
    walk_mode_guard = py_trees.composites.Selector(
        name=f"WalkModeGuard({player_id})",
        memory=False,
        children=[
            py_trees.composites.Sequence(
                name=f"WalkModeRecovery({player_id})",
                memory=False,
                children=[
                    IsNotWalkMode(player_id),
                    IsWalkModeRequired(player_id),
                    TriggerEnterWalkMode(kit, player_id),
                ],
            ),
            py_trees.behaviours.Success(name=f"WalkModeOk({player_id})"),
        ],
    )
    return py_trees.composites.Sequence(
        name=f"PlayerSafety({player_id})",
        memory=False,
        children=[allowed_guard, fall_down_guard, walk_mode_guard],
    )


def create_safety_overrides_subtree(
    kit: "SoccerKit",
) -> py_trees.behaviour.Behaviour:
    """Team-level SafetyOverrides; per-player overlays run in parallel without blocking each other."""

    return py_trees.composites.Parallel(
        name="SafetyOverrides",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[
            _create_player_safety_subtree(kit, player_id)
            for player_id in kit.config.player_ids
        ],
    )
