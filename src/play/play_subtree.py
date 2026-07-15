"""PLAY-stage subtree factory; a complete playbook map for newcomers.

Shape:

Sequence(PlayingPhase)
|-- IsGameInState(PLAYING)
`-- Sequence(PlaybookCore)
    |-- AssignRoles
    `-- Roles (Parallel)
        |-- Selector(Player(1))
        |   |-- Sequence(KickoffHold(1))
        |   |   |-- IsOpponentKickoffActive
        |   |   `-- GoReadyTarget(1)
        |   |-- Sequence(PenaltyAvoid(1))
        |   |   |-- IsOpponentRestartActive
        |   |   `-- AvoidOpponentRestart(1)
        |   |-- Sequence(AsChaser(1))
        |   |-- Sequence(AsSupporter(1))
        |   |-- Sequence(AsGoalkeeper(1))
        |   `-- WaitForBall(1) fallback for no assigned role
        |-- Player(2) ...
        `-- Player(3) ...

Each ``As<role.name>`` branch is an :class:`IsRole` check plus the subtree
returned by ``role.build_subtree(build)``. Player Selector branch order follows
``playbook.role_registry`` registration order: chaser, supporter, then
goalkeeper by default. SafetyGuards stop the team before this subtree when
GameController or ball data is missing in PLAYING, or when ``stopped=true``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import GameState
from ..behavior_tree.nodes.actions import (
    AvoidOpponentRestart,
    GoReadyTarget,
)
from ..behavior_tree.nodes.conditions import (
    IsGameInState,
    IsOpponentKickoffActive,
    IsOpponentRestartActive,
)
from .nodes import AssignRoles, IsRole, WaitForBall
from .playbook import Playbook

if TYPE_CHECKING:
    from ..runtime import SoccerKit


def _create_role_subtree(
    kit: "SoccerKit",
    playbook: Playbook,
    player_id: int,
) -> py_trees.behaviour.Behaviour:
    """Per-player subtree that selects a branch by dynamic role.

    Branch order equals ``playbook.role_registry`` registration order, with WaitForBall as the final fallback.
    """

    branches: list[py_trees.behaviour.Behaviour] = []

    kickoff_hold = py_trees.composites.Sequence(
        name=f"KickoffHold({player_id})",
        memory=False,
        children=[
            IsOpponentKickoffActive(kit),
            GoReadyTarget(kit, player_id),
        ],
    )
    branches.append(kickoff_hold)

    # Personal avoidance guard during opponent set plays.
    personal_avoid_guard = py_trees.composites.Sequence(
        name=f"PenaltyAvoid({player_id})",
        memory=False,
        children=[
            IsOpponentRestartActive(kit),
            AvoidOpponentRestart(kit, player_id),
        ],
    )
    branches.append(personal_avoid_guard)

    # Normal role-tactic execution branches.
    for role in playbook.role_registry:
        branches.append(
            py_trees.composites.Sequence(
                name=f"As{role.name.capitalize()}({player_id})",
                memory=False,
                children=[
                    IsRole(player_id, role.name),
                    role.build_subtree(kit, player_id),
                ],
            )
        )
    branches.append(WaitForBall(kit, playbook, player_id))

    return py_trees.composites.Selector(
        name=f"Player({player_id})",
        memory=False,
        children=branches,
    )


def create_play_subtree(
    kit: "SoccerKit",
    playbook: Playbook,
) -> py_trees.behaviour.Behaviour:
    """PLAY subtree: assign roles by Playbook, run players in parallel, and let each player handle top-level avoidance and stopping."""

    role_branches = py_trees.composites.Parallel(
        name="Roles",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[
            _create_role_subtree(kit, playbook, player_id)
            for player_id in kit.config.player_ids
        ],
    )
    playbook_core = py_trees.composites.Sequence(
        name="PlaybookCore",
        memory=False,
        children=[
            # SafetyGuards own input validity; PLAY tactics can assume fresh
            # GameController and ball data.
            AssignRoles(playbook),
            role_branches,
        ],
    )

    return py_trees.composites.Sequence(
        name="PlayingPhase",
        memory=False,
        children=[IsGameInState(GameState.PLAYING), playbook_core],
    )
