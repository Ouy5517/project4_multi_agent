"""READY subtree factory, decoupled from PLAY dynamic roles and using Team.ready_target.

Shape:

Sequence(ReadyPhase)
|-- IsGameInState(READY)
`-- Parallel(ReadySlots)
    |-- GoReadyTarget(1)
    |-- GoReadyTarget(2)
    `-- GoReadyTarget(3)

READY can run without fresh ball data; :class:`ReadyStance` falls back to base
slot targets when the ball is absent.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import GameState
from .nodes.actions import GoReadyTarget
from .nodes.conditions import IsGameInState

if TYPE_CHECKING:
    from ..runtime import SoccerKit


def create_ready_subtree(kit: "SoccerKit") -> py_trees.behaviour.Behaviour:
    """READY phase: each player moves to its own READY target."""

    ready_branches = py_trees.composites.Parallel(
        name="ReadySlots",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[
            GoReadyTarget(kit, player_id) for player_id in kit.config.player_ids
        ],
    )
    return py_trees.composites.Sequence(
        name="ReadyPhase",
        memory=False,
        children=[IsGameInState(GameState.READY), ready_branches],
    )
