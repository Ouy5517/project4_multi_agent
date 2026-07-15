"""Centralized blackboard keys and a thin wrapper.

Each key is annotated with its business meaning and data type, forming a starter data-flow map.

``BlackboardClient`` is a thin shell around
``py_trees.blackboard.Client`` that centralizes access registration and exposes
``read`` / ``write`` helpers that are easier to read than raw ``set/get`` calls.
"""

from __future__ import annotations

from typing import Any

import py_trees


class BlackboardKeys:
    """Complete set of blackboard keys used by BT nodes."""

    # Data layer
    NOW = "/clock/now"                  #  Current tick time from time.monotonic.
    PLAY_CONTEXT = "/play_context"     #  Filtered PlayContext; stale game_state/ball/pose are None.

    # Tactic layer
    # Written by AssignRoles at the front of PLAY; type RoleAssignment | None.
    # All PLAY role-condition leaves read from this slot.
    ROLES = "/team/roles"

    # Runtime handshake
    # Written by the caller in tick; CommitTeamCommands hands commands to it.
    EXECUTOR = "/runtime/executor"      #  Optional team command executor.

    # Global stop flag
    # Set True when SafetyGuards stop the team; CommitTeamCommands then stops overriding
    # commands with penalty handling. Reset by UpdateClock every tick.
    SAFETY_ACTIVE = "/safety/active"

    # Per-role command slots; dynamic keys use cmd_key().
    _CMD_PREFIX = "/cmd"

    # Per-robot hardware status slots; dynamic keys use robot_status_key().
    # Written by UpdateRobotStatus every tick; StopAll/StopPlayer use mode to decide
    # whether to write no-op, and SafetyOverrides use it for get-up or walk-mode recovery.
    # Value type is RobotRuntimeStatus.
    _ROBOT_STATUS_PREFIX = "/robot_status"


def cmd_key(player_id: int) -> str:
    """Return the command-slot key for one player."""

    return f"{BlackboardKeys._CMD_PREFIX}/{player_id}"


def robot_status_key(player_id: int) -> str:
    """Return the hardware-status key for one player."""

    return f"{BlackboardKeys._ROBOT_STATUS_PREFIX}/{player_id}"


class BlackboardClient:
    """Thin py_trees.Blackboard wrapper that centralizes access registration."""

    def __init__(self, name: str):
        self._client = py_trees.blackboard.Client(name=name)
        self._registered: set[tuple[str, str]] = set()

    def read(self, key: str, default: Any = None) -> Any:
        self._ensure(key, "read")
        try:
            return self._client.get(key)
        except KeyError:
            return default

    def write(self, key: str, value: Any) -> None:
        self._ensure(key, "write")
        self._client.set(key, value, overwrite=True)

    def _ensure(self, key: str, mode: str) -> None:
        token = (key, mode)
        if token in self._registered:
            return
        access = (
            py_trees.common.Access.READ
            if mode == "read"
            else py_trees.common.Access.WRITE
        )
        self._client.register_key(key=key, access=access)
        self._registered.add(token)
