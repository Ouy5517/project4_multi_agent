"""Data-layer leaves that write external inputs and shared state to the blackboard each tick.

``UpdateGameState``
The tree root chains these nodes in a ``Sequence`` because they depend on each other:
``UpdateClock`` writes time, ``UpdatePlayContext`` reads callbacks, freshness filters run,
and ``UpdateRobotStatus`` pulls hardware status.

PLAY-stage role assignment used to live here (old ``UpdateChaser``), but now
belongs to :class:`src.play.nodes.AssignRoles` so it only runs when play actually starts.

To add a new global input, add a ``_DataLeaf`` subclass and mount it in the DataLayer ``Sequence``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import py_trees

from ...soccer_framework import RobotRuntimeStatus, PlayContext, PlayContextProvider
from ..blackboard import BlackboardKeys, BlackboardClient, robot_status_key

if TYPE_CHECKING:
    from ...runtime import SoccerKit


_BALL_STALE_SEC = 1.5
_ROBOT_POSE_STALE_SEC = 2.0
_GAME_STATE_STALE_SEC = 2.0


class _DataLeaf(py_trees.behaviour.Behaviour):
    """Shared base for data-layer leaves; always returns SUCCESS."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = BlackboardClient(name=name)


class UpdateClock(_DataLeaf):
    """Write current time to ``/clock/now`` and reset frame-level flags."""

    def __init__(self, get_now: Callable[[], float]):
        super().__init__("UpdateClock")
        self._get_now = get_now

    def update(self) -> py_trees.common.Status:
        self.blackboard.write(BlackboardKeys.NOW, self._get_now())
        self.blackboard.write(BlackboardKeys.SAFETY_ACTIVE, False)
        return py_trees.common.Status.SUCCESS


class UpdatePlayContext(_DataLeaf):
    """Snapshot the context from :class:`PlayContextProvider` directly into ``/play_context``.

    Holding the provider here avoids an extra callback chain through runtime,
    strategy, tree, and a temporary cached context.
    """

    def __init__(self, provider: PlayContextProvider):
        super().__init__("UpdatePlayContext")
        self._provider = provider

    def update(self) -> py_trees.common.Status:
        context = self._provider.get_snapshot()
        self.blackboard.write(BlackboardKeys.PLAY_CONTEXT, context)
        return py_trees.common.Status.SUCCESS


class UpdateGameState(_DataLeaf):
    """Clear stale GameControlState in place using the data-layer watchdog.

    Freshness filtering is centralized in the data layer and mutates
    ``context.game_state`` in place, so strategy code reads the filtered value without a separate key.
    These thresholds protect against stopped publishers or blocked callbacks; they are
    not tactic tuning knobs.

    ``last_seen_at == 0.0`` means no topic callback has written it yet, often in
    tests, so it is not filtered to match the old ``last_topic_at <= 0.0`` behavior.
    """

    def __init__(self):
        super().__init__("UpdateGameState")

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        gs = context.game_state
        if gs is not None and not gs.is_recent(now, _GAME_STATE_STALE_SEC):
            context.game_state = None
        return py_trees.common.Status.SUCCESS


class UpdateRecentBall(_DataLeaf):
    """Clear a stale ball in place using the data-layer watchdog.

    Like game and robot filtering, this mutates ``context.ball`` directly and does
    not use a separate blackboard key.
    """

    def __init__(self):
        super().__init__("UpdateRecentBall")

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        if context.ball is not None and not context.ball.is_recent(
            now, _BALL_STALE_SEC
        ):
            context.ball = None
        return py_trees.common.Status.SUCCESS


class UpdateRobotPoses(_DataLeaf):
    """Clear stale teammate/opponent poses in place using the data-layer watchdog.

    ``context.teammates[i].pose``
    The snapshot is deep-copied by the provider, so these in-place edits do not
    pollute provider internals.

    Once stale poses become None, existing ``if robot.pose is None`` checks in
    strategy and motion code work without extra ``last_seen_at`` checks.
    """

    def __init__(self):
        super().__init__("UpdateRobotPoses")

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        for robot in context.teammates.values():
            if robot.pose is not None and not robot.is_recent(
                now, _ROBOT_POSE_STALE_SEC
            ):
                robot.pose = None
        for opponent in context.opponents.values():
            if opponent.pose is not None and not opponent.is_recent(
                now, _ROBOT_POSE_STALE_SEC
            ):
                opponent.pose = None
        return py_trees.common.Status.SUCCESS


class UpdateRobotStatus(_DataLeaf):
    """Pull one player's hardware status onto the blackboard.

    This is read-only: it calls throttled ``poll_runtime_status`` via
    :class:`RobotServices`, writes ``/robot_status/{player_id}``, and lets SafetyOverrides own side effects.

    Without bound services, such as tests or dry-run, it writes a default
    :class:`RobotRuntimeStatus` so guards naturally take the normal branch.
    """

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"UpdateRobotStatus({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        now = self.blackboard.read(BlackboardKeys.NOW)
        services = self._kit.robot_services()
        if services is None or now is None:
            status = RobotRuntimeStatus()
        else:
            status = services.poll_runtime_status(self._player_id, now)
        self.blackboard.write(robot_status_key(self._player_id), status)
        return py_trees.common.Status.SUCCESS
