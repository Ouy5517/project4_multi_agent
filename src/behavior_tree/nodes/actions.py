"""Non-PLAY action leaves and command aggregation.

PLAY-stage dynamic-role action leaves live in :mod:`src.play`; this module keeps
only playbook-agnostic actions:

- stop actions such as :class:`StopPlayer` and :class:`StopAll`
- READY/restart movement actions
- safety override actions such as :class:`TriggerGetUp` and
  :class:`TriggerEnterWalkMode`
- :class:`CommitTeamCommands`, which packs per-player commands and submits them
  to :class:`TeamCommandExecutor`
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ...soccer_framework import (
    BallState,
    GameControlState,
    RobotCommand,
    RobotRuntimeStatus,
    PlayContext,
)
from ..blackboard import BlackboardKeys, BlackboardClient, cmd_key, robot_status_key

if TYPE_CHECKING:
    from ...runtime import SoccerKit


class _ActionLeaf(py_trees.behaviour.Behaviour):
    """Shared base that writes a command to this player's ``/cmd/{player_id}`` slot."""

    def __init__(self, name: str, kit: "SoccerKit", player_id: int):
        super().__init__(name)
        self._kit = kit
        self._player_id = player_id
        self.blackboard = BlackboardClient(name=name)

    def update(self) -> py_trees.common.Status:
        command = self._compute_command()
        if command is None:
            return py_trees.common.Status.FAILURE
        self.blackboard.write(cmd_key(self._player_id), command)
        return py_trees.common.Status.SUCCESS

    def _compute_command(self) -> RobotCommand | None:
        raise NotImplementedError

    def _read_context(self) -> PlayContext | None:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        return context if isinstance(context, PlayContext) else None

    def _read_game(self) -> GameControlState | None:
        context = self._read_context()
        return context.game_state if context is not None else None

    def _read_ball(self) -> BallState | None:
        context = self._read_context()
        return context.ball if context is not None else None


# Common


def _stop_or_noop_for_status(
    blackboard: BlackboardClient,
    player_id: int,
    reason: str,
) -> RobotCommand:
    status = blackboard.read(robot_status_key(player_id))
    if (
        isinstance(status, RobotRuntimeStatus)
        and status.mode is not None
        and status.mode != "walk"
    ):
        return RobotCommand.noop(reason)
    return RobotCommand.stop(reason)


class StopPlayer(_ActionLeaf):
    """Send a stop command to one player."""

    def __init__(self, kit: "SoccerKit", player_id: int, reason: str):
        super().__init__(f"StopPlayer({player_id})", kit, player_id)
        self._reason = reason

    def _compute_command(self) -> RobotCommand:
        self._kit.kicker.clear_player(self._player_id)
        return _stop_or_noop_for_status(
            self.blackboard, self._player_id, self._reason,
        )


class StopAll(py_trees.behaviour.Behaviour):
    """Stop the whole team for global guards and fallback branches.

    Also sets ``/safety/active`` to True so :class:`CommitTeamCommands` will not
    override this tick's reason with penalty handling.
    """

    def __init__(self, kit: "SoccerKit", reason: str):
        super().__init__(f"StopAll({reason})")
        self._kit = kit
        self._reason = reason
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        self._kit.kicker.clear_all()
        for player_id in self._kit.config.player_ids:
            self.blackboard.write(
                cmd_key(player_id),
                _stop_or_noop_for_status(self.blackboard, player_id, self._reason),
            )
        self.blackboard.write(BlackboardKeys.SAFETY_ACTIVE, True)
        return py_trees.common.Status.SUCCESS


# READY


class GoReadyTarget(_ActionLeaf):
    """Move to the legal ReadySlot target during READY."""

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"GoReadyTarget({player_id})", kit, player_id)

    def _compute_command(self) -> RobotCommand | None:
        context = self._read_context()
        game = self._read_game()
        ball = self._read_ball()
        if context is None or game is None:
            return None
        target = self._kit.ready_stance.ready_target_for(
            self._kit.config.ready_slot_for_player(self._player_id),
            game,
            ball,
        )
        slot = self._kit.config.ready_slot_for_player(self._player_id)
        return self._kit.motion.move_to_target(
            self._player_id, context, target, f"ready {slot.value}",
            avoid_opponents=True,
        )


class AvoidOpponentRestart(_ActionLeaf):
    """Move to a legal avoidance target during opponent restarts."""

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"AvoidOpponentRestart({player_id})", kit, player_id)

    def _compute_command(self) -> RobotCommand | None:
        context = self._read_context()
        game = self._read_game()
        ball = self._read_ball()
        if context is None or game is None:
            return None
        self._kit.kicker.clear_player(self._player_id)
        if ball is None:
            return _stop_or_noop_for_status(
                self.blackboard,
                self._player_id,
                "avoid opponent restart: waiting for ball",
            )

        robot = context.teammates.get(self._player_id)
        if robot is None or robot.pose is None:
            return _stop_or_noop_for_status(
                self.blackboard,
                self._player_id,
                "avoid opponent restart: waiting for pose",
            )

        avoid_distance = self._kit.config.strategy.opponent_restart_avoid_distance_m
        distance_to_ball = math.hypot(
            robot.pose.x - ball.x,
            robot.pose.y - ball.y,
        )
        if distance_to_ball >= avoid_distance:
            return _stop_or_noop_for_status(
                self.blackboard,
                self._player_id,
                "avoid opponent restart: clear of ball",
            )

        slot = self._kit.config.ready_slot_for_player(self._player_id)
        target = self._kit.targeting.opponent_restart_target(
            self._player_id,
            slot,
            context,
            self._kit.ready_stance.base_ready_target,
        )
        return self._kit.motion.move_to_target(
            self._player_id,
            context,
            target,
            "avoid opponent restart",
            arrive_distance=0.25,
            hold_vyaw=self._kit.targeting.opponent_restart_hold_vyaw(self._player_id, game),
            avoid_opponents=True,
        )


# Command aggregation


class CommitTeamCommands(py_trees.behaviour.Behaviour):
    """Collect every ``/cmd/{player_id}`` and hand them to the executor.

    This node is the only handshake point between the BT and runtime:
    1. Fill a stop command for players without a command.
    2. Submit the full command dict to the executor in ``/runtime/executor``.
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("CommitTeamCommands")
        self._kit = kit
        self.blackboard = BlackboardClient(name=self.name)
        self.last_committed: dict[int, RobotCommand] = {}
        self.last_executed: dict[int, RobotCommand] = {}

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.FAILURE

        safety_active = bool(self.blackboard.read(BlackboardKeys.SAFETY_ACTIVE))

        commands: dict[int, RobotCommand] = {}
        for player_id in self._kit.config.player_ids:
            cmd = self.blackboard.read(cmd_key(player_id))
            if cmd is None:
                cmd = _stop_or_noop_for_status(
                    self.blackboard, player_id, "tree produced no command",
                )
            commands[player_id] = cmd
            # Clear the slot so a branch that skips writing next tick cannot read a stale command.
            self.blackboard.write(cmd_key(player_id), None)

        executor = self.blackboard.read(BlackboardKeys.EXECUTOR)
        if executor is not None:
            executed = executor.execute_team_commands(commands)
        else:
            executed = commands

        self.last_committed = commands
        self.last_executed = executed
        return py_trees.common.Status.SUCCESS


# Safety overrides: get up and enter walk mode


class TriggerGetUp(py_trees.behaviour.Behaviour):
    """Trigger ``robot.get_up()``; the manager throttles retries and this node always returns SUCCESS.

    Always returning SUCCESS is a key SafetyOverrides convention: get-up is only
    a side effect, and the following ``StopPlayer`` overwrites this tick's command with stop.
    """

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"TriggerGetUp({player_id})")
        self._kit = kit
        self._player_id = player_id
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        services = self._kit.robot_services()
        now = self.blackboard.read(BlackboardKeys.NOW)
        if services is not None and now is not None:
            services.trigger_get_up(self._player_id, now)
        return py_trees.common.Status.SUCCESS


class TriggerEnterWalkMode(py_trees.behaviour.Behaviour):
    """Trigger one ``ensure_walk_mode`` call; this node always returns SUCCESS.

    The switch is asynchronous and :class:`UpdateRobotStatus` confirms it next
    tick; this tick does not override the command because a standing robot can keep the BT command already produced.
    """

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"TriggerEnterWalkMode({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        services = self._kit.robot_services()
        if services is not None:
            services.ensure_walk_mode(
                self._player_id, f"player {self._player_id} not in walk mode",
            )
        return py_trees.common.Status.SUCCESS
