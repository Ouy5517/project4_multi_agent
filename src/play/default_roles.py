"""Default dynamic role implementations for SoccerSim.

Each role extends :class:`RoleStrategy` and assembles its subtree through
:meth:`build_subtree`. Methods such as ``target``, ``wants_to_kick``, and
``kick_target`` are implementation helpers for role utility nodes, not
base-class contracts.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import PlayContext, Pose2D, ReadySlot
from .nodes import AttackSubtreeConfig, MoveToTarget, build_attack_subtree
from .role import RoleStrategy

if TYPE_CHECKING:
    from ..runtime import SoccerKit


# ----------------------------------------------------------------------
# Chaser: approach and kick the ball
# ----------------------------------------------------------------------

# Approach alignment distance behind the ball while chasing, in meters.
_CHASER_APPROACH_OFFSET = 0.4


class ChaserRole(RoleStrategy):
    """Default chaser; kick target is split by ReadySlot into center shot or side clearance."""

    name = "chaser"

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball
        kt = self.kick_target(kit, player_id, context)
        kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
        return kit.motion.approach_target(
            ball,
            kick_theta,
            _CHASER_APPROACH_OFFSET,
        )

    def kick_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        slot = kit.config.ready_slot_for_player(player_id)
        if slot == ReadySlot.SIDE:
            return kit.targeting.select_clear_or_pass_target(
                player_id,
                context,
                kit.is_player_allowed,
            )
        return kit.targeting.select_kick_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def _approach_reason(self, kit: "SoccerKit", player_id: int) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        return f"{slot.value} approach ball"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        player_id: int,
        target: Pose2D,
    ) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        default = "side clear" if slot == ReadySlot.SIDE else "center kick"
        return kit.targeting.kick_reason(target, default=default)

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, player_id, context),
                kick_target_fn=lambda context: self.kick_target(
                    kit,
                    player_id,
                    context,
                ),
                reason_fn=lambda: self._approach_reason(kit, player_id),
                kick_reason_fn=lambda target: self._kick_reason(
                    kit,
                    player_id,
                    target,
                ),
            ),
        )


# ----------------------------------------------------------------------
# Supporter: attacking support position
# ----------------------------------------------------------------------


class SupporterRole(RoleStrategy):
    """Attacking support role; uses :meth:`Targeting.support_target` for positioning."""

    name = "supporter"

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        return kit.targeting.support_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "supporter hold",
        )


# ----------------------------------------------------------------------
# Defender: extension defensive position, defaulting to supporter target
# ----------------------------------------------------------------------


class DefenderRole(RoleStrategy):
    """Extension defensive role; default implementation reuses the supporter target."""

    name = "defender"

    def __init__(self) -> None:
        super().__init__()
        self._fallback = SupporterRole()

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        return self._fallback.target(kit, player_id, context)

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "defender hold",
        )


# ----------------------------------------------------------------------
# Goalkeeper: guard the goal and clear dangerous balls
# ----------------------------------------------------------------------


class GoalkeeperRole(RoleStrategy):
    """Goalkeeper guarding and defensive-area clearance."""

    name = "goalkeeper"

    # Approach alignment distance for goalkeeper challenges, tighter than the chaser, in meters.
    _APPROACH_OFFSET = 0.22

    def target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        # When the ball is dangerous and kickable, target the approach point behind the ball to enter IsInKickRange.
        # Otherwise return to the goal-line guard target.
        ball = context.known_ball
        if self.wants_to_kick(kit, context):
            kt = self.kick_target(kit, context)
            kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
            return kit.motion.approach_target(
                ball,
                kick_theta,
                self._APPROACH_OFFSET,
            )
        return kit.ready_stance.goalkeeper_guard_target(ball)

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> bool:
        return kit.targeting.ball_in_own_defensive_area(context.known_ball)

    def kick_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball
        if kit.targeting.ball_near_sideline(ball):
            return kit.targeting.sideline_recovery_target(ball)
        return Pose2D(
            kit.field.opponent_goal_x(),
            0.0,
            kit.field.attack_theta(),
        )

    def _guard_reason(self) -> str:
        return "goalkeeper guard"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        target: Pose2D,
    ) -> str:
        return kit.targeting.kick_reason(target, default="goalkeeper clear")

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, context),
                kick_target_fn=lambda context: self.kick_target(kit, context),
                wants_kick_fn=lambda context: self.wants_to_kick(kit, context),
                reason_fn=self._guard_reason,
                kick_reason_fn=lambda target: self._kick_reason(kit, target),
            ),
        )
