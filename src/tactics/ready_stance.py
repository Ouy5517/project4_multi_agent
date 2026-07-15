"""READY-stage positioning: base targets for three ReadySlots plus SetPlay variants.

Pure model extracted from :class:`SoccerKit`; it depends only on
:class:`SoccerConfig` and :class:`TeamFieldFrame`, owns no cross-tick state, and is
held by :class:`SoccerKit`.
"""

from __future__ import annotations

from ..soccer_framework import (
    BallState,
    GameControlState,
    Pose2D,
    ReadySlot,
    SetPlay,
    SoccerConfig,
)
from .geometry import TeamFieldFrame, clamp


class ReadyStance:
    """READY-stage target-position calculation.

    Three pieces of logic:

    :meth:`base_ready_target`: base positions for CENTER, SIDE, and KEEPER.
    :meth:`ready_target_for`: final target from current SetPlay and ball position,
    including own restart, opponent restart, or base target.
    :meth:`goalkeeper_guard_target`: goalkeeper guard-position formula.
    """

    def __init__(self, config: SoccerConfig, field: TeamFieldFrame):
        self.config = config
        self.field = field

    def base_ready_target(
        self,
        slot: ReadySlot,
        own_restart: bool,
    ) -> Pose2D:
        """Base target for the three ReadySlots: center, side, and keeper.

        ``own_restart`` means the restart belongs to us. Own restarts push the
        attacker toward the center circle; opponent restarts pull back for safety.
        """
        field_length = self.config.field_length
        field_width = self.config.field_width
        circle_radius = self.config.center_circle_radius
        goal_area_length = self.config.goal_area_length

        goal_x = self.field.own_goal_x() + goal_area_length + 0.25
        side_y = min(field_width / 2.0 - 0.45, max(0.9, field_width * 0.30))
        attack_x = -max(
            circle_radius * (0.95 if own_restart else 1.6),
            field_length * (0.12 if own_restart else 0.20),
        )
        attack_line_x = self.field.own_half_x(attack_x, margin=0.15)

        if slot == ReadySlot.CENTER:
            return Pose2D(
                x=attack_line_x,
                y=0.0,
                theta=self.field.attack_theta(),
            )
        if slot == ReadySlot.SIDE:
            return Pose2D(
                x=attack_line_x,
                y=side_y,
                theta=self.field.attack_theta(),
            )
        return Pose2D(x=goal_x, y=0.0, theta=self.field.attack_theta())

    def ready_target_for(
        self,
        slot: ReadySlot,
        game: GameControlState,
        ball: BallState | None,
    ) -> Pose2D:
        """Compute READY positioning from the current SetPlay and ball position.

        No SetPlay or no ball: use base target.
        Own restart: stand close to the ball, ready to restart.
        Opponent restart: avoid the configured area around the ball.
        """
        own_restart = game.is_restart_for_team(self.config.team_id)
        base_target = self.base_ready_target(slot, own_restart)
        if game.set_play == SetPlay.NONE or ball is None:
            return base_target
        if own_restart:
            return self._own_set_play_ready_target(slot, ball, base_target)
        return self.field.avoid_ball_target(base_target, ball)

    def goalkeeper_guard_target(
        self,
        ball: BallState | None,
    ) -> Pose2D:
        """Goalkeeper guard formula; the default goalkeeper role calls this."""
        keeper_x = self.field.own_goal_x() + self.config.goal_area_length + 0.50
        keeper_y = clamp((ball.y * 0.38) if ball else 0.0, -1.35, 1.35)
        return Pose2D(
            keeper_x,
            keeper_y,
            self.field.face_ball_theta(keeper_x, keeper_y, ball),
        )

    def _own_set_play_ready_target(
        self,
        slot: ReadySlot,
        ball: BallState,
        base_target: Pose2D,
    ) -> Pose2D:
        """Own-restart close-ball positioning: center behind the ball, side diagonally behind for support."""
        if slot == ReadySlot.CENTER:
            return self.field.clamp_inside_field(
                Pose2D(
                    x=ball.x - 0.45,
                    y=ball.y,
                    theta=self.field.face_ball_theta(ball.x, ball.y, ball),
                )
            )
        if slot == ReadySlot.SIDE:
            y_offset = -1.1 if ball.y > 0.0 else 1.1
            return self.field.clamp_inside_field(
                Pose2D(
                    x=ball.x - 1.3,
                    y=ball.y + y_offset,
                    theta=self.field.face_ball_theta(ball.x, ball.y, ball),
                )
            )
        return base_target
