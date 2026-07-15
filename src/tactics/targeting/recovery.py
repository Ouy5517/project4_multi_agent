"""Sideline recovery targets.

When the ball is near the sideline, skip normal attacking scores and use this
hard recovery target to pull the ball back infield. This is an interception layer before attack decisions.
"""

from __future__ import annotations

from collections.abc import Callable

from ...soccer_framework import BallState, Pose2D, ReadySlot, SoccerConfig
from ..geometry import clamp
from ..geometry import TeamFieldFrame
from .predicates import sideline_sign


__all__ = [
    "BaseReadyTarget",
    "sideline_recovery_target",
]


# Shared with upper layers: READY target callback signature, (slot, allow_in_attack_half) -> Pose2D.
BaseReadyTarget = Callable[[ReadySlot, bool], Pose2D]


def sideline_recovery_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    ball: BallState,
) -> Pose2D:
    """Pull the ball back infield when it is close to the sideline.

    Offset opposite the sideline by ``sideline_recovery_infield_m`` and advance
    by ``sideline_recovery_advance_m`` along x. Dynamic margin prevents target_y from clamping back to the sideline.
    """

    sign_y = sideline_sign(ball.y)
    target_x = ball.x + config.strategy.sideline_recovery_advance_m
    target_y = ball.y - sign_y * config.strategy.sideline_recovery_infield_m
    safe_y_margin = min(
        config.field_width / 2.0 - 0.35,
        config.strategy.sideline_recovery_infield_m,
    )
    target_y = clamp(
        target_y,
        -config.field_width / 2.0 + safe_y_margin,
        config.field_width / 2.0 - safe_y_margin,
    )
    return field.clamp_inside_field(
        Pose2D(target_x, target_y, field.attack_theta()),
        margin=0.35,
    )
