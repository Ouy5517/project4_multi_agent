"""Stateless field predicates and player scoring functions.

This module only performs pure calculations from ``BallState`` / ``PlayContext``
plus geometry thresholds to bool/float results. It does not create :class:`Pose2D`
or dispatch commands; other targeting modules call these predicates before higher-level decisions.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    Pose2D,
    ReadySlot,
    SoccerConfig,
    PlayContext,
)


__all__ = [
    "ball_beyond_goal_line",
    "ball_beyond_own_goal_line",
    "ball_claim_score",
    "ball_in_own_defensive_area",
    "ball_is_in_midfield_or_own_half",
    "ball_near_sideline",
    "pose_for_slot",
    "side_should_challenge",
    "sideline_sign",
]


# Field-geometry predicates


def ball_in_own_defensive_area(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is in our dangerous area where the goalkeeper should clear it."""

    area_x = -config.field_length * 0.18
    area_y = min(
        config.field_width / 2.0 - 0.35,
        config.penalty_area_width / 2.0
        + config.strategy.goalkeeper_challenge_margin_m,
    )
    return ball.x < area_x and abs(ball.y) <= area_y


def ball_beyond_goal_line(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball crossed either goal line, ours or the opponent's."""

    half_length = config.field_length / 2.0
    margin = config.strategy.goal_line_recovery_margin_m
    return abs(ball.x) > half_length + margin


def ball_beyond_own_goal_line(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball crossed our goal line in the ``-x`` direction."""

    half_length = config.field_length / 2.0
    margin = config.strategy.goal_line_recovery_margin_m
    return ball.x < -half_length - margin


def ball_near_sideline(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is close enough to the sideline to trigger sideline recovery."""

    sideline_y = config.field_width / 2.0
    return (
        abs(ball.y)
        >= sideline_y - config.strategy.sideline_recovery_margin_m
    )


def ball_is_in_midfield_or_own_half(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is around midfield or our half; SIDE uses this to decide whether to challenge."""

    return ball.x < config.field_length * 0.12


def sideline_sign(y: float) -> float:
    """Return +1 for the upper side of the field and -1 for the lower side."""

    return 1.0 if y >= 0.0 else -1.0


# Player scoring


def ball_claim_score(
    config: SoccerConfig,
    slot: ReadySlot,
    pose: Pose2D,
    ball: BallState,
) -> float:
    """Estimate the cost for a player to claim the current ball; lower score is better for chaser.

    KEEPER gets a high field-length cost unless the ball is dangerous, pushing it
    to the end. CENTER is slightly cheaper than SIDE to prefer central challenges.
    """

    distance = math.hypot(ball.x - pose.x, ball.y - pose.y)
    if slot == ReadySlot.KEEPER:
        if ball_in_own_defensive_area(config, ball):
            return distance - 0.75
        return distance + config.field_length
    if slot == ReadySlot.CENTER:
        return distance - 0.20
    return distance - 0.10


def pose_for_slot(
    config: SoccerConfig,
    context: PlayContext,
    slot: ReadySlot,
) -> Pose2D | None:
    """Find the teammate pose currently assigned to a ReadySlot; return ``None`` if missing."""

    for player_id, robot in context.teammates.items():
        if config.ready_slot_for_player(player_id) == slot:
            return robot.pose
    return None


def side_should_challenge(
    config: SoccerConfig,
    context: PlayContext,
) -> bool:
    """Whether the SIDE slot should challenge: always in midfield/own half, and in attack only when clearly closer than CENTER."""

    ball = context.known_ball
    if ball_is_in_midfield_or_own_half(config, ball):
        return True
    center = pose_for_slot(config, context, ReadySlot.CENTER)
    side = pose_for_slot(config, context, ReadySlot.SIDE)
    if center is None or side is None:
        return False
    center_dist = math.hypot(ball.x - center.x, ball.y - center.y)
    side_dist = math.hypot(ball.x - side.x, ball.y - side.y)
    return side_dist + 0.35 < center_dist
