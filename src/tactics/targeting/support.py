"""Support-position targets plus teammate-spacing pushout.

SafetyGuards ensure PLAY support targets only run with fresh ball and
GameController data. Supporters screen the shot lane in defense, offer a
backward outlet near midfield, move ahead in attack, and avoid stacking.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    Pose2D,
    SoccerConfig,
    PlayContext,
)
from ..geometry import clamp
from ..geometry import TeamFieldFrame
from .attack import PlayerAllowed


__all__ = ["support_target"]


def support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Compute this tick's supporter target Pose2D.

    In our defensive half, screen the direct ball-to-goal lane. Around midfield,
    stay behind and lateral to the ball. When the ball is advanced, move ahead
    and lateral to become a legal forward passing option.
    Pushout: use :func:`_spaced_support_target` to avoid overlapping other supporters.
    """

    side = 1.0 if player_id % 2 == 0 else -1.0
    lateral = config.strategy.support_lateral_m * side
    ball = context.known_ball
    if ball.x < -0.5:
        x = ball.x - config.strategy.support_depth_m
        own_goal_x = field.own_goal_x()
        lane_ratio = (x - own_goal_x) / max(0.1, ball.x - own_goal_x)
        lane_y = ball.y * clamp(lane_ratio, 0.0, 1.0)
        y = lane_y + side * min(0.45, config.strategy.support_lateral_m * 0.4)
    elif ball.x > config.strategy.support_depth_m + 0.35:
        x = ball.x + config.strategy.support_forward_m
        y = ball.y + lateral
    else:
        x = field.own_half_x(ball.x - config.strategy.support_depth_m, margin=0.35)
        y = ball.y + lateral
    y = clamp(
        y,
        -config.field_width / 2.0 + 0.45,
        config.field_width / 2.0 - 0.45,
    )
    target = field.clamp_inside_field(
        Pose2D(x, y, field.face_ball_theta(x, y, ball))
    )
    return _spaced_support_target(
        config,
        field,
        player_id,
        context,
        target,
        is_player_allowed,
    )


# Teammate spacing pushout


def _spaced_support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    target: Pose2D,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """If target is closer than min_spacing to the nearest teammate, push it along "teammate -> target" out to ``min_spacing``.

    Steps:
    1. Find the nearest legal teammate.
    2. If distance is large enough, do nothing.
    3. Otherwise scale the "teammate -> target" unit vector to min_spacing.
    4. Clamp inside the field and finally face the ball.

    Degenerate case: when target almost overlaps the teammate, no direction can
    be scaled, so fall back to ``lane_sign`` based on which side of the ball target
    is on; if target is exactly on the ball, split by player_id parity.

    In extreme corners with teammate pressure, clamping can make the final target
    slightly closer than min_spacing. With at most three teammates this is rare; if
    strict final distance is needed, iterate once more after clamping.
    """

    min_spacing = config.strategy.support_min_spacing_m
    if min_spacing <= 0.0:
        return target

    ball = context.known_ball
    game = context.known_game
    teammate_poses = tuple(
        robot.pose
        for teammate_id, robot in context.teammates.items()
        if teammate_id != player_id
        and robot.pose is not None
        and is_player_allowed(game, teammate_id)
    )
    if not teammate_poses:
        return target

    closest = min(
        teammate_poses,
        key=lambda pose: math.hypot(pose.x - target.x, pose.y - target.y),
    )
    dx = target.x - closest.x
    dy = target.y - closest.y
    distance = math.hypot(dx, dy)
    if distance >= min_spacing:
        return target

    if distance <= 1e-6:
        # Target overlaps the nearest teammate; use the original lane_sign fallback direction.
        lane_sign = 1.0 if target.y >= ball.y else -1.0
        if abs(target.y - ball.y) < 1e-6:
            lane_sign = 1.0 if player_id % 2 == 0 else -1.0
        dx, dy = 0.0, lane_sign
        distance = 1.0

    scale = min_spacing / distance
    pushed = field.clamp_inside_field(
        Pose2D(
            closest.x + dx * scale,
            closest.y + dy * scale,
            target.theta,
        )
    )
    return Pose2D(
        pushed.x,
        pushed.y,
        field.face_ball_theta(pushed.x, pushed.y, ball),
    )
