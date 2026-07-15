"""Avoidance targets during opponent restarts.

When GameController marks an opponent ``set_play``, all our players must stay
outside the configured ball-avoidance radius. This module passes position targets
through constraints for radius safety, penalty-area legality, and field bounds.

Control-flow shape:
Entry :func:`opponent_restart_target` chooses a base target from normal READY
position or escape mode, then runs :func:`_apply_safety_chain`.
:func:`_apply_safety_chain` composes radius safety then penalty-area legality.
Penalty-area pushout can move a target closer to the ball, so
:func:`_goal_kick_safe` calls :func:`_radius_safe` again as a fallback.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    GameControlState,
    GameState,
    Pose2D,
    ReadySlot,
    SetPlay,
    SoccerConfig,
    PlayContext,
)
from ..geometry import TeamFieldFrame
from .predicates import sideline_sign
from .recovery import BaseReadyTarget


__all__ = [
    "opponent_restart_hold_vyaw",
    "opponent_restart_target",
]


def opponent_restart_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    slot: ReadySlot,
    context: PlayContext,
    base_ready_target: BaseReadyTarget,
) -> Pose2D:
    """Position target for this tick during an opponent restart.

    Compute ``base_target`` through two possible paths, then run :func:`_apply_safety_chain`:

    If the player is already too close to the ball, use :func:`_escape_initial_target`
    and add ``min_distance + 0.35`` buffer in escape mode.
    Otherwise use ``base_ready_target`` directly.

    See :func:`_apply_safety_chain` for safety-chain details.
    """

    base_target = base_ready_target(slot, False)
    ball = context.known_ball
    game = context.known_game

    min_distance = config.strategy.opponent_restart_avoid_distance_m
    robot = context.teammates.get(player_id)
    if robot is not None and robot.pose is not None:
        distance_to_ball = math.hypot(
            robot.pose.x - ball.x,
            robot.pose.y - ball.y,
        )
        if distance_to_ball < min_distance + 0.25:
            escape_target = _escape_initial_target(
                field, robot.pose, ball, game,
                min_distance=min_distance + 0.35,
            )
            return _apply_safety_chain(
                config, field, escape_target, ball, game,
                min_distance=min_distance + 0.35,
            )

    return _apply_safety_chain(
        config, field, base_target, ball, game, min_distance,
    )


def opponent_restart_hold_vyaw(
    config: SoccerConfig,
    player_id: int,
    game: GameControlState,
) -> float:
    """During opponent restart in PLAYING, apply a small yaw rate so players scan for the ball.

    Return 0 before PLAYING or while stopped. Direction alternates by player_id
    parity, with magnitude capped by min(0.12, ``max_angular_speed``).
    """

    if game.state != GameState.PLAYING or game.stopped:
        return 0.0
    direction = 1.0 if player_id % 2 else -1.0
    return direction * min(0.12, config.strategy.max_angular_speed)


# Safety chain


def _apply_safety_chain(
    config: SoccerConfig,
    field: TeamFieldFrame,
    target: Pose2D,
    ball: BallState,
    game: GameControlState,
    min_distance: float,
) -> Pose2D:
    """Safety constraint chain for restart positioning.

    Order:
    1. :func:`_radius_safe` ensures target is at least ``min_distance`` from the ball.
    2. :func:`_goal_kick_safe` pushes target out of the opponent penalty area on GOAL_KICK only.

    Note: ``_goal_kick_safe`` calls ``_radius_safe`` again because lateral pushout
    can move the target closer than min_distance. That nonlinear call is a property
    of the constraints, not a control-flow bug.
    """

    target = _radius_safe(field, target, ball, min_distance)
    target = _goal_kick_safe(config, field, target, ball, game, min_distance)
    return target


# Single-step constraints


def _escape_initial_target(
    field: TeamFieldFrame,
    robot_pose: Pose2D,
    ball: BallState,
    game: GameControlState,
    min_distance: float,
) -> Pose2D:
    """Compute an initial escape target away from the ball when a player is already too close.

    Move along "ball -> current pose" out to ``min_distance``; THROW_IN adds a
    backfield bias. The target still must pass :func:`_apply_safety_chain`.
    """

    dx = robot_pose.x - ball.x
    dy = robot_pose.y - ball.y
    if math.hypot(dx, dy) <= 1e-6:
        dx = -1.0
        dy = -sideline_sign(ball.y)

    if game.set_play == SetPlay.THROW_IN:
        dx -= 0.8
        dy -= sideline_sign(ball.y) * 0.9

    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        distance = 1.0
    target_x = ball.x + dx / distance * min_distance
    target_y = ball.y + dy / distance * min_distance
    target = Pose2D(
        x=target_x,
        y=target_y,
        theta=field.face_ball_theta(target_x, target_y, ball),
    )
    if game.set_play == SetPlay.THROW_IN:
        target = Pose2D(
            field.own_half_x(target.x, margin=0.45),
            target.y,
            target.theta,
        )
    return target


def _radius_safe(
    field: TeamFieldFrame,
    target: Pose2D,
    ball: BallState,
    min_distance: float,
) -> Pose2D:
    """Ensure target is at least ``min_distance`` from the ball.

    If too close, scale along the original direction. If scaling fails because
    target is exactly on the ball, fall back to "back from ball plus opposite sideline";
    if still invalid, fall back to our half.
    """

    candidate = field.clamp_inside_field(target, margin=0.35)
    dx = candidate.x - ball.x
    dy = candidate.y - ball.y
    distance = math.hypot(dx, dy)
    if distance >= min_distance:
        return candidate

    if distance <= 1e-6:
        dx = -1.0
        dy = -sideline_sign(ball.y)
        distance = math.hypot(dx, dy)

    candidate = field.clamp_inside_field(
        Pose2D(
            x=ball.x + dx / distance * min_distance,
            y=ball.y + dy / distance * min_distance,
            theta=target.theta,
        ),
        margin=0.35,
    )
    if math.hypot(candidate.x - ball.x, candidate.y - ball.y) >= min_distance:
        return candidate

    fallback_y = ball.y - sideline_sign(ball.y) * min_distance
    fallback_x = field.own_half_x(
        ball.x - min_distance,
        margin=0.45,
    )
    return field.clamp_inside_field(
        Pose2D(
            fallback_x,
            fallback_y,
            field.face_ball_theta(fallback_x, fallback_y, ball),
        ),
        margin=0.35,
    )


def _goal_kick_safe(
    config: SoccerConfig,
    field: TeamFieldFrame,
    target: Pose2D,
    ball: BallState,
    game: GameControlState,
    min_distance: float,
) -> Pose2D:
    """During opponent goal kicks, also avoid standing inside the opponent penalty area.

    If target is inside the penalty area, push it laterally outside the area, then
    run :func:`_radius_safe` again because lateral pushout may move it closer to the ball.
    """

    if game.set_play != SetPlay.GOAL_KICK:
        return target

    goal_sign = 1.0 if ball.x >= 0.0 else -1.0
    penalty_front_x = goal_sign * (
        config.field_length / 2.0
        - config.penalty_area_length
        - 0.55
    )
    inside_penalty_x = (
        target.x >= penalty_front_x
        if goal_sign > 0.0
        else target.x <= penalty_front_x
    )
    inside_penalty_y = (
        abs(target.y) <= config.penalty_area_width / 2.0 + 0.35
    )
    if not (inside_penalty_x and inside_penalty_y):
        return target

    lateral_sign = -1.0 if ball.y >= 0.0 else 1.0
    lateral_y = lateral_sign * (
        config.penalty_area_width / 2.0 + min_distance
    )
    safe_x = penalty_front_x - goal_sign * 0.45
    candidate = field.clamp_inside_field(
        Pose2D(
            safe_x,
            lateral_y,
            field.face_ball_theta(
                safe_x,
                lateral_y,
                ball,
            ),
        ),
        margin=0.35,
    )
    if math.hypot(candidate.x - ball.x, candidate.y - ball.y) >= min_distance:
        return candidate

    return _radius_safe(field, candidate, ball, min_distance)
