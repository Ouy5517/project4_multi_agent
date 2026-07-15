"""Coordinate geometry helpers around Pose2D and the team-view field frame.

These functions serve tactic calculations such as avoidance, targets, and walking
control; they are not part of the framework data contract. ``Pose2D`` itself comes
from :mod:`soccer_framework.types`.
"""

from __future__ import annotations

import math

from ..soccer_framework import BallState, Pose2D, SoccerConfig


__all__ = [
    "clamp",
    "normalize_angle",
    "relative_to_field",
    "field_to_relative",
    "TeamFieldFrame",
]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_to_field(rel_x: float, rel_y: float, robot_pose: Pose2D) -> Pose2D:
    cos_t = math.cos(robot_pose.theta)
    sin_t = math.sin(robot_pose.theta)
    return Pose2D(
        x=robot_pose.x + rel_x * cos_t - rel_y * sin_t,
        y=robot_pose.y + rel_x * sin_t + rel_y * cos_t,
        theta=0.0,
    )


def field_to_relative(field_x: float, field_y: float, robot_pose: Pose2D) -> Pose2D:
    dx = field_x - robot_pose.x
    dy = field_y - robot_pose.y
    cos_t = math.cos(robot_pose.theta)
    sin_t = math.sin(robot_pose.theta)
    return Pose2D(
        x=dx * cos_t + dy * sin_t,
        y=-dx * sin_t + dy * cos_t,
        theta=0.0,
    )


class TeamFieldFrame:
    """Team-view field helpers for target geometry and legal clamping.

    Strategy code uses team-view coordinates: our goal is ``-x``, the opponent
    goal is ``+x``, and attack direction is always ``+x``. Mirroring from global
    coordinates should happen in input adapters such as ``PlayContextProvider``.
    """

    def __init__(self, config: SoccerConfig):
        self.config = config

    def own_goal_x(self) -> float:
        """Return our goal x coordinate in the team-view field frame."""

        return -self.config.field_length / 2.0

    def opponent_goal_x(self) -> float:
        """Return the opponent goal x coordinate in the team-view field frame."""

        return self.config.field_length / 2.0

    def attack_theta(self) -> float:
        """Return the heading angle for facing the opponent goal."""

        return 0.0

    def own_half_x(
        self,
        x: float,
        margin: float = 0.0,
    ) -> float:
        """Clamp an x coordinate so it stays in our half of the field.

        ``margin`` keeps a small distance from midfield so READY/restart targets do not sit on the line.
        """

        return min(x, -margin)

    def clamp_inside_field(self, pose: Pose2D, margin: float = 0.25) -> Pose2D:
        """Clamp a pose target inside the legal field rectangle.

        Only x/y are clamped; theta is unchanged. ``margin`` keeps safe distance
        from sidelines and goal lines.
        """

        return Pose2D(
            x=clamp(
                pose.x,
                -self.config.field_length / 2.0 + margin,
                self.config.field_length / 2.0 - margin,
            ),
            y=clamp(
                pose.y,
                -self.config.field_width / 2.0 + margin,
                self.config.field_width / 2.0 - margin,
            ),
            theta=pose.theta,
        )

    def face_ball_theta(
        self,
        x: float,
        y: float,
        ball: BallState | None,
    ) -> float:
        """Return the heading from a target point toward the ball.

        When no ball position is available, face attack direction so positioning remains reasonable.
        """

        if ball is None:
            return self.attack_theta()
        return math.atan2(ball.y - y, ball.x - x)

    def avoid_ball_target(
        self,
        target: Pose2D,
        ball: BallState,
    ) -> Pose2D:
        """Move a target outward if it is too close to a restart ball.

        Used for opponent restarts: preserve the target direction relative to the
        ball, push it outside the legal radius, then clamp inside the field. theta is unchanged.
        """

        min_distance = self.config.strategy.opponent_restart_avoid_distance_m
        dx = target.x - ball.x
        dy = target.y - ball.y
        distance = math.hypot(dx, dy)
        if distance >= min_distance:
            return self.clamp_inside_field(target)

        if distance <= 1e-6:
            # If target is exactly on the ball, no direction can be scaled; default backward toward our goal.
            dx = -1.0
            dy = 0.0
            distance = 1.0
        scale = min_distance / distance
        # Scale along the "ball -> original target" ray to the minimum safe distance.
        return self.clamp_inside_field(
            Pose2D(
                x=ball.x + dx * scale,
                y=ball.y + dy * scale,
                theta=target.theta,
            )
        )
