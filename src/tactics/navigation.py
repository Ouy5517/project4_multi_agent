"""Obstacle collection tools: stateless geometry reused by motion control and tactic target selection.

This module provides :class:`Obstacle` and :class:`ObstacleCollector`, converting
opponents, teammates, and goal structure from :class:`PlayContext` into uniform circular obstacles.

Design principle: pure geometry with no cross-tick state. Motion-control state
such as via-side memory is owned by :class:`src.tactics.motion.MotionController`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..soccer_framework import (
    SoccerConfig,
    PlayContext,
)
from .geometry import TeamFieldFrame


@dataclass(frozen=True)
class Obstacle:
    """Circular obstacle in field coordinates."""

    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class ObstacleCollector:
    """Stateless helper that collects obstacles from :class:`PlayContext`.

    It keeps no cross-tick state. :class:`MotionController` calls it each tick for
    opponents, teammates, and goal structure; :class:`Targeting` uses it for opponent positions in pass-lane scoring.
    """

    config: SoccerConfig
    field: TeamFieldFrame

    def opponent_obstacles(
        self,
        context: PlayContext,
    ) -> tuple[Obstacle, ...]:
        """Convert visible opponent robots into circular obstacles.

        Opponents with ``pose`` None are skipped because stale poses were cleared
        by :class:`UpdateRobotPoses`. Radius uses ``opponent_obstacle_radius``, larger
        than teammates because opponents are less predictable.
        """
        obstacles: list[Obstacle] = []
        for robot in context.opponents.values():
            if robot.pose is None:
                continue
            obstacles.append(
                Obstacle(
                    robot.pose.x,
                    robot.pose.y,
                    self.config.strategy.opponent_obstacle_radius,
                )
            )
        return tuple(obstacles)

    def teammate_obstacles(
        self,
        player_id: int,
        context: PlayContext,
    ) -> tuple[Obstacle, ...]:
        """Convert visible teammates, excluding self, into circular obstacles.

        Radius uses ``teammate_obstacle_radius``, smaller than opponents because
        teammates are more predictable. Pose filtering matches :meth:`opponent_obstacles`.
        """
        obstacles: list[Obstacle] = []
        for teammate_id, robot in context.teammates.items():
            if teammate_id == player_id or robot.pose is None:
                continue
            obstacles.append(
                Obstacle(
                    robot.pose.x,
                    robot.pose.y,
                    self.config.strategy.teammate_obstacle_radius,
                )
            )
        return tuple(obstacles)

    def goal_structure_obstacles(self) -> tuple[Obstacle, ...]:
        """Treat each goal as an impassable U-shaped structure sampled into circular obstacles.

        A goal has four posts and three net sides. Robots must not pass through posts or net, so:

        Four posts become four small-radius obstacles matching the rule dimensions.
        Three net sides are uniformly sampled so corridor checks see a continuous wall.

        ``net_step`` is 0.35 m, smaller than opponent radius plus safety margin, so
        any approach angle should hit at least one sample.
        """
        half_length = self.config.field_length / 2.0
        half_goal_width = self.config.goal_width / 2.0
        goal_depth = 0.6  #  Rule doc note/3v3_rule.md: goal depth is 0.6 m.
        post_radius = 0.18
        net_radius = 0.20
        net_step = 0.35

        obstacles: list[Obstacle] = []
        for sign_x in (-1.0, 1.0):
            front_x = sign_x * half_length
            back_x = sign_x * (half_length + goal_depth)
            for sign_y in (-1.0, 1.0):
                obstacles.append(
                    Obstacle(x=front_x, y=sign_y * half_goal_width, radius=post_radius)
                )
                obstacles.append(
                    Obstacle(x=back_x, y=sign_y * half_goal_width, radius=post_radius)
                )
            obstacles.extend(
                _sample_net_segment(
                    back_x, -half_goal_width, back_x, half_goal_width,
                    step=net_step, radius=net_radius,
                )
            )
            for sign_y in (-1.0, 1.0):
                obstacles.extend(
                    _sample_net_segment(
                        front_x, sign_y * half_goal_width,
                        back_x, sign_y * half_goal_width,
                        step=net_step, radius=net_radius,
                    )
                )
        return tuple(obstacles)

    def collect_all(
        self,
        player_id: int,
        context: PlayContext,
    ) -> tuple[Obstacle, ...]:
        """Collect all obstacles for one avoidance check: opponents, teammates, and goal structure.

        Freshness filtering is already done by :class:`UpdateRobotPoses`, which
        sets stale poses to None; this method only skips ``pose is None`` robots.
        """
        return (
            self.opponent_obstacles(context)
            + self.teammate_obstacles(player_id, context)
            + self.goal_structure_obstacles()
        )


def _sample_net_segment(
    x0: float, y0: float, x1: float, y1: float,
    *, step: float, radius: float,
) -> list[Obstacle]:
    """Uniformly sample circular obstacles along a segment without duplicating endpoints."""
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= step:
        return []
    n = max(1, int(math.floor(length / step)) - 1)
    return [
        Obstacle(
            x=x0 + (x1 - x0) * (i + 1) / (n + 1),
            y=y0 + (y1 - y0) * (i + 1) / (n + 1),
            radius=radius,
        )
        for i in range(n)
    ]
