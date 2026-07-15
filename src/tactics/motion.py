"""Motion controller that combines avoidance, walking control, and kick commands.

The controller translates "where to go" into a :class:`RobotCommand` while
keeping path detours, yaw-avoidance bias, and unicycle walking control together.
It is a reactive controller with no global path planning: each tick computes
detour points from current obstacles and outputs velocity directly.

The biped base is most stable with ``vx + vyaw`` commands. Lateral ``vy`` is
left at zero in combined movement commands, so avoidance is split into path
detours that change the target and yaw avoidance that changes angular velocity.

PLAY and READY share walking parameters. The remaining phase difference is the
``avoid_opponents`` flag: off for PLAY, on for READY and recovery paths.
"""

from __future__ import annotations

import math

from ..soccer_framework import (
    BallState,
    KickIntent,
    MoveIntent,
    Pose2D,
    RobotCommand,
    SoccerConfig,
    PlayContext,
)
from .geometry import clamp, field_to_relative, normalize_angle
from .kick_hysteresis import KickHysteresis
from .navigation import Obstacle, ObstacleCollector
from .geometry import TeamFieldFrame


# Walking-control parameters shared by PLAY and READY
# Use a compromise of the old PLAY/READY presets: arrive_distance in the middle,
# turn_threshold and floor from the more stable READY values. During chasing,
# small angle error will not trigger floor; near the ball,
# distance < arrive_distance goes through align unaffected by floor.
#
# 2026-06-26 adjustment: lower floor, relax arrival angle, and add dead zone to fix
# extra spinning in Ready; see note/motion_spinning_issue_analysis.md.
_ARRIVE_DISTANCE = 0.15
_ARRIVE_ANGLE = 0.20  # Relaxed to ~11.5 deg to avoid repeated micro-adjustments near target.
_TURN_THRESHOLD = 0.5
_ANGULAR_SPEED_FLOOR = 0.25  # Lower floor to reduce overshoot for small angle errors.
_ANGULAR_DEAD_ZONE = 0.15  # No floor below this angle error; pure proportional control.
_LINEAR_SPEED_FLOOR = 0.3
_LINEAR_GAIN = 0.9


class MotionController:
    """Reactive motion controller that integrates avoidance, walking control, and kicking.

    A full :meth:`move_to_target` call follows this flow:

    1. **Path detour**: find the first blocking obstacle on the original path and
    create a side via point as the new target; side choice is remembered across ticks.
    2. **Walking control**: compute unicycle velocity, ``vx + vyaw``, toward the possibly adjusted target.
    3. **Yaw avoidance**: add vyaw bias when nearby teammates or opponents require extra turning.

    The three layers change different quantities, target / velocity / angular
    velocity, and never touch ``vy``, satisfying biped constraints.
    """

    def __init__(
        self,
        config: SoccerConfig,
        field: TeamFieldFrame,
        kicker: KickHysteresis,
        obstacles: ObstacleCollector,
    ):
        self._config = config
        self._field = field
        self._kicker = kicker
        self._obstacles = obstacles
        self._avoid_side_by_player: dict[int, float] = {}

    # Public interface

    def move_to_target(
        self,
        player_id: int,
        context: PlayContext,
        target: Pose2D,
        reason: str,
        arrive_distance: float | None = None,
        hold_vyaw: float = 0.0,
        avoid_opponents: bool = False,
    ) -> RobotCommand:
        """Generate a movement command with avoidance applied.

        ``arrive_distance`` overrides the arrival threshold; ``hold_vyaw`` keeps a
        nonzero turn rate after arrival; ``avoid_opponents`` includes opponents in
        yaw avoidance. PLAY passes False so chasers are not pushed away from opponents,
        while READY/recovery/opponent restarts pass True.
        """
        robot = context.teammates.get(player_id)
        if robot is None or robot.pose is None:
            return RobotCommand.stop(f"{reason}: waiting for pose")

        # Path detour: compute via point
        adjusted_target = self._avoidance_target(player_id, robot.pose, target, context)
        adjusted_reason = (
            f"{reason} via obstacle" if adjusted_target != target else reason
        )

        # Walking control: compute vx + vyaw
        arrive_dist = _ARRIVE_DISTANCE if arrive_distance is None else arrive_distance
        command = self._compute_velocity(
            robot.pose,
            adjusted_target,
            adjusted_reason,
            arrive_dist,
            hold_vyaw,
        )

        # Yaw avoidance: add vyaw bias
        return self._apply_yaw_avoidance(
            player_id,
            context,
            command,
            avoid_opponents,
        )

    def kick_command(
        self,
        player_id: int,
        context: PlayContext,
        kick_theta: float,
        reason: str,
    ) -> RobotCommand:
        """Generate a kick command and mark the player as kicking to trigger kick hysteresis."""
        ball = context.known_ball
        robot = context.teammates.get(player_id)
        if robot is None or robot.pose is None:
            return RobotCommand.stop(f"{reason}: waiting for pose")
        self._kicker.mark_kicking(player_id)
        rel_ball = field_to_relative(ball.x, ball.y, robot.pose)
        return RobotCommand(
            intent=KickIntent(
                direction=normalize_angle(kick_theta - robot.pose.theta),
                power=self._config.strategy.soccer_kick_power,
                ball_x=rel_ball.x,
                ball_y=rel_ball.y,
            ),
            reason=reason,
        )

    def approach_target(
        self,
        ball: BallState,
        kick_theta: float,
        approach_offset: float = 0.4,
    ) -> Pose2D:
        """Approach target behind the ball: step back by ``approach_offset`` opposite the kick direction, then clamp inside the field."""
        return self._field.clamp_inside_field(
            Pose2D(
                x=ball.x - approach_offset * math.cos(kick_theta),
                y=ball.y - approach_offset * math.sin(kick_theta),
                theta=kick_theta,
            )
        )

    # Path detour

    def _avoidance_target(
        self,
        player_id: int,
        start: Pose2D,
        target: Pose2D,
        context: PlayContext,
    ) -> Pose2D:
        """Insert a detour via point on the original path.

        Finds the first blocking obstacle and generates a side via point as the
        new target. With no obstacle, returns target and clears the player's side memory.
        The via point is clamped inside the field.
        """
        obstacle = self._first_blocking_obstacle(player_id, start, target, context)
        if obstacle is None:
            self._avoid_side_by_player.pop(player_id, None)
            return target
        side_sign = self._avoid_side_by_player.get(player_id)
        if side_sign is None:
            side_sign = self._choose_avoid_side(start, target, obstacle)
        self._avoid_side_by_player[player_id] = side_sign
        via = self._via_pose(start, target, obstacle, side_sign)
        return self._field.clamp_inside_field(via)

    def _first_blocking_obstacle(
        self,
        player_id: int,
        start: Pose2D,
        target: Pose2D,
        context: PlayContext,
    ) -> Obstacle | None:
        """Find the first obstacle that truly blocks the start-to-target segment.

        Decision logic, projected in the path-local frame:

        ``along`` must fall in the middle of the path; obstacles near start or target are ignored.
        ``lateral`` must be smaller than obstacle radius plus safety margin, meaning the obstacle intrudes into the corridor.
        Among blockers, return the one with the smallest ``along`` value, nearest the start.
        """
        seg_dx = target.x - start.x
        seg_dy = target.y - start.y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            return None
        dir_x = seg_dx / seg_len
        dir_y = seg_dy / seg_len
        left_x = -dir_y
        left_y = dir_x
        best: tuple[float, Obstacle] | None = None
        for obstacle in self._obstacles.collect_all(player_id, context):
            rel_x = obstacle.x - start.x
            rel_y = obstacle.y - start.y
            along = rel_x * dir_x + rel_y * dir_y
            if (
                along <= self._config.strategy.obstacle_start_ignore_distance
                or along
                >= seg_len - self._config.strategy.obstacle_target_ignore_distance
            ):
                continue
            lateral = abs(rel_x * left_x + rel_y * left_y)
            corridor = obstacle.radius + self._config.strategy.obstacle_safety_margin
            if lateral >= corridor:
                continue
            if best is None or along < best[0]:
                best = (along, obstacle)
        return best[1] if best is not None else None

    def _choose_avoid_side(
        self,
        start: Pose2D,
        target: Pose2D,
        obstacle: Obstacle,
    ) -> float:
        """Decide whether to detour left (+1) or right (-1) the first time an obstacle is seen.

        Project the obstacle onto the path's left normal: obstacle on the left
        means detour right (-1), obstacle on the right means detour left (+1), minimizing the detour.
        """
        seg_dx = target.x - start.x
        seg_dy = target.y - start.y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            return 1.0
        left_x = -seg_dy / seg_len
        left_y = seg_dx / seg_len
        lateral = (obstacle.x - start.x) * left_x + (obstacle.y - start.y) * left_y
        return -1.0 if lateral > 0.0 else 1.0

    def _via_pose(
        self,
        start: Pose2D,
        target: Pose2D,
        obstacle: Obstacle,
        side_sign: float,
    ) -> Pose2D:
        """Generate a side detour via point around an obstacle as the new target.

        1. Project the obstacle onto the path to get ``closest``.
        2. Offset along the path's left normal by radius plus margin, with ``side_sign`` choosing left/right.
        3. Point the via pose back toward the original target so walking naturally returns to path.
        """
        seg_dx = target.x - start.x
        seg_dy = target.y - start.y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            return target
        dir_x = seg_dx / seg_len
        dir_y = seg_dy / seg_len
        left_x = -dir_y
        left_y = dir_x
        rel_x = obstacle.x - start.x
        rel_y = obstacle.y - start.y
        along = rel_x * dir_x + rel_y * dir_y
        closest_x = start.x + dir_x * along
        closest_y = start.y + dir_y * along
        offset = obstacle.radius + self._config.strategy.obstacle_safety_margin
        via_x = closest_x + left_x * side_sign * offset
        via_y = closest_y + left_y * side_sign * offset
        return Pose2D(via_x, via_y, math.atan2(target.y - via_y, target.x - via_x))

    # Walking control

    def _compute_velocity(
        self,
        pose: Pose2D,
        target: Pose2D,
        reason: str,
        arrive_distance: float,
        hold_vyaw: float,
    ) -> RobotCommand:
        """Unicycle-style movement: pure turning at long angles, then vx plus small vyaw when aligned.

        Output always satisfies one of:

        Pure stop: ``vx=vy=vyaw=0``
        Pure turn: ``vyaw != 0`` and ``vx=vy=0``
        Forward plus turn: ``vx > 0``, ``vy=0``, and small ``vyaw``
        """
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)
        final_theta_error = normalize_angle(target.theta - pose.theta)

        # Arrival check
        if distance < arrive_distance and abs(final_theta_error) < _ARRIVE_ANGLE:
            if abs(hold_vyaw) > 1e-6:
                return RobotCommand(
                    intent=MoveIntent(vyaw=hold_vyaw),
                    reason=f"{reason}: active hold",
                )
            return RobotCommand.stop(f"{reason}: arrived")

        # Already close but misaligned: turn in place to target theta to avoid overshooting.
        if distance < arrive_distance:
            return RobotCommand(
                intent=MoveIntent(vyaw=self._angular_velocity(final_theta_error)),
                reason=f"{reason} align",
            )

        angle_to_target = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_target - pose.theta)

        # Large angle error: pure turn instead of walking while turning.
        if abs(angle_error) > _TURN_THRESHOLD:
            return RobotCommand(
                intent=MoveIntent(vyaw=self._angular_velocity(angle_error)),
                reason=f"{reason} turn",
            )

        # Forward plus tracking turn; vx is cosine-scaled to reduce lateral drift, and vy is forced to 0.
        vx = self._linear_speed(distance, angle_error)
        vyaw = self._angular_velocity(angle_error)
        return RobotCommand(intent=MoveIntent(vx=vx, vy=0.0, vyaw=vyaw), reason=reason)

    def _linear_speed(self, distance: float, angle_error: float) -> float:
        """vx equals gain * distance * cos(err), then applies floor and max clamps."""
        if distance <= 1e-6:
            return 0.0
        raw = _LINEAR_GAIN * distance * math.cos(angle_error)
        magnitude = abs(raw)
        floor = min(_LINEAR_SPEED_FLOOR, self._config.strategy.max_linear_speed)
        if magnitude < floor:
            return floor if raw >= 0.0 else -floor
        return clamp(raw, -self._config.strategy.max_linear_speed, self._config.strategy.max_linear_speed)

    def _angular_velocity(self, angle_error: float) -> float:
        """omega equals clamp(2 * err, +/-max), then applies a floor.

        Small-angle dead zone: when error is below ``_ANGULAR_DEAD_ZONE``, skip the
        floor and use pure proportional control to avoid floor-amplified overshoot near target angle.
        """
        if abs(angle_error) <= 1e-6:
            return 0.0
        omega = clamp(
            2.0 * angle_error,
            -self._config.strategy.max_angular_speed,
            self._config.strategy.max_angular_speed,
        )
        # Inside the dead zone, use pure proportional control and do not raise to floor.
        if abs(angle_error) < _ANGULAR_DEAD_ZONE:
            return omega
        floor = min(_ANGULAR_SPEED_FLOOR, self._config.strategy.max_angular_speed)
        if abs(omega) < floor:
            return floor if omega > 0.0 else -floor
        return omega

    # Yaw avoidance

    def _apply_yaw_avoidance(
        self,
        player_id: int,
        context: PlayContext,
        command: RobotCommand,
        include_opponents: bool,
    ) -> RobotCommand:
        """Map nearby-neighbor threats to a vyaw bias added onto the command.

        A biped cannot add field-frame lateral velocity like an omni robot; instead
        it turns a bit more and walks along the new direction around the neighbor.

        ``include_opponents`` controls whether opponents count as neighbors: True
        in READY/recovery, False in PLAY to avoid pushing chasers away from opponents.
        """
        intent = command.intent
        # Apply bias only to forward MoveIntent commands; kick, pure turn, pure strafe, and stop are unchanged.
        if not isinstance(intent, MoveIntent) or abs(intent.vx) < 1e-6:
            return command

        robot = context.teammates.get(player_id)
        if robot is None or robot.pose is None:
            return command

        # Field-frame forward direction equals robot heading because vx is body-forward and vy is assumed 0.
        forward_vx = intent.vx * math.cos(robot.pose.theta)
        forward_vy = intent.vx * math.sin(robot.pose.theta)
        max_bias_per_neighbor = self._config.strategy.yaw_avoid_bias_max

        def neighbor_yaw_contribution(rel_x: float, rel_y: float) -> float:
            scale = self._yaw_avoid_scale(rel_x, rel_y, forward_vx, forward_vy)
            if scale <= 0.0:
                return 0.0
            speed = math.hypot(forward_vx, forward_vy)
            if speed <= 1e-6:
                return 0.0
            forward_unit_x = forward_vx / speed
            forward_unit_y = forward_vy / speed
            left_unit_x = -forward_unit_y
            left_unit_y = forward_unit_x
            rel_lateral = rel_x * left_unit_x + rel_y * left_unit_y
            # Neighbor on the left means turn right; directly behind is resolved by player_id parity.
            if abs(rel_lateral) < 0.05:
                side_sign = 1.0 if player_id % 2 == 0 else -1.0
            else:
                side_sign = -1.0 if rel_lateral > 0.0 else 1.0
            return side_sign * scale * max_bias_per_neighbor

        yaw_bias = 0.0
        for teammate_id, teammate in context.teammates.items():
            if teammate_id == player_id or teammate.pose is None:
                continue
            yaw_bias += neighbor_yaw_contribution(
                teammate.pose.x - robot.pose.x,
                teammate.pose.y - robot.pose.y,
            )

        if include_opponents:
            for opponent in context.opponents.values():
                if opponent.pose is None:
                    continue
                yaw_bias += neighbor_yaw_contribution(
                    opponent.pose.x - robot.pose.x,
                    opponent.pose.y - robot.pose.y,
                )

        if abs(yaw_bias) < 1e-6:
            return command

        new_vyaw = clamp(
            intent.vyaw + yaw_bias,
            -self._config.strategy.max_angular_speed,
            self._config.strategy.max_angular_speed,
        )
        return RobotCommand(
            intent=MoveIntent(vx=intent.vx, vy=intent.vy, vyaw=new_vyaw),
            reason=f"{command.reason} yaw avoid",
        )

    def _yaw_avoid_scale(
        self,
        rel_x: float,
        rel_y: float,
        field_vx: float,
        field_vy: float,
    ) -> float:
        """Compute one neighbor's avoidance-bias strength in [0, 1].

        Consider both current distance and predicted closest distance:

        Currently inside ``min_distance`` -> positive scale proportional to closeness.
        Predicted closest distance within ``horizon`` below ``min_distance`` -> positive scale for early avoidance.

        Take the max so both "already inside" and "will enter" trigger avoidance.
        With zero speed this falls back to pure distance; 0 means no current threat.
        """
        min_distance = self._config.strategy.yaw_avoid_min_distance_m
        horizon = self._config.strategy.yaw_avoid_horizon_sec
        distance = math.hypot(rel_x, rel_y)
        if min_distance <= 0.0 or horizon <= 0.0 or distance <= 1e-6:
            return 0.0

        speed_sq = field_vx * field_vx + field_vy * field_vy
        if speed_sq <= 1e-9:
            if distance >= min_distance:
                return 0.0
            return clamp((min_distance - distance) / min_distance, 0.0, 1.0)

        closing_time = clamp(
            (rel_x * field_vx + rel_y * field_vy) / speed_sq,
            0.0,
            horizon,
        )
        closest_x = rel_x - field_vx * closing_time
        closest_y = rel_y - field_vy * closing_time
        closest_distance = math.hypot(closest_x, closest_y)
        scale = 0.0
        if closest_distance < min_distance:
            scale = max(scale, (min_distance - closest_distance) / min_distance)
        if distance < min_distance:
            scale = max(scale, (min_distance - distance) / min_distance)
        return clamp(scale, 0.0, 1.0)
