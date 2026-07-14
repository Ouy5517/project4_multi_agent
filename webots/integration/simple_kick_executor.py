#!/usr/bin/python3.10
"""
Simple Kick Executor — uses Move API + Webots physics collision for kicking.

No FancyKick, VisualKick, kShoot, or Soccer mode required.
Ball is pushed by robot foot via short forward Move command.

State machine: OBSERVE→MOVE_BEHIND→ROTATE→APPROACH→KICK→STOP→VERIFY→RECOVER
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


# ── State Machine ──

class KickState(Enum):
    OBSERVE = "OBSERVE"
    MOVE_BEHIND_BALL = "MOVE_BEHIND_BALL"
    ROTATE_TO_TARGET = "ROTATE_TO_TARGET"
    APPROACH_BALL = "APPROACH_BALL"
    KICK_FORWARD = "KICK_FORWARD"
    STOP = "STOP"
    VERIFY = "VERIFY"
    RECOVER = "RECOVER"
    DONE = "DONE"
    FAILED = "FAILED"


@dataclass
class KickConfig:
    """Safe parameters loaded from YAML."""
    approach_speed: float = 0.04
    alignment_speed: float = 0.08
    kick_speed: float = 0.08
    kick_duration: float = 0.30
    max_kick_speed: float = 0.12
    max_kick_duration: float = 0.50
    behind_ball_distance: float = 0.20
    contact_distance: float = 0.14
    yaw_tolerance: float = 0.12
    ball_move_threshold: float = 0.05
    max_attempts: int = 3
    stop_after: float = 0.5

    @classmethod
    def from_yaml(cls, path: str = None) -> "KickConfig":
        if path is None:
            path = str(Path(__file__).parent.parent / "config" / "simple_kick.yaml")
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})


@dataclass
class KickResult:
    """Record of a kick attempt."""
    robot_id: str
    target: Tuple[float, float]
    ball_init: Tuple[float, float, float]
    ball_final: Tuple[float, float, float]
    horizontal_disp: float
    direction_error: float  # radians
    success: bool
    state: KickState
    attempts: int
    reason: str
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id, "target": list(self.target),
            "ball_init": list(self.ball_init), "ball_final": list(self.ball_final),
            "horizontal_disp": round(self.horizontal_disp, 4),
            "direction_error": round(self.direction_error, 4),
            "success": self.success, "state": self.state.value,
            "attempts": self.attempts, "reason": self.reason,
            "timestamp": self.timestamp,
        }


class SimpleKickExecutor:
    """
    Kicks the ball via Move API + Webots physics collision.

    Algorithm:
    1. Compute position behind ball (ball - direction_to_target * distance)
    2. Move robot behind ball
    3. Rotate to face target
    4. Approach ball until contact_distance
    5. Short forward Move (kick)
    6. Stop immediately
    7. Verify ball displacement
    """

    def __init__(self, config: KickConfig = None, rpc_client=None):
        self.cfg = config or KickConfig()
        self.rpc = rpc_client  # Must have .call(api_id, body) method
        self.log: List[KickResult] = []

    # ── Public API ──

    def execute_kick(
        self, robot_id: str, robot_pos: Tuple[float, float],
        ball_pos: Tuple[float, float, float],
        target: Tuple[float, float],
        get_ball_pos: callable = None,
    ) -> KickResult:
        """Execute a simple kick toward target. Returns KickResult."""
        ctx = _KickContext(self, robot_id, robot_pos, ball_pos, target, get_ball_pos)

        for attempt in range(1, self.cfg.max_attempts + 1):
            ctx.attempt = attempt

            # OBSERVE
            ctx.ball_init = ctx.get_ball_pos()
            if ctx.ball_init is None:
                return ctx.fail("Cannot observe ball position")

            # MOVE_BEHIND_BALL
            behind = self._behind_ball_pos(ctx.ball_init, target)
            dist = self._dist(ctx.robot_pos, behind)
            if dist > 0.05:
                ctx.transition(KickState.MOVE_BEHIND_BALL)
                self._move_to(ctx, behind)
                ctx.update_robot_pos(behind)

            # ROTATE_TO_TARGET
            ctx.transition(KickState.ROTATE_TO_TARGET)
            target_angle = self._angle_to(ctx.robot_pos, target)
            yaw_err = abs(target_angle)
            if yaw_err > self.cfg.yaw_tolerance:
                self._rotate(ctx, target_angle)

            # APPROACH_BALL
            ball_now = ctx.get_ball_pos()
            dist_to_ball = self._dist(ctx.robot_pos, (ball_now[0], ball_now[1]))
            if dist_to_ball > self.cfg.contact_distance:
                ctx.transition(KickState.APPROACH_BALL)
                self._approach(ctx, ball_now)

            # KICK_FORWARD
            ctx.transition(KickState.KICK_FORWARD)
            kick_speed = min(self.cfg.kick_speed, self.cfg.max_kick_speed)
            kick_dur = min(self.cfg.kick_duration, self.cfg.max_kick_duration)
            self._rpc_move(kick_speed, 0.0, 0.0)
            time.sleep(kick_dur)

            # STOP
            ctx.transition(KickState.STOP)
            self._rpc_move(0.0, 0.0, 0.0)
            time.sleep(self.cfg.stop_after)

            # VERIFY
            ctx.transition(KickState.VERIFY)
            ball_final = ctx.get_ball_pos()
            if ball_final is None:
                continue

            disp = math.hypot(ball_final[0] - ctx.ball_init[0],
                              ball_final[1] - ctx.ball_init[1])
            actual_dir = math.atan2(ball_final[1] - ctx.ball_init[1],
                                    ball_final[0] - ctx.ball_init[0])
            dir_err = self._angle_diff(actual_dir, target_angle)

            if disp >= self.cfg.ball_move_threshold:
                ctx.transition(KickState.DONE)
                return ctx.success(ball_final, disp, dir_err,
                                   f"Ball moved {disp:.3f}m, dir_err={dir_err:.2f}rad")

            if attempt < self.cfg.max_attempts:
                ctx.robot_pos = (ball_now[0] - target[0] * 0.3,
                                 ball_now[1] - target[1] * 0.3)

        ctx.transition(KickState.FAILED)
        return ctx.fail(f"Ball did not move after {self.cfg.max_attempts} attempts")

    def execute_pass(self, from_robot: str, from_pos, ball_pos,
                     to_pos, get_ball_pos=None) -> KickResult:
        """Pass ball toward teammate's receive position."""
        return self.execute_kick(from_robot, from_pos, ball_pos, to_pos, get_ball_pos)

    def execute_shot(self, robot_id: str, robot_pos, ball_pos,
                     goal_center, get_ball_pos=None) -> KickResult:
        """Shoot ball toward goal."""
        return self.execute_kick(robot_id, robot_pos, ball_pos, goal_center, get_ball_pos)

    def execute_clearance(self, robot_id: str, robot_pos, ball_pos,
                          away_from_goal, get_ball_pos=None) -> KickResult:
        """Clear ball away from own goal."""
        return self.execute_kick(robot_id, robot_pos, ball_pos, away_from_goal, get_ball_pos)

    # ── Geometry helpers ──

    @staticmethod
    def _behind_ball_pos(ball_pos, target) -> Tuple[float, float]:
        """Position behind the ball (opposite side of target)."""
        dx = ball_pos[0] - target[0]
        dy = ball_pos[1] - target[1]
        d = math.hypot(dx, dy) or 1.0
        return (ball_pos[0] + dx / d * 0.20,
                ball_pos[1] + dy / d * 0.20)

    @staticmethod
    def _angle_to(from_pos, to_pos) -> float:
        return math.atan2(to_pos[1] - from_pos[1], to_pos[0] - from_pos[0])

    @staticmethod
    def _angle_diff(a, b) -> float:
        d = (a - b) % (2 * math.pi)
        return d if d <= math.pi else d - 2 * math.pi

    @staticmethod
    def _dist(a, b) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ── RPC helpers ──

    def _rpc_move(self, vx, vy, vyaw):
        if self.rpc:
            self.rpc.call(2001, json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw}))

    def _move_to(self, ctx, target):
        dist = self._dist(ctx.robot_pos, target)
        dur = min(dist / max(self.cfg.approach_speed, 0.01), 2.0)
        angle = self._angle_to(ctx.robot_pos, target)
        self._rotate(ctx, angle)
        self._rpc_move(self.cfg.approach_speed, 0.0, 0.0)
        time.sleep(dur)

    def _rotate(self, ctx, target_angle):
        yaw = target_angle
        dur = min(abs(yaw) / max(self.cfg.alignment_speed, 0.01), 2.0)
        vyaw = self.cfg.alignment_speed if yaw > 0 else -self.cfg.alignment_speed
        self._rpc_move(0.0, 0.0, vyaw)
        time.sleep(dur)

    def _approach(self, ctx, ball_pos):
        dist = self._dist(ctx.robot_pos, (ball_pos[0], ball_pos[1]))
        dur = min(dist / max(self.cfg.approach_speed, 0.01), 2.0)
        self._rpc_move(self.cfg.approach_speed, 0.0, 0.0)
        time.sleep(dur)


class _KickContext:
    """Internal mutable state during a kick sequence."""

    def __init__(self, executor, robot_id, robot_pos, ball_pos, target, get_ball_pos):
        self.exec = executor
        self.robot_id = robot_id
        self.robot_pos = robot_pos
        self.target = target
        self._get_ball = get_ball_pos or (lambda: ball_pos)
        self.ball_init = ball_pos
        self.attempt = 0
        self.state = KickState.OBSERVE

    def get_ball_pos(self):
        return self._get_ball()

    def update_robot_pos(self, pos):
        self.robot_pos = pos

    def transition(self, state: KickState):
        self.state = state

    def success(self, ball_final, disp, dir_err, reason):
        r = KickResult(
            robot_id=self.robot_id, target=self.target,
            ball_init=self.ball_init, ball_final=ball_final,
            horizontal_disp=disp, direction_error=dir_err,
            success=True, state=self.state, attempts=self.attempt, reason=reason,
        )
        self.exec.log.append(r)
        return r

    def fail(self, reason):
        r = KickResult(
            robot_id=self.robot_id, target=self.target,
            ball_init=self.ball_init, ball_final=self.ball_init,
            horizontal_disp=0.0, direction_error=0.0,
            success=False, state=self.state, attempts=self.attempt, reason=reason,
        )
        self.exec.log.append(r)
        return r
