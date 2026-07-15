from __future__ import annotations

import math
from dataclasses import dataclass

from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.physics.geometry import clamp, wrap_to_pi


@dataclass
class MotionStats:
    maximum_acceleration: float = 0.0
    maximum_jerk: float = 0.0
    maximum_yaw_rate: float = 0.0
    maximum_yaw_acceleration: float = 0.0
    abrupt_motion_count: int = 0
    animation_blend_interruptions: int = 0


class MotionCommandSmoother:
    def __init__(
        self,
        position_tau: float = 0.18,
        yaw_tau: float = 0.16,
        max_acceleration: float = 0.9,
        max_jerk: float = 4.0,
        max_yaw_rate: float = math.radians(90.0),
        max_yaw_acceleration: float = math.radians(180.0),
    ) -> None:
        self.position_tau = position_tau
        self.yaw_tau = yaw_tau
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        self.max_yaw_rate = max_yaw_rate
        self.max_yaw_acceleration = max_yaw_acceleration
        self.filtered: BaseTarget | None = None
        self.speed = 0.0
        self.acceleration = 0.0
        self.yaw_rate = 0.0
        self.stats = MotionStats()
        self._freeze_until = 0.0
        self._frozen_target: BaseTarget | None = None

    def smooth(self, target: BaseTarget, dt: float, sim_time: float, behavior: str, kick_action: str | None) -> BaseTarget:
        if kick_action and behavior in {"PASS", "SHOOT", "CLEAR", "DRIBBLE", "COUNTER_ATTACK", "INTERCEPT_BALL"}:
            if self._frozen_target is None or sim_time >= self._freeze_until:
                self._frozen_target = target
                self._freeze_until = sim_time + 0.35
            target = self._frozen_target
        elif sim_time >= self._freeze_until:
            self._frozen_target = None

        if self.filtered is None:
            self.filtered = target
            self.speed = min(target.max_speed, 0.08)
            return BaseTarget(target.x, target.y, target.yaw, self.speed, min(target.max_yaw_rate, self.max_yaw_rate), min(target.acceleration_limit, self.max_acceleration))

        pos_alpha = 1.0 - math.exp(-dt / max(1e-6, self.position_tau))
        yaw_alpha = 1.0 - math.exp(-dt / max(1e-6, self.yaw_tau))
        x = self.filtered.x + (target.x - self.filtered.x) * pos_alpha
        y = self.filtered.y + (target.y - self.filtered.y) * pos_alpha
        yaw = wrap_to_pi(self.filtered.yaw + wrap_to_pi(target.yaw - self.filtered.yaw) * yaw_alpha)

        desired_speed = min(target.max_speed, 0.20 if behavior in {"PRESS_BALL", "INTERCEPT_BALL"} else target.max_speed)
        desired_accel = clamp((desired_speed - self.speed) / max(dt, 1e-6), -1.5, self.max_acceleration)
        accel_delta = clamp(desired_accel - self.acceleration, -self.max_jerk * dt, self.max_jerk * dt)
        self.acceleration += accel_delta
        self.speed = max(0.0, min(target.max_speed, self.speed + self.acceleration * dt))

        desired_yaw_rate = min(target.max_yaw_rate, self.max_yaw_rate)
        yaw_accel = clamp((desired_yaw_rate - self.yaw_rate) / max(dt, 1e-6), -self.max_yaw_acceleration, self.max_yaw_acceleration)
        self.yaw_rate = max(0.0, min(self.max_yaw_rate, self.yaw_rate + yaw_accel * dt))

        jerk = abs(accel_delta / max(dt, 1e-6))
        self.stats.maximum_acceleration = max(self.stats.maximum_acceleration, abs(self.acceleration))
        self.stats.maximum_jerk = max(self.stats.maximum_jerk, jerk)
        self.stats.maximum_yaw_rate = max(self.stats.maximum_yaw_rate, self.yaw_rate)
        self.stats.maximum_yaw_acceleration = max(self.stats.maximum_yaw_acceleration, abs(yaw_accel))
        if abs(self.acceleration) > self.max_acceleration * 1.25 or jerk > self.max_jerk * 1.25:
            self.stats.abrupt_motion_count += 1

        self.filtered = BaseTarget(x, y, yaw, self.speed, max(math.radians(10.0), self.yaw_rate), min(target.acceleration_limit, self.max_acceleration))
        return self.filtered
