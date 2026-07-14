from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_soccer.physics.geometry import clamp, wrap_to_pi


@dataclass
class BaseTarget:
    x: float
    y: float
    yaw: float
    max_speed: float = 0.24
    max_yaw_rate: float = math.radians(30.0)
    acceleration_limit: float = 0.45


class PlanarBaseController:
    """Actuator-only assisted planar locomotion controller."""

    def __init__(self, model: mujoco.MjModel, robot: str, turn_first: bool = False) -> None:
        self.model = model
        self.robot = robot
        self.joint_ids = {
            axis: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_{axis}")
            for axis in ("x", "y", "yaw")
        }
        self.act_ids = {
            axis: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{robot}_base_{axis}_act")
            for axis in ("x", "y", "yaw")
        }
        self.base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base")
        self.initial_xy: tuple[float, float] | None = None
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0
        self.speed = 0.0
        self.target: BaseTarget | None = None
        self.path_length = 0.0
        self.max_continuous_motion = 0.0
        self._continuous_motion = 0.0
        self.max_turn = 0.0
        self._last_xy: tuple[float, float] | None = None
        self._start_yaw: float | None = None
        self.turn_first = turn_first
        self.last_motion_step = 0.0
        self.last_yaw_step = 0.0
        self.last_command_speed = 0.0
        self._previous_command_speed = 0.0
        self._previous_acceleration = 0.0
        self._previous_yaw_rate = 0.0
        self.max_acceleration = 0.0
        self.max_jerk = 0.0
        self.max_yaw_rate_seen = 0.0
        self.max_yaw_acceleration = 0.0
        self.abrupt_motion_count = 0

    def set_target(self, target: BaseTarget) -> None:
        self.target = target

    def pose(self, data: mujoco.MjData) -> tuple[float, float, float]:
        body_pos = data.xpos[self.base_body]
        qadr = self.model.jnt_qposadr[self.joint_ids["yaw"]]
        yaw = float(data.qpos[qadr])
        return float(body_pos[0]), float(body_pos[1]), yaw

    def update(self, data: mujoco.MjData, dt: float) -> bool:
        if self.initial_xy is None:
            x, y, yaw = self.pose(data)
            self.initial_xy = (x, y)
            self.ref_x = x
            self.ref_y = y
            self.ref_yaw = yaw
            self._start_yaw = yaw
            self._last_xy = (x, y)
        self._track_metrics(data)
        if self.target is None:
            ix, iy = self.initial_xy
            self._write_ctrl(data, self.ref_x - ix, self.ref_y - iy, self.ref_yaw)
            self.last_motion_step = 0.0
            self.last_yaw_step = 0.0
            self.last_command_speed = 0.0
            return True

        x, y, yaw = self.pose(data)
        dx = self.target.x - x
        dy = self.target.y - y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx) if dist > 0.08 else self.target.yaw
        heading_err = wrap_to_pi(desired_heading - yaw)
        turn_gate = self.turn_first and dist > 0.08 and abs(heading_err) > math.radians(20.0)
        desired_speed = min(self.target.max_speed, math.sqrt(max(0.0, 2.0 * self.target.acceleration_limit * dist)))
        if turn_gate:
            desired_speed *= 0.22
        if desired_speed > self.speed:
            self.speed = min(desired_speed, self.speed + self.target.acceleration_limit * dt)
        else:
            self.speed = max(desired_speed, self.speed - self.target.acceleration_limit * dt)
        step = min(dist, self.speed * dt)
        if dist > 1e-6:
            self.ref_x += (dx / dist) * step
            self.ref_y += (dy / dist) * step

        yaw_goal = desired_heading if self.turn_first and dist > 0.08 else self.target.yaw
        yaw_err = wrap_to_pi(yaw_goal - yaw)
        yaw_step = clamp(yaw_err, -self.target.max_yaw_rate * dt, self.target.max_yaw_rate * dt)
        self.ref_yaw += yaw_step
        self.last_motion_step = step
        self.last_yaw_step = yaw_step
        self.last_command_speed = self.speed
        self._track_command_smoothness(dt, yaw_step)

        ix, iy = self.initial_xy
        ctrl_x = clamp(self.ref_x - ix, -4.0, 4.0)
        ctrl_y = clamp(self.ref_y - iy, -3.0, 3.0)
        self._write_ctrl(data, ctrl_x, ctrl_y, self.ref_yaw)
        return dist <= 0.04 and abs(yaw_err) <= math.radians(3.0)

    def _track_command_smoothness(self, dt: float, yaw_step: float) -> None:
        inv_dt = 1.0 / max(dt, 1e-6)
        acceleration = (self.speed - self._previous_command_speed) * inv_dt
        jerk = (acceleration - self._previous_acceleration) * inv_dt
        yaw_rate = abs(yaw_step) * inv_dt
        yaw_acceleration = (yaw_rate - self._previous_yaw_rate) * inv_dt
        self.max_acceleration = max(self.max_acceleration, abs(acceleration))
        self.max_jerk = max(self.max_jerk, abs(jerk))
        self.max_yaw_rate_seen = max(self.max_yaw_rate_seen, yaw_rate)
        self.max_yaw_acceleration = max(self.max_yaw_acceleration, abs(yaw_acceleration))
        if abs(jerk) > 6.0 or abs(acceleration) > 2.5 or yaw_rate > math.radians(130.0):
            self.abrupt_motion_count += 1
        self._previous_command_speed = self.speed
        self._previous_acceleration = acceleration
        self._previous_yaw_rate = yaw_rate

    def _write_ctrl(self, data: mujoco.MjData, x: float, y: float, yaw: float) -> None:
        data.ctrl[self.act_ids["x"]] = x
        data.ctrl[self.act_ids["y"]] = y
        data.ctrl[self.act_ids["yaw"]] = yaw

    def _track_metrics(self, data: mujoco.MjData) -> None:
        x, y, yaw = self.pose(data)
        if self._last_xy is not None:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if step > 1.3e-3:
                self.path_length += step
                self._continuous_motion += step
                self.max_continuous_motion = max(self.max_continuous_motion, self._continuous_motion)
            else:
                self._continuous_motion = 0.0
        if self._start_yaw is not None:
            self.max_turn = max(self.max_turn, abs(wrap_to_pi(yaw - self._start_yaw)))
        self._last_xy = (x, y)
