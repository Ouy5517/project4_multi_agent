from __future__ import annotations

import math
from dataclasses import dataclass


def distance_xy(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class RobotTrack:
    path_length_m: float = 0.0
    current_segment_m: float = 0.0
    maximum_continuous_motion_m: float = 0.0
    maximum_turn_deg: float = 0.0
    last_xy: list[float] | None = None
    last_yaw: float | None = None
    initial_yaw: float | None = None

    def update(self, xy: list[float], yaw: float) -> None:
        if self.last_xy is not None:
            step = distance_xy(xy, self.last_xy)
            self.path_length_m += step
            if step > 1e-5:
                self.current_segment_m += step
                self.maximum_continuous_motion_m = max(self.maximum_continuous_motion_m, self.current_segment_m)
            else:
                self.current_segment_m = 0.0
        if self.last_yaw is not None:
            self.maximum_turn_deg = max(self.maximum_turn_deg, abs(math.degrees(wrap_angle(yaw - self.last_yaw))))
        if self.initial_yaw is None:
            self.initial_yaw = yaw
        self.maximum_turn_deg = max(self.maximum_turn_deg, abs(math.degrees(wrap_angle(yaw - self.initial_yaw))))
        self.last_xy = list(xy)
        self.last_yaw = yaw


class AssistedLocomotion:
    def __init__(self, timestep_s: float, field_x: float = 3.15, field_y: float = 2.15) -> None:
        self.timestep_s = timestep_s
        self.field_x = field_x
        self.field_y = field_y
        self.tracks: dict[str, RobotTrack] = {}
        self.relative_offsets: dict[str, list[float]] = {}
        self.root_heights: dict[str, float] = {}
        self.max_step_m = 0.0

    def register(self, name: str, robot_node, assistance_node) -> None:
        robot_pos = robot_node.getField("translation").getSFVec3f()
        assist_pos = assistance_node.getField("translation").getSFVec3f()
        self.relative_offsets[name] = [assist_pos[i] - robot_pos[i] for i in range(3)]
        self.root_heights[name] = robot_pos[2]
        yaw = self.yaw(robot_node)
        self.tracks[name] = RobotTrack(last_xy=[robot_pos[0], robot_pos[1]], last_yaw=yaw, initial_yaw=yaw)

    def yaw(self, node) -> float:
        rot = node.getField("rotation").getSFRotation()
        return float(rot[3] if abs(rot[2]) >= 0.5 else 0.0)

    def pose(self, node) -> tuple[list[float], float]:
        pos = node.getField("translation").getSFVec3f()
        return pos, self.yaw(node)

    def step_toward(self, name: str, robot_node, assistance_node, target_xy: list[float], target_yaw: float, speed: float) -> bool:
        pos_field = robot_node.getField("translation")
        rot_field = robot_node.getField("rotation")
        pos = pos_field.getSFVec3f()
        yaw = self.yaw(robot_node)
        target = [
            max(-self.field_x, min(self.field_x, float(target_xy[0]))),
            max(-self.field_y, min(self.field_y, float(target_xy[1]))),
        ]
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.hypot(dx, dy)
        max_step = max(0.0, float(speed)) * self.timestep_s
        if dist > 1e-9:
            step = min(dist, max_step)
            nx = pos[0] + dx / dist * step
            ny = pos[1] + dy / dist * step
        else:
            step = 0.0
            nx, ny = pos[0], pos[1]
        yaw_error = wrap_angle(target_yaw - yaw)
        max_yaw_step = math.radians(35.0) * self.timestep_s
        yaw_step = max(-max_yaw_step, min(max_yaw_step, yaw_error))
        new_yaw = yaw + yaw_step
        root_z = self.root_heights.get(name, pos[2])
        pos_field.setSFVec3f([nx, ny, root_z])
        rot_field.setSFRotation([0.0, 0.0, 1.0, new_yaw])
        try:
            robot_node.resetPhysics()
        except Exception:
            pass
        offset = self.relative_offsets[name]
        assistance_node.getField("translation").setSFVec3f([nx + offset[0], ny + offset[1], root_z + offset[2]])
        assistance_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, new_yaw])
        self.max_step_m = max(self.max_step_m, step)
        self.tracks[name].update([nx, ny], new_yaw)
        return dist < 0.03 and abs(math.degrees(wrap_angle(yaw_error))) < 3.0

    def relative_error(self, name: str, robot_node, assistance_node) -> float:
        robot_pos = robot_node.getField("translation").getSFVec3f()
        assist_pos = assistance_node.getField("translation").getSFVec3f()
        offset = self.relative_offsets[name]
        expected = [robot_pos[i] + offset[i] for i in range(3)]
        return math.sqrt(sum((assist_pos[i] - expected[i]) ** 2 for i in range(3)))
