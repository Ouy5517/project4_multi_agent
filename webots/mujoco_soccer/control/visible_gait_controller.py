from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco


GAIT_JOINTS = [
    "Left_Hip_Pitch",
    "Right_Hip_Pitch",
    "Left_Knee_Pitch",
    "Right_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Right_Ankle_Pitch",
    "Left_Hip_Roll",
    "Right_Hip_Roll",
    "Left_Shoulder_Pitch",
    "Right_Shoulder_Pitch",
    "Left_Elbow_Pitch",
    "Right_Elbow_Pitch",
]


@dataclass
class JointStats:
    samples: dict[str, list[float]] = field(default_factory=dict)
    active_seconds: float = 0.0

    def add(self, name: str, value: float) -> None:
        self.samples.setdefault(name, []).append(value)

    def amplitude(self, names: list[str]) -> float:
        values: list[float] = []
        for name in names:
            values.extend(self.samples.get(name, []))
        if not values:
            return 0.0
        return max(values) - min(values)


class VisibleGaitController:
    def __init__(self, model: mujoco.MjModel, robot: str, prefix: str, path_coupled: bool = False) -> None:
        self.model = model
        self.robot = robot
        self.prefix = prefix
        self.act_ids = {}
        self.joint_ids = {}
        for joint in GAIT_JOINTS + ["Waist", "AAHead_yaw", "Head_pitch"]:
            name = f"{prefix}_{joint}"
            self.joint_ids[joint] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.act_ids[joint] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        self.stats = JointStats()
        self.path_coupled = path_coupled
        self.phase = 0.0
        self.fade = 0.0

    def update(
        self,
        data: mujoco.MjData,
        sim_time: float,
        dt: float,
        moving: bool,
        push_pose: float = 0.0,
        motion_step: float = 0.0,
        yaw_step: float = 0.0,
    ) -> None:
        if self.path_coupled:
            step_length = 0.30
            turn_equiv = min(0.035, abs(yaw_step) * 0.10)
            if moving or push_pose > 1e-4:
                self.phase = (self.phase + 2.0 * math.pi * (motion_step + turn_equiv) / step_length) % (2.0 * math.pi)
                self.fade = min(1.0, self.fade + dt / 0.20)
            else:
                self.fade = max(0.0, self.fade - dt / 0.25)
                if self.fade <= 1e-4:
                    self.phase = 0.0
            phase = self.phase
            fade = self.fade
        else:
            phase = 2.0 * math.pi * (sim_time % 1.0)
            fade = 1.0 if moving else 0.0
        left = math.sin(phase)
        right = -left
        if self.path_coupled:
            turn_bias = min(1.0, abs(yaw_step) / max(dt, 1e-6) / math.radians(35.0))
            targets = {
                "Left_Hip_Pitch": 0.30 * left + 0.12 * push_pose,
                "Right_Hip_Pitch": 0.28 * right + 0.28 * push_pose,
                "Left_Knee_Pitch": 0.12 + 0.42 * max(0.0, -left) + 0.05 * max(0.0, right),
                "Right_Knee_Pitch": 0.13 + 0.38 * max(0.0, -right) + 0.10 * push_pose,
                "Left_Ankle_Pitch": -0.12 * left,
                "Right_Ankle_Pitch": -0.11 * right - 0.12 * push_pose,
                "Left_Hip_Roll": 0.08 * math.cos(phase) + 0.04 * turn_bias,
                "Right_Hip_Roll": -0.08 * math.cos(phase) - 0.03 * turn_bias,
                "Left_Shoulder_Pitch": 0.24 * right - 0.04 * turn_bias,
                "Right_Shoulder_Pitch": 0.24 * left - 0.16 * push_pose - 0.04 * turn_bias,
                "Left_Elbow_Pitch": 0.16 + 0.13 * max(0.0, left),
                "Right_Elbow_Pitch": 0.16 + 0.13 * max(0.0, right),
                "Waist": 0.07 * math.sin(phase) + 0.06 * push_pose,
                "AAHead_yaw": 0.04 * math.sin(phase * 0.5),
                "Head_pitch": 0.08 - 0.04 * push_pose,
            }
        else:
            targets = {
                "Left_Hip_Pitch": 0.24 * left + 0.12 * push_pose,
                "Right_Hip_Pitch": 0.24 * right + 0.28 * push_pose,
                "Left_Knee_Pitch": 0.18 + 0.32 * max(0.0, -left),
                "Right_Knee_Pitch": 0.18 + 0.32 * max(0.0, -right) + 0.10 * push_pose,
                "Left_Ankle_Pitch": -0.10 * left,
                "Right_Ankle_Pitch": -0.10 * right - 0.12 * push_pose,
                "Left_Hip_Roll": 0.06 * math.cos(phase),
                "Right_Hip_Roll": -0.06 * math.cos(phase),
                "Left_Shoulder_Pitch": 0.32 * right,
                "Right_Shoulder_Pitch": 0.32 * left - 0.16 * push_pose,
                "Left_Elbow_Pitch": 0.20 + 0.16 * max(0.0, left),
                "Right_Elbow_Pitch": 0.20 + 0.16 * max(0.0, right),
                "Waist": 0.08 * math.sin(phase),
                "AAHead_yaw": 0.05 * math.sin(phase * 0.5),
                "Head_pitch": 0.08,
            }
        if moving:
            self.stats.active_seconds += dt
        for joint, value in targets.items():
            qid = self.joint_ids[joint]
            low, high = self.model.jnt_range[qid]
            clipped = max(float(low), min(float(high), fade * value))
            data.ctrl[self.act_ids[joint]] = clipped
            qadr = self.model.jnt_qposadr[qid]
            self.stats.add(joint, float(data.qpos[qadr]))
