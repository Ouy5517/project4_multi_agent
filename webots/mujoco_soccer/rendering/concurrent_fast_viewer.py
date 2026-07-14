from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from mujoco_soccer.rendering.frame_pacing_monitor import FramePacingMonitor
from mujoco_soccer.rendering.realtime_clock import RealtimeClock
from mujoco_soccer.strategy.world_state_adapter import ROBOTS


@dataclass
class ConcurrentFastViewer:
    model: mujoco.MjModel
    data: mujoco.MjData
    camera: str = "broadcast_wide"
    target_fps: float = 60.0
    real_time_factor: float = 1.0

    def __post_init__(self) -> None:
        self.handle: Any | None = None
        self.opened = False
        self.clock = RealtimeClock(self.real_time_factor)
        self.monitor = FramePacingMonitor(self.target_fps)
        self._final_metrics: dict[str, float | int | bool] | None = None
        self._steps_per_render = 1.0 / (self.target_fps * float(self.model.opt.timestep))
        self._render_step_accumulator = 0.0
        self._last_present_step = 0
        self._base_bodies: list[int] = []
        self._yaw_joints: list[int] = []
        self._sample_joints: list[int] = []
        self._ball_body = -1

    @property
    def render_interval_steps(self) -> int:
        return max(1, round(self._steps_per_render))

    def open(self) -> None:
        import mujoco.viewer

        self.handle = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self.monitor.renderer_created()
        self.opened = True
        self._cache_state_ids()
        self._configure_camera()

    def _cache_state_ids(self) -> None:
        self._base_bodies = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base") for robot in ROBOTS]
        self._yaw_joints = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_yaw") for robot in ROBOTS]
        self._ball_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        sample_names = [
            "BLUE1_Left_Hip_Pitch",
            "BLUE2_Left_Hip_Pitch",
            "RED1_Left_Hip_Pitch",
            "RED2_Left_Hip_Pitch",
            "BLUE1_Right_Knee_Pitch",
            "BLUE2_Right_Knee_Pitch",
            "RED1_Right_Knee_Pitch",
            "RED2_Right_Knee_Pitch",
        ]
        self._sample_joints = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in sample_names
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        ]

    def _configure_camera(self) -> None:
        if self.handle is None:
            return
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        if cam_id >= 0:
            self.handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.handle.cam.fixedcamid = cam_id
        if hasattr(self.handle, "show_left_ui"):
            self.handle.show_left_ui = False
        if hasattr(self.handle, "show_right_ui"):
            self.handle.show_right_ui = False
        if hasattr(self.handle, "opt"):
            self.handle.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
            self.handle.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0

    def present(self, sim_time: float) -> None:
        if self.handle is None:
            return
        self.handle.sync()
        self.monitor.record_frame(self._state_vector())

    def sync(self, step_count: int, sim_time: float) -> None:
        if self.handle is None:
            return
        self._render_step_accumulator += 1.0
        if self._render_step_accumulator + 1e-9 >= self._steps_per_render:
            self.present(sim_time)
            self._render_step_accumulator -= self._steps_per_render
            self._last_present_step = step_count
        self.clock.pace(sim_time)

    def is_running(self) -> bool:
        if self.handle is None:
            return False
        if hasattr(self.handle, "is_running"):
            return bool(self.handle.is_running())
        return True

    def _state_vector(self) -> np.ndarray:
        values: list[float] = []
        for body in self._base_bodies:
            pos = self.data.xpos[body]
            values.extend([float(pos[0]), float(pos[1])])
        for joint in self._yaw_joints:
            qadr = self.model.jnt_qposadr[joint]
            values.append(float(self.data.qpos[qadr]))
        for joint in self._sample_joints:
            qadr = self.model.jnt_qposadr[joint]
            values.append(float(self.data.qpos[qadr]))
        ball = self.data.xpos[self._ball_body]
        values.extend([float(ball[0]), float(ball[1]), float(ball[2])])
        return np.asarray(values, dtype=np.float64)

    def close(self) -> None:
        self._final_metrics = self.monitor.metrics(float(self.data.time))
        if self.handle is not None:
            self.handle.close()
            self.handle = None

    def metrics(self, log_queue_high_watermark: int = 0) -> dict[str, float | int | bool]:
        if self._final_metrics is not None:
            return dict(self._final_metrics)
        return self.monitor.metrics(float(self.data.time), log_queue_high_watermark)
