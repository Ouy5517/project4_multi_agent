from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco

from mujoco_soccer.rendering.realtime_scheduler import RealtimeScheduler


@dataclass
class FastVisualViewer:
    model: mujoco.MjModel
    data: mujoco.MjData
    camera: str = "broadcast_wide"
    target_fps: float = 30.0
    real_time_factor: float = 1.0

    def __post_init__(self) -> None:
        self.handle: Any | None = None
        self.scheduler = RealtimeScheduler(float(self.model.opt.timestep), self.target_fps, self.real_time_factor)
        self.opened = False
        self._final_metrics: dict[str, float | int | bool] | None = None

    @property
    def render_interval_steps(self) -> int:
        return self.scheduler.render_interval_steps

    def open(self) -> None:
        import mujoco.viewer

        self.handle = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self.opened = True
        self._configure_camera()
        self.scheduler = RealtimeScheduler(float(self.model.opt.timestep), self.target_fps, self.real_time_factor)

    def _configure_camera(self) -> None:
        if self.handle is None:
            return
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        if cam_id < 0:
            return
        self.handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.handle.cam.fixedcamid = cam_id
        if hasattr(self.handle, "show_left_ui"):
            self.handle.show_left_ui = False
        if hasattr(self.handle, "show_right_ui"):
            self.handle.show_right_ui = False

    def sync(self, step_count: int, sim_time: float) -> None:
        if self.handle is None:
            return
        if self.scheduler.should_render(step_count):
            self.handle.sync()
            self.scheduler.record_frame()
        self.scheduler.pace(sim_time)

    def close(self) -> None:
        self._final_metrics = self.scheduler.metrics()
        if self.handle is not None:
            self.handle.close()
            self.handle = None

    def metrics(self) -> dict[str, float | int | bool]:
        if self._final_metrics is not None:
            return self._final_metrics
        return self.scheduler.metrics()
