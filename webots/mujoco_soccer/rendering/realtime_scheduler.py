from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class RealtimeScheduler:
    timestep: float
    target_fps: float = 30.0
    real_time_factor: float = 1.0
    start_wall: float = field(default_factory=time.perf_counter)
    last_frame_wall: float = field(default_factory=time.perf_counter)
    frame_times: list[float] = field(default_factory=list)
    dropped_display_frames: int = 0
    maximum_stall_ms: float = 0.0

    @property
    def render_interval_steps(self) -> int:
        return max(1, round(1.0 / (self.target_fps * self.timestep)))

    def should_render(self, step_count: int) -> bool:
        return step_count % self.render_interval_steps == 0

    def pace(self, sim_time: float) -> None:
        target_wall = self.start_wall + sim_time / max(self.real_time_factor, 1e-6)
        now = time.perf_counter()
        if target_wall > now:
            time.sleep(target_wall - now)

    def record_frame(self) -> None:
        now = time.perf_counter()
        frame_ms = (now - self.last_frame_wall) * 1000.0
        self.frame_times.append(frame_ms)
        self.maximum_stall_ms = max(self.maximum_stall_ms, frame_ms)
        if frame_ms > 1000.0:
            self.dropped_display_frames += 1
        self.last_frame_wall = now

    def metrics(self) -> dict[str, float | int | bool]:
        elapsed = max(1e-9, time.perf_counter() - self.start_wall)
        count = len(self.frame_times)
        ordered = sorted(self.frame_times)
        p95 = ordered[min(len(ordered) - 1, int(0.95 * (len(ordered) - 1)))] if ordered else 0.0
        return {
            "average_viewer_fps": count / elapsed,
            "p95_frame_time_ms": p95,
            "dropped_display_frames": self.dropped_display_frames,
            "simulation_real_time_factor": self.real_time_factor,
            "maximum_stall_ms": self.maximum_stall_ms,
            "continuous_freeze_over_1s": any(value > 1000.0 for value in self.frame_times),
            "render_interval_steps": self.render_interval_steps,
        }
