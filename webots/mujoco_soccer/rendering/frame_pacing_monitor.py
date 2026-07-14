from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class FramePacingMonitor:
    target_fps: float = 60.0
    frame_times_ms: list[float] = field(default_factory=list)
    dropped_frames: int = 0
    render_quality_level: int = 0
    renderer_creation_count: int = 0
    video_writer_started: bool = False
    decision_overrun_count: int = 0
    _last_frame_wall: float = field(default_factory=time.perf_counter)
    _start_wall: float = field(default_factory=time.perf_counter)
    _seen_first_frame: bool = False
    present_timestamps: list[float] = field(default_factory=list)
    state_change_timestamps: list[float] = field(default_factory=list)
    state_change_count: int = 0
    unchanged_present_count: int = 0
    longest_unchanged_frame_run: int = 0
    _current_unchanged_frame_run: int = 0
    maximum_visual_state_gap_ms: float = 0.0
    _last_state_vector: np.ndarray | None = None
    _last_state_change_wall: float | None = None

    def renderer_created(self) -> None:
        self.renderer_creation_count += 1

    def record_frame(self, state_vector: np.ndarray | None = None) -> None:
        now = time.perf_counter()
        self.present_timestamps.append(now)
        dt_ms = (now - self._last_frame_wall) * 1000.0
        if not self._seen_first_frame:
            self._seen_first_frame = True
            self._last_frame_wall = now
            if state_vector is not None:
                self._last_state_vector = state_vector.copy()
                self._last_state_change_wall = now
                self.state_change_timestamps.append(now)
            return
        self.frame_times_ms.append(dt_ms)
        if len(self.frame_times_ms) > 1 and dt_ms > (1000.0 / max(1.0, self.target_fps)) * 1.5:
            self.dropped_frames += 1
        if state_vector is not None:
            self._record_state_change(now, state_vector)
        self._last_frame_wall = now

    def _record_state_change(self, now: float, state_vector: np.ndarray) -> None:
        if self._last_state_vector is None:
            self._last_state_vector = state_vector.copy()
            self._last_state_change_wall = now
            self.state_change_timestamps.append(now)
            return
        delta = float(np.linalg.norm(state_vector - self._last_state_vector))
        if delta > 1.0e-4:
            self.state_change_count += 1
            self.state_change_timestamps.append(now)
            self._current_unchanged_frame_run = 0
            if self._last_state_change_wall is not None:
                gap_ms = (now - self._last_state_change_wall) * 1000.0
                self.maximum_visual_state_gap_ms = max(self.maximum_visual_state_gap_ms, gap_ms)
            self._last_state_change_wall = now
            self._last_state_vector = state_vector.copy()
        else:
            self.unchanged_present_count += 1
            self._current_unchanged_frame_run += 1
            self.longest_unchanged_frame_run = max(
                self.longest_unchanged_frame_run,
                self._current_unchanged_frame_run,
            )

    def record_decision_time(self, seconds: float, budget_seconds: float) -> None:
        if seconds > budget_seconds:
            self.decision_overrun_count += 1

    def metrics(self, simulation_time: float, log_queue_high_watermark: int = 0) -> dict[str, float | int | bool]:
        elapsed = max(1e-9, time.perf_counter() - self._start_wall)
        values = sorted(self.frame_times_ms)
        p50 = statistics.median(values) if values else 0.0
        p95 = values[min(len(values) - 1, int(0.95 * (len(values) - 1)))] if values else 0.0
        p99 = values[min(len(values) - 1, int(0.99 * (len(values) - 1)))] if values else 0.0
        maximum = max(values) if values else 0.0
        display_freeze_count = sum(1 for value in values if value > 50.0)
        main_thread_stall_count = sum(1 for value in values if value > 100.0)
        present_elapsed = max(1e-9, self.present_timestamps[-1] - self.present_timestamps[0]) if len(self.present_timestamps) >= 2 else elapsed
        actual_present_fps = (len(self.present_timestamps) - 1) / present_elapsed if len(self.present_timestamps) >= 2 else 0.0
        change_elapsed = max(1e-9, self.state_change_timestamps[-1] - self.state_change_timestamps[0]) if len(self.state_change_timestamps) >= 2 else present_elapsed
        render_state_change_hz = self.state_change_count / change_elapsed if self.state_change_count else 0.0
        comparable_frames = max(1, len(self.present_timestamps) - 1)
        effective_motion_frame_ratio = self.state_change_count / comparable_frames
        return {
            "average_viewer_fps": len(self.frame_times_ms) / elapsed,
            "actual_present_fps": actual_present_fps,
            "render_state_change_hz": render_state_change_hz,
            "unique_visual_frame_ratio": effective_motion_frame_ratio,
            "effective_motion_frame_ratio": effective_motion_frame_ratio,
            "maximum_visual_state_gap_ms": self.maximum_visual_state_gap_ms,
            "p50_frame_ms": p50,
            "p95_frame_ms": p95,
            "p99_frame_ms": p99,
            "p95_present_interval_ms": p95,
            "p99_present_interval_ms": p99,
            "maximum_frame_ms": maximum,
            "display_freeze_count": display_freeze_count,
            "main_thread_stall_count": main_thread_stall_count,
            "longest_unchanged_frame_run": self.longest_unchanged_frame_run,
            "dropped_frames": self.dropped_frames,
            "render_quality_level": self.render_quality_level,
            "simulation_real_time_factor": simulation_time / elapsed,
            "decision_overrun_count": self.decision_overrun_count,
            "log_queue_high_watermark": log_queue_high_watermark,
            "continuous_freeze_over_1s": any(value > 1000.0 for value in values),
            "renderer_creation_count": self.renderer_creation_count,
            "view_started_video_writer": self.video_writer_started,
        }
