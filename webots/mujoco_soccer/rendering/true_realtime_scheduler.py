from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class TrueRealtimeScheduler:
    physics_dt: float
    render_hz: float = 60.0
    decision_hz: float = 20.0
    coordination_hz: float = 10.0
    control_hz: float = 100.0
    playback_speed: float = 1.0
    max_catchup_steps: int = 10
    wall_start: float = field(default_factory=time.perf_counter)
    sim_start: float = 0.0
    next_render_wall: float = field(default_factory=time.perf_counter)
    next_decision_sim: float = 0.0
    next_coordination_sim: float = 0.0
    next_control_sim: float = 0.0
    main_loop_iterations: int = 0
    sleep_seconds: float = 0.0

    def reset(self, sim_time: float) -> None:
        now = time.perf_counter()
        self.wall_start = now
        self.next_render_wall = now
        self.sim_start = sim_time
        self.next_decision_sim = sim_time
        self.next_coordination_sim = sim_time
        self.next_control_sim = sim_time
        self.main_loop_iterations = 0
        self.sleep_seconds = 0.0

    @property
    def render_dt(self) -> float:
        return 1.0 / self.render_hz

    @property
    def decision_dt(self) -> float:
        return 1.0 / self.decision_hz

    @property
    def coordination_dt(self) -> float:
        return 1.0 / self.coordination_hz

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_hz

    def target_sim_time(self, now: float | None = None) -> float:
        wall = time.perf_counter() if now is None else now
        return self.sim_start + (wall - self.wall_start) * self.playback_speed

    def should_render(self, now: float) -> bool:
        return now >= self.next_render_wall

    def mark_rendered(self) -> None:
        self.next_render_wall += self.render_dt
        now = time.perf_counter()
        if self.next_render_wall < now - self.render_dt:
            self.next_render_wall = now

    def tiny_sleep(self) -> None:
        sleep_time = min(self.next_render_wall - time.perf_counter(), 0.002)
        if sleep_time > 0:
            time.sleep(sleep_time)
            self.sleep_seconds += sleep_time
