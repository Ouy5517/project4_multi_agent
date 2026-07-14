from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class RealtimeClock:
    real_time_factor: float = 1.0
    start_wall: float = field(default_factory=time.perf_counter)

    def pace(self, sim_time: float) -> None:
        target_wall = self.start_wall + sim_time / max(self.real_time_factor, 1e-6)
        now = time.perf_counter()
        if target_wall > now:
            time.sleep(target_wall - now)

    def elapsed(self) -> float:
        return time.perf_counter() - self.start_wall
