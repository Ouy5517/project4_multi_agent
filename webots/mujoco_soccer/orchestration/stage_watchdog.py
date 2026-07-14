from __future__ import annotations

from dataclasses import dataclass


@dataclass
class StageWatchdog:
    total_limit: float = 120.0

    def timed_out(self, sim_time: float, enter_time: float, timeout: float) -> bool:
        return sim_time - enter_time > timeout or sim_time > self.total_limit

