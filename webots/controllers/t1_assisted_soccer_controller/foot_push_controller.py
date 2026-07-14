from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PushState:
    active: bool = False
    foot: str = "RIGHT"
    started_at: float = 0.0
    duration: float = 0.9

    def start(self, foot: str, sim_time: float, duration: float = 0.9) -> None:
        self.active = True
        self.foot = foot.upper()
        self.started_at = sim_time
        self.duration = max(0.4, float(duration))

    def fraction(self, sim_time: float) -> float:
        if not self.active:
            return 0.0
        return max(0.0, min(1.0, (sim_time - self.started_at) / self.duration))

    def update(self, sim_time: float) -> bool:
        if self.active and self.fraction(sim_time) >= 1.0:
            self.active = False
            return True
        return False
