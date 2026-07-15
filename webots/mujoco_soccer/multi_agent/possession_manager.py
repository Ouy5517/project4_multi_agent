from __future__ import annotations

import math


class PossessionManager:
    def __init__(self) -> None:
        self.state = "FREE"
        self.confidence = 0.0
        self.last_owner: str | None = None
        self.last_team: str | None = None
        self.possession_since = 0.0
        self.changes = 0
        self.contested_count = 0

    def update(self, sim_time: float, ball_xy: tuple[float, float], robot_xy: dict[str, tuple[float, float]], contacts: list[dict[str, object]]) -> None:
        if contacts:
            robot = str(contacts[-1]["robot"])
            team = "BLUE" if "BLUE" in robot else "RED"
            new_state = f"{team}_CONTROL"
            if new_state != self.state and self.state != "FREE":
                self.changes += 1
            self.state = new_state
            self.last_owner = robot
            self.last_team = team.lower()
            self.confidence = 0.9
            self.possession_since = sim_time
        close = [r for r, xy in robot_xy.items() if math.hypot(xy[0] - ball_xy[0], xy[1] - ball_xy[1]) < 0.55]
        teams = {"BLUE" if "BLUE" in r else "RED" for r in close}
        if len(close) >= 2 and len(teams) > 1:
            if self.state != "CONTESTED":
                self.contested_count += 1
            self.state = "CONTESTED"
            self.confidence = 0.55
        elif self.confidence > 0.05:
            self.confidence *= 0.995

