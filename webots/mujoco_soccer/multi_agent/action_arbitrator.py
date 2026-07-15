from __future__ import annotations

from dataclasses import dataclass

from mujoco_soccer.multi_agent.robot_agent import ACTIVE_KICKS, AgentCommand


@dataclass
class KickLock:
    owner: str | None = None
    expiry_time: float = 0.0
    action: str | None = None
    target: tuple[float, float] | None = None


class ActionArbitrator:
    def __init__(self) -> None:
        self.locks = {"blue": KickLock(), "red": KickLock()}
        self.conflicts = 0

    def arbitrate(self, sim_time: float, commands: dict[str, AgentCommand]) -> dict[str, AgentCommand]:
        result = dict(commands)
        for team, robots in {"blue": ["T1_BLUE_1", "T1_BLUE_2"], "red": ["T1_RED_1", "T1_RED_2"]}.items():
            lock = self.locks[team]
            if lock.expiry_time <= sim_time:
                lock.owner = None
            candidates = [r for r in robots if result[r].behavior in ACTIVE_KICKS and result[r].kick_action]
            if not candidates:
                continue
            owner = lock.owner if lock.owner in candidates else candidates[0]
            if len(candidates) > 1:
                self.conflicts += 1
            lock.owner = owner
            lock.expiry_time = sim_time + 1.2
            lock.action = result[owner].behavior
            lock.target = result[owner].kick_target
            for robot in candidates:
                if robot != owner:
                    cmd = result[robot]
                    cmd.kick_action = None
                    cmd.behavior = "OPEN_FOR_PASS" if team == "blue" else "BLOCK_LINE"
        return result

