from __future__ import annotations

from mujoco_soccer.multi_agent.shared_world_state import SharedWorldState


class TeamCoordinator:
    def intent(self, world: SharedWorldState, roles: dict[str, str]) -> dict[str, object] | None:
        if 10.0 <= world.sim_time <= 28.0:
            receiver = "T1_BLUE_2"
            r = world.robots[receiver]
            return {
                "type": "PASS_INTENT",
                "sender": "T1_BLUE_1",
                "receiver": receiver,
                "target": [r.x, r.y],
                "expected_start": world.sim_time,
                "expected_arrival": world.sim_time + 1.4,
            }
        return None

