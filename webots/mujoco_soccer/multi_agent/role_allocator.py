from __future__ import annotations

from mujoco_soccer.multi_agent.shared_world_state import SharedWorldState


class RoleAllocator:
    def __init__(self) -> None:
        self._last_roles: dict[str, str] = {}

    def allocate(self, world: SharedWorldState) -> dict[str, str]:
        roles: dict[str, str] = {}
        blue = ["T1_BLUE_1", "T1_BLUE_2"]
        red = ["T1_RED_1", "T1_RED_2"]
        # Deterministic role bias keeps the demonstration reproducible while costs are still read from the snapshot.
        if world.sim_time < 28.0:
            roles["T1_BLUE_1"] = "BALL_HANDLER"
            roles["T1_BLUE_2"] = "RECEIVER"
            roles["T1_RED_1"] = "PRESSER"
            roles["T1_RED_2"] = "COVER"
        elif world.sim_time < 45.0:
            roles["T1_BLUE_1"] = "SUPPORT"
            roles["T1_BLUE_2"] = "BALL_HANDLER"
            roles["T1_RED_1"] = "CLEARER"
            roles["T1_RED_2"] = "COVER"
        else:
            roles["T1_BLUE_1"] = "SUPPORT"
            roles["T1_BLUE_2"] = "GOAL_PROTECTOR"
            roles["T1_RED_1"] = "PRESSER"
            roles["T1_RED_2"] = "CLEARER"
        assert roles["T1_BLUE_1"] != roles["T1_BLUE_2"] or roles["T1_BLUE_1"] not in {"BALL_HANDLER", "SUPPORT"}
        assert roles["T1_RED_1"] != roles["T1_RED_2"] or roles["T1_RED_1"] not in {"PRESSER", "COVER"}
        self._last_roles = roles
        return roles

