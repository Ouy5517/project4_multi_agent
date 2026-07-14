from __future__ import annotations

from dataclasses import dataclass

import mujoco

from mujoco_soccer.control.planar_base_controller import BaseTarget, PlanarBaseController
from mujoco_soccer.control.visible_gait_controller import VisibleGaitController


PREFIXES = {
    "T1_BLUE_1": "BLUE1",
    "T1_BLUE_2": "BLUE2",
    "T1_RED_1": "RED1",
    "T1_RED_2": "RED2",
}


@dataclass
class RobotController:
    name: str
    base: PlanarBaseController
    gait: VisibleGaitController
    moving: bool = False
    push_pose: float = 0.0

    @classmethod
    def create(
        cls,
        model: mujoco.MjModel,
        name: str,
        visual_v2: bool = False,
        turn_first: bool | None = None,
    ) -> "RobotController":
        use_turn_first = visual_v2 if turn_first is None else turn_first
        return cls(
            name=name,
            base=PlanarBaseController(model, name, turn_first=use_turn_first),
            gait=VisibleGaitController(model, name, PREFIXES[name], path_coupled=visual_v2),
        )

    def set_target(self, target: BaseTarget) -> None:
        self.base.set_target(target)
        self.moving = True

    def update(self, data: mujoco.MjData, sim_time: float, dt: float) -> bool:
        arrived = self.base.update(data, dt)
        self.moving = not arrived or abs(self.push_pose) > 1e-4
        self.gait.update(
            data,
            sim_time,
            dt,
            self.moving,
            self.push_pose,
            motion_step=self.base.last_motion_step,
            yaw_step=self.base.last_yaw_step,
        )
        return arrived
