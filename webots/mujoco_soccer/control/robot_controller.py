from __future__ import annotations

from dataclasses import dataclass, field

import mujoco

from mujoco_soccer.control.kick_swing import KickSwingState, style_from_action
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
    kick_swing: KickSwingState = field(default_factory=KickSwingState)
    _prev_speed: float = 0.0

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

    def start_kick_swing(self, action: str | None = None, style: str | None = None) -> None:
        """启动踢球肢体动画 (pass/shoot=kick, dribble= dribble)"""
        resolved = style or style_from_action(action)
        self.kick_swing.start(resolved)

    def update(self, data: mujoco.MjData, sim_time: float, dt: float) -> bool:
        arrived = self.base.update(data, dt)
        self.kick_swing.step(dt)

        # 兼容: 外部设置的 push_pose 与踢腿相位包络取较大值
        swing_push = self.kick_swing.push_pose_equivalent()
        effective_push = max(self.push_pose, swing_push)

        speed = float(self.base.last_command_speed)
        # 减速 → 刹车姿态 (速度明显下降且仍在动)
        braking = 0.0
        if self._prev_speed > 0.04 and speed < self._prev_speed - 0.01:
            braking = min(1.0, (self._prev_speed - speed) / max(0.08, self._prev_speed))
        elif not arrived and speed < 0.06 and self.base.target is not None:
            # 接近目标缓行
            x, y, _ = self.base.pose(data)
            dist = ((self.base.target.x - x) ** 2 + (self.base.target.y - y) ** 2) ** 0.5
            if dist < 0.45:
                braking = min(1.0, (0.45 - dist) / 0.45)
        self._prev_speed = speed

        self.moving = (not arrived) or abs(effective_push) > 1e-4 or self.kick_swing.active
        self.gait.update(
            data,
            sim_time,
            dt,
            self.moving,
            effective_push,
            motion_step=self.base.last_motion_step,
            yaw_step=self.base.last_yaw_step,
            kick_offsets=self.kick_swing.joint_offsets(),
            braking=braking,
        )
        return arrived
