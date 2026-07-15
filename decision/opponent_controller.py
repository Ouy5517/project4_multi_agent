"""
黄队轻量对抗控制器
==================
不侵入蓝队 DecisionFSM：在主循环里用同一套 MockRobotAction，
让黄队主动追球 / 卡防 / 清球，形成队内传球时的对抗感。

黄队进攻方向: -X (蓝方球门 OUR_GOAL_X)
黄队防守球门: +X (GOAL_X)
"""

from __future__ import annotations

import math
from typing import List

from common.config import (
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_X,
    OUR_GOAL_X,
    ROBOT_KICK_RANGE,
)
from common.robot_action import RobotActionInterface
from common.world_state import Robot, WorldState


class YellowOpponentController:
    """每帧为黄队下发移动/踢球指令。"""

    def __init__(self, robot_action: RobotActionInterface) -> None:
        self._action = robot_action
        self._roles: dict[int, str] = {}

    @property
    def last_roles(self) -> dict[int, str]:
        return dict(self._roles)

    def update(self, ws: WorldState) -> None:
        opponents: List[Robot] = list(ws.opponents)
        if not opponents:
            self._roles = {}
            return

        ball = ws.ball
        ranked = sorted(
            opponents,
            key=lambda r: (r.x - ball.x) ** 2 + (r.y - ball.y) ** 2,
        )
        self._roles = {}

        # 1) 最近者追球，够得到就踢向蓝门
        chaser = ranked[0]
        self._roles[chaser.id] = "CHASE"
        self._drive_chaser(chaser, ball.x, ball.y, ball.vx, ball.vy)

        # 2) 第二近：预判拦截点 (球前进方向一侧切断)
        if len(ranked) >= 2:
            interceptor = ranked[1]
            self._roles[interceptor.id] = "INTERCEPT"
            tx, ty = self._intercept_point(ball.x, ball.y, ball.vx, ball.vy)
            self._action.move_to(interceptor.id, tx, ty)

        # 3) 其余：站在「球 — 黄方球门」连线上保护右门
        for defender in ranked[2:]:
            self._roles[defender.id] = "DEFEND"
            tx, ty = self._defend_yellow_goal(ball.x, ball.y)
            self._action.move_to(defender.id, tx, ty)

    def _drive_chaser(
        self, robot: Robot, bx: float, by: float, bvx: float, bvy: float
    ) -> None:
        dist = math.hypot(robot.x - bx, robot.y - by)
        if dist <= ROBOT_KICK_RANGE * 1.05:
            direction = math.atan2(0.0 - by, OUR_GOAL_X - bx)
            if self._action.kick(robot.id, 70.0, direction):
                return
        # 略超前预瞄球速
        lead_t = 0.25
        tx = bx + bvx * lead_t
        ty = by + bvy * lead_t
        self._action.move_to(robot.id, tx, ty)

    @staticmethod
    def _intercept_point(bx: float, by: float, bvx: float, bvy: float) -> tuple[float, float]:
        speed = math.hypot(bvx, bvy)
        if speed > 0.15:
            # 球速够大: 挡在球前进路径上
            ux, uy = bvx / speed, bvy / speed
            tx = bx + ux * 0.9
            ty = by + uy * 0.9
        else:
            # 静态球: 站在球与黄门之间偏前
            goal_x, goal_y = GOAL_X, 0.0
            dx, dy = bx - goal_x, by - goal_y
            dist = math.hypot(dx, dy) or 1.0
            tx = goal_x + dx * 0.55
            ty = goal_y + dy * 0.55
        return _clamp_field(tx, ty)

    @staticmethod
    def _defend_yellow_goal(bx: float, by: float) -> tuple[float, float]:
        goal_x, goal_y = GOAL_X, 0.0
        dx, dy = bx - goal_x, by - goal_y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return _clamp_field(goal_x - 0.8, goal_y)
        # 40% 从球门向球，贴近门线保护
        ratio = 0.35
        tx = goal_x + dx * ratio
        ty = goal_y + dy * ratio
        return _clamp_field(tx, ty)


def _clamp_field(x: float, y: float) -> tuple[float, float]:
    return (
        max(-FIELD_WIDTH / 2 + 0.2, min(FIELD_WIDTH / 2 - 0.2, x)),
        max(-FIELD_HEIGHT / 2 + 0.2, min(FIELD_HEIGHT / 2 - 0.2, y)),
    )
