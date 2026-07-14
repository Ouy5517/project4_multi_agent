"""
带球策略模块
=============
实现机器人带球推进逻辑：
- 接近球 (从目标方向的反方向接近)
- 向指定方向带球推进 (小力度反复踢)
- 带球路径规划
"""

from typing import List, Tuple, Optional
import math
from common.config import (
    ROBOT_KICK_RANGE, ROBOT_MAX_SPEED, DRIBBLE_SPEED,
    ROBOT_RADIUS, FIELD_WIDTH, FIELD_HEIGHT
)
from common.world_state import WorldState, Robot, Ball


class DribbleStrategy:
    """带球策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    # ================================================================
    # 接近球
    # ================================================================

    def approach_ball(self, robot_id: int, target_direction: Tuple[float, float] = None
                      ) -> Tuple[bool, float, float]:
        """
        命令机器人接近球。
        返回 (是否已到达, 目标X, 目标Y)。
        从目标方向的反方向接近球 (例如如果要向右带球, 从球的左边接近)。
        """
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return (False, 0.0, 0.0)

        ball = self._ws.ball
        dist = self._ws.distance(robot, ball)

        # 已在踢球范围
        if dist <= ROBOT_KICK_RANGE:
            return (True, robot.x, robot.y)

        # 计算接近点 (球的背后, 相对于推进方向)
        if target_direction:
            approach_offset = 0.3  # 在球后方 0.3m
            tx = ball.x - math.cos(target_direction[0] if isinstance(target_direction, tuple)
                    and len(target_direction) == 2 else 0) * approach_offset
            ty = ball.y - math.sin(0) * approach_offset
        else:
            # 默认: 从左侧接近 (为向右带球做准备)
            tx = ball.x - ROBOT_KICK_RANGE * 0.8
            ty = ball.y

        return (False, tx, ty)

    # ================================================================
    # 带球推进
    # ================================================================

    def dribble_toward(self, robot_id: int, target_x: float, target_y: float
                       ) -> Tuple[bool, float, float, float]:
        """
        向目标方向带球推进。
        返回 (是否仍在带球, 踢球方向, 踢球力度, 到目标距离)。

        策略: 在球的后面, 朝目标方向小力度反复踢球。
        """
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return (False, 0.0, 0.0, 0)

        ball = self._ws.ball
        dist_to_ball = self._ws.distance(robot, ball)
        dist_to_target = math.sqrt((ball.x - target_x)**2 + (ball.y - target_y)**2)

        # 球不在控制范围内，需要先接近
        if dist_to_ball > ROBOT_KICK_RANGE * 1.5:
            return (False, 0.0, 0.0, dist_to_target)

        # 球已接近目标，停止带球
        if dist_to_target < 0.3:
            return (False, 0.0, 0.0, dist_to_target)

        # 计算踢球方向 (指向目标)
        dx = target_x - ball.x
        dy = target_y - ball.y
        direction = math.atan2(dy, dx)

        # 带球力度 (小力度, 保持控制)
        power = 15.0

        # 让机器人保持在球后面
        self._position_behind_ball(robot_id, direction)

        return (True, direction, power, dist_to_target)

    def _position_behind_ball(self, robot_id: int, kick_direction: float):
        """移动到球的后面 (相对于踢球方向)"""
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return
        ball = self._ws.ball
        # 在球的相反方向 (0.2m 后)
        behind_x = ball.x - math.cos(kick_direction) * 0.2
        behind_y = ball.y - math.sin(kick_direction) * 0.2
        return behind_x, behind_y

    # ================================================================
    # 带球路径规划
    # ================================================================

    def plan_dribble_path(self, robot_id: int, target_x: float, target_y: float
                          ) -> List[Tuple[float, float]]:
        """
        规划带球路径 (简化: 直接走向目标, 如果遇到对手则绕行)。
        """
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return []

        ball = self._ws.ball
        waypoints = []

        # 检查是否有对手阻挡直线上
        for opp in self._ws.opponents:
            d = self._point_to_segment_distance(
                (opp.x, opp.y), (ball.x, ball.y), (target_x, target_y))
            if d < ROBOT_RADIUS * 3:
                # 添加绕行点
                mid_x = (ball.x + target_x) / 2
                mid_y = (ball.y + target_y) / 2 + (1 if opp.y > 0 else -1)
                waypoints.append((mid_x, mid_y))

        waypoints.append((target_x, target_y))
        return waypoints

    # ================================================================
    # 查询
    # ================================================================

    def is_ball_controlled(self, robot_id: int) -> bool:
        """检查球是否在机器人控制范围内"""
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return False
        return self._ws.distance(robot, self._ws.ball) <= ROBOT_KICK_RANGE

    # ================================================================
    # 工具
    # ================================================================

    @staticmethod
    def _point_to_segment_distance(p, a, b):
        """点到线段距离"""
        px, py = p
        x1, y1 = a
        x2, y2 = b
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx*dx + dy*dy
        if seg_len_sq == 0:
            return math.sqrt((px-x1)**2 + (py-y1)**2)
        t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy) / seg_len_sq))
        return math.sqrt((px - (x1 + t*dx))**2 + (py - (y1 + t*dy))**2)
