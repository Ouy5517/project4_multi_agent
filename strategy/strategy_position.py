"""
跑位策略模块
=============
计算支援/接应/散开站位：
- 支援接应站位 (在持球者后方形成三角)
- 空当位置寻找
- 散开站位 (覆盖场地宽度)
"""

from typing import List, Tuple, Optional
import math
from common.config import (
    SUPPORT_DISTANCE, FIELD_WIDTH, FIELD_HEIGHT,
    ROBOT_RADIUS, GOAL_X
)
from common.world_state import WorldState, Robot


class PositionStrategy:
    """跑位策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    # ================================================================
    # 支援站位
    # ================================================================

    def calculate_support_position(self, ball_carrier_id: int,
                                    robot_id: int) -> Tuple[float, float]:
        """
        计算支援接应位置。
        策略：在持球者后方约45度位置，形成三角进攻阵型。
        """
        carrier = self._ws.get_robot_by_id(ball_carrier_id)
        if carrier is None:
            return self.calculate_default_position(robot_id)

        ball = self._ws.ball

        # 向量: 持球者 → 对方球门
        goal_x = GOAL_X
        dx_goal = goal_x - carrier.x
        dy_goal = 0 - carrier.y  # 指向球门中心

        # 根据机器人编号决定是左支援还是右支援
        if robot_id % 2 == 0:
            # 上方支援
            offset_x = -0.5
            offset_y = SUPPORT_DISTANCE
        else:
            # 下方支援
            offset_x = -0.5
            offset_y = -SUPPORT_DISTANCE

        target_x = carrier.x + offset_x
        target_y = carrier.y + offset_y

        # 裁剪到场地内
        target_x = max(-FIELD_WIDTH/2, min(FIELD_WIDTH/2, target_x))
        target_y = max(-FIELD_HEIGHT/2, min(FIELD_HEIGHT/2, target_y))

        return (target_x, target_y)

    # ================================================================
    # 空当位置
    # ================================================================

    def calculate_open_space(self, robot_id: int) -> Tuple[float, float]:
        """
        寻找空当位置。
        策略：在对方半场寻找远离对手的开放区域。
        """
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return (0, 0)

        # 在对方半场采样，找对手密度最低的位置
        best_pos = (GOAL_X * 0.3, 0)
        best_score = -1

        for x_ratio in [0.2, 0.4, 0.6]:
            for y_ratio in [-0.6, -0.3, 0, 0.3, 0.6]:
                sx = FIELD_WIDTH / 2 * x_ratio
                sy = FIELD_HEIGHT / 2 * y_ratio

                # 距离最近对手的距离作为分数
                min_opp_dist = float('inf')
                for opp in self._ws.opponents:
                    d = math.sqrt((sx - opp.x)**2 + (sy - opp.y)**2)
                    min_opp_dist = min(min_opp_dist, d)

                if min_opp_dist > best_score:
                    best_score = min_opp_dist
                    best_pos = (sx, sy)

        return best_pos

    # ================================================================
    # 默认站位
    # ================================================================

    def calculate_default_position(self, robot_id: int) -> Tuple[float, float]:
        """根据 ID 和球位置计算默认站位"""
        ball = self._ws.ball

        # 在球和己方球门之间的位置
        if robot_id == 0:
            return (ball.x + 0.5, ball.y)
        elif robot_id == 1:
            return (ball.x - 1.0, ball.y + 1.5)
        else:
            return (ball.x - 1.5, ball.y - 0.5)

    def calculate_spread_positions(self) -> dict:
        """
        散开站位。
        将支援者分布在场地宽度上。
        """
        positions = {}
        supporters = [r for r in self._ws.teammates
                      if r.role.value == "supporter"]

        if not supporters:
            return positions

        spread = FIELD_HEIGHT * 0.6
        for i, robot in enumerate(supporters):
            t = i / max(len(supporters) - 1, 1)
            y = -spread / 2 + t * spread
            x = max(-FIELD_WIDTH/2 + 1, self._ws.ball.x - 2)
            positions[robot.id] = (x, y)

        return positions
