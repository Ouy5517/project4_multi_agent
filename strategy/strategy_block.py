"""
卡位策略模块
=============
实现防守卡位与拦截：
- 防守站位 (站在球与己方球门连线之间)
- 对手路径预判
- 拦截路径计算
- 威胁检测
"""

from typing import List, Tuple, Optional
import math
from common.config import (
    BLOCK_DISTANCE, OPPONENT_THREAT_RANGE,
    FIELD_WIDTH, FIELD_HEIGHT, ROBOT_RADIUS,
    GOAL_WIDTH, ROBOT_MAX_SPEED
)
from common.world_state import WorldState, Robot, Ball


class BlockStrategy:
    """卡位策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    # ================================================================
    # 防守站位
    # ================================================================

    def calculate_defensive_position(self, robot_id: int) -> Tuple[float, float]:
        """
        计算防守站位:
        站在 '球 — 己方球门' 连线上, 保护球门方向。
        """
        ball = self._ws.ball
        goal_center = self._ws.our_goal.center

        # 球到己方球门的向量
        dx = ball.x - goal_center[0]
        dy = ball.y - goal_center[1]
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < 0.1:
            return (goal_center[0] + 0.5, goal_center[1])

        # 在球和目标之间, 离球更近
        ratio = 0.4  # 40% 从目标向球
        target_x = goal_center[0] + dx * ratio
        target_y = goal_center[1] + dy * ratio

        # 裁剪
        target_x = max(-FIELD_WIDTH/2, min(FIELD_WIDTH/2, target_x))
        target_y = max(-FIELD_HEIGHT/2, min(FIELD_HEIGHT/2, target_y))

        return (target_x, target_y)

    # ================================================================
    # 对手封锁
    # ================================================================

    def calculate_block_position(self, robot_id: int,
                                  opponent_id: int) -> Tuple[float, float]:
        """
        针对特定对手的站位。
        站在对手与己方球门之间。
        """
        opp = self._ws.get_robot_by_id(opponent_id)
        if opp is None:
            return self.calculate_defensive_position(robot_id)

        goal_center = self._ws.our_goal.center
        dx = opp.x - goal_center[0]
        dy = opp.y - goal_center[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.1:
            return (goal_center[0] + 1, goal_center[1])

        # 在对手和球门之间, 靠近对手
        ratio = 0.0  # 正好在对手面前
        target_x = goal_center[0] + dx * ratio + (dx / dist) * (-BLOCK_DISTANCE)
        target_y = goal_center[1] + dy * ratio + (dy / dist) * (-BLOCK_DISTANCE)

        target_x = max(-FIELD_WIDTH/2, min(FIELD_WIDTH/2, target_x))
        target_y = max(-FIELD_HEIGHT/2, min(FIELD_HEIGHT/2, target_y))

        return (target_x, target_y)

    # ================================================================
    # 拦截预判
    # ================================================================

    def predict_opponent_path(self, opponent_id: int,
                               steps: int = 5) -> List[Tuple[float, float]]:
        """预判对手未来路径 (假设匀速直线)"""
        opp = self._ws.get_robot_by_id(opponent_id)
        if opp is None:
            return []

        path = []
        # 简单预判: 对手朝球移动
        ball = self._ws.ball
        dx = ball.x - opp.x
        dy = ball.y - opp.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.01:
            return [(opp.x, opp.y)] * steps

        for i in range(steps):
            t = i * 0.2  # 每 0.2s
            tx = opp.x + (dx / dist) * ROBOT_MAX_SPEED * t
            ty = opp.y + (dy / dist) * ROBOT_MAX_SPEED * t
            path.append((tx, ty))

        return path

    # ================================================================
    # 威胁检测
    # ================================================================

    def is_goal_threatened(self) -> bool:
        """
        检查对方是否对球门构成威胁。
        条件: 对手在己方半场控球, 且距离球门较近。
        使用 ws.our_goal, 蓝/黄视角通用。
        """
        closest_opp = self._ws.closest_opponent_to_ball()
        if closest_opp is None:
            return False

        if self._ws.distance(closest_opp, self._ws.ball) > ROBOT_RADIUS * 2:
            return False

        our_gx = self._ws.our_goal.x
        # 与己方球门同侧半场视为己方半场
        if closest_opp.x * our_gx <= 0:
            return False

        dist_to_goal = math.sqrt(
            (closest_opp.x - our_gx)**2 + closest_opp.y**2)
        return dist_to_goal < OPPONENT_THREAT_RANGE

    def get_threat_level(self) -> float:
        """
        获取当前威胁等级 (0-1)。
        """
        if not self.is_goal_threatened():
            return 0.0

        closest_opp = self._ws.closest_opponent_to_ball()
        if closest_opp is None:
            return 0.0

        our_gx = self._ws.our_goal.x
        dist_to_goal = math.sqrt(
            (closest_opp.x - our_gx)**2 + closest_opp.y**2)

        threat = 1.0 - min(dist_to_goal / OPPONENT_THREAT_RANGE, 1.0)
        return threat
