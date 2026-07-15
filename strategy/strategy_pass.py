"""
传球策略模块
=============
实现两机器人之间的传球逻辑：
- 传球路线可行性判断（障碍物检测）
- 传球方向和力度计算
- 最佳接球队友选择
"""

from typing import List, Optional, Tuple
import math
from common.config import (
    PASS_MIN_DISTANCE, PASS_MAX_DISTANCE, ROBOT_RADIUS,
    ROBOT_KICK_RANGE, OPPONENT_THREAT_RANGE
)
from common.world_state import WorldState, Robot, Ball, RobotRole


class PassStrategy:
    """传球策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        """更新世界状态引用"""
        self._ws = ws

    # ================================================================
    # 传球可行性判断
    # ================================================================

    def is_pass_path_clear(self, from_robot: Robot, to_robot: Robot) -> bool:
        """
        判断传球路线是否被对手阻挡。
        计算每个对手到传球线段的距离，判断是否有对手在路上。
        """
        for opp in self._ws.opponents:
            if opp.id == to_robot.id:
                continue
            d = self._point_to_segment_distance(
                (opp.x, opp.y),
                (from_robot.x, from_robot.y),
                (to_robot.x, to_robot.y)
            )
            if d < ROBOT_RADIUS * 2.5:
                # 检查对手是否在传球路径上 (而不是在后面)
                if self._is_between(from_robot, to_robot, opp):
                    return False
        return True

    def can_pass(self, from_robot: Robot, to_robot: Robot) -> bool:
        """综合判断是否能传球"""
        dist = self._ws.distance(from_robot, to_robot)
        if dist < PASS_MIN_DISTANCE or dist > PASS_MAX_DISTANCE:
            return False
        return self.is_pass_path_clear(from_robot, to_robot)

    # ================================================================
    # 传球参数计算
    # ================================================================

    def calculate_pass_direction(self, from_pos: Tuple[float, float],
                                  to_pos: Tuple[float, float]) -> float:
        """计算传球方向角 (弧度)"""
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        return math.atan2(dy, dx)

    def calculate_pass_power(self, distance: float) -> float:
        """
        根据距离计算传球力度 (0-100)。
        力度映射: PASS_MIN_DISTANCE → 30, PASS_MAX_DISTANCE → 80
        """
        ratio = (distance - PASS_MIN_DISTANCE) / (PASS_MAX_DISTANCE - PASS_MIN_DISTANCE)
        ratio = max(0, min(1, ratio))
        return 30 + ratio * 50

    def calculate_receive_point(self, receiver: Robot, ball_velocity: Tuple[float, float]
                                 ) -> Tuple[float, float]:
        """
        预测接球点。
        简化实现: 预测球在 0.5 秒后的位置。
        """
        vx, vy = ball_velocity
        return (receiver.x + vx * 0.5, receiver.y + vy * 0.5)

    # ================================================================
    # 接球队友选择
    # ================================================================

    def evaluate_pass_options(self, passer_id: int) -> List['PassOption']:
        """
        评估所有潜在接球队友。
        返回按评分降序排列的传球选项列表。
        """
        passer = self._ws.get_robot_by_id(passer_id)
        if passer is None:
            return []

        options = []
        for teammate in self._ws.teammates:
            if teammate.id == passer_id:
                continue

            dist = self._ws.distance(passer, teammate)
            if dist < PASS_MIN_DISTANCE or dist > PASS_MAX_DISTANCE:
                continue

            path_clear = self.is_pass_path_clear(passer, teammate)
            is_open = self._is_robot_open(teammate)

            # 计算评分
            score = self._score_pass_option(passer, teammate, dist, path_clear, is_open)

            options.append(PassOption(
                receiver_id=teammate.id,
                score=score,
                target_x=teammate.x,
                target_y=teammate.y,
                distance=dist,
                is_clear=path_clear
            ))

        options.sort(key=lambda o: o.score, reverse=True)
        return options

    def find_best_receiver(self, passer_id: int) -> Optional['PassOption']:
        """选择最佳接球队友"""
        options = self.evaluate_pass_options(passer_id)
        return options[0] if options else None

    # ================================================================
    # 传球执行
    # ================================================================

    def execute_pass(self, passer_id: int, receiver_id: int) -> Tuple[bool, float, float]:
        """
        执行传球，返回 (是否成功发起, 方向, 力度)。
        注意: 实际踢球由调用方通过 robot_action.kick() 执行。
        """
        passer = self._ws.get_robot_by_id(passer_id)
        receiver = self._ws.get_robot_by_id(receiver_id)

        if passer is None or receiver is None:
            return (False, 0.0, 0.0)

        if not self.can_pass(passer, receiver):
            return (False, 0.0, 0.0)

        direction = self.calculate_pass_direction(
            (passer.x, passer.y), (receiver.x, receiver.y))
        dist = self._ws.distance(passer, receiver)
        power = self.calculate_pass_power(dist)

        return (True, direction, power)

    # ================================================================
    # 内部辅助方法
    # ================================================================

    def _is_robot_open(self, robot: Robot) -> bool:
        """检查机器人是否 '空位' (附近没有对手)"""
        for opp in self._ws.opponents:
            if self._ws.distance(robot, opp) < OPPONENT_THREAT_RANGE:
                return False
        return True

    def _score_pass_option(self, passer: Robot, receiver: Robot,
                           dist: float, path_clear: bool, is_open: bool) -> float:
        """
        传球选项评分 (0.0 - 1.0):
        - 路径畅通: +0.3
        - 接球者空位: +0.25
        - 向前推进: +0.25 (接球者比对方向前)
        - 距离合适: +0.2 (中等距离最好)
        """
        score = 0.0
        if path_clear:
            score += 0.3
        if is_open:
            score += 0.25
        # 向前推进
        if receiver.x > passer.x:
            score += 0.25
        # 距离评分 (钟形, 最优在 2-3m)
        optimal_dist = 2.5
        dist_score = 1.0 - min(abs(dist - optimal_dist) / PASS_MAX_DISTANCE, 1.0)
        score += dist_score * 0.2

        return score

    @staticmethod
    def _point_to_segment_distance(point: Tuple[float, float],
                                    seg_start: Tuple[float, float],
                                    seg_end: Tuple[float, float]) -> float:
        """计算点到线段的最短距离"""
        px, py = point
        x1, y1 = seg_start
        x2, y2 = seg_end

        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx*dx + dy*dy

        if seg_len_sq == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)

        t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    @staticmethod
    def _is_between(p1: Robot, p2: Robot, target: Robot) -> bool:
        """检查 target 是否在 p1-p2 之间 (点积法)"""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dot = (target.x - p1.x) * dx + (target.y - p1.y) * dy
        return 0 < dot < (dx*dx + dy*dy)


class PassOption:
    """传球选项"""
    def __init__(self, receiver_id: int, score: float, target_x: float,
                 target_y: float, distance: float, is_clear: bool):
        self.receiver_id = receiver_id
        self.score = score
        self.target_x = target_x
        self.target_y = target_y
        self.distance = distance
        self.is_clear = is_clear

    def __repr__(self):
        return (f"PassOption(to={self.receiver_id}, score={self.score:.2f}, "
                f"dist={self.distance:.1f}m, clear={self.is_clear})")
