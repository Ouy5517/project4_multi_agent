"""
射门策略模块
=============
实现射门方向和力度计算：
- 射门机会评估 (距离、角度、阻挡检测)
- 射门方向计算 (找球门空当)
- 力度自适应
"""

from typing import List, Tuple, Optional
import math
from common.config import (
    SHOOT_RANGE, ROBOT_KICK_RANGE, GOAL_WIDTH,
    ROBOT_RADIUS, ROBOT_KICK_POWER_MAX
)
from common.world_state import WorldState, Robot, Goal


class ShootStrategy:
    """射门策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    # ================================================================
    # 射门机会评估
    # ================================================================

    def evaluate_shoot_opportunity(self, robot_id: int) -> 'ShootEvaluation':
        """综合评估射门机会"""
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return ShootEvaluation(False, 0, 0, 0, 0)

        # 距离检查
        goal = self._ws.opponent_goal
        dist_to_goal = math.sqrt(
            (robot.x - goal.x)**2 + (robot.y - goal.center[1])**2)

        if dist_to_goal > SHOOT_RANGE:
            return ShootEvaluation(False, 0, 0, 0, 0)

        # 检查是否控制球
        if not self._ws.has_possession(robot_id):
            return ShootEvaluation(False, 0, 0, 0, 0)

        # 找最佳射门点
        direction, best_y = self._find_best_shot_angle(robot, goal)
        blockers = self._find_blockers(robot, direction)

        # 计算力度
        power = self.calculate_shoot_power(dist_to_goal)

        # 评分
        score = self._score_shot(robot, dist_to_goal, blockers, direction)

        is_viable = score > 0.3 and len(blockers) < 2

        return ShootEvaluation(
            is_viable=is_viable,
            best_angle=direction,
            best_target_y=best_y,
            power=power,
            score=score,
            blocked_by=blockers
        )

    # ================================================================
    # 射门参数计算
    # ================================================================

    def calculate_shoot_direction(self, robot_id: int) -> float:
        """计算最佳射门角度"""
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return 0.0

        goal = self._ws.opponent_goal
        direction, _ = self._find_best_shot_angle(robot, goal)
        return direction

    def calculate_shoot_power(self, distance: float) -> float:
        """
        根据距离计算射门力度 (0-100)。
        距离越远力度越大，但不过度。
        """
        if distance <= 1.0:
            return 40  # 近距离不用力过大
        elif distance >= SHOOT_RANGE:
            return 80
        else:
            ratio = (distance - 1.0) / (SHOOT_RANGE - 1.0)
            return 40 + ratio * 40

    # ================================================================
    # 射门执行
    # ================================================================

    def execute_shoot(self, robot_id: int) -> Tuple[bool, float, float]:
        """
        执行射门，返回 (是否发起, 方向, 力度)。
        """
        evaluation = self.evaluate_shoot_opportunity(robot_id)
        if not evaluation.is_viable:
            return (False, 0.0, 0.0)

        return (True, evaluation.best_angle, evaluation.power)

    # ================================================================
    # 内部方法
    # ================================================================

    def _find_best_shot_angle(self, robot: Robot, goal: Goal) -> Tuple[float, float]:
        """
        寻找最佳射门角度。
        在球门线上采样多个点, 选择无阻挡或阻挡最少的。
        """
        samples = 5
        best_direction = 0
        best_y = goal.center[1]
        best_score = -1

        for i in range(samples):
            t = i / (samples - 1)
            target_y = goal.y_min + t * goal.width
            dx = goal.x - robot.x
            dy = target_y - robot.y
            direction = math.atan2(dy, dx)

            # 检查这条线路上有多少阻挡
            blockers = self._count_blockers_on_line(robot, direction, goal)
            score = 1.0 / (1 + len(blockers))

            # 优先中间区域
            center_bias = 1.0 - abs(target_y - goal.center[1]) / (goal.width / 2)
            score += center_bias * 0.3

            if score > best_score:
                best_score = score
                best_direction = direction
                best_y = target_y

        return (best_direction, best_y)

    def _find_blockers(self, robot: Robot, direction: float) -> List[int]:
        """找出射门路线上的阻挡对手"""
        return self._count_blockers_on_line(robot, direction, self._ws.opponent_goal)

    def _count_blockers_on_line(self, robot: Robot, direction: float, goal: Goal) -> List[int]:
        """统计射门路线上的对手"""
        blockers = []
        line_end_x = goal.x
        line_end_y = goal.center[1]

        # 模拟射门路线
        ray_len = 10.0
        ray_end_x = robot.x + math.cos(direction) * ray_len
        ray_end_y = robot.y + math.sin(direction) * ray_len

        for opp in self._ws.opponents:
            d = self._point_to_segment_distance(
                (opp.x, opp.y),
                (robot.x, robot.y),
                (ray_end_x, ray_end_y)
            )
            if d < ROBOT_RADIUS * 2:
                # 确保对手在射门路线上 (不是身后)
                dot = (opp.x - robot.x) * (ray_end_x - robot.x) + \
                      (opp.y - robot.y) * (ray_end_y - robot.y)
                if dot > 0:
                    blockers.append(opp.id)

        return blockers

    def _score_shot(self, robot: Robot, distance: float,
                    blockers: List[int], direction: float) -> float:
        """射门评分 (0-1)"""
        score = 1.0

        # 距离惩罚
        if distance > SHOOT_RANGE * 0.7:
            dist_factor = (SHOOT_RANGE - distance) / (SHOOT_RANGE * 0.3)
            score *= max(0.2, dist_factor)

        # 阻挡惩罚
        score *= max(0.1, 1.0 - len(blockers) * 0.4)

        return score

    @staticmethod
    def _point_to_segment_distance(p, a, b):
        px, py = p
        x1, y1 = a
        x2, y2 = b
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx*dx + dy*dy
        if seg_len_sq == 0:
            return math.sqrt((px-x1)**2 + (py-y1)**2)
        t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy) / seg_len_sq))
        return math.sqrt((px-(x1+t*dx))**2 + (py-(y1+t*dy))**2)


class ShootEvaluation:
    """射门评估结果"""
    def __init__(self, is_viable: bool, best_angle: float, best_target_y: float,
                 power: float, score: float, blocked_by: List[int] = None):
        self.is_viable = is_viable
        self.best_angle = best_angle
        self.best_target_y = best_target_y
        self.power = power
        self.score = score
        self.blocked_by = blocked_by or []

    def __repr__(self):
        return (f"ShootEval(viable={self.is_viable}, score={self.score:.2f}, "
                f"power={self.power:.0f}, blocked_by={self.blocked_by})")
