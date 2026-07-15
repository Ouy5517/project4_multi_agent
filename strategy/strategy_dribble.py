"""
带球策略模块
=============
接近球采用 Booster Chase: kickDir + direct / circle-back。
"""

from typing import List, Tuple, Optional
import math
from common.config import (
    ROBOT_KICK_RANGE, ROBOT_MAX_SPEED, DRIBBLE_SPEED,
    ROBOT_RADIUS, FIELD_WIDTH, FIELD_HEIGHT
)
from common.world_state import WorldState, Robot, Ball
from strategy.booster_skills import (
    adjust_behind_ball,
    calc_kick_dir,
    chase_approach_point,
    should_enter_kick,
)


class DribbleStrategy:
    """带球策略"""

    def __init__(self, world_state: WorldState):
        self._ws = world_state
        self._last_approach_mode = "direct"

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    def approach_ball(
        self,
        robot_id: int,
        target_direction: Optional[float] = None,
    ) -> Tuple[bool, float, float]:
        """返回 (是否已到达可踢/可对准区, 目标X, 目标Y)。"""
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return (False, 0.0, 0.0)

        ball = self._ws.ball
        if target_direction is None:
            kick_dir, _ = calc_kick_dir(self._ws)
        else:
            kick_dir = float(target_direction)

        dist = self._ws.distance(robot, ball)
        if dist <= ROBOT_KICK_RANGE or should_enter_kick(robot, ball.x, ball.y, kick_dir):
            tx, ty = adjust_behind_ball(ball.x, ball.y, kick_dir)
            self._last_approach_mode = "adjust"
            return (True, tx, ty)

        tx, ty, mode = chase_approach_point(robot, ball.x, ball.y, kick_dir)
        self._last_approach_mode = mode
        return (False, tx, ty)

    def dribble_toward(self, robot_id: int, target_x: float, target_y: float
                       ) -> Tuple[bool, float, float, float]:
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return (False, 0.0, 0.0, 0)

        ball = self._ws.ball
        dist_to_ball = self._ws.distance(robot, ball)
        dist_to_target = math.sqrt((ball.x - target_x)**2 + (ball.y - target_y)**2)

        if dist_to_ball > ROBOT_KICK_RANGE * 1.5:
            return (False, 0.0, 0.0, dist_to_target)
        if dist_to_target < 0.3:
            return (False, 0.0, 0.0, dist_to_target)

        direction = math.atan2(target_y - ball.y, target_x - ball.x)
        power = 15.0
        self._position_behind_ball(robot_id, direction)
        return (True, direction, power, dist_to_target)

    def _position_behind_ball(self, robot_id: int, kick_direction: float):
        ball = self._ws.ball
        return adjust_behind_ball(ball.x, ball.y, kick_direction)

    def plan_dribble_path(self, robot_id: int, target_x: float, target_y: float
                          ) -> List[Tuple[float, float]]:
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return []
        ball = self._ws.ball
        waypoints = []
        for opp in self._ws.opponents:
            d = self._point_to_segment_distance(
                (opp.x, opp.y), (ball.x, ball.y), (target_x, target_y))
            if d < ROBOT_RADIUS * 3:
                mid_x = (ball.x + target_x) / 2
                mid_y = (ball.y + target_y) / 2 + (1 if opp.y > 0 else -1)
                waypoints.append((mid_x, mid_y))
        waypoints.append((target_x, target_y))
        return waypoints

    def is_ball_controlled(self, robot_id: int) -> bool:
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return False
        return self._ws.distance(robot, self._ws.ball) <= ROBOT_KICK_RANGE

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
        return math.sqrt((px - (x1 + t*dx))**2 + (py - (y1 + t*dy))**2)
