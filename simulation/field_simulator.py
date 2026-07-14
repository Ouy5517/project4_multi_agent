"""
2D 场地物理仿真引擎
====================
脱离 Webots/Docker 的自包含物理模拟器，支持：
- 机器人移动（匀速运动模型）
- 足球物理（带摩擦力减速）
- 踢球交互（方向和力度）
- 边界裁剪
- 碰撞检测
"""

from typing import Dict, List, Tuple, Optional
import math
from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH, GOAL_X, OUR_GOAL_X,
    ROBOT_MAX_SPEED, ROBOT_RADIUS, ROBOT_KICK_RANGE,
    BALL_FRICTION, BALL_RADIUS, BALL_MIN_VELOCITY,
    KICK_POWER_SCALE, DT, TEAM_BLUE, TEAM_YELLOW,
    NUM_ROBOTS_PER_TEAM
)
from common.world_state import Ball, Robot, Team, RobotRole


class Simulator:
    """2D 足球场物理仿真引擎"""

    def __init__(self, num_blue: int = NUM_ROBOTS_PER_TEAM,
                 num_yellow: int = NUM_ROBOTS_PER_TEAM):
        self.ball = Ball(x=0.0, y=0.0)
        self.blue_robots: Dict[int, Robot] = {}
        self.yellow_robots: Dict[int, Robot] = {}
        self.timestamp: float = 0.0
        self.tick_count: int = 0

        # 内部状态
        self._move_targets: Dict[int, Tuple[float, float]] = {}
        self._turn_targets: Dict[int, float] = {}
        self._kick_queue: List[Tuple[int, float, float]] = []  # (robot_id, power, direction)
        self._init_positions = {}  # 用于 reset

        self._initialize_positions(num_blue, num_yellow)

    # ================================================================
    # 初始化
    # ================================================================

    def _initialize_positions(self, num_blue: int, num_yellow: int):
        """设置初始开球位置"""
        # 蓝队 (己方) — 左半场，进攻方向向右 (+X)
        blue_positions = [
            (-1.0, 0.0),    # 前锋/中锋
            (-2.0, 1.5),    # 左边锋
            (-2.5, 0.0),    # 后卫
        ]
        for i in range(min(num_blue, len(blue_positions))):
            px, py = blue_positions[i]
            robot = Robot(id=i, team=Team.BLUE, x=px, y=py, theta=0.0)
            self.blue_robots[i] = robot
            self._init_positions[i] = (px, py, 0.0)

        # 黄队 (对手) — 右半场，进攻方向向左 (-X)
        yellow_positions = [
            (1.0, 0.0),     # 前锋
            (2.0, -1.5),    # 右边锋
            (2.5, 0.0),     # 后卫
        ]
        for i in range(min(num_yellow, len(yellow_positions))):
            rid = 10 + i
            px, py = yellow_positions[i]
            robot = Robot(id=rid, team=Team.YELLOW, x=px, y=py, theta=math.pi)
            self.yellow_robots[rid] = robot
            self._init_positions[rid] = (px, py, math.pi)

    # ================================================================
    # 主更新循环
    # ================================================================

    def update(self, dt: float = DT):
        """执行一个时间步长"""
        # 1. 处理踢球队列
        self._process_kicks()

        # 2. 更新机器人位置
        self._update_robots(dt)

        # 3. 更新足球物理
        self._update_ball(dt)

        # 4. 更新冷却
        self._update_cooldowns(dt)

        self.timestamp += dt
        self.tick_count += 1

    def _process_kicks(self):
        """处理待执行的踢球动作"""
        for robot_id, power, direction in self._kick_queue:
            robot = self._get_robot(robot_id)
            if robot is None:
                continue
            # 检查球是否在踢球范围内
            dist = math.sqrt((robot.x - self.ball.x)**2 + (robot.y - self.ball.y)**2)
            if dist <= ROBOT_KICK_RANGE * 1.2:
                force = power * KICK_POWER_SCALE
                self.ball.vx += math.cos(direction) * force
                self.ball.vy += math.sin(direction) * force
        self._kick_queue.clear()

    def _update_robots(self, dt: float):
        """更新机器人位置 (朝目标匀速移动)"""
        for robots_dict in [self.blue_robots, self.yellow_robots]:
            for rid, robot in robots_dict.items():
                # 转向
                if rid in self._turn_targets:
                    target_theta = self._turn_targets[rid]
                    diff = self._angle_diff(target_theta, robot.theta)
                    max_turn = ROBOT_MAX_SPEED * dt
                    if abs(diff) <= max_turn:
                        robot.theta = target_theta
                        del self._turn_targets[rid]
                    else:
                        robot.theta += math.copysign(max_turn, diff)

                # 移动
                if rid in self._move_targets:
                    tx, ty = self._move_targets[rid]
                    dx = tx - robot.x
                    dy = ty - robot.y
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist < 0.05:  # 足够近, 到达目标
                        robot.x, robot.y = tx, ty
                        del self._move_targets[rid]
                    else:
                        step = ROBOT_MAX_SPEED * dt
                        if step >= dist:
                            robot.x, robot.y = tx, ty
                        else:
                            robot.x += (dx / dist) * step
                            robot.y += (dy / dist) * step

    def _update_ball(self, dt: float):
        """更新足球物理 (运动 + 摩擦)"""
        # 位置更新
        self.ball.x += self.ball.vx * dt
        self.ball.y += self.ball.vy * dt

        # 摩擦减速
        self.ball.vx *= BALL_FRICTION
        self.ball.vy *= BALL_FRICTION

        # 停止阈值
        if abs(self.ball.vx) < BALL_MIN_VELOCITY:
            self.ball.vx = 0.0
        if abs(self.ball.vy) < BALL_MIN_VELOCITY:
            self.ball.vy = 0.0

        # 场地边界反弹 (X方向)
        if self.ball.x < -FIELD_WIDTH / 2:
            self.ball.x = -FIELD_WIDTH / 2
            self.ball.vx *= -0.5
        elif self.ball.x > FIELD_WIDTH / 2:
            self.ball.x = FIELD_WIDTH / 2
            self.ball.vx *= -0.5

        # 场地边界反弹 (Y方向)
        if self.ball.y < -FIELD_HEIGHT / 2:
            self.ball.y = -FIELD_HEIGHT / 2
            self.ball.vy *= -0.5
        elif self.ball.y > FIELD_HEIGHT / 2:
            self.ball.y = FIELD_HEIGHT / 2
            self.ball.vy *= -0.5

    def _update_cooldowns(self, dt: float):
        """更新所有机器人的冷却时间"""
        for robots_dict in [self.blue_robots, self.yellow_robots]:
            for robot in robots_dict.values():
                if robot.kick_cooldown > 0:
                    robot.kick_cooldown = max(0, robot.kick_cooldown - dt)

    # ================================================================
    # 指令接口
    # ================================================================

    def set_move_target(self, robot_id: int, x: float, y: float):
        """设置机器人移动目标"""
        self._move_targets[robot_id] = (x, y)

    def set_turn_target(self, robot_id: int, theta: float):
        """设置机器人转向目标"""
        self._turn_targets[robot_id] = theta % (2 * math.pi)

    def load_world_state(self, ws):
        """Load robot and ball positions from a WorldState"""
        for r in ws.teammates:
            if r.id in self.blue_robots:
                self.blue_robots[r.id].x = r.x
                self.blue_robots[r.id].y = r.y
                self.blue_robots[r.id].theta = r.theta
        for r in ws.opponents:
            if r.id in self.yellow_robots:
                self.yellow_robots[r.id].x = r.x
                self.yellow_robots[r.id].y = r.y
                self.yellow_robots[r.id].theta = r.theta
        self.ball.x = ws.ball.x
        self.ball.y = ws.ball.y
        self.ball.vx = ws.ball.vx
        self.ball.vy = ws.ball.vy

    def clear_move_target(self, robot_id: int):
        """清除移动目标"""
        self._move_targets.pop(robot_id, None)

    def queue_kick(self, robot_id: int, power: float, direction: float):
        """排队踢球动作"""
        self._kick_queue.append((robot_id, power, direction))

    def reset(self):
        """重置所有位置"""
        self.ball = Ball(x=0.0, y=0.0)
        self.timestamp = 0.0
        self.tick_count = 0
        self._move_targets.clear()
        self._turn_targets.clear()
        self._kick_queue.clear()
        for rid, (x, y, theta) in self._init_positions.items():
            robot = self._get_robot(rid)
            if robot:
                robot.x, robot.y, robot.theta = x, y, theta
                robot.kick_cooldown = 0.0
                robot.role = RobotRole.IDLE

    # ================================================================
    # 查询接口
    # ================================================================

    def get_ball(self) -> Ball:
        return self.ball

    def get_robots(self, team: Team) -> List[Robot]:
        """获取指定队伍的所有机器人"""
        if team == Team.BLUE:
            return list(self.blue_robots.values())
        return list(self.yellow_robots.values())

    def get_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        """根据 ID 获取机器人"""
        return self._get_robot(robot_id)

    def _get_robot(self, robot_id: int) -> Optional[Robot]:
        if robot_id in self.blue_robots:
            return self.blue_robots[robot_id]
        if robot_id in self.yellow_robots:
            return self.yellow_robots[robot_id]
        return None

    # ================================================================
    # 工具方法
    # ================================================================

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """计算最小角度差 [-pi, pi]"""
        diff = (target - current) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        return diff
