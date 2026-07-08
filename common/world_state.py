"""
WorldState 数据结构与提供者
============================
定义足球场世界状态的完整数据模型，包括：
- Ball: 足球的位置和速度
- Robot: 机器人的位置、朝向、角色、队伍
- Goal: 球门位置
- WorldState: 完整世界快照
- WorldStateProvider: 从仿真器读取数据并提供给决策层
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
from enum import Enum
import math
from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH,
    GOAL_X, OUR_GOAL_X, TEAM_BLUE, TEAM_YELLOW,
    ROBOT_KICK_RANGE
)


class Team(Enum):
    """队伍枚举"""
    BLUE = TEAM_BLUE
    YELLOW = TEAM_YELLOW


class RobotRole(Enum):
    """机器人角色"""
    BALL_CARRIER = "ball_carrier"   # 持球者
    SUPPORTER = "supporter"          # 支援者
    DEFENDER = "defender"            # 防守者
    IDLE = "idle"                    # 空闲


@dataclass
class Ball:
    """足球状态"""
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)

    @property
    def speed(self) -> float:
        return math.sqrt(self.vx ** 2 + self.vy ** 2)

    @property
    def is_moving(self) -> bool:
        from common.config import BALL_MIN_VELOCITY
        return self.speed > BALL_MIN_VELOCITY


@dataclass
class Robot:
    """机器人状态"""
    id: int
    team: Team
    x: float
    y: float
    theta: float = 0.0              # 朝向角度 (弧度, 0=右/东)
    role: RobotRole = RobotRole.IDLE
    kick_cooldown: float = 0.0      # 踢球冷却剩余时间

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)


@dataclass
class Goal:
    """球门"""
    x: float                        # 球门中心 X
    y_min: float                    # 球门底部 Y
    y_max: float                    # 球门顶部 Y

    @property
    def center(self) -> Tuple[float, float]:
        return (self.x, (self.y_min + self.y_max) / 2)

    @property
    def width(self) -> float:
        return self.y_max - self.y_min


@dataclass
class WorldState:
    """
    完整世界状态快照 (每帧不可变)
    蓝队 = 己方 (teammates), 黄队 = 对手 (opponents)
    """
    ball: Ball
    teammates: List[Robot]          # 蓝队机器人
    opponents: List[Robot]          # 黄队机器人
    our_goal: Goal                  # 蓝队球门 (左)
    opponent_goal: Goal             # 黄队球门 (右)
    field_width: float = FIELD_WIDTH
    field_height: float = FIELD_HEIGHT
    timestamp: float = 0.0

    # --- 查询方法 ---

    def get_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        """根据 ID 查找机器人"""
        for r in self.teammates + self.opponents:
            if r.id == robot_id:
                return r
        return None

    def all_robots(self) -> List[Robot]:
        """所有机器人"""
        return self.teammates + self.opponents

    def closest_teammate_to_ball(self) -> Optional[Robot]:
        """离球最近的己方机器人"""
        if not self.teammates:
            return None
        return min(self.teammates, key=lambda r: self._dist(r, self.ball))

    def closest_opponent_to_ball(self) -> Optional[Robot]:
        """离球最近的对手"""
        if not self.opponents:
            return None
        return min(self.opponents, key=lambda r: self._dist(r, self.ball))

    def distance(self, a, b) -> float:
        """
        计算两点距离
        a, b 可以是 Robot, Ball, 或 (x, y) 元组
        """
        p1 = (a.x, a.y) if hasattr(a, 'x') else a
        p2 = (b.x, b.y) if hasattr(b, 'x') else b
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def is_in_field(self, x: float, y: float) -> bool:
        """检查坐标是否在场地内"""
        return (
            -self.field_width / 2 <= x <= self.field_width / 2 and
            -self.field_height / 2 <= y <= self.field_height / 2
        )

    def has_possession(self, robot_id: int) -> bool:
        """检查机器人是否控制球 (在踢球范围内)"""
        robot = self.get_robot_by_id(robot_id)
        if robot is None:
            return False
        return self.distance(robot, self.ball) <= ROBOT_KICK_RANGE

    @staticmethod
    def _dist(a, b) -> float:
        """内部距离计算"""
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


class WorldStateProvider:
    """
    WorldState 提供者
    从仿真器读取数据，构建 WorldState 快照
    """

    def __init__(self, simulator=None):
        """
        Args:
            simulator: Simulator 实例 (实际运行时) 或 None (Mock 模式)
        """
        self._sim = simulator
        self._mock_ws = None

    def get(self) -> WorldState:
        """获取当前世界状态"""
        if self._sim is not None:
            return self._from_simulator()
        elif self._mock_ws is not None:
            ws = self._mock_ws
            ws.timestamp += 1.0 / 30.0  # 模拟时间推进
            return ws
        else:
            return create_default_world_state()

    def set_mock(self, ws: WorldState):
        """设置 Mock 世界状态 (用于测试)"""
        self._mock_ws = ws

    def _from_simulator(self) -> WorldState:
        """从仿真器构建 WorldState"""
        sim = self._sim
        # 从仿真器读取原始数据
        ball_raw = sim.get_ball()
        blue_raw = sim.get_robots(Team.BLUE)
        yellow_raw = sim.get_robots(Team.YELLOW)

        ball = Ball(x=ball_raw.x, y=ball_raw.y,
                    vx=ball_raw.vx, vy=ball_raw.vy)

        teammates = [Robot(id=r.id, team=Team.BLUE,
                           x=r.x, y=r.y, theta=r.theta,
                           role=r.role, kick_cooldown=r.kick_cooldown)
                     for r in blue_raw]

        opponents = [Robot(id=r.id, team=Team.YELLOW,
                           x=r.x, y=r.y, theta=r.theta,
                           role=RobotRole.IDLE, kick_cooldown=r.kick_cooldown)
                     for r in yellow_raw]

        return WorldState(
            ball=ball,
            teammates=teammates,
            opponents=opponents,
            our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            timestamp=sim.timestamp
        )


# ============================================================
# Mock 数据工厂
# ============================================================

def create_default_world_state() -> WorldState:
    """创建默认开球 WorldState"""
    return WorldState(
        ball=Ball(x=0.0, y=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=-1.0, y=0.0, theta=0.0),
            Robot(id=1, team=Team.BLUE, x=-2.0, y=1.5, theta=0.0),
            Robot(id=2, team=Team.BLUE, x=-2.5, y=0.0, theta=0.0),
        ],
        opponents=[
            Robot(id=10, team=Team.YELLOW, x=1.0, y=0.0, theta=3.14),
            Robot(id=11, team=Team.YELLOW, x=2.0, y=-1.5, theta=3.14),
            Robot(id=12, team=Team.YELLOW, x=2.5, y=0.0, theta=3.14),
        ],
        our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
        opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
    )


def create_pass_scenario() -> WorldState:
    """传球场景: 0号持球, 1号在有利位置等待接球"""
    return WorldState(
        ball=Ball(x=-2.0, y=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=-2.0, y=0.2, theta=0.0),
            Robot(id=1, team=Team.BLUE, x=0.0, y=1.5, theta=1.57),
            Robot(id=2, team=Team.BLUE, x=-3.5, y=0.0, theta=0.0),
        ],
        opponents=[
            Robot(id=10, team=Team.YELLOW, x=3.0, y=0.0, theta=3.14),
            Robot(id=11, team=Team.YELLOW, x=1.5, y=-2.0, theta=3.14),
            Robot(id=12, team=Team.YELLOW, x=3.5, y=0.0, theta=3.14),
        ],
        our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
        opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
    )


def create_shoot_scenario() -> WorldState:
    """射门场景: 0号在对方半场, 持球可射门"""
    return WorldState(
        ball=Ball(x=2.5, y=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=2.5, y=0.2, theta=0.0),
            Robot(id=1, team=Team.BLUE, x=1.0, y=1.5, theta=0.0),
            Robot(id=2, team=Team.BLUE, x=-1.0, y=0.0, theta=0.0),
        ],
        opponents=[
            Robot(id=10, team=Team.YELLOW, x=3.5, y=1.0, theta=3.14),
            Robot(id=11, team=Team.YELLOW, x=2.0, y=-2.0, theta=3.14),
            Robot(id=12, team=Team.YELLOW, x=4.0, y=0.0, theta=3.14),
        ],
        our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
        opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
    )


def create_threat_scenario() -> WorldState:
    """防守威胁场景: 对手持球接近己方球门"""
    return WorldState(
        ball=Ball(x=-3.0, y=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=-1.0, y=0.0, theta=3.14),
            Robot(id=1, team=Team.BLUE, x=-2.0, y=1.5, theta=3.14),
            Robot(id=2, team=Team.BLUE, x=-3.5, y=0.0, theta=3.14),
        ],
        opponents=[
            Robot(id=10, team=Team.YELLOW, x=-3.0, y=0.2, theta=3.14),
            Robot(id=11, team=Team.YELLOW, x=1.0, y=-1.5, theta=3.14),
            Robot(id=12, team=Team.YELLOW, x=2.0, y=0.0, theta=3.14),
        ],
        our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
        opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
    )


# 场景注册表
SCENARIOS = {
    "default": create_default_world_state,
    "pass": create_pass_scenario,
    "shoot": create_shoot_scenario,
    "threat": create_threat_scenario,
}
