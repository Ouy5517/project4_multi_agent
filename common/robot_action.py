"""
RobotAction 接口定义与 Mock 实现
=================================
定义机器人动作的抽象接口，并提供 Mock 实现用于脱离仿真器测试。
实际对接 Booster SDK 时，只需替换为 RealRobotAction 实现。

SDK 对应关系 (未来扩展):
- move_to → B1LocoClient.Move(vx, vy, vyaw)
- kick    → B1LocoClient.VisualKick() / Shoot()
- turn_to → B1LocoClient.Move(0, 0, vyaw)
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional
import math
from common.events import ActionEvent
from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT,
    ROBOT_MAX_SPEED, ROBOT_KICK_RANGE, ROBOT_KICK_POWER_MAX,
    ROBOT_KICK_COOLDOWN, KICK_POWER_SCALE
)


class RobotActionInterface(ABC):
    """机器人动作抽象接口"""

    @abstractmethod
    def move_to(self, robot_id: int, x: float, y: float):
        """命令机器人向目标坐标移动"""
        ...

    @abstractmethod
    def turn_to(self, robot_id: int, theta: float):
        """命令机器人转向指定角度"""
        ...

    @abstractmethod
    def kick(self, robot_id: int, power: float, direction: float) -> bool:
        """命令机器人踢球 (power: 0-100, direction: 弧度)"""
        ...

    @abstractmethod
    def stop(self, robot_id: int):
        """停止所有运动"""
        ...

    @abstractmethod
    def reset(self):
        """重置所有机器人到初始位置"""
        ...

    def is_moving(self, robot_id: int) -> bool:
        """查询机器人是否正在移动 (默认 False, 子类按需覆写)"""
        return False


class MockRobotAction(RobotActionInterface):
    """
    Mock 动作实现
    将动作指令转发给仿真器 (Simulator)
    """

    def __init__(self, simulator):
        self._sim = simulator
        self._events = []
        self._event_seq = 0

    def drain_events(self):
        events = list(self._events)
        self._events.clear()
        return events

    def _record(self, robot_id: int, action: str, params: dict,
                accepted: bool = True, reject_code: Optional[str] = None):
        self._event_seq += 1
        event = ActionEvent(
            event_id=f"act-{self._event_seq}",
            tick=getattr(self._sim, "tick_count", 0),
            timestamp=getattr(self._sim, "timestamp", 0.0),
            robot_id=robot_id,
            action=action,
            params=params,
            accepted=accepted,
            reject_code=reject_code,
        )
        self._events.append(event)
        return event

    def move_to(self, robot_id: int, x: float, y: float):
        """移动机器人到目标位置"""
        # 裁剪到场地内
        x = max(-FIELD_WIDTH / 2, min(FIELD_WIDTH / 2, x))
        y = max(-FIELD_HEIGHT / 2, min(FIELD_HEIGHT / 2, y))
        self._sim.set_move_target(robot_id, x, y)
        return self._record(robot_id, "move_to", {"x": x, "y": y})

    def turn_to(self, robot_id: int, theta: float):
        """转向指定角度"""
        theta = theta % (2 * math.pi)
        self._sim.set_turn_target(robot_id, theta)
        return self._record(robot_id, "turn_to", {"theta": theta})

    def kick(self, robot_id: int, power: float, direction: float) -> bool:
        """踢球"""
        # 验证力度
        power = max(0, min(ROBOT_KICK_POWER_MAX, power))
        # 检查冷却
        robot = self._sim.get_robot_by_id(robot_id)
        if robot is None:
            self._record(robot_id, "kick", {"power": power, "direction": direction}, False, "ROBOT_NOT_FOUND")
            return False
        if robot.kick_cooldown > 0:
            self._record(robot_id, "kick", {"power": power, "direction": direction}, False, "KICK_COOLDOWN")
            return False
        # 检查球是否在范围内
        ball = self._sim.get_ball()
        dist = math.sqrt((robot.x - ball.x) ** 2 + (robot.y - ball.y) ** 2)
        if dist > ROBOT_KICK_RANGE:
            self._record(robot_id, "kick", {"power": power, "direction": direction}, False, "KICK_OUT_OF_RANGE")
            return False

        self._sim.queue_kick(robot_id, power, direction)
        robot.kick_cooldown = ROBOT_KICK_COOLDOWN
        self._record(robot_id, "kick", {"power": power, "direction": direction})
        return True

    def stop(self, robot_id: int):
        """停止移动"""
        self._sim.clear_move_target(robot_id)
        return self._record(robot_id, "stop", {})

    def reset(self):
        """重置仿真"""
        self._sim.reset()
        return self._record(-1, "reset", {})

    def is_moving(self, robot_id: int) -> bool:
        """查询机器人是否有活跃的移动目标"""
        return robot_id in self._sim._move_targets


class ActionCommand:
    """记录一次动作指令 (用于日志)"""

    def __init__(self, robot_id: int, action_type: str, params: dict):
        self.robot_id = robot_id
        self.type = action_type    # "move", "turn", "kick", "stop"
        self.params = params

    def __repr__(self):
        return f"Action({self.type}, robot={self.robot_id}, {self.params})"
