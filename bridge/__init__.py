"""
桥接层 (Bridge Layer)
=====================
对外部仿真环境（MuJoCo/Webots/实机）的接口适配层。

本目录包含:
- real_world_state.py: 从外部仿真器/ROS2 读取 WorldState
- real_robot_action.py: 通过 Booster SDK 控制真实机器人

当前状态:
- 接口冻结前: 两个模块均为桩代码 (stub), 等待项目二/三的接口确定后填入
- Mock 模式下不使用本目录
- 通过 main.py --mode real 调用

与项目二/三的接口约定参见: docs/接口冻结协议.md
"""

from bridge.real_world_state import RealWorldStateProvider
from bridge.real_robot_action import RealRobotAction

__all__ = ["RealWorldStateProvider", "RealRobotAction"]
