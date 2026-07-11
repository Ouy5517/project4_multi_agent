# RobotAction 接口说明

> 版本：v1.0（已冻结）  
> 对应代码：`common/robot_action.py`

## 概述

`RobotActionInterface` 定义机器人动作抽象接口。策略层和决策层只依赖此接口，不直接调用仿真器或 SDK。

- **Mock 模式**：`MockRobotAction` 将指令转发给 `Simulator`
- **Real 模式**：`RealRobotAction`（`bridge/real_robot_action.py`）对接外部 SDK

## 接口方法

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `move_to` | `robot_id, x, y` | `None` | 移动到目标坐标（自动裁剪到场地内） |
| `turn_to` | `robot_id, theta` | `None` | 转向指定角度（弧度） |
| `kick` | `robot_id, power, direction` | `bool` | 踢球，power 0-100，direction 弧度；成功返回 True |
| `stop` | `robot_id` | `None` | 停止移动 |
| `reset` | 无 | `None` | 重置所有机器人到初始位置 |
| `is_moving` | `robot_id` | `bool` | 查询是否正在移动（可选） |

## kick 约束

- 力度范围：`0 ~ ROBOT_KICK_POWER_MAX`（100）
- 球必须在 `ROBOT_KICK_RANGE` 范围内
- 踢球后有 `ROBOT_KICK_COOLDOWN` 冷却时间

## SDK 映射（未来扩展）

| RobotAction | Booster SDK 对应 |
|-------------|------------------|
| `move_to` | `B1LocoClient.Move(vx, vy, vyaw)` |
| `turn_to` | `B1LocoClient.Move(0, 0, vyaw)` |
| `kick` | `B1LocoClient.VisualKick()` / `Shoot()` |
| `stop` | SDK 停止接口 |

## 使用示例

```python
from simulation.field_simulator import Simulator
from common.robot_action import MockRobotAction

sim = Simulator()
action = MockRobotAction(sim)

action.move_to(0, 2.0, 0.0)
action.kick(0, power=60, direction=0.0)
action.stop(0)
```

## 扩展原则

1. 新增动作方法先在 `RobotActionInterface` 中定义
2. `MockRobotAction` 和 `RealRobotAction` 同步实现
3. 策略模块不得直接 import SDK 或 ROS2
