# WorldState 字段表

> 版本：v1.0（已冻结）  
> 对应代码：`common/world_state.py`

## 概述

`WorldState` 是每帧不可变的世界快照，供决策层和策略层统一读取。蓝队为己方（`teammates`），黄队为对手（`opponents`）。

## 顶层字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `ball` | `Ball` | 足球状态 |
| `teammates` | `List[Robot]` | 己方（蓝队）机器人列表 |
| `opponents` | `List[Robot]` | 对手（黄队）机器人列表 |
| `our_goal` | `Goal` | 己方球门（左侧） |
| `opponent_goal` | `Goal` | 对方球门（右侧） |
| `field_width` | `float` | 场地宽度（米），默认 9.0 |
| `field_height` | `float` | 场地高度（米），默认 6.0 |
| `timestamp` | `float` | 仿真时间戳（秒） |

## Ball 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `x`, `y`, `z` | `float` | 位置坐标（Mock 模式 z=0） |
| `vx`, `vy`, `vz` | `float` | 速度分量 |
| `position` | `(x, y)` | 属性，2D 位置 |
| `speed` | `float` | 属性，水平速率 |
| `is_moving` | `bool` | 属性，是否在运动 |

## Robot 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | `int` | 机器人唯一 ID |
| `team` | `Team` | 队伍（BLUE / YELLOW） |
| `x`, `y`, `z` | `float` | 位置坐标 |
| `theta` | `float` | 朝向（弧度，0=东/右） |
| `role` | `RobotRole` | 角色：ball_carrier / supporter / defender / idle |
| `kick_cooldown` | `float` | 踢球冷却剩余时间 |

## Goal 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `x` | `float` | 球门中心 X 坐标 |
| `y_min`, `y_max` | `float` | 球门 Y 范围 |
| `center` | `(x, y)` | 属性，球门中心 |
| `width` | `float` | 属性，球门宽度 |

## 查询方法

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `get_robot_by_id(id)` | `Robot \| None` | 按 ID 查找机器人 |
| `all_robots()` | `List[Robot]` | 全部机器人 |
| `closest_teammate_to_ball()` | `Robot \| None` | 离球最近的队友 |
| `closest_opponent_to_ball()` | `Robot \| None` | 离球最近的对手 |
| `distance(a, b)` | `float` | 两点距离 |
| `is_in_field(x, y)` | `bool` | 坐标是否在场地内 |
| `has_possession(robot_id)` | `bool` | 机器人是否在控球范围内 |

## 预设场景

| 场景名 | 工厂函数 | 用途 |
|--------|----------|------|
| `default` | `create_default_world_state()` | 开球默认布局 |
| `pass` | `create_pass_scenario()` | 传球演示 |
| `shoot` | `create_shoot_scenario()` | 射门演示 |
| `threat` | `create_threat_scenario()` | 防守/卡位演示 |

## Mock 输入样例

```python
from common.world_state import create_pass_scenario

ws = create_pass_scenario()
# ball at (-2.0, 0.0), robot 0 持球, robot 1 在 (0.0, 1.5) 等待接球
```
