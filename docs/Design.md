# 概要设计说明书

> 项目：Booster T1 多机器人足球协同决策系统  
> 版本：v1.0

## 1. 总体架构

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────┐
│  Simulator  │────→│ WorldStateProvider│────→│ DecisionFSM │
│ (2D Mock)   │     │                  │     │             │
└─────────────┘     └──────────────────┘     └──────┬──────┘
       ↑                                            │
       │                                            ▼
       │                                    ┌───────────────┐
       │                                    │ Strategy Layer│
       │                                    │ pass/dribble/ │
       │                                    │ shoot/position│
       │                                    │ /block        │
       │                                    └───────┬───────┘
       │                                            │
       └──────────── MockRobotAction ←──────────────┘
```

## 2. 模块划分

| 层次 | 模块 | 文件 |
|------|------|------|
| 公共层 | 配置、数据结构、动作接口 | `common/` |
| 策略层 | 传球/带球/射门/跑位/卡位 | `strategy/` |
| 决策层 | 状态机、角色分配、日志 | `decision/` |
| 仿真层 | 2D 物理引擎 | `simulation/` |
| 桥接层 | Real 模式适配 | `bridge/` |
| 入口层 | 主程序、可视化 | `main.py` |

## 3. 数据流

1. `Simulator.step()` 更新物理状态
2. `WorldStateProvider.get()` 构建快照
3. `DecisionFSM.update()` 分配角色、切换状态
4. 策略模块计算目标，调用 `RobotAction`
5. `MockRobotAction` 将指令写入仿真器
6. 决策日志记录到内存，可选导出 CSV

## 4. 决策流程

```
每帧:
  1. 读取 WorldState
  2. 分配角色 (ball_carrier / supporter / defender)
  3. 对每个机器人:
     - ball_carrier: CHASE → (PASS | SHOOT | DRIBBLE)
     - supporter: POSITION (跑位)
     - defender: BLOCK (若威胁) 或 POSITION
  4. 执行策略 → RobotAction
  5. 记录 DecisionLog
```

## 5. 扩展设计

### Mock / Real 双模式

- `--mode mock`：内置仿真器 + MockRobotAction
- `--mode real`：RealWorldStateProvider + RealRobotAction（bridge 层）

### 与 Booster 官方对接

```
ROS2/SDK → bridge/ → WorldState / RobotAction → strategy/ + decision/
```

策略层不依赖 ROS2，仅通过 bridge 适配。

## 6. 目录结构

```
project4_multi_agent/
├── common/          # 公共接口
├── strategy/        # 策略模块
├── decision/        # 决策引擎
├── simulation/      # 2D 仿真
├── bridge/          # Real 模式桥接
├── tests/           # 测试
├── docs/            # 文档
├── outputs/         # 日志/CSV/视频
└── main.py          # 入口
```
