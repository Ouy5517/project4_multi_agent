# 接口说明文档

> Booster T1 多机器人足球协同决策系统  
> 版本：v1.0

## 1. 系统架构

```
Simulator / RealBridge
        ↓
WorldStateProvider → WorldState
        ↓
DecisionFSM → Strategy Modules
        ↓
RobotActionInterface → MockRobotAction / RealRobotAction
        ↓
Simulator / Booster SDK
```

## 2. 核心模块

| 模块 | 路径 | 职责 |
|------|------|------|
| 配置 | `common/config.py` | 场地尺寸、速度、阈值等常量 |
| 世界状态 | `common/world_state.py` | 数据结构 + WorldStateProvider |
| 动作接口 | `common/robot_action.py` | RobotActionInterface + Mock 实现 |
| 决策引擎 | `decision/decision_fsm.py` | 状态机、角色分配、决策日志 |
| 传球策略 | `strategy/strategy_pass.py` | 传球路径、接球点、执行 |
| 带球策略 | `strategy/strategy_dribble.py` | 追球、带球推进 |
| 射门策略 | `strategy/strategy_shoot.py` | 射门条件判断与执行 |
| 跑位策略 | `strategy/strategy_position.py` | 支援跑位 |
| 卡位策略 | `strategy/strategy_block.py` | 防守卡位 |
| 仿真器 | `simulation/field_simulator.py` | 2D 物理仿真 |
| 桥接层 | `bridge/` | Real 模式适配（预留） |

## 3. 决策状态机

```
IDLE → CHASE → (DRIBBLE | PASS | SHOOT) → ...
                  ↓
                BLOCK (防守)
```

### 状态说明

| 状态 | 触发条件 | 行为 |
|------|----------|------|
| IDLE | 初始/无任务 | 等待角色分配 |
| CHASE | 离球最近 | 追球 |
| DRIBBLE | 控球且无法传/射 | 带球推进 |
| PASS | 队友位置更优 | 传球 |
| SHOOT | 射门条件满足 | 射门 |
| BLOCK | 对手威胁球门 | 卡位防守 |

### 角色分配

| 角色 | 分配规则 |
|------|----------|
| ball_carrier | 离球最近的己方机器人 |
| supporter | 其他进攻支援机器人 |
| defender | 负责防守的机器人 |

## 4. 运行入口

```powershell
# 激活虚拟环境
.\.venv\Scripts\Activate.ps1
$env:PYTHONUTF8="1"

# Mock 模式演示
python main.py --headless --duration 10
python main.py --scenario pass --duration 30 --export-csv
python main.py --scenario shoot --duration 30 --export-csv
python main.py --scenario threat --duration 30 --export-csv

# 运行测试
python -m pytest -q
```

## 5. 决策日志格式

CSV 字段：`tick, timestamp, robot_id, state, role, x, y, action, reason`

导出路径：`outputs/decision_log.csv` 或 `outputs/csv/`

## 6. 详细字段文档

- [WorldState 字段表](WorldState字段表.md)
- [RobotAction 接口说明](RobotAction接口说明.md)
