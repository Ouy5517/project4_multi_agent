# Booster T1 多机器人足球协同决策系统

## 项目概述

**题目四：Booster T1 多机器人足球协同决策系统**

基于 Booster T1 机器人，采用软件工程思想，实现多机器人之间的传球、带球、跑位、卡位和简单攻防决策。

### 核心功能
- 基于 WorldState 读取球、队友、对手和球门位置
- 两机器人之间的固定点传球
- 带球推进 (Dribble)
- 射门决策
- 跑位与卡位策略
- 决策状态机 (CHASE → DRIBBLE → PASS → SHOOT → BLOCK)
- 角色分配 (持球者/支援者/防守者)
- 决策过程日志

## 项目结构

```
project4_multi_agent/
├── common/                     # 公共接口
│   ├── config.py               # 配置参数
│   ├── world_state.py          # WorldState 数据结构
│   └── robot_action.py         # RobotAction 接口
├── strategy/                   # 策略模块
│   ├── strategy_pass.py        # 传球策略
│   ├── strategy_dribble.py     # 带球策略
│   ├── strategy_shoot.py       # 射门策略
│   ├── strategy_position.py    # 跑位策略
│   └── strategy_block.py       # 卡位策略
├── decision/                   # 决策引擎
│   └── decision_fsm.py         # 决策状态机
├── simulation/                 # 仿真引擎
│   └── field_simulator.py      # 2D 物理仿真
├── tests/                      # 测试
│   └── test_*.py               # 单元测试 & 集成测试
├── main.py                     # 主程序
├── run.sh                      # 一键运行
├── README.md                   # 本文档
└── outputs/                    # 日志输出
```

## 快速开始

### 环境要求
- Python 3.8+
- 可选: pytest, matplotlib (用于高级可视化和测试)

### 运行演示

```bash
# 一键启动 (30秒默认演示)
bash run.sh

# 自定义时长
bash run.sh 60

# 运行测试
bash run.sh test

# 启动轻量 MuJoCo 2.5D 综合演示（传球、带球、跑位、卡位、2v2 攻防）
source "$HOME/.venvs/robocup-p4/bin/activate"
bash run.sh view

# 安装开发依赖
bash run.sh install
```

### 直接使用 Python

```bash
# 默认演示
python3 main.py

# 无渲染模式 (仅日志)
python3 main.py --headless --duration 60

# 导出决策日志 CSV
python3 main.py --export-csv --duration 30

# 查看帮助
python3 main.py --help
```

## 系统架构

### 数据流

```
Simulator → WorldStateProvider → DecisionFSM → Strategy Modules
    ↑                                              ↓
    └────────── RobotAction (Mock) ←───────────────┘
```

### 决策状态机

```
        ┌──────┐
        │ IDLE │
        └──┬───┘
           │ role=ball_carrier
           ▼
        ┌───────┐
   ┌───→│ CHASE │←────────────────────┐
   │    └──┬┬──┘                      │
   │      ││ ball reached             │
   │      │└────────────┐             │
   │      ▼              ▼            │
   │  ┌───────┐     ┌───────┐         │
   │  │DRIBBLE│     │ PASS  │         │
   │  └───┬───┘     └───┬───┘         │
   │      │              │             │
   │      ▼              │    complete │
   │  ┌───────┐          │             │
   │  │ SHOOT │←─────────┘             │
   │  └───┬───┘                        │
   │      │ complete                   │
   │      └────────────────────────────┘
   │
   │  (role=defender, threat detected)
   │      ┌───────┐
   └──────│ BLOCK │
          └───────┘
```

### 模块依赖

```
config.py ← world_state.py ← field_simulator.py ← robot_action.py
                ↑                    ↑                    ↑
                └── strategy_*.py ───┴── decision_fsm.py ── main.py
```

## 小组分工

| 序号 | 角色 | 负责模块 | 关键产出 |
|------|------|----------|----------|
| 1 | 项目负责人 | 总体协调 | 项目计划、会议记录、答辩 |
| 2 | 需求与文档 | 需求分析+文档 | SRS、验收标准 |
| 3 | 架构与接口 | WorldState 设计 | world_state.py、接口文档 |
| 4 | 传球策略 | 传球模块 | strategy_pass.py |
| 5 | 带球+射门 | 带球+射门 | strategy_dribble.py, strategy_shoot.py |
| 6 | 跑位+卡位 | 跑位+卡位 | strategy_position.py, strategy_block.py |
| 7 | 决策引擎 | 状态机 | decision_fsm.py |
| 8 | 测试 | 测试验证 | 测试报告 (≥15条用例) |
| 9 | 集成演示 | 系统集成 | main.py, run.sh, README.md |

## 最低验收标准

- [x] 两个机器人基于统一 WorldState 完成一次有效传球
- [x] 带球推进功能可用
- [x] 决策过程日志完整输出

## 技术栈

- Python 3.8+
- 标准库: dataclasses, csv, json, argparse, math, enum
- 可选: matplotlib (可视化), pytest (测试)

## AI 使用说明

| 工具 | 使用场景 |
|------|----------|
| Claude Code | 代码架构设计、模块实现、测试生成、文档撰写 |

## License

本项目为《软件工程导论》课程作业，仅用于教育目的。
