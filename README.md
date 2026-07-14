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
│   ├── decision_fsm.py         # 决策状态机
│   └── match_controller.py     # 比赛计分与定点球
├── simulation/                 # 仿真引擎
│   ├── field_simulator.py      # 2D 物理仿真
│   ├── mujoco_simulator.py     # MuJoCo 仿真器
│   └── mujoco_visualizer.py    # MuJoCo 3D 渲染
├── experiments/                # 独立实验脚本
│   ├── solo_shoot_test.py      # 单球员射空门测试 (FSM)
│   └── shoot_angle_lab.py      # 多角度射门精度实验
├── tests/                      # 测试
│   └── test_*.py               # 单元测试 & 集成测试
├── main.py                     # 主程序 (对抗模式)
├── run.sh                      # 一键运行
├── run_shoot.sh                # 射门测试独立启动脚本
├── README.md                   # 本文档
└── outputs/                    # 日志输出
```

## 快速开始

### 环境要求
- Python 3.8+
- 可选: pytest, matplotlib (用于高级可视化和测试)

### 运行演示

```bash
# === 一键启动 (推荐) ===
bash run.sh                          # 交互菜单

# 对抗模式
bash run.sh 3d                       # MuJoCo 3D 对抗
bash run.sh 3d pass                  # MuJoCo 3D + 传球场景
bash run.sh 3d shoot                 # MuJoCo 3D + 射门场景
bash run.sh 2d                       # Matplotlib 2D 对抗
bash run.sh ascii pass 30            # ASCII + 传球 + 30s

# 单球员射空门
bash run.sh solo                     # 无渲染快速测试
bash run.sh solo ascii 3.0 0.0 10   # ASCII 可视化
bash run.sh solo mujoco 2.5 1.0 10  # MuJoCo 3D

# 多角度射门实验
python experiments/shoot_angle_lab.py --angles 9 --power 70

# 运行测试
bash run.sh test

# 安装开发依赖
bash run.sh install
```

### 直接使用 Python

```bash
# === 对抗模式 ===
python main.py                                    # 默认演示
python main.py --scenario pass --viz matplotlib   # 传球 + 2D 图形
python main.py --scenario shoot --viz mujoco      # 射门 + 3D
python main.py --headless --duration 60           # 无渲染 60s
python main.py --export-csv --duration 30         # 导出决策日志

# === 单球员射空门 ===
python experiments/solo_shoot_test.py             # 默认位置 (3.0, 0.0)
python experiments/solo_shoot_test.py --ball-x 2.5 --ball-y 1.0
python experiments/solo_shoot_test.py --viz ascii --duration 15

# === 多角度射门实验 ===
python experiments/shoot_angle_lab.py             # 7角度, 力度90
python experiments/shoot_angle_lab.py --angles 9 --power 70 --viz mujoco

# === 查看帮助 ===
python main.py --help
python experiments/solo_shoot_test.py --help
python experiments/shoot_angle_lab.py --help
```

## 测试与演示

项目支持三种测试/演示模式：**对抗模式**、**单球员射空门**、**多角度射门实验**。

### 1. 对抗模式 (双队 FSM 争球)

完整的 3v3 对抗，蓝队攻右门、黄队攻左门，含争球、越位、门底禁抢、进球开球、出界任意球等规则。

```bash
# === 通过 run.sh (推荐) ===
./run.sh                          # 交互菜单选择场景和可视化

# 一键启动各模式
./run.sh 3d                       # MuJoCo 3D 可视化 (默认场景)
./run.sh 3d pass                  # MuJoCo 3D + 传球场景
./run.sh 3d shoot                 # MuJoCo 3D + 射门场景
./run.sh 3d threat                # MuJoCo 3D + 防守场景
./run.sh 2d                       # Matplotlib 2D 图形窗口
./run.sh 2d pass                  # Matplotlib 2D + 传球场景
./run.sh ascii                    # ASCII 终端可视化
./run.sh headless 60              # 无渲染模式跑 60 秒

# === 直接使用 Python ===
# 默认演示 (ASCII 可视化, 30s)
python main.py

# 指定场景
python main.py --scenario pass    # 传球场景
python main.py --scenario shoot   # 射门场景
python main.py --scenario threat  # 防守场景

# 可视化选项
python main.py --viz matplotlib --scenario pass --duration 30
python main.py --viz mujoco --scenario shoot --duration 60

# 无渲染 + 导出 CSV
python main.py --headless --duration 120 --export-csv

# 导出 GIF 动画
python main.py --viz matplotlib --scenario pass --export-gif outputs/pass.gif
```

### 2. 单球员射空门 (Solo Shoot Test)

仅拉取 **1 个球员**，场上**无对手**，通过完整 FSM 决策管线 (IDLE → CHASE → SHOOT) 测试射门能力。

```bash
# === 通过 run.sh ===
./run.sh solo                     # 默认: 无渲染, 球在 (3.0, 0.0)
./run.sh solo ascii 3.0 0.0 10   # ASCII 可视化, 自定义球位和时长
./run.sh solo mujoco 2.5 1.0 10  # MuJoCo 3D 可视化

# === 通过 run_shoot.sh (独立射门脚本, 推荐) ===
./run_shoot.sh                              # 交互菜单 (预设球位 + 可视化选择)
./run_shoot.sh 3.0 0.0 mujoco               # 正面球位 + 3D 渲染
./run_shoot.sh 2.5 1.0 ascii 15             # 偏右 + ASCII + 15 秒
./run_shoot.sh 2.0 -1.2 none 20             # 偏左 + 无渲染 + 20 秒
./run_shoot.sh batch                         # 批量测试全部 7 个预设球位

# === 直接使用 Python ===
# 默认位置射门
python experiments/solo_shoot_test.py

# 自定义球位
python experiments/solo_shoot_test.py --ball-x 3.0 --ball-y 1.0

# 可视化
python experiments/solo_shoot_test.py --viz none      # 无渲染 (默认)
python experiments/solo_shoot_test.py --viz ascii     # 终端 ASCII
python experiments/solo_shoot_test.py --viz mujoco    # MuJoCo 3D

# 完整参数
python experiments/solo_shoot_test.py \
    --ball-x 2.5 --ball-y -1.2 \
    --viz ascii --duration 15
```

**参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--ball-x` | 3.0 | 发球点 X 坐标 (球门在 4.5) |
| `--ball-y` | 0.0 | 发球点 Y 坐标 (球门宽 2.0) |
| `--duration` | 10 | 最大仿真时长 (秒) |
| `--viz` | none | 可视化: none / ascii / mujoco |

**结果分类：**
- `GOAL` — 进球
- `MISS_WIDE` — 球偏出界外
- `MISS_STOPPED` — 球中途停止
- `MISS_TIMEOUT` — 超时未进球

### 3. 多角度射门实验 (Shoot Angle Lab)

绕开 FSM，直接控制球员从多个角度踢球，用于量化评估射门精度。

```bash
# 默认 7 个角度, 力度 90
python experiments/shoot_angle_lab.py

# 9 个角度, 力度 70
python experiments/shoot_angle_lab.py --angles 9 --power 70

# 自定义发球点
python experiments/shoot_angle_lab.py --ball-x 2.5 --ball-y 1.0

# MuJoCo 3D 可视化 (一秒踢一脚)
python experiments/shoot_angle_lab.py --viz mujoco --kick-interval 1.0

# 重复整组角度 3 轮
python experiments/shoot_angle_lab.py --angles 7 --repeat 3
```

**参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--angles` | 7 | 球门线上采样角度数 |
| `--power` | 90 | 踢球力度 (0-100) |
| `--ball-x` | 3.0 | 发球点 X 坐标 |
| `--ball-y` | 0.0 | 发球点 Y 坐标 |
| `--settle` | 3.0 | 单次最多等待秒数 |
| `--kick-interval` | 1.0 | 相邻踢球最小间隔 (秒) |
| `--viz` | none | 可视化: none / mujoco |
| `--repeat` | 1 | 整组角度重复次数 |

### 4. 运行单元测试

```bash
# === 通过 run.sh ===
./run.sh test

# === 直接使用 pytest ===
pytest tests/ -v --tb=short

# 运行单个测试文件
pytest tests/test_strategy_shoot.py -v
pytest tests/test_decision_fsm.py -v
pytest tests/test_shoot_angle_lab.py -v
pytest tests/test_integration.py -v

# 带覆盖率
pytest tests/ -v --cov=. --cov-report=term-missing
```

### 测试模式对比

| 模式 | FSM 决策 | 对手 | 球员数 | 适用场景 |
|------|----------|------|--------|----------|
| 对抗模式 (`main.py`) | ✅ 完整双队 | 3v3 | 6 | 完整比赛演示 |
| 单球员射空门 (`solo_shoot_test.py`) | ✅ 完整单队 | 0 | 1 | 射门决策调试 |
| 多角度实验 (`shoot_angle_lab.py`) | ❌ 直接踢球 | 0 | 1 | 射门精度量化 |
| 单元测试 (`pytest`) | 各模块独立 | — | — | 回归验证 |

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
