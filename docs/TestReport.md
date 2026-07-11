# 测试报告

> 测试日期：2026-07-11  
> 环境：Windows 10, Python 3.13.14, pytest 9.1.1  
> 命令：`$env:PYTHONUTF8="1"; python -m pytest -v`

## 1. 测试概要

| 指标 | 结果 |
|------|------|
| 总用例数 | 57 |
| 通过 | 57 |
| 失败 | 0 |
| 耗时 | 0.19s |

## 2. 用例分类统计

| 类型 | 数量 | 文件 |
|------|------|------|
| WorldState 测试 | 15 | `test_world_state.py` |
| 传球策略测试 | 7 | `test_strategy_pass.py` |
| 带球策略测试 | 5 | `test_strategy_dribble.py` |
| 射门策略测试 | 6 | `test_strategy_shoot.py` |
| 跑位策略测试 | 3 | `test_strategy_position.py` |
| 卡位策略测试 | 5 | `test_strategy_block.py` |
| 状态机测试 | 9 | `test_decision_fsm.py` |
| 集成测试 | 7 | `test_integration.py` |

## 3. 代表性用例明细

### 3.1 WorldState

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_closest_teammate_to_ball | 最近队友 | default 场景 | robot 0 | robot 0 | PASS |
| test_has_possession | 控球判断 | shoot 场景 | True | True | PASS |

### 3.2 传球策略

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_path_clear_no_opponents | 路径畅通 | pass 场景 | True | True | PASS |
| test_can_pass | 可传球 | pass 场景 | True | True | PASS |
| test_execute_pass | 执行传球 | pass 场景 | kick 成功 | kick 成功 | PASS |

### 3.3 带球策略

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_dribble_toward_goal | 向球门带球 | shoot 场景 | 目标在对方半场 | 符合 | PASS |
| test_approach_ball_not_controlled | 追球 | default 场景 | 移动指令 | 移动指令 | PASS |

### 3.4 射门策略

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_shoot_viable_when_close | 可射门 | shoot 场景 | True | True | PASS |
| test_shoot_not_viable_when_far | 距离远不射 | default 场景 | False | False | PASS |

### 3.5 跑位/卡位

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_support_position | 支援位 | pass 场景 | 场地内坐标 | 场地内 | PASS |
| test_is_goal_threatened | 威胁检测 | threat 场景 | True | True | PASS |
| test_block_position | 卡位位置 | threat 场景 | 球与门之间 | 符合 | PASS |

### 3.6 状态机

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_ball_carrier_goes_to_chase | 追球状态 | default | CHASE | CHASE | PASS |
| test_simulation_with_pass_scenario | 传球仿真 | pass 30 tick | 含 PASS | 含 PASS | PASS |
| test_simulation_with_threat_scenario | 威胁仿真 | threat 30 tick | 含 BLOCK | 含 BLOCK | PASS |

### 3.7 集成测试

| 用例 | 目标 | 输入 | 预期 | 实际 | 结果 |
|------|------|------|------|------|------|
| test_full_loop_no_crash | 完整循环 | 100 tick | 无崩溃 | 无崩溃 | PASS |
| test_pass_scenario_completes | 传球场景 | pass 场景 | 正常完成 | 正常完成 | PASS |
| test_decision_log_export | CSV 导出 | 50 tick + export | 文件存在 | 文件存在 | PASS |

## 4. 场景演示验证

| 场景 | 命令 | 决策状态分布（30s） | CSV |
|------|------|---------------------|-----|
| default | `--headless --duration 10` | CHASE/PASS/BLOCK/IDLE | - |
| pass | `--scenario pass --export-csv` | PASS:693, CHASE:600 | `outputs/csv/decision_log_pass.csv` |
| shoot | `--scenario shoot --export-csv` | DRIBBLE:102, PASS:693 | `outputs/csv/decision_log_shoot.csv` |
| threat | `--scenario threat --export-csv` | BLOCK:163 | `outputs/csv/decision_log_threat.csv` |

## 5. 失败案例说明

当前全部 57 条自动化测试通过。以下为策略层面的边界行为（非测试失败）：

| 场景 | 表现 | 原因 | 处理 |
|------|------|------|------|
| 距离过远 | 不执行传球/射门 | 超出 ROBOT_KICK_RANGE / SHOOT_RANGE | 策略回退到 CHASE 或 DRIBBLE |
| 对手拦截 | 传球路径被阻挡 | `PassStrategy.is_path_clear()` 返回 False | 选择其他队友或改为带球 |
| 角度不佳 | 不射门 | `ShootStrategy.is_viable()` 返回 False | 改为 PASS 或 DRIBBLE |

## 6. 完整测试日志

详见：`outputs/logs/pytest_result.txt`
