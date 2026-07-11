# 项目总结报告（模板）

> 项目：Booster T1 多机器人足球协同决策系统（题目四）  
> 状态：进行中 — 核心功能与测试已完成，演示视频待录制

---

## 1. 项目名称

Booster T1 多机器人足球协同决策系统

## 2. 项目背景与意义

基于 Booster T1 机器人平台，采用软件工程方法实现多机器人足球协同决策，涵盖传球、带球、射门、跑位、卡位和状态机决策。

## 3. 需求分析

详见 [SRS.md](SRS.md)

## 4. 系统总体设计

详见 [Design.md](Design.md)

## 5. 详细设计

- 核心模块：WorldState、RobotAction、DecisionFSM、Strategy 五模块
- 接口文档：[API.md](API.md)
- 状态机：IDLE → CHASE → (DRIBBLE | PASS | SHOOT | BLOCK)

## 6. 系统实现

### 开发环境

- OS：Windows 10
- Python：3.13.14
- 依赖：pytest 9.1.1, matplotlib 3.11.0

### 运行方法

```powershell
.\.venv\Scripts\Activate.ps1
$env:PYTHONUTF8="1"
python main.py --scenario pass --duration 30 --export-csv --headless
python -m pytest -q
```

## 7. 系统测试

详见 [TestReport.md](TestReport.md) — 57 条用例全部通过。

## 8. 项目管理

- 小组分工：见 README.md
- 会议记录：[MeetingRecords.md](MeetingRecords.md)
- 版本管理：Git + GitHub

## 9. 成果展示

| 成果 | 路径 |
|------|------|
| 决策日志 CSV | `outputs/csv/` |
| 测试日志 | `outputs/logs/pytest_result.txt` |
| 演示视频 | `outputs/videos/`（待录制） |

## 10. 问题分析与改进方向

| 问题 | 改进 |
|------|------|
| Windows emoji 编码 | 设置 PYTHONUTF8=1 |
| Real 模式未接实机 | bridge/ 层预留接口 |
| 3v3 策略较简单 | 参考 RoboCup Demo 分层 |

## 11. 参考资料

见 [AIUsage.md](AIUsage.md) 第 5 节。

---

*本报告为模板，请组员补充个人贡献、截图和视频说明后定稿。*
