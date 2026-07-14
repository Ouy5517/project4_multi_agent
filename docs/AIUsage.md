# AI 使用说明

> 项目：Booster T1 多机器人足球协同决策系统

## 1. 使用工具

| 工具 | 用途 |
|------|------|
| Cursor Agent (Claude) | 环境搭建、文档生成、运行验证 |
| Claude Code | 代码架构设计、模块实现、测试生成（仓库原始开发） |

## 2. 本次 AI 辅助内容

| 任务 | AI 参与方式 | 人工审核 |
|------|-------------|----------|
| 阅读 booster.pdf 完成路线 | AI 通读并提取 Windows 可执行步骤 | 已确认 |
| 虚拟环境 + 依赖安装 | AI 执行 pip install | 已验证 |
| 运行演示与测试 | AI 执行 main.py / pytest | 57/57 通过 |
| 创建 docs/ 文档 | AI 基于代码生成接口文档和测试报告 | 需组员补充 |
| 创建 outputs/ 目录 | AI 导出 CSV 和测试日志 | 已生成 |

## 3. 未使用 AI 的部分

- 核心策略算法（strategy/）
- 决策状态机逻辑（decision/）
- 2D 仿真物理引擎（simulation/）
- 单元测试用例设计（tests/）

## 4. 使用原则

1. AI 生成文档基于实际代码，不虚构接口
2. 策略和决策核心逻辑由组员自行实现和审查
3. AI 辅助内容在提交前需人工验证
4. 参考 Booster 官方 Demo 仅借鉴架构，不直接复制代码

## 5. 参考资料

- [Booster 官方开源资料](https://www.booster.tech/cn/open-source/)
- [Booster Robotics SDK](https://github.com/BoosterRobotics/booster_robotics_sdk)
- [Booster ROS2 SDK](https://github.com/BoosterRobotics/booster_robotics_sdk_ros2)
- [Booster RoboCup Demo](https://github.com/BoosterRobotics/robocup_demo)
- [课程仓库](https://github.com/Ouy5517/project4_multi_agent)
