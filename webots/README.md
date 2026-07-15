# Booster T1 多机器人足球协同决策系统

## Final Release Quick Start

实时四机器人并发比赛：

```bash
./scripts/start_final_soccer_demo.sh --match
```

稳定功能展示：

```bash
./scripts/start_final_soccer_demo.sh --showcase
```

生成 60FPS 演示视频：

```bash
./scripts/start_final_soccer_demo.sh --record
```

一键最终验收：

```bash
./scripts/run_final_acceptance.sh
```

当前最终发布版使用 MuJoCo 四个独立 `RobotAgent` 并发决策，保留 Assisted Planar Locomotion、原生关节 actuator 步态和足球物理碰撞移动。视觉机器人是 NAO-inspired 代理，不是官方 NAO 模型，也不是完整官方 Booster T1 动力学模型。60FPS 视频是完整真实物理运行日志的 trajectory replay 插值重放，不是实时屏幕直接录制。

这是一个第一阶段 Mock 版多机器人足球协同决策系统。系统基于统一 `WorldState` 读取球、队友、对手和球门位置，输出传球、接球、带球、射门、卡位、追球等高层动作。

当前版本不依赖 ROS、数据库或 Booster SDK，不控制真实机器人，重点是策略逻辑、模块边界、日志和可测试性。

## 目录结构

```text
booster_soccer_project/
├── main.py
├── common/
│   ├── world_state.py
│   ├── robot_action.py
│   └── evaluation_log.py
├── strategy/
│   ├── team_strategy.py
│   └── state_machine.py
├── robot_adapter/
│   ├── mock_adapter.py
│   ├── webots_adapter.py
│   └── booster_action_map.py
├── scenarios/
│   ├── pass_success.json
│   ├── dribble_when_marked.json
│   ├── pass_receive_shoot.json
│   └── defend_marking.json
├── outputs/
├── tests/
│   └── test_strategy.py
├── docs/
└── README.md
```

## 环境准备

项目使用 Python 3。若已有 `coop_env`，可直接激活：

```bash
cd ~/Workspace/booster_soccer_project
source coop_env/bin/activate
```

如需重新安装测试依赖：

```bash
pip install -r requirements.txt
```

## 运行方式

```bash
python main.py
```

默认使用 Mock 适配器，只打印策略动作意图：

```bash
python main.py --adapter mock
```

使用 Webots 说明性适配器时，程序仍不控制真实机器人，也不调用 Booster SDK，只打印每个策略动作在 Webots / Booster T1 示例程序中应如何执行：

```bash
python main.py --adapter webots
```

程序会依次加载 `scenarios/` 下的 JSON 场景，打印场景名称、WorldState 摘要、状态机状态、动作、原因，并将日志写入：

```text
outputs/decision_log.jsonl
```

## 测试方式

```bash
pytest
```

测试覆盖：

- 空位传球输出 `PASS` 和 `MOVE_TO_RECEIVE`
- 队友被盯防时输出 `DRIBBLE`
- 传球、接球、射门连续流程最终输出 `SHOOT`
- 无人持球时最近机器人输出 `CHASE_BALL`

## 2D 可视化演示

项目提供一个基于 pygame 的 2D 足球场演示窗口，会读取 `scenarios/` 下的 JSON 场景，绘制足球场、我方机器人、对手机器人、足球、双方球门，并根据 `TeamStrategy.decide(world_state)` 的结果显示动作箭头和决策原因。

```bash
python demo/field_viewer.py --scenario scenarios/pass_success.json
```

窗口中使用 `T1_A`、`T1_B`、`O1`、`O2`、`Ball`、`Our Goal`、`Enemy Goal` 标注对象。`PASS`、`DRIBBLE`、`SHOOT`、`MARK_OPPONENT`、`CHASE_BALL` 会在场上显示对应动作箭头；右侧面板显示当前策略动作、目标点和决策原因。按 `Q` 或 `Esc` 退出。

## 策略说明

`TeamStrategy.decide(world_state)` 返回 `list[RobotAction]`，核心规则如下：

- 我方无人持球：最近机器人执行 `CHASE_BALL`
- 持球机器人进入 `shoot_distance`：优先 `SHOOT`
- 队友未被盯防：持球机器人 `PASS`，队友 `MOVE_TO_RECEIVE`
- 队友被盯防：持球机器人 `DRIBBLE`，队友 `MARK_OPPONENT`
- 对手靠近我方球门：支援机器人优先 `MARK_OPPONENT`

队友被盯防的判断为：任意对手与该队友距离小于 `pass_safe_distance`。

## Webots / Booster T1 使用说明

当前项目只提供策略动作到 Webots / Booster T1 示例动作的映射说明，不调用真实 Booster SDK，也不修改 `~/Workspace/booster_robotics_sdk`。

推荐按四个终端运行：

```bash
# 终端 1：打开 Webots 场景
webots path/to/your_scene.wbt
```

```bash
# 终端 2：启动 booster-runner
booster-runner
```

```bash
# 终端 3：运行 Booster T1 示例客户端
~/Workspace/booster_robotics_sdk/build/b1_loco_example_client 127.0.0.1
```

```bash
# 终端 4：运行本项目的 Webots 说明性适配器
cd ~/Workspace/booster_soccer_project
python main.py --adapter webots
```

`robot_adapter/booster_action_map.py` 中维护策略动作到 Booster T1 高层动作的映射，`robot_adapter/webots_adapter.py` 会打印 `robot_id`、`action_type`、`target`、建议输入命令和动作解释。

| 策略动作 | 说明 | Webots / SDK 示例动作 | 建议输入命令 |
| --- | --- | --- | --- |
| `PASS` | 面向队友方向，短距离推球或踢球 | 先转向目标队友，再低速前进或执行 kick 踢球动作 | `a/d` 转向目标，然后 `w` 低速推进或 `kick` |
| `MOVE_TO_RECEIVE` | 移动到接球区域 | 根据目标点方向选择 `w/a/s/d` 移动到接球点 | `w/a/s/d` |
| `DRIBBLE` | 带球向敌方球门方向推进 | 保持面向推进方向，低速 `w` 前进 | `w` |
| `SHOOT` | 面向球门后射门 | 先转向球门，再执行 `kick` 或 Soccer Agent 踢球动作 | `a/d` 转向球门，然后 `kick` |
| `MARK_OPPONENT` | 移动到对手和我方球门之间进行卡位 | 移动到防守站位，保持在对手射门线路上 | `w/a/s/d` |
| `CHASE_BALL` | 追球 | 朝球的位置移动 | `w/a/s/d` |
| `STOP` | 停止 | 停止移动 | `l` |

后续若接入真实控制层，可以保持策略层不变：

1. 在 Webots 中读取仿真球、机器人、对手和球门状态，转换为 `WorldState`
2. 将 `RobotAction` 映射为真实 Webots 控制命令
3. 新增真实适配器，例如 `BoosterRobotAdapter`，实现与 `MockRobotAdapter.execute()` 相同的接口
4. 继续复用 `EvaluationLogger` 做策略日志和对比评估
# Final Submission Status - 2026-07-12

This project contains the Booster T1 + Webots multi-robot soccer cooperation demo.

Final verified path: **Path B**.

- Real mode command: `./scripts/start_final_submission_demo.sh real`
- Mock mode command: `./scripts/start_final_submission_demo.sh mock`
- Stop command: `./scripts/stop_final_submission_demo.sh`
- Check command: `./scripts/check_final_submission.sh`

Real mode uses one real mck-controlled robot, `T1_BLUE_1`; `T1_BLUE_2`, `T1_RED_1`, and `T1_RED_2` are passive Webots robots. The real run `20260712_152137_6c4b6c` did not reach mck ready: mck segfaulted after repeated recording reentrancy warnings from `lcm_matlab_backend.cpp` and `socket_backend.cpp`. Therefore real physical dribble was not completed and the physical minimum acceptance was not reached.

Mock mode run `mock_20260712_152051_d6f360` completed the 2v2 cooperation demo using real `TeamStrategy.decide()` decisions and `MockRobotActionAdapter` only at the action layer. The actual strategy returns were PASS, DRIBBLE, SHOOT, and BLOCK.

Evidence is under `results/final_submission/`.

Native Webots Motor control was added after the mck failure:

- Start: `./scripts/start_native_physical_kick.sh kick`
- Assisted: `./scripts/start_native_physical_kick.sh assisted-kick`
- Check: `./scripts/check_native_physical_kick.sh`

Final native outcome is **N3**: device probing and PositionSensor setup worked,
but unassisted standing failed and the single assisted run timed out before a
kick/contact. No native physical ball movement is claimed.

## Assisted 2v2 Physical Soccer Demo

Run the assisted four-robot Webots demo with:

```bash
./scripts/start_four_robot_physical_demo.sh
```

This mode uses Supervisor-assisted robot root motion, native Webots Motor gait animation, and physics-only soccer ball movement. It does not use `mck` or `rpc_service_node`. Details are in `docs/FOUR_ROBOT_PHYSICAL_DEMO.md`.
# MuJoCo 2v2 Addition

Added `mujoco_soccer/`, a MuJoCo-only four-robot 2v2 soccer demo using assisted planar locomotion, visible native joint gait, and physical foot-ball collision. It does not start Webots, mck, RPC, or ROS controllers.

Run:

```bash
./scripts/start_mujoco_four_robot_demo.sh
```

See `docs/MUJOCO_FOUR_ROBOT_DEMO.md` and the latest `results/mujoco_four_robot_demo/*/summary.json`.

## MuJoCo Visual V2 Presentation Demo

Visual V2 adds an independent NAO-inspired primitive visual proxy, brighter indoor soccer field, clean viewer/recorder, broadcast camera, key screenshots, and 1280x720 presentation video while preserving the accepted MuJoCo physics baseline.

Run:

```bash
./scripts/start_mujoco_visual_soccer_demo_v2.sh --normal
```

Visual V2 does not use an official NAO model or official NAO mesh. The visual shells are non-colliding, zero-density MuJoCo primitives; soccer ball motion remains physical foot-ball contact only. See:

- `docs/MUJOCO_VISUAL_V2.md`
- `docs/NAO_STYLE_PROXY_DESIGN.md`
- `docs/MOTION_QUALITY_RULES.md`

Final accepted MuJoCo run:

- `run_id`: `full_final_acceptance`
- `demo_success`: `true`
- summary: `results/mujoco_four_robot_demo/full_final_acceptance/summary.json`
- screenshot: `results/mujoco_four_robot_demo/full_final_acceptance/final_frame.png`

## MuJoCo Concurrent 2v2 Match

`concurrent-match` is an independent MuJoCo mode that runs four robot agents on each decision tick. Each robot observes the same immutable world snapshot, selects its own behavior, then all four commands are applied before one shared MuJoCo physics step. This mode does not use the previous single active-robot sequence and does not modify the accepted Visual V3 deterministic baseline.

Run:

```bash
./scripts/start_mujoco_concurrent_match.sh --view --seed 42
./scripts/start_mujoco_concurrent_match.sh --record --seed 42
./scripts/start_mujoco_concurrent_match.sh --no-render --seed 42
```

Outputs are written to `results/mujoco_concurrent_match/<run_id>/`, including `summary.json`, `concurrency_acceptance.json`, `agent_decisions.jsonl`, `shared_world_state.jsonl`, `team_roles.jsonl`, `possession.jsonl`, `contacts.jsonl`, `robot_commands.jsonl`, `robot_states.jsonl`, `ball_motion.jsonl`, `goals.jsonl`, `events.jsonl`, `final_frame.png`, and recorded `demo.mp4` when `--record` is used.

Smooth realtime frontend:

```bash
./scripts/start_mujoco_concurrent_match_smooth.sh --view --seed 42
./scripts/start_mujoco_concurrent_match_smooth.sh --benchmark --duration 20 --seed 42
./scripts/start_mujoco_concurrent_match_smooth.sh --record --seed 42
```

The smooth path keeps the four-Agent concurrent match logic intact while using a 60Hz MuJoCo passive viewer, async JSONL logging, target/velocity/yaw smoothing, path-coupled gait, and frontend acceptance output in `frontend_smoothness_acceptance.json`. Its `--record` path generates a 60 FPS trajectory replay from physical simulation logs and writes provenance metadata.
