# T1 MuJoCo 接入与 Pre-RL 修改总结

日期：2026-07-15

## 目标

本次修改的目标是把项目推进到“需要机器学习 / 强化学习之前”的边界：

- 在 MuJoCo 中接入官方 Booster T1 STL 外观模型。
- 保留当前已经稳定的平面辅助移动控制。
- 增加可切换的动作后端接口。
- 增加基于规则的关节轨迹后端，用于 `move_to`、`pass`、`shoot`、`block`、`clear`、`intercept`、`dribble`。
- 预留 `rl_policy` 后端，但在没有训练策略前保持显式未实现。

## 当前架构

高层多智能体策略仍然输出语义级足球行为：

- `PASS`
- `SHOOT`
- `BLOCK_LINE`
- `CLEAR`
- `INTERCEPT_BALL`
- `PRESS_BALL`
- `DRIBBLE`
- `OPEN_FOR_PASS`
- `RECEIVE_PASS`

这些行为会通过 `mujoco_soccer.control.action_interface.SoccerActionInterface` 转换成底层动作原语。

当前支持的动作后端：

| 后端 | 状态 | 作用 |
| --- | --- | --- |
| `assisted_planar` | 默认可用 | 当前稳定的平面辅助移动 + 可视步态 + 踢球动画。 |
| `trajectory_joint` | 可用 | 仍保留平面位移，但动作会经过显式规则关节轨迹层。 |
| `rl_policy` | 预留 | 没有训练好的策略适配器前会抛出 `NotImplementedError`。 |

## 修改文件

### `webots/mujoco_soccer/tools_generate_proxy_model.py`

模型生成器现在会从以下目录加载官方 Booster T1 STL：

```text
booster_assets/robots/T1/meshes
```

生成的 MuJoCo 模型仍然保留原有运行契约：

- 机器人基座 body：`T1_BLUE_1_base`、`T1_BLUE_2_base`、`T1_RED_1_base`、`T1_RED_2_base`
- 平面辅助关节：`{robot}_base_x`、`{robot}_base_y`、`{robot}_base_yaw`
- 平面辅助 actuator：`{robot}_base_x_act`、`{robot}_base_y_act`、`{robot}_base_yaw_act`
- 球体 geom：`soccer_ball_geom`
- 脚部碰球代理 geom：`*_FOOT_BALL_PROXY`

因此，目前已经能看到官方 T1 外观 mesh，但底层移动仍然是辅助平面移动，不是自由基座人形机器人真实步态。

### `webots/mujoco_soccer/control/action_interface.py`

新增可切换动作后端抽象：

```python
SoccerActionInterface(controllers, backend="assisted_planar")
```

后端工厂：

```python
create_action_backend(name, controllers)
```

已实现：

- `AssistedPlanarBackend`
- `TrajectoryJointBackend`
- `RlPolicyBackend`

`rl_policy` 当前会明确抛出：

```text
NotImplementedError: rl_policy backend requires a trained policy adapter
```

这样做的目的是避免误以为 RL 后端已经可以直接运行。

### `webots/mujoco_soccer/control/joint_trajectory.py`

新增规则关节轨迹模块。

该模块为以下动作提供确定性的关节 offset 曲线：

- `pass`
- `shoot`
- `clear`
- `intercept`
- `dribble`
- `block`

核心接口：

```python
trajectory_offsets(action, progress)
JointTrajectoryState
```

这是进入 ML/RL 之前的最后一层手写规则控制。

### `webots/mujoco_soccer/control/robot_controller.py`

每个机器人控制器新增 `JointTrajectoryState`。

现在控制器会混合三类关节输入：

- 原有可视步态 offset；
- 原有踢球摆腿 offset；
- 新增规则关节轨迹 offset。

### `webots/mujoco_soccer/multi_agent/concurrent_match.py`

并发比赛运行器新增 `action_backend` 参数。

运行 summary 现在会记录当前动作后端：

```json
"action_backend": "trajectory_joint"
```

### `webots/mujoco_soccer/run_demo.py`

新增命令行参数：

```powershell
--action-backend assisted_planar
--action-backend trajectory_joint
--action-backend rl_policy
```

### 配置文件

已更新：

- `webots/mujoco_soccer/config/simulation.yaml`
- `webots/mujoco_soccer/config/final_release.yaml`

新增配置：

```yaml
action:
  backend: assisted_planar
  available_backends:
    - assisted_planar
    - trajectory_joint
    - rl_policy
```

### 测试

新增或更新：

- `webots/mujoco_soccer/tests/test_action_interface.py`
- `webots/mujoco_soccer/tests/test_joint_trajectory.py`
- `webots/mujoco_soccer/tests/test_visual_v3_goals.py`

## 验证结果

在 `webots/` 目录下执行：

```powershell
$env:PYTHONPATH="F:\AAADEMO\project4_multi_agent-main\project4_multi_agent-main\webots"
python -m pytest mujoco_soccer\tests\test_action_interface.py mujoco_soccer\tests\test_joint_trajectory.py mujoco_soccer\tests\test_visual_v3_goals.py tests\test_concurrent_multi_agent.py::test_concurrent_match_short_run_logs_four_agent_decisions -q
```

结果：

```text
19 passed
```

规则关节轨迹后端 smoke test：

```powershell
python -m mujoco_soccer.run_demo --mode concurrent-match --no-render --duration 1 --seed 42 --run-id final_pre_rl_check --action-backend trajectory_joint
```

关键结果：

```text
finished: true
action_backend: trajectory_joint
decision_counts: 每台机器人 20
nan_detected: false
joint_limit_violation: false
ball_mutation_detected: false
```

## 运行方式

默认平面辅助后端：

```powershell
python -m mujoco_soccer.run_demo --mode concurrent-match --view --duration 20 --action-backend assisted_planar
```

规则关节轨迹后端：

```powershell
python -m mujoco_soccer.run_demo --mode concurrent-match --view --duration 20 --action-backend trajectory_joint
```

RL 后端占位：

```powershell
python -m mujoco_soccer.run_demo --mode concurrent-match --no-render --duration 1 --action-backend rl_policy
```

当前预期结果：

```text
NotImplementedError: rl_policy backend requires a trained policy adapter
```

## 当前仍然不是完整真实人形控制

虽然已经接入真实 T1 mesh 和规则关节轨迹，但当前系统仍不是完整物理真实的 T1 控制器。

仍然简化的部分：

- 机器人基座仍由平面 `x/y/yaw` 关节驱动。
- 没有自由基座平衡控制。
- 足地接触不决定机器人移动。
- 脚和球的接触仍使用简化 foot proxy geom。
- 关节轨迹是手写规则，不是优化或学习得到的策略。
- 没有加入电机延迟、力矩饱和动态、电池限制、传感器噪声等真实硬件因素。

## 下一步：进入 ML/RL 或高级控制的边界

下一步真正有意义的推进是把平面辅助移动替换为真实人形控制：

1. 自由基座 T1 站立控制。
2. 足地接触参数调优。
3. 平衡控制器：WBC、MPC 或学习策略。
4. 走路、转身、停止、靠近球的 locomotion policy。
5. 踢球、传球、射门的 skill policy 或轨迹优化器。
6. 如果目标是真机，还需要 sim-to-real 验证。

从这一步开始，机器学习 / 强化学习才真正有价值。当前项目已经具备进入这一步之前所需的稳定真实外观 MuJoCo 路径和清晰动作后端边界。
