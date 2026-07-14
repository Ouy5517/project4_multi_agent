# 将足球演示机器人替换为 Booster T1（根目录 MuJoCo 路径）

> 目标：在**不改动** WorldState / 决策状态机 / 2D 冲量踢球逻辑的前提下，  
> 把 `main.py --viz mujoco` 里的简模人形换成 **Booster T1 MuJoCo Visual Proxy** 外观。  
> 日期：2026-07-14

---

## 0. 背景与约束（必读）

### 仓库现状

| 路径 | 现状 |
|------|------|
| 根目录 `assets/soccer_full.xml` | mocap + 简模胶囊人，决策演示主路径 |
| `webots/mujoco_soccer` | 已有 T1 **procedural visual proxy**（由 `tools_generate_proxy_model.py` 生成） |
| 官方 STL 网格 `T1_release_meshes/` | Webots 世界文件有引用，**未随仓库提供** |
| `assets/t1_*.xml` | 空 stub，不包含几何 |

### 本方案选择（推荐）

**保留根目录栈，只换外观：**

```
保留: robot_{id} mocap + LimbAnimator + 冲量踢球 + ring/pass_line
替换: 子树几何 → 缩放后的 T1 visual_v2 代理结构
禁止: 整包换成 t1_2v2_soccer.xml（base 执行器 / 2v2 / 脚撞球，会破坏现有契约）
```

### 硬契约（替换后必须仍满足）

1. Body 名：`robot_0/1/2/10/11/12`，且 `mocap="true"`
2. Geom：`robot_{id}_ring`（决策光圈）
3. 动画关节（短名，供 `LimbAnimator`）：  
   `robot_{id}_{r_hip,r_knee,l_hip,l_knee,r_shoulder,l_shoulder}`
4. 球：`ball` + `ball_joint`；传球线：`pass_line_*`
5. 全身几何 `contype="0" conaffinity="0"`（纯视觉，踢篮球走冲量）

### 比例

- Proxy 真高约 1.2 m；现场地 9×6、球 r=0.05。
- 采用 **SCALE ≈ 0.45**，把 T1 外观压到与场地协调的高度；脚底尽量贴近地面。

---

## 步骤一览

| # | 步骤 | 产出 |
|---|------|------|
| 1 | 环境确认 | mujoco 可 import |
| 2 | 生成 / 核对 T1 proxy 参考模型 | `webots/mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml` |
| 3 | 改写 `assets/build_soccer_full.py` 嵌入缩放 T1 | 新 `soccer_full.xml` |
| 4 | （可选增强）扩展关节驱动 / 相机 | `mujoco_simulator.py` 微调 |
| 5 | 回归测试 + 人工打开 Viewer | pytest 绿；传球可见 T1 |
| 6 | （可选后续）接入官方 STL | `assets/meshes/`（需外部资源） |

---

## 步骤 1 — 环境确认

```powershell
cd C:\Users\caihao112\Desktop\gitunico\project4_multi_agent
.\.venv\Scripts\python.exe -c "import mujoco; print(mujoco.__version__)"
```

预期：打印版本号（如 `3.10.0`）。

---

## 步骤 2 — 生成 T1 Visual Proxy 参考模型

```powershell
$env:PYTHONPATH="C:\Users\caihao112\Desktop\gitunico\project4_multi_agent\webots"
.\.venv\Scripts\python.exe -m mujoco_soccer.tools_generate_proxy_model
.\.venv\Scripts\python.exe -c "import mujoco; m=mujoco.MjModel.from_xml_path(r'webots\mujoco_soccer\models\t1_2v2_soccer_visual_v2.xml'); print('bodies', m.nbody, 'joints', m.njnt)"
```

要点：只作外观结构参考；**不要**把该 XML 直接设为根目录 `MuJoCoSimulator` 的加载文件。

---

## 步骤 3 — 改写场景生成器（核心）

修改 `assets/build_soccer_full.py`：

1. 为每个 `robot_{id}` 生成：
   - mocap 根 body + ring
   - 子树：缩放版 T1 torso（visual_v2 白壳 + 队色胸口/腿臂）
   - **无** `base_x/y/yaw` 执行器
   - **无** 可撞球的 `FOOT_BALL_PROXY`（全部 contype=0）
2. 动画关节使用短名挂在：
   - 右/左 Hip Pitch → `r_hip` / `l_hip`
   - 右/左 Knee Pitch → `r_knee` / `l_knee`
   - 右/左 Shoulder Pitch → `r_shoulder` / `l_shoulder`
3. 其余 T1 关节（Waist / Ankle / Roll…）可保留并固定 qpos=0，增加真实感外形。
4. 队色：蓝方 `0.04 0.25 0.90`；黄/橙红方 `0.95 0.55 0.08`（对应 id 10–12）。
5. 运行：

```powershell
.\.venv\Scripts\python.exe assets\build_soccer_full.py
```

---

## 步骤 4 — 仿真器兼容检查 / 轻量增强

检查 `simulation/mujoco_simulator.py`：

- `_robot_mocap_ids` / `_limb_qpos` / `_ring_geom_ids` 仍能全部解析
- `ROBOT_Z` 可视情况微调（脚悬空则略降，如 `-0.02`）
-（可选）在 `_apply_limb_poses` 后把 Knee/Hip 映射到踝关节轻度跟随，让脚更自然

**不改** `decision/`、`strategy/`、`common/world_state.py`。

---

## 步骤 5 — 验证

```powershell
.\.venv\Scripts\python.exe -c "from simulation.mujoco_simulator import MuJoCoSimulator; s=MuJoCoSimulator(num_blue=3,num_yellow=3); print('mocap', sorted(s._robot_mocap_ids)); print('limb', {k:len(v) for k,v in s._limb_qpos.items()})"
.\.venv\Scripts\python.exe -m pytest tests\test_limb_animator.py tests\ -q
.\.venv\Scripts\python.exe main.py --viz mujoco --scenario pass --duration 30
```

验收标准：

- [x] 6 个 mocap + 每机 6 个肢体关节
- [x] 传球/射门时仍有摆腿，低力度仍为 dribble
- [x] 现有单元测试全部通过
- [x] Viewer 中外观接近 T1（白壳头身 + 队色面板 + 分腿分臂）

**落地结果（2026-07-14）：**

```
mocap [0, 1, 2, 10, 11, 12]
limb  每机 6 个短名关节
follow 每机 6 个跟随关节 (踝/肘/腰/头)
pytest: 69 passed
SCALE=0.45
```

---

## 步骤 6 — 官方网格（已落地）

官方资源仓库：

```text
https://github.com/BoosterRobotics/booster_assets.git
```

克隆位置（本地，已 `.gitignore`）：

```text
project4_multi_agent/booster_assets/
```

生成命令：

```powershell
.\.venv\Scripts\python.exe assets\build_soccer_full.py
```

实现要点：

1. `compiler meshdir="../booster_assets/robots/T1/meshes"`
2. 使用官方 `T1_23dof` 运动学 + STL（Trunk/头臂腿脚等）
3. 仍挂在 `robot_{id}` mocap 下；短名关节驱动动画
4. 几何 `contype=0`，踢球仍走冲量
5. 蓝/黄队通过 Trunk/髋/肩 mesh 的 rgba 区分

验收：

```powershell
.\.venv\Scripts\python.exe -c "from simulation.mujoco_simulator import MuJoCoSimulator; s=MuJoCoSimulator(); print(sorted(s._robot_mocap_ids), {k:len(v) for k,v in s._limb_qpos.items()})"
.\.venv\Scripts\python.exe -m pytest tests -q
.\.venv\Scripts\python.exe main.py --viz mujoco --scenario pass --duration 30
```

---

## 明确不做的事

- 不用 `mujoco_soccer` 的 `PlanarBaseController` 替换根目录 2D 仿真
- 不把机器人 ID 改成 `T1_BLUE_1` 字符串（决策层用 int）
- 不把球改成 r=0.11 而不同步 `common/config.py`（会破坏策略距离阈值）

---

## 变更文件清单（本方案落地）

| 文件 | 操作 |
|------|------|
| `docs/T1_ROOT_REPLACEMENT_STEPS.md` | 新建（本文档） |
| `assets/build_soccer_full.py` | 重写：嵌入缩放 T1 |
| `assets/soccer_full.xml` | 重新生成 |
| `simulation/mujoco_simulator.py` | 必要时微调 `ROBOT_Z` / 映射 |
