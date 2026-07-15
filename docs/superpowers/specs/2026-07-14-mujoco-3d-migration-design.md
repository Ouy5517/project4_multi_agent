# MuJoCo 3D 可视化迁移 — 设计说明书

> 项目：Booster T1 多机器人足球协同决策系统  
> 日期：2026-07-14  
> 版本：v1.0

## 1. 目标

将项目现有 ASCII/Matplotlib 2D 可视化升级为 MuJoCo 3D 渲染，**策略层、决策层、2D 仿真层不做任何改动**，仅在渲染层新增 MuJoCo 通道。

## 2. 架构

```
                       ┌──────────────────────┐
                       │    main.py            │
                       │  --viz mujoco (新增)   │
                       └──────┬───────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         ▼                    ▼                     ▼
┌─────────────┐  ┌──────────────┐  ┌──────────────────┐
│ ASCII        │  │ Matplotlib   │  │ MuJoCoVisualizer │  ← 新增
│ Visualizer   │  │ Visualizer   │  │ (NEW)            │
└─────────────┘  └──────────────┘  └────────┬─────────┘
                                            │
                                    ┌───────┴────────┐
                                    │ MuJoCoSimulator │  ← 增强
                                    │ (已有,需扩展)    │
                                    └───────┬────────┘
                                            │
                                    ┌───────┴────────┐
                                    │ soccer_full.xml │  ← 新增
                                    │ (完整3D场景)     │
                                    └────────────────┘

┌──────────┐  ┌──────────┐  ┌──────────────┐
│ strategy/│  │decision/ │  │ simulation/  │  ← 完全不改
│ *.py     │  │*.py      │  │field_simulator│
└──────────┘  └──────────┘  └──────────────┘
```

### 数据流

```
Simulator.update()           # 2D 物理 (不变)
    → WorldStateProvider.get()   # 构建快照 (不变)
    → DecisionFSM.update()       # 决策 (不变)
    → MockRobotAction            # 指令写入仿真器 (不变)
    → MuJoCoSimulator.sync_to_mujoco()  # 同步到 3D (新增步骤)
    → mujoco.viewer.sync()       # 刷新 3D 画面
```

## 3. 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `assets/soccer_full.xml` | **新增** | 3v3 完整 MuJoCo 场景 |
| `assets/meshes/` | **新增** | 目录，放置 Booster T1 的 STL/OBJ 模型 |
| `simulation/mujoco_simulator.py` | **增强** | 6 机器人 + 队伍颜色 + 状态光圈 |
| `simulation/mujoco_visualizer.py` | **新增** | viewer 生命周期封装 |
| `main.py` | **微调** | 新增 `--viz mujoco` CLI 选项 |

## 4. MuJoCo 场景设计 (`soccer_full.xml`)

### 4.1 场地参数 (基于 Booster T1 实际尺寸)

Booster T1 规格：高度 1.2m，宽度 0.47m，深度 0.23m，重量 30kg，步行速度 0.8 m/s。

场景使用统一的米制单位，场地保持 9m × 6m。

### 4.2 场景元素

```
soccer_full.xml:
├── <asset>
│   ├── skybox (渐变天空)
│   ├── 草地纹理 (checker, 比 minimal 更大更细)
│   ├── 球门材质 (白色半透明)
│   └── 机器人材质 (蓝/黄)
│
├── <worldbody>
│   ├── 灯光 × 2 (主光 + 补光)
│   ├── 地面 (plane, 草地纹理)
│   ├── 场地标线 (边线 × 4, 中线 × 1, 中圈 × 1)
│   ├── 球门框 (左/右, box组合)
│   ├── 足球 (freejoint + sphere, 半径 0.05m)
│   ├── 蓝方机器人 × 3 (mocap body + 几何体组合)
│   │   └── robot_0, robot_1, robot_2
│   └── 黄方机器人 × 3 (mocap body + 几何体组合)
│       └── robot_10, robot_11, robot_12
```

### 4.3 机器人外观 (几何体近似的 Booster T1 人形)

由于当前无 STL 模型文件，使用几何体组合近似：

```
robot_X body (mocap="true"):
├── 躯干: box 0.15×0.10×0.25 (宽×深×高), 蓝色/黄色
├── 头部: sphere 半径 0.06, 白色, 位于躯干上方
├── 左腿: cylinder 半径 0.03 长度 0.25, 位于躯干下方左侧
├── 右腿: cylinder 半径 0.03 长度 0.25, 位于躯干下方右侧
├── 左臂: cylinder 半径 0.02 长度 0.20, 位于躯干侧面
├── 右臂: cylinder 半径 0.02 长度 0.20, 位于躯干侧面
└── 朝向指示: capsule 从中心向前 0.15m, 白色
```

总高度约 0.55m（按比例缩放到适合 9×6m 场地）。实际 T1 高 1.2m，场地按 FIFA 标准约 105×68m，本项目场地缩小为 9×6m，因此机器人也等比缩小。

**如果后续有 STL 文件**，只需在 `<asset>` 中添加 `<mesh>` 并在 body 中引用来替换几何体：

```xml
<asset>
  <mesh name="t1_body" file="meshes/booster_t1.stl" scale="0.001 0.001 0.001"/>
</asset>
```

### 4.4 状态可视化

| 决策状态 | 地板光圈颜色 | 含义 |
|----------|-------------|------|
| IDLE | 灰色 #888888 | 空闲 |
| CHASE | 蓝色 #2196F3 | 追球 |
| DRIBBLE | 橙色 #FF9800 | 带球 |
| PASS | 绿色 #4CAF50 | 传球 |
| SHOOT | 红色 #E91E63 | 射门 |
| BLOCK | 紫色 #9C27B0 | 防守 |

光圈通过动态创建/更新平面圆 geom 实现，每帧根据 fsm 状态切换颜色。

## 5. MuJoCoSimulator 增强

在现有 `mujoco_simulator.py` 基础上增加：

```python
class MuJoCoSimulator(Simulator):
    # 新增功能:
    # 1. 支持 num_yellow > 0 (对手机器人 mocap)
    # 2. sync_to_mujoco() 同步所有 6 个机器人
    # 3. update_status_rings(fsm) — 根据决策状态更新光圈颜色
    # 4. update_pass_lines(fsm) — 动态绘制传球线
    # 5. camera_presets — 相机预设位置
```

## 6. MuJoCoVisualizer 封装

```python
class MuJoCoVisualizer:
    """与 ASCIIVisualizer / MatplotlibVisualizer 接口兼容"""
    
    def __init__(self, simulator: MuJoCoSimulator, title: str = ""):
        self._sim = simulator
        self._viewer = None  # 延迟创建
    
    def render(self, ws: WorldState, fsm: DecisionFSM):
        """每帧调用: 同步状态 + 更新光圈 + 刷新画面"""
        self._sim.sync_to_mujoco()
        self._sim.update_status_rings(fsm)
        self._viewer.sync()
    
    def close(self):
        """关闭 viewer"""
```

## 7. main.py 改动

```diff
+    elif viz_mode == 'mujoco':
+        from simulation.mujoco_simulator import MuJoCoSimulator
+        from simulation.mujoco_visualizer import MuJoCoVisualizer
+        
+        # 替换 simulator 为 MuJoCo 版本
+        simulator = MuJoCoSimulator(
+            num_blue=NUM_ROBOTS_PER_TEAM,
+            num_yellow=NUM_ROBOTS_PER_TEAM
+        )
+        world_provider = WorldStateProvider(simulator)
+        robot_action = MockRobotAction(simulator)
+        visualizer = MuJoCoVisualizer(simulator)
```

## 8. 命令行接口

```bash
# 基础用法
python3 main.py --viz mujoco

# 指定场景
python3 main.py --viz mujoco --scenario pass

# 自定义时长
python3 main.py --viz mujoco --duration 60

# 无渲染 + CSV导出 (兼容现有)
python3 main.py --headless --export-csv --duration 30

# Matplotlib (兼容现有, 不变)
python3 main.py --viz matplotlib --scenario shoot
```

## 9. 机器人模型加载策略

| 情况 | 行为 |
|------|------|
| `assets/meshes/booster_t1.stl` 存在 | 加载真实 3D 模型替换几何体 |
| 文件不存在 | 使用几何体组合 (box + sphere + cylinder) 近似 |

MuJoCo XML 方案：
- `soccer_full.xml` — 使用几何体的场景（始终可用）
- `soccer_full_mesh.xml` — 引用 mesh 文件的场景（需要 STL 存在时生成）

Python 侧自动检测并使用合适的 XML。

## 10. 不做的范围

- ❌ 不修改 strategy/ 任何文件
- ❌ 不修改 decision/ 任何文件
- ❌ 不修改 common/world_state.py (已有 z 字段)
- ❌ 不修改 common/robot_action.py
- ❌ 不替换 2D 物理引擎
- ❌ 不增加 Webots 支持（如需可后续）
- ❌ 不增加多相机/录屏（保持简单的自由视角）

## 11. 验收标准

- [ ] `python3 main.py --viz mujoco` 可启动 3D 窗口
- [ ] 3v3 场景，蓝/黄 6 个机器人 + 足球同时可见
- [ ] 鼠标拖拽旋转/缩放/平移正常工作
- [ ] `--scenario pass` 传球场景正确运行
- [ ] `--scenario shoot` 射门场景正确运行
- [ ] `--scenario threat` 防守场景正确运行
- [ ] 机器人底部光圈颜色随决策状态变化
- [ ] `--headless` 模式仍正常工作（无 MuJoCo 依赖）
- [ ] `--viz matplotlib` 仍正常工作
- [ ] 现有 57 个 pytest 测试全部通过
