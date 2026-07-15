# 演示录屏讲稿（约 2.5 分钟）

配合 `demo_record.ps1` 使用。每幕按 Enter 推进，边录边念即可。

---

## 开场（第 1 幕，~10 秒）

> 大家好，这是软件工程短学期题目四——Booster T1 多机器人足球协同决策系统。
>
> 项目采用分层架构：`common` 定义统一世界状态和动作接口，`strategy` 实现传球、带球、射门、跑位和卡位策略，`decision` 是决策状态机，`simulation` 提供 2D Mock 仿真。
>
> 入口是 `main.py`，支持 mock 和 real 两种模式。

**画面**：终端显示项目目录结构。

---

## 默认场景（第 2 幕，~15 秒）

> 现在运行默认开球场景。系统每帧读取 WorldState，自动分配三个角色：持球者、支援者和防守者。
>
> 可以看到 ASCII 可视化：蓝方追球、黄方防守，状态机在 CHASE、PASS、BLOCK 之间切换。

**画面**：`python main.py --duration 15`（带 ASCII 球场渲染）。

---

## 传球场景（第 3 幕，~20 秒）

> 接下来是传球场景。0 号机器人持球，1 号在有利位置等待接球。
>
> 决策引擎判断路径安全后执行 PASS，日志中会记录 CHASE 到 PASS 的状态变化，两个机器人共同参与。

**画面**：headless 运行 + 末尾决策统计（PASS 次数）。

---

## 射门与防守（第 4 幕，~40 秒）

> 射门场景中，持球者接近对方球门，系统评估射门条件。
>
> 威胁场景中，对手持球逼近己方球门，防守机器人进入 BLOCK 状态执行卡位。这体现了策略切换能力。

**画面**：shoot 和 threat 两次运行的统计摘要。

---

## 测试与日志（第 5 幕，~20 秒）

> 最后运行自动化测试。当前共 57 条用例，覆盖 WorldState、五种策略、状态机和集成场景，全部通过。
>
> 决策过程可导出为 CSV，字段包含 tick、robot_id、state、role 和 action，满足课程对决策日志的要求。

**画面**：pytest 结果 + CSV 前几行预览。

---

## 收尾（~5 秒）

> 以上是本项目的核心演示。详细文档在 docs 目录，演示视频和日志在 outputs 目录。谢谢。

---

## 录制前 checklist

- [ ] 终端字体调大（16pt+），便于录屏阅读
- [ ] 先跑一遍彩排：`.\demo_record.ps1 -Quick -NoPause`
- [ ] 正式录制：`.\demo_record.ps1`（每幕按 Enter）
- [ ] 保存视频到 `outputs/videos/`

## 可视化说明

项目支持两种可视化方式：

| 方式 | 命令 | 说明 |
|------|------|------|
| 终端文本 | 默认 / `--viz ascii` | 每 10 帧打印坐标和状态 |
| **2D 图形窗口** | `--viz matplotlib` | 绿色球场、机器人、球轨迹、**传球虚线箭头** |
| **3D MuJoCo** | `demo_mujoco_pass.py` | 两个圆柱机器人 + 3D 球场窗口 |
| 导出 GIF | 加 `--export-gif PATH` | 录屏替代方案，无需手动录屏 |

### 传球可视化（推荐）

```powershell
# 实时图形窗口 — 可看到传球绿色虚线连到接球队友
python main.py --scenario pass --duration 30 --viz matplotlib

# 一键启动（双击 pass_viz.bat）
pass_viz.bat

# 3D MuJoCo 最小传球 demo（两个圆柱机器人）
python demo_mujoco_pass.py
demo_mujoco_pass.bat

# 导出 GIF 用于演示视频/PPT
python main.py --scenario pass --duration 30 --viz matplotlib --export-gif outputs/videos/pass_demo.gif
```

图形窗口中：
- 蓝色圆 = 己方机器人，边框颜色 = 当前状态（绿色边框 = PASS）
- 黄色圆 = 对手
- 白色圆 = 足球，黄色轨迹 = 球的运动路径
- **绿色虚线箭头** = 传球方向（持球者 → 接球队友）
- 绿色虚线圆 = 接球区域

## 常用命令

```powershell
cd c:\Users\13315\Desktop\gitap

# 推荐：用 bat 启动（不受执行策略限制）
demo_record.bat -Quick -NoPause
demo_record.bat

# 或直接调用 PowerShell（单次绕过策略）
powershell -ExecutionPolicy Bypass -File .\demo_record.ps1 -Quick -NoPause
```

> 若直接运行 `.\demo_record.ps1` 报「禁止运行脚本」，见下方「执行策略说明」。

## 执行策略说明

Windows 默认可能禁止运行 `.ps1` 脚本（`Restricted` 策略），这是安全机制，不是项目错误。

可选解决办法（任选其一）：

1. **用 bat 启动（推荐）**：`demo_record.bat -Quick -NoPause`
2. **单次绕过**：`powershell -ExecutionPolicy Bypass -File .\demo_record.ps1`
3. **永久放开当前用户**（需管理员权限一次）：
   ```powershell
   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
   ```
   之后可直接 `.\demo_record.ps1`
