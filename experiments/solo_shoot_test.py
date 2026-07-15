#!/usr/bin/env python3
"""
单球员射空门测试 (Solo Shoot Test)
===================================
仅拉取一个球员，无人防守，通过完整的 FSM 决策管线测试射门能力。

与 shoot_angle_lab.py 的区别:
  - shoot_angle_lab: 绕开 FSM，直接 queue_kick 做多角度采样
  - solo_shoot_test: 走完整 FSM 决策链路 (IDLE → CHASE → SHOOT)，考验实际决策引擎

用法:
    python experiments/solo_shoot_test.py                      # 默认位置 (球在 3.0, 0.0)
    python experiments/solo_shoot_test.py --ball-x 3 --ball-y 1.0
    python experiments/solo_shoot_test.py --ball-x 2 --ball-y -1.2
    python experiments/solo_shoot_test.py --viz none           # 无渲染快速测试
    python experiments/solo_shoot_test.py --viz ascii          # 终端 ASCII 可视化
    python experiments/solo_shoot_test.py --viz mujoco         # MuJoCo 3D 可视化
    python experiments/solo_shoot_test.py --duration 15        # 最多跑 15 秒
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from common.config import (
    DT,
    FPS,
    FIELD_WIDTH,
    FIELD_HEIGHT,
    GOAL_WIDTH,
    GOAL_X,
    OUR_GOAL_X,
)
from common.world_state import (
    Ball,
    Goal,
    Team,
    WorldState,
    WorldStateProvider,
    create_shoot_angle_scenario,
)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM
from decision.match_controller import MatchController


# ================================================================
# 结果类型
# ================================================================

class ShotResult:
    GOAL = "GOAL"
    MISS_WIDE = "MISS_WIDE"
    MISS_TIMEOUT = "MISS_TIMEOUT"
    MISS_STOPPED = "MISS_STOPPED"


@dataclass
class SoloShootReport:
    """一次射空门测试的报告"""
    ball_start: Tuple[float, float]
    ball_final: Tuple[float, float]
    result: str
    simulation_time: float
    max_duration: float
    fsm_final_state: str = ""
    fsm_shoot_entered: bool = False
    kicks_attempted: int = 0
    trajectory: List[Tuple[float, float]] = None

    def __post_init__(self):
        if self.trajectory is None:
            self.trajectory = []


# ================================================================
# ASCII 可视化 (简化版终端渲染)
# ================================================================

class SoloASCIIViz:
    """简化 ASCII 场地渲染，用于单球员测试"""

    def __init__(self):
        self.frame_count = 0

    def render(self, sim: Simulator, fsm: DecisionFSM, tick: int):
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return

        ball = sim.ball
        robot = sim.blue_robots.get(0)
        fsm_state = fsm.get_state(0) if robot else "?"

        print(f"\n  ── Tick {tick:4d} | t={sim.timestamp:5.1f}s ──")
        print(f"  ⚽ Ball:  ({ball.x:+.2f}, {ball.y:+.2f})  speed={ball.speed:.2f} m/s")
        if robot:
            print(f"  🔵 T1#0:  ({robot.x:+.2f}, {robot.y:+.2f})  "
                  f"θ={math.degrees(robot.theta):+.0f}°  state={fsm_state.value}")
        print(f"  🥅 Goal:  x={GOAL_X:.1f}  y=[{-GOAL_WIDTH/2:+.1f}, {+GOAL_WIDTH/2:+.1f}]")


# ================================================================
# MuJoCo 可视化适配器 (复用 shoot_angle_lab 的模式)
# ================================================================

class _MujocoSoloViz:
    """单球员 MuJoCo 3D 可视化"""

    def __init__(self, sim: Simulator):
        from simulation.mujoco_visualizer import MuJoCoVisualizer
        self._viz = MuJoCoVisualizer(sim, title="Solo Shoot Test — 1 T1 vs Empty Goal")

    def render(self, sim: Simulator, fsm) -> bool:
        ws = WorldState(
            ball=sim.ball,
            teammates=list(sim.blue_robots.values()),
            opponents=[],
            our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            timestamp=sim.timestamp,
        )
        return self._viz.render(ws, fsm)


# ================================================================
# 核心逻辑
# ================================================================

def _clamp_field(x: float, y: float) -> Tuple[float, float]:
    """裁剪到场地内"""
    x = max(-FIELD_WIDTH / 2 + 0.3, min(FIELD_WIDTH / 2 - 0.3, x))
    y = max(-FIELD_HEIGHT / 2 + 0.3, min(FIELD_HEIGHT / 2 - 0.3, y))
    return x, y


def place_initial_state(sim: Simulator, ball_x: float, ball_y: float) -> None:
    """把球和机器人放到射门起始位置 (球后方, 朝向球门)"""
    ws = create_shoot_angle_scenario(ball_x=ball_x, ball_y=ball_y)
    sim.ball = Ball(x=ws.ball.x, y=ws.ball.y, vx=0.0, vy=0.0)
    r = sim.blue_robots[0]
    r.x, r.y, r.theta = ws.teammates[0].x, ws.teammates[0].y, ws.teammates[0].theta
    r.kick_cooldown = 0.0
    sim._kick_queue.clear()
    sim._move_targets.clear()
    sim._turn_targets.clear()


def classify_outcome(
    sim: Simulator,
    match: MatchController,
    *,
    ball_stopped: bool = False,
    timed_out: bool = False,
) -> Optional[str]:
    """判断当前帧的结果"""
    bx, by = sim.ball.x, sim.ball.y

    # 进球检测
    scorer = match.detect_goal(bx, by)
    if scorer == Team.BLUE:
        return ShotResult.GOAL

    # 出界检测
    out = match.detect_out_of_play(bx, by)
    if out is not None:
        return ShotResult.MISS_WIDE

    # 超时
    if timed_out:
        return ShotResult.MISS_TIMEOUT

    # 球停了 (未进球, 未出界)
    if ball_stopped:
        return ShotResult.MISS_STOPPED

    return None


def run_solo_shoot(args: argparse.Namespace) -> SoloShootReport:
    """
    执行单球员射空门测试。
    返回 SoloShootReport。
    """
    # ── 1. 初始化 ──
    sim = Simulator(num_blue=1, num_yellow=0)
    place_initial_state(sim, args.ball_x, args.ball_y)

    ball_start = (sim.ball.x, sim.ball.y)

    # WorldStateProvider — 从仿真器读取状态
    provider = WorldStateProvider(sim)
    action = MockRobotAction(sim)

    # DecisionFSM — 关键: goalkeeper_id=-1 避免唯一球员被分配为门将
    fsm = DecisionFSM(
        provider.get(),
        action,
        num_robots=1,
        team=Team.BLUE,
        goalkeeper_id=-1,
    )

    # MatchController — 用于进球/出界检测
    match = MatchController()

    # ── 2. 可视化 ──
    viz = None
    if args.viz == "ascii":
        viz = SoloASCIIViz()
    elif args.viz == "mujoco":
        try:
            from simulation.mujoco_simulator import MuJoCoSimulator
            # 用 MuJoCo 仿真器替换普通仿真器
            mujoco_sim = MuJoCoSimulator(num_blue=1, num_yellow=0)
            place_initial_state(mujoco_sim, args.ball_x, args.ball_y)
            mujoco_sim.set_visible_robots([0])
            sim = mujoco_sim
            # 重建 provider / action / fsm 指向新仿真器
            provider = WorldStateProvider(sim)
            action = MockRobotAction(sim)
            fsm = DecisionFSM(
                provider.get(),
                action,
                num_robots=1,
                team=Team.BLUE,
                goalkeeper_id=-1,
            )
            viz = _MujocoSoloViz(sim)
        except ImportError as e:
            print(f"  ⚠ MuJoCo 未安装 ({e}), 回退到无渲染模式")
            args.viz = "none"

    # ── 3. 主循环 ──
    total_ticks = int(args.duration / DT)
    result: Optional[str] = None
    fsm_shoot_entered = False
    kicks_attempted = 0
    trajectory: List[Tuple[float, float]] = [(sim.ball.x, sim.ball.y)]
    stop_timer = 0.0
    ball_stopped_threshold = 0.05  # m/s
    stop_timeout = 3.0  # 球停 n 秒后判定 MISS_STOPPED
    last_print_tick = -FPS  # 每秒打印一次状态

    print(f"\n{'='*60}")
    print(f"  Solo Shoot Test — 单球员射空门")
    print(f"  球位: ({args.ball_x:.2f}, {args.ball_y:.2f})")
    print(f"  最大时长: {args.duration}s")
    print(f"  可视化: {args.viz}")
    print(f"{'='*60}\n")

    start_time = time.time()

    for tick in range(total_ticks):
        loop_start = time.time()

        # 3a. 更新物理
        sim.update(DT)

        # 3b. 获取世界状态
        ws = provider.get()

        # 3c. FSM 决策
        fsm.update(ws, DT)

        # 跟踪 FSM 状态
        state = fsm.get_state(0)
        if state.value == "SHOOT":
            fsm_shoot_entered = True

        # 3d. 检测结果
        ball_stopped = sim.ball.speed < ball_stopped_threshold
        if ball_stopped:
            stop_timer += DT
        else:
            stop_timer = 0.0

        timed_out = tick >= total_ticks - 1
        outcome = classify_outcome(
            sim, match,
            ball_stopped=(stop_timer >= stop_timeout),
            timed_out=timed_out,
        )

        # 记录轨迹 (每 3 帧记录一次)
        if tick % 3 == 0:
            trajectory.append((sim.ball.x, sim.ball.y))

        # 统计踢球次数 (从 FSM 状态转换中统计进入 SHOOT 的次数)
        kicks_attempted += sum(
            1 for t in fsm.transitions if t.to_state.value == "SHOOT"
        )

        # 3e. 渲染
        if viz is not None:
            if args.viz == "mujoco":
                if not viz.render(sim, fsm):
                    print("\n  用户关闭了 3D 窗口，结束测试")
                    result = result or ShotResult.MISS_TIMEOUT
                    break
            else:
                viz.render(sim, fsm, tick)

        # 每秒打印状态
        if tick - last_print_tick >= FPS:
            last_print_tick = tick
            elapsed = time.time() - start_time
            print(f"  [t={sim.timestamp:4.1f}s] "
                  f"ball=({sim.ball.x:+.2f}, {sim.ball.y:+.2f}) "
                  f"speed={sim.ball.speed:.2f} "
                  f"state={state.value:7s} "
                  f"elapsed={elapsed:.1f}s")

        # 结果已出
        if outcome is not None:
            result = outcome
            break

        # 维持帧率
        elapsed_frame = time.time() - loop_start
        if elapsed_frame < DT:
            time.sleep(DT - elapsed_frame)

    # ── 4. 收尾 ──
    elapsed_total = time.time() - start_time
    if result is None:
        result = ShotResult.MISS_TIMEOUT

    # 关闭可视化
    if viz is not None and hasattr(viz, '_viz') and hasattr(viz._viz, 'close'):
        viz._viz.close()

    # 打印决策摘要
    summary = fsm.get_decision_summary()
    state_dist = summary.get('state_distribution', {})

    return SoloShootReport(
        ball_start=ball_start,
        ball_final=(sim.ball.x, sim.ball.y),
        result=result,
        simulation_time=sim.timestamp,
        max_duration=args.duration,
        fsm_final_state=fsm.get_state(0).value,
        fsm_shoot_entered=fsm_shoot_entered,
        kicks_attempted=kicks_attempted,
        trajectory=trajectory,
    )


# ================================================================
# 报告输出
# ================================================================

def print_report(report: SoloShootReport):
    """格式化输出测试报告"""
    print(f"\n{'='*60}")
    print(f"  测试报告")
    print(f"{'='*60}")

    # 结果图标 (ASCII, 避免 Windows GBK 编码错误)
    icon = {"GOAL": "[GOAL]", "MISS_WIDE": "[WIDE]", "MISS_TIMEOUT": "[TIMEOUT]", "MISS_STOPPED": "[STOP]"}
    print(f"  结果:       {icon.get(report.result, '[????]')} {report.result}")

    print(f"  球起始:     ({report.ball_start[0]:+.2f}, {report.ball_start[1]:+.2f})")
    print(f"  球终点:     ({report.ball_final[0]:+.2f}, {report.ball_final[1]:+.2f})")
    print(f"  仿真时长:   {report.simulation_time:.2f}s / {report.max_duration:.0f}s")
    print(f"  FSM 最终状态: {report.fsm_final_state}")
    print(f"  进入过 SHOOT: {'是' if report.fsm_shoot_entered else '否'}")
    print(f"  踢球次数:   {report.kicks_attempted}")
    print(f"  轨迹点数:   {len(report.trajectory)}")

    # 简短分析
    if report.result == "GOAL":
        print(f"\n  *** 射门成功！球进了空门。 ***")
    elif report.result == "MISS_WIDE":
        print(f"\n  --- 球偏出界外。")
    elif report.result == "MISS_STOPPED":
        print(f"\n  --- 球中途停止，力度不足或方向偏差。")
    elif report.result == "MISS_TIMEOUT":
        if not report.fsm_shoot_entered:
            print(f"\n  --- FSM 未进入 SHOOT 状态，可能球位太远或角度不好。")
        else:
            print(f"\n  --- 超时未进球。")

    print(f"{'='*60}\n")


# ================================================================
# CLI
# ================================================================

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Solo Shoot Test — 单球员射空门测试 (FSM 决策管线)",
    )
    p.add_argument(
        "--ball-x", type=float, default=3.0,
        help="发球点 X 坐标 (默认 3.0, 球门在 %.1f)" % GOAL_X,
    )
    p.add_argument(
        "--ball-y", type=float, default=0.0,
        help="发球点 Y 坐标 (默认 0.0, 球门宽 %.1f)" % GOAL_WIDTH,
    )
    p.add_argument(
        "--duration", type=float, default=10.0,
        help="最大仿真时长 (秒, 默认 10)",
    )
    p.add_argument(
        "--viz", choices=["none", "ascii", "mujoco"], default="none",
        help="可视化方式 (默认 none)",
    )
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # 参数校验
    if abs(args.ball_x) > FIELD_WIDTH / 2:
        print(f"  ⚠ ball_x={args.ball_x} 超出场地范围 [{-FIELD_WIDTH/2:.1f}, {FIELD_WIDTH/2:.1f}]")
    if abs(args.ball_y) > FIELD_HEIGHT / 2:
        print(f"  ⚠ ball_y={args.ball_y} 超出场地范围 [{-FIELD_HEIGHT/2:.1f}, {FIELD_HEIGHT/2:.1f}]")

    report = run_solo_shoot(args)
    print_report(report)

    return 0 if report.result == "GOAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
