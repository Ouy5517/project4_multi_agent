#!/usr/bin/env python3
"""
MuJoCo 最小 3D 传球演示
========================
两个圆柱机器人 + 决策状态机传球, MuJoCo 3D 窗口实时渲染。

用法:
    python demo_mujoco_pass.py
    python demo_mujoco_pass.py --duration 60
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco.viewer

from common.config import DT, FPS, MAX_TICKS, NUM_ROBOTS_PER_TEAM
from common.world_state import WorldStateProvider, create_pass_scenario_2v0
from common.robot_action import MockRobotAction
from simulation.mujoco_simulator import MuJoCoSimulator
from decision.decision_fsm import DecisionFSM


NUM_ROBOTS = 2  # 最小 demo: 2 个圆柱机器人


def parse_args():
    p = argparse.ArgumentParser(description="MuJoCo 3D 传球最小演示")
    p.add_argument("--duration", type=int, default=30, help="仿真秒数 (默认 30)")
    return p.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("  MuJoCo 3D 传球最小演示")
    print("  2 圆柱机器人 + 决策状态机 + MuJoCo Viewer")
    print("=" * 60)
    print(f"  时长: {args.duration}s  |  机器人: {NUM_ROBOTS}")
    print("  操作: 鼠标拖拽旋转视角, 滚轮缩放, 关闭窗口结束")
    print("=" * 60)

    try:
        simulator = MuJoCoSimulator(num_blue=NUM_ROBOTS, num_yellow=0)
    except FileNotFoundError as e:
        print(f"  错误: {e}")
        sys.exit(1)

    # 加载传球场景初始布局
    pass_ws = create_pass_scenario_2v0()
    simulator.load_world_state(pass_ws)

    world_provider = WorldStateProvider(simulator)
    robot_action = MockRobotAction(simulator)
    fsm = DecisionFSM(pass_ws, robot_action, num_robots=NUM_ROBOTS)

    total_ticks = min(int(args.duration / DT), MAX_TICKS)
    print(f"\n  总步数: {total_ticks}\n")

    with mujoco.viewer.launch_passive(
            simulator.model, simulator.data) as viewer:
        # 初始相机
        viewer.cam.lookat[:] = [0, 0, 0]
        viewer.cam.distance = 12
        viewer.cam.elevation = -35
        viewer.cam.azimuth = 90

        start = time.time()
        for tick in range(total_ticks):
            if not viewer.is_running():
                break

            loop_start = time.time()

            # 先决策、再物理，保证本帧动作立刻生效
            world_state = world_provider.get()
            fsm.update(world_state, DT)
            simulator.update(DT)

            viewer.sync()

            elapsed = time.time() - loop_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

            if tick > 0 and tick % (FPS * 5) == 0:
                summary = fsm.get_decision_summary()
                states = summary.get("state_distribution", {})
                print(f"  [t={world_state.timestamp:.0f}s] states={states}")

    elapsed_total = time.time() - start
    summary = fsm.get_decision_summary()
    print(f"\n{'=' * 60}")
    print(f"  演示结束  实际耗时: {elapsed_total:.1f}s")
    print(f"  决策统计: {summary.get('state_distribution', {})}")
    print(f"{'=' * 60}\n")


if __name__ == "__main__":
    main()
