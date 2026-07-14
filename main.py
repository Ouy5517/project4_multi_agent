#!/usr/bin/env python3
"""
Booster T1 多机器人足球协同决策系统 — 主程序
================================================
题目四：Booster T1 多机器人足球协同决策系统
基于软件工程方法实现的多机器人足球协同决策演示系统。

用法:
    python3 main.py                      # 默认 Mock 模式演示
    python3 main.py --mode mock          # Mock 模式 (自包含 2D 仿真)
    python3 main.py --mode real          # 真实模式 (对接外部仿真器/SDK)
    python3 main.py --scenario pass      # 传球场景
    python3 main.py --scenario shoot     # 射门场景
    python3 main.py --scenario threat    # 防守场景
    python3 main.py --headless           # 无渲染模式 (仅日志)
    python3 main.py --duration 120       # 运行 120 秒
"""

import sys
import os
import time
import argparse
import json

# 确保项目根目录在路径中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common.config import (
    FPS, DT, MAX_TICKS, NUM_ROBOTS_PER_TEAM,
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH
)
from common.world_state import (
    WorldStateProvider, WorldState, Ball, Robot, Goal, Team, RobotRole
)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from simulation.scenarios import (
    ScenarioValidationError,
    load_scenario_into_simulator,
)
from decision.decision_fsm import DecisionFSM
from strategy.strategy_pass import PassStrategy
from strategy.strategy_dribble import DribbleStrategy
from strategy.strategy_shoot import ShootStrategy

# 真实模式桥接层 (导入失败不阻塞, 仅 --mode real 时需要)
try:
    from bridge.real_world_state import RealWorldStateProvider
    from bridge.real_robot_action import RealRobotAction
    _BRIDGE_AVAILABLE = True
except ImportError as e:
    RealWorldStateProvider = None
    RealRobotAction = None
    _BRIDGE_AVAILABLE = False


# ================================================================
# ASCII 可视化
# ================================================================

class ASCIIVisualizer:
    """终端 ASCII 可视化, 每帧打印场地状态"""

    FIELD_W_CHARS = 60
    FIELD_H_CHARS = 20

    def __init__(self):
        self.frame_count = 0

    def render(self, ws: WorldState, fsm: DecisionFSM):
        """打印当前帧"""
        self.frame_count += 1

        # 只每10帧渲染一次或关键帧
        if self.frame_count % 10 != 0:
            return

        print(f"\n{'='*60}")
        print(f"Tick: {fsm.tick_count:4d} | Time: {ws.timestamp:5.1f}s")
        print(f"{'='*60}")

        # 球信息
        b = ws.ball
        print(f"  ⚽ Ball: ({b.x:+.1f}, {b.y:+.1f}) "
              f"speed={b.speed:.2f} m/s")

        # 己方机器人
        print(f"\n  🔵 BLUE Team (us):")
        for r in ws.teammates:
            state = fsm.get_state(r.id).value if hasattr(fsm, 'get_state') else "?"
            print(f"    ID={r.id} | Pos=({r.x:+.1f}, {r.y:+.1f}) | "
                  f"Role={r.role.value:14s} | State={state}")

        # 对手
        print(f"  🟡 YELLOW Team (opponent):")
        for r in ws.opponents:
            print(f"    ID={r.id} | Pos=({r.x:+.1f}, {r.y:+.1f})")

        # 最近转换
        if fsm.transitions:
            print(f"\n  📋 Recent transitions:")
            for t in fsm.transitions[-3:]:
                print(f"    Robot {t.robot_id}: {t.from_state.value} → "
                      f"{t.to_state.value} ({t.reason})")

        print()


# ================================================================
# 命令行参数
# ================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Booster T1 多机器人足球协同决策系统"
    )
    parser.add_argument(
        '--mode', choices=['mock', 'real'], default='mock',
        help='运行模式: mock=自包含2D仿真器 (默认), real=对接外部仿真器/SDK'
    )
    scenario_choices = [
        "default",
        "pass", "shoot", "threat",
        "pass_fixed", "dribble_open", "position_block",
    ]
    parser.add_argument(
        '--scenario', default='default',
        choices=scenario_choices,
        help='预设场景 (仅在 mock 模式下生效)'
    )
    parser.add_argument(
        '--duration', type=int, default=30,
        help='仿真时长 (秒, 默认 30)'
    )
    parser.add_argument(
        '--headless', action='store_true',
        help='无渲染模式 (不输出 ASCII 可视化)'
    )
    parser.add_argument(
        '--log-dir', default='outputs',
        help='日志输出目录 (默认: outputs/)'
    )
    parser.add_argument(
        '--export-csv', action='store_true',
        help='导出决策日志为 CSV 文件'
    )
    return parser.parse_args()


# ================================================================
# 主函数
# ================================================================

def main():
    args = parse_args()

    print("=" * 60)
    print("  Booster T1 多机器人足球协同决策系统")
    print("  题目四：Multi-Robot Soccer Cooperative Decision System")
    print("=" * 60)
    print(f"  运行模式: {args.mode}")
    print(f"  仿真时长: {args.duration} 秒")
    print(f"  帧率:     {FPS} FPS")
    print(f"  蓝方:     {NUM_ROBOTS_PER_TEAM} 机器人")
    print(f"  黄方:     {NUM_ROBOTS_PER_TEAM} 机器人")
    print("=" * 60)

    # ================================================================
    # 初始化 (根据模式选择不同的 Provider 和 Action)
    # ================================================================
    simulator = None  # 仅 mock 模式使用

    if args.mode == 'mock':
        simulator = Simulator()
        if args.scenario != 'default':
            scenario_name = {
                "pass": "pass_fixed",
                "shoot": "dribble_open",
                "threat": "position_block",
            }.get(args.scenario, args.scenario)
            try:
                load_scenario_into_simulator(simulator, scenario_name)
            except ScenarioValidationError as exc:
                print(f"  错误: 场景加载失败: {exc}")
                sys.exit(3)
        world_provider = WorldStateProvider(simulator)
        robot_action = MockRobotAction(simulator)

        print(f"  Mock 场景: {args.scenario}")
        print(f"  仿真器:    内置 2D 物理引擎")

    elif args.mode == 'real':
        if not _BRIDGE_AVAILABLE:
            print("  错误: bridge 模块不可用, 请检查 bridge/ 目录")
            sys.exit(1)

        # 桩代码初始化 (等接口确定后替换为真实数据源)
        world_provider = RealWorldStateProvider(source=None)
        robot_action = RealRobotAction(sdk_client=None)

        print(f"  仿真器:    外部 (MuJoCo/Webots/实机) — 当前为桩代码")
        print(f"  注意:      --mode real 需要项目二/三的接口就绪后才能真实运行")

    # ================================================================
    # 决策引擎 (两种模式共用)
    # ================================================================
    world_state = world_provider.get()
    fsm = DecisionFSM(world_state, robot_action, NUM_ROBOTS_PER_TEAM)

    # 可视化
    visualizer = None if args.headless else ASCIIVisualizer()

    # 日志目录
    os.makedirs(args.log_dir, exist_ok=True)

    total_ticks = int(args.duration / DT)
    if total_ticks > MAX_TICKS:
        total_ticks = MAX_TICKS

    print(f"\n  总步数: {total_ticks}")
    print(f"  开始仿真...\n")

    start_time = time.time()

    # ================================================================
    # 主循环 (两种模式共用)
    # ================================================================
    for tick in range(total_ticks):
        loop_start = time.time()

        # 1. 更新物理仿真 (仅 Mock 模式)
        if args.mode == 'mock' and simulator is not None:
            simulator.update(DT)

        # 2. 获取世界状态
        world_state = world_provider.get()

        # 3. 运行决策引擎
        fsm.update(world_state, DT)

        # 4. 渲染
        if visualizer:
            visualizer.render(world_state, fsm)

        # 5. 检查球是否进球
        _check_goal(world_state)

        # 维持帧率
        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

        # 每 5 秒打印摘要
        if tick % (FPS * 5) == 0 and tick > 0:
            elapsed_total = time.time() - start_time
            print(f"  [{tick}/{total_ticks}] "
                  f"t={world_state.timestamp:.0f}s "
                  f"real_time={elapsed_total:.1f}s")

    # ================================================================
    # 结束
    # ================================================================
    elapsed_total = time.time() - start_time
    print(f"\n{'='*60}")
    print(f"  仿真完成!")
    print(f"  总步数:   {total_ticks}")
    print(f"  仿真时间: {args.duration}s")
    print(f"  实际耗时: {elapsed_total:.1f}s")
    print(f"  运行模式: {args.mode}")
    print(f"{'='*60}")

    # 打印决策摘要
    summary = fsm.get_decision_summary()
    print(f"\n  📊 决策统计:")
    print(f"    总决策记录: {summary.get('total_decisions', 0)}")
    print(f"    状态分布:   {summary.get('state_distribution', {})}")
    print(f"    角色分布:   {summary.get('role_distribution', {})}")

    # 导出 CSV
    if args.export_csv:
        csv_path = os.path.join(args.log_dir, "decision_log.csv")
        fsm.export_csv(csv_path)
        print(f"\n  📁 决策日志已导出: {csv_path}")


def _check_goal(ws: WorldState):
    """检查球是否进入球门 (简化版)"""
    ball = ws.ball
    # 左边球门 (对手进球)
    if (ball.x <= -FIELD_WIDTH/2 and
            -GOAL_WIDTH/2 <= ball.y <= GOAL_WIDTH/2):
        pass  # 黄队得分 - 可扩展
    # 右边球门 (己方进球)
    if (ball.x >= FIELD_WIDTH/2 and
            -GOAL_WIDTH/2 <= ball.y <= GOAL_WIDTH/2):
        pass  # 蓝队得分 - 可扩展


if __name__ == '__main__':
    main()
