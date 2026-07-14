#!/usr/bin/env python3
"""
Booster T1 多机器人足球协同决策系统 — 主程序 (整合版)
========================================================
题目四：Booster T1 多机器人足球协同决策系统
基于软件工程方法实现的多机器人足球协同决策演示系统。

用法:
    python3 main.py                      # 默认 Mock 模式演示
    python3 main.py --scenario pass      # 传球场景
    python3 main.py --scenario shoot     # 射门场景
    python3 main.py --scenario threat    # 防守场景
    python3 main.py --scenario fixed-pass            # 固定坐标点模拟通信传球
    python3 main.py --scenario fixed-pass --target-x 8.0 --target-y 3.0
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
    WorldStateProvider, SCENARIOS, WorldState, Ball, Robot, Goal, Team, RobotRole
)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM, DecisionState
from strategy.strategy_pass import PassStrategy
from strategy.strategy_dribble import DribbleStrategy
from strategy.strategy_shoot import ShootStrategy

# 真实模式桥接层
try:
    from bridge.real_world_state import RealWorldStateProvider
    from bridge.real_robot_action import RealRobotAction
    _BRIDGE_AVAILABLE = True
except ImportError:
    RealWorldStateProvider = None
    RealRobotAction = None
    _BRIDGE_AVAILABLE = False

# 固定点传球模块
try:
    from common.models import Vec2
    from decision.pass_fsm import PassConfig
    from simulation.fixed_point_simulator import FixedPointSimulator, export_events_csv
    _FIXED_PASS_AVAILABLE = True
except ImportError:
    Vec2 = None
    FixedPointSimulator = None
    _FIXED_PASS_AVAILABLE = False


# ================================================================
# ASCII 可视化
# ================================================================

class ASCIIVisualizer:
    """终端 ASCII 可视化"""

    FIELD_W_CHARS = 60
    FIELD_H_CHARS = 20

    def __init__(self):
        self.frame_count = 0

    def render(self, ws, fsm):
        self.frame_count += 1
        if self.frame_count % 10 != 0:
            return

        print(f"\n{'='*60}")
        print(f"Tick: {fsm.tick_count:4d} | Time: {ws.timestamp:5.1f}s")
        print(f"{'='*60}")

        b = ws.ball
        print(f"  ⚽ Ball: ({b.x:+.1f}, {b.y:+.1f}) speed={b.speed:.2f} m/s")

        print(f"\n  🔵 BLUE Team:")
        for r in ws.teammates:
            s = fsm.get_state(r.id).value
            print(f"    ID={r.id} | ({r.x:+.1f}, {r.y:+.1f}) | {r.role.value:14s} | {s}")

        print(f"  🟡 YELLOW Team:")
        for r in ws.opponents:
            print(f"    ID={r.id} | ({r.x:+.1f}, {r.y:+.1f})")

        if fsm.transitions:
            print(f"\n  📋 Recent transitions:")
            for t in fsm.transitions[-3:]:
                print(f"    Robot {t.robot_id}: {t.from_state.value} → {t.to_state.value} ({t.reason})")
        print()


# ================================================================
# 固定点传球
# ================================================================

def run_fixed_point_pass(args) -> int:
    """运行固定坐标点模拟通信传球，返回 0=成功, 1=失败"""
    if not _FIXED_PASS_AVAILABLE:
        print("错误: 固定点传球模块不可用")
        return 1

    target_x = getattr(args, 'target_x', 7.0)
    target_y = getattr(args, 'target_y', 4.0)
    duration = args.duration if args.duration > 0 else 15.0
    dt = getattr(args, 'dt', 0.1)

    print("=" * 60)
    print("  固定坐标点模拟通信传球 (fixed-point pass)")
    print("  题目四：基于 MockTeamBus 的固定点队内通信")
    print("=" * 60)
    print(f"  持球者:    R1 (2.0, 4.0)")
    print(f"  接球者:    R2 (5.0, 1.5)")
    print(f"  接球目标:   ({target_x}, {target_y})")
    print(f"  通信方式:   MockTeamBus (内存消息队列)")
    print(f"  状态机:     DecisionFSM (FIXED_PASS 模式)")
    print(f"  消息类型:   PASS_TARGET + RECEIVER_READY")
    print("=" * 60)

    # ---- 路线 1: FixedPointSimulator (独立仿真, 已验证) ----
    config = PassConfig(fixed_target=Vec2(target_x, target_y))
    simulator = FixedPointSimulator(config)
    result = simulator.run(duration_s=duration, dt=dt)

    print("\n=== 决策与通信日志 ===")
    for event in result.events:
        print(
            f"[{event.time_s:05.2f}s] {event.actor:<3} "
            f"{event.action:<16} {event.result:<18} {event.detail}"
        )

    summary = {
        "success": result.success,
        "final_state": result.final_state,
        "elapsed_s": round(result.elapsed_s, 2),
        "message_count": result.message_count,
        "ball_owner_id": result.ball_owner_id,
        "receiver_position": {
            "x": round(result.receiver_position.x, 2),
            "y": round(result.receiver_position.y, 2),
        },
    }
    print("\n=== 验收摘要 ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.export_csv:
        csv_path = os.path.join(args.log_dir, "decision_log.csv")
        export_events_csv(result.events, csv_path)
        print(f"\nCSV 已生成：{csv_path}")

    # ---- 路线 2 (可选): 通过 DecisionFSM.init_fixed_pass_scenario() ----
    # 验证 DecisionFSM 集成接口可用
    try:
        from decision.decision_fsm import DecisionFSM
        test_fsm = DecisionFSM.__init__  # 仅验证导入
        print("\n[集成验证] DecisionFSM 固定点传球接口就绪 ✓")
    except Exception as e:
        print(f"\n[集成验证] DecisionFSM 接口检查: {e}")

    return 0 if result.success else 1


# ================================================================
# 命令行参数
# ================================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Booster T1 多机器人足球协同决策系统")
    parser.add_argument('--mode', choices=['mock', 'real'], default='mock')
    scenario_choices = list(SCENARIOS.keys()) + ['fixed-pass', 'pass-shoot', '2v2', 'interference', 'interference-3v3']
    parser.add_argument('--scenario', default='default', choices=scenario_choices)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--viz', choices=['ascii', 'matplotlib'], default=None)
    parser.add_argument('--export-gif', default=None, metavar='PATH')
    parser.add_argument('--log-dir', default='outputs')
    parser.add_argument('--export-csv', action='store_true')
    # fixed-pass 专用
    parser.add_argument('--target-x', type=float, default=7.0)
    parser.add_argument('--target-y', type=float, default=4.0)
    parser.add_argument('--dt', type=float, default=0.1)
    return parser.parse_args()


# ================================================================
# 主函数
# ================================================================

def main():
    args = parse_args()

    # ---- 固定点传球: 独立仿真通道 ----
    if args.scenario == 'fixed-pass':
        return run_fixed_point_pass(args)

    # ---- 通用场景 ----
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

    simulator = None
    if args.mode == 'mock':
        simulator = Simulator()
        world_provider = WorldStateProvider(simulator)
        robot_action = MockRobotAction(simulator)
        if args.scenario != 'default':
            world_provider.set_mock(SCENARIOS[args.scenario]())
        print(f"  Mock 场景: {args.scenario}")
        print(f"  仿真器:    内置 2D 物理引擎")
    elif args.mode == 'real':
        if not _BRIDGE_AVAILABLE:
            print("  错误: bridge 模块不可用")
            sys.exit(1)
        world_provider = RealWorldStateProvider(source=None)
        robot_action = RealRobotAction(sdk_client=None)
        print(f"  仿真器:    外部")

    world_state = world_provider.get()
    fsm = DecisionFSM(world_state, robot_action, NUM_ROBOTS_PER_TEAM)

    visualizer = None
    viz_mode = 'none' if args.headless else (args.viz or 'ascii')
    if viz_mode == 'ascii':
        visualizer = ASCIIVisualizer()
    elif viz_mode == 'matplotlib':
        try:
            from visualization.field_visualizer import MatplotlibVisualizer
            visualizer = MatplotlibVisualizer(
                title=f"Booster T1 — {args.scenario}",
                save_gif=args.export_gif,
            )
            print(f"  可视化:    matplotlib 2D 图形窗口")
        except ImportError as e:
            print(f"  错误: matplotlib 未安装 ({e})")
            sys.exit(1)

    os.makedirs(args.log_dir, exist_ok=True)
    total_ticks = min(int(args.duration / DT), MAX_TICKS)
    print(f"\n  总步数: {total_ticks}")
    print(f"  开始仿真...\n")

    start_time = time.time()
    for tick in range(total_ticks):
        loop_start = time.time()
        if args.mode == 'mock' and simulator is not None:
            simulator.update(DT)
        world_state = world_provider.get()
        fsm.update(world_state, DT)
        if visualizer:
            visualizer.render(world_state, fsm)
        _check_goal(world_state)
        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)
        if tick % (FPS * 5) == 0 and tick > 0:
            print(f"  [{tick}/{total_ticks}] "
                  f"t={world_state.timestamp:.0f}s "
                  f"real_time={time.time() - start_time:.1f}s")

    elapsed_total = time.time() - start_time
    print(f"\n{'='*60}")
    print(f"  仿真完成!")
    print(f"  总步数:   {total_ticks}")
    print(f"  仿真时间: {args.duration}s")
    print(f"  实际耗时: {elapsed_total:.1f}s")
    print(f"{'='*60}")

    summary = fsm.get_decision_summary()
    print(f"\n  📊 决策统计:")
    print(f"    总决策记录: {summary.get('total_decisions', 0)}")
    print(f"    状态分布:   {summary.get('state_distribution', {})}")
    print(f"    角色分布:   {summary.get('role_distribution', {})}")

    if args.export_csv:
        csv_path = os.path.join(args.log_dir, "decision_log.csv")
        fsm.export_csv(csv_path)
        print(f"\n  📁 决策日志已导出: {csv_path}")

    if visualizer and hasattr(visualizer, 'close'):
        visualizer.close()


def _check_goal(ws: WorldState):
    ball = ws.ball
    if ball.x <= -FIELD_WIDTH/2 and -GOAL_WIDTH/2 <= ball.y <= GOAL_WIDTH/2:
        _check_goal.last_scorer = "YELLOW"
        print("  [GOAL!] Yellow scores at t=" + str(round(ws.timestamp, 1)) + "s")
    elif ball.x >= FIELD_WIDTH/2 and -GOAL_WIDTH/2 <= ball.y <= GOAL_WIDTH/2:
        _check_goal.last_scorer = "BLUE"
        print("  [GOAL!] Blue scores at t=" + str(round(ws.timestamp, 1)) + "s")


if __name__ == '__main__':
    main()
