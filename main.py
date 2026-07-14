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
    python3 main.py --viz matplotlib     # 2D 图形可视化 (传球连线等)
    python3 main.py --scenario pass --viz matplotlib --duration 30
    python3 main.py --scenario pass --viz matplotlib --export-gif outputs/videos/pass.gif
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
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM
from decision.match_controller import MatchController
from common.world_state import (
    WorldStateProvider, SCENARIOS, WorldState, Ball, Robot, Goal, Team, RobotRole
)
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
    parser.add_argument(
        '--scenario', default='default',
        choices=list(SCENARIOS.keys()),
        help='预设场景 (仅在 mock 模式下生效)'
    )
    parser.add_argument(
        '--duration', type=int, default=30,
        help='仿真时长 (秒, 默认 30)'
    )
    parser.add_argument(
        '--headless', action='store_true',
        help='无渲染模式 (不输出可视化)'
    )
    parser.add_argument(
        '--viz', choices=['ascii', 'matplotlib', 'mujoco'], default=None,
        help='可视化方式: ascii=终端文本 (默认), matplotlib=2D图形窗口, mujoco=MuJoCo 3D窗口'
    )
    parser.add_argument(
        '--export-gif', default=None, metavar='PATH',
        help='将 matplotlib 可视化导出为 GIF (需 --viz matplotlib)'
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
        # MuJoCo 模式使用 MuJoCoSimulator (继承 Simulator, 接口兼容)
        if args.viz == 'mujoco':
            from simulation.mujoco_simulator import MuJoCoSimulator
            simulator = MuJoCoSimulator(
                num_blue=NUM_ROBOTS_PER_TEAM,
                num_yellow=NUM_ROBOTS_PER_TEAM
            )
            print(f"  仿真器:    2D 物理 + MuJoCo 3D 渲染")
        else:
            simulator = Simulator()

        world_provider = WorldStateProvider(simulator)
        robot_action = MockRobotAction(simulator)

        # 预设场景 (MuJoCo 模式需要 load_world_state 来同步初始位置)
        if args.scenario != 'default':
            scenario_ws = SCENARIOS[args.scenario]()
            if args.viz == 'mujoco':
                simulator.load_world_state(scenario_ws)
            world_provider.set_mock(scenario_ws)

        if args.viz != 'mujoco':
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
    blue_gk_id = NUM_ROBOTS_PER_TEAM - 1
    yellow_ids = [10 + i for i in range(NUM_ROBOTS_PER_TEAM)]
    yellow_gk_id = yellow_ids[-1]
    blue_fsm = DecisionFSM(
        world_state,
        robot_action,
        num_robots=NUM_ROBOTS_PER_TEAM,
        team=Team.BLUE,
        goalkeeper_id=blue_gk_id,
    )
    yellow_fsm = DecisionFSM(
        world_state.perspective_for(Team.YELLOW),
        robot_action,
        robot_ids=yellow_ids,
        team=Team.YELLOW,
        goalkeeper_id=yellow_gk_id,
    )
    # 兼容旧代码/可视化仍用 fsm 指蓝队
    fsm = blue_fsm
    match = MatchController(
        blue_gk_id=blue_gk_id,
        yellow_gk_id=yellow_gk_id,
        blue_ids=list(range(NUM_ROBOTS_PER_TEAM)),
        yellow_ids=list(yellow_ids),
    )
    if args.mode == 'mock' and simulator is not None:
        match.begin_kickoff(simulator, blue_fsm, yellow_fsm, kicking_team=Team.BLUE)

    # 可视化
    visualizer = None
    viz_mode = 'none' if args.headless else (args.viz or 'ascii')

    print(f"  双队 FSM:  蓝攻右门 / 黄攻左门 (争球对抗)")
    print(f"  门将:      蓝#{blue_gk_id} / 黄#{yellow_gk_id}")
    print(f"  规则:      防重叠 / 越位 / 门底禁抢 / 进球开球 / 出界任意球")
    print(f"  算法:      Booster kickDir / circle-back / 门线 / Assist / cost / isAngleGood")

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
            if args.export_gif:
                print(f"  GIF 导出:  {args.export_gif}")
        except ImportError as e:
            print(f"  错误: matplotlib 未安装 ({e})")
            print(f"  请运行: pip install matplotlib")
            sys.exit(1)
    elif viz_mode == 'mujoco':
        try:
            from simulation.mujoco_visualizer import MuJoCoVisualizer
            visualizer = MuJoCoVisualizer(
                simulator,
                title=f"Booster T1 — {args.scenario}",
            )
            print(f"  可视化:    MuJoCo 3D 窗口")
            print(f"  操作:      鼠标拖拽旋转 | 滚轮缩放 | 右键平移 | 关窗口结束")
        except ImportError as e:
            print(f"  错误: MuJoCo 未安装 ({e})")
            print(f"  请运行: pip install mujoco")
            sys.exit(1)

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

        match.update_cooldown(DT)

        if simulator is not None:
            match.tick_set_piece(simulator)

        # 2. 获取世界状态
        world_state = world_provider.get()

        # 3. 双队决策 (定点球保持期内暂停)
        if not match.frozen:
            blue_view = world_state
            yellow_view = world_state.perspective_for(Team.YELLOW)
            blue_fsm.update(blue_view, DT)
            yellow_fsm.update(yellow_view, DT)
            _sync_roles_to_sim(simulator, blue_view)
            _sync_roles_to_sim(simulator, yellow_view)

        # 4. 渲染
        if visualizer:
            if viz_mode == 'mujoco':
                if not visualizer.render(world_state, blue_fsm, yellow_fsm):
                    print(f"\n  用户关闭了 3D 窗口, 结束仿真")
                    break
            else:
                visualizer.render(world_state, blue_fsm)

        # 5. 进球 / 出界
        if simulator is not None:
            scorer = match.detect_goal(simulator.ball.x, simulator.ball.y)
            if scorer is not None:
                match.handle_goal(
                    scorer, simulator, blue_fsm, yellow_fsm,
                    timestamp=world_state.timestamp,
                )
            else:
                out_kind = match.detect_out_of_play(
                    simulator.ball.x, simulator.ball.y,
                )
                if out_kind is not None:
                    bx, by = simulator.ball.x, simulator.ball.y
                    if out_kind == "goalline":
                        attacking = Team.BLUE if bx < 0 else Team.YELLOW
                    else:
                        attacking = Team.BLUE if bx > 0 else Team.YELLOW
                    match.begin_freekick(
                        simulator, blue_fsm, yellow_fsm,
                        attacking_team=attacking,
                        ball_x=bx, ball_y=by,
                    )

        # 维持帧率
        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

        # 每 5 秒打印摘要
        if tick % (FPS * 5) == 0 and tick > 0:
            elapsed_total = time.time() - start_time
            print(f"  [{tick}/{total_ticks}] "
                  f"t={world_state.timestamp:.0f}s "
                  f"score={match.scoreboard()} "
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
    print(f"  最终比分: {match.scoreboard()}")
    print(f"{'='*60}")

    # 打印决策摘要
    blue_summary = blue_fsm.get_decision_summary()
    yellow_summary = yellow_fsm.get_decision_summary()
    print(f"\n  📊 蓝队决策统计:")
    print(f"    总决策记录: {blue_summary.get('total_decisions', 0)}")
    print(f"    状态分布:   {blue_summary.get('state_distribution', {})}")
    print(f"    角色分布:   {blue_summary.get('role_distribution', {})}")
    print(f"\n  📊 黄队决策统计:")
    print(f"    总决策记录: {yellow_summary.get('total_decisions', 0)}")
    print(f"    状态分布:   {yellow_summary.get('state_distribution', {})}")
    print(f"    角色分布:   {yellow_summary.get('role_distribution', {})}")

    # 导出 CSV
    if args.export_csv:
        csv_path = os.path.join(args.log_dir, "decision_log.csv")
        fsm.export_csv(csv_path)
        print(f"\n  📁 决策日志已导出: {csv_path}")

    if visualizer and hasattr(visualizer, 'close'):
        visualizer.close()


def _sync_roles_to_sim(simulator, team_view: WorldState) -> None:
    """把本队视角中的角色写回仿真器机器人, 供下一帧 WorldState 读取。"""
    if simulator is None:
        return
    for robot in team_view.teammates:
        sim_r = simulator.get_robot_by_id(robot.id)
        if sim_r is not None:
            sim_r.role = robot.role


if __name__ == '__main__':
    main()
