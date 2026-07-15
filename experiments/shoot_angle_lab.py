"""
单人多角度射门实验 (Shoot Angle Lab)
=====================================
场景约定:
  - 场上仅 1 个蓝队 T1 (id=0) + 1 个球 + 右侧球门
  - MuJoCo 下隐藏 XML 中其余机器人, 只显示 robot_0
  - 不跑对抗 FSM / 不跑传球决策
  - 默认约 1 秒踢一次 (踢完后等到间隔再复位下一脚)

用法:
  python experiments/shoot_angle_lab.py
  python experiments/shoot_angle_lab.py --angles 9 --power 70
  python experiments/shoot_angle_lab.py --viz mujoco --kick-interval 1.0
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
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_WIDTH,
    GOAL_X,
    OUR_GOAL_X,
)
from common.world_state import Ball, Goal, Team, WorldState, create_shoot_angle_scenario
from decision.match_controller import MatchController
from simulation.field_simulator import Simulator


@dataclass
class ShotTrial:
    index: int
    angle_deg: float
    angle_rad: float
    target_y: float
    power: float
    result: str  # GOAL / MISS_WIDE / MISS_SHORT / TIMEOUT
    ball_final: Tuple[float, float]
    settle_time: float


def _clamp_field(x: float, y: float) -> Tuple[float, float]:
    x = max(-FIELD_WIDTH / 2 + 0.3, min(FIELD_WIDTH / 2 - 0.3, x))
    y = max(-FIELD_HEIGHT / 2 + 0.3, min(FIELD_HEIGHT / 2 - 0.3, y))
    return x, y


def sample_goal_targets(n: int) -> List[float]:
    """在球门线上均匀采样目标 y (含两端内缩, 避免贴门柱)。"""
    if n <= 1:
        return [0.0]
    half = GOAL_WIDTH / 2 * 0.85
    if n == 2:
        return [-half, half]
    step = 2.0 * half / (n - 1)
    return [-half + i * step for i in range(n)]


def place_shot(
    sim: Simulator,
    *,
    ball_x: float,
    ball_y: float,
    kick_dir: float,
    behind: float = 0.22,
) -> None:
    """把球和机器人放到球后方, 朝向 kick_dir。"""
    bx, by = _clamp_field(ball_x, ball_y)
    sim.ball = Ball(x=bx, y=by, vx=0.0, vy=0.0)
    rx = bx - math.cos(kick_dir) * behind
    ry = by - math.sin(kick_dir) * behind
    rx, ry = _clamp_field(rx, ry)
    robot = sim.blue_robots[0]
    robot.x, robot.y, robot.theta = rx, ry, kick_dir
    robot.kick_cooldown = 0.0
    sim.clear_move_target(0)
    sim._kick_queue.clear()
    sim._turn_targets[0] = kick_dir
    if hasattr(sim, "sync_to_mujoco"):
        sim.sync_to_mujoco()


def classify_outcome(
    ball_x: float,
    ball_y: float,
    match: MatchController,
    *,
    stopped: bool = False,
) -> str:
    scorer = match.detect_goal(ball_x, ball_y)
    if scorer == Team.BLUE:
        return "GOAL"
    out = match.detect_out_of_play(ball_x, ball_y)
    if out is not None:
        return "MISS_WIDE"
    if stopped:
        return "MISS_SHORT"
    return "IN_FLIGHT"


def _pace_until(
    deadline: float,
    sim: Simulator,
    viz,
) -> None:
    """墙钟等到 deadline, MuJoCo 模式下持续刷新画面。"""
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            break
        if viz is not None and hasattr(viz, "render_simple"):
            viz.render_simple(sim)
            time.sleep(min(DT, remain))
        else:
            time.sleep(min(0.05, remain))


def run_trial(
    sim: Simulator,
    match: MatchController,
    *,
    index: int,
    target_y: float,
    ball_x: float,
    ball_y: float,
    power: float,
    settle_s: float,
    kick_interval: float = 1.0,
    last_kick_at: Optional[float] = None,
    viz=None,
) -> Tuple[ShotTrial, float]:
    """
    执行一次射门。
    返回 (trial, 本次 queue_kick 的墙钟时间)。
    保证与上一次踢球至少间隔 kick_interval 秒。
    """
    kick_dir = math.atan2(target_y - ball_y, GOAL_X - ball_x)
    place_shot(sim, ball_x=ball_x, ball_y=ball_y, kick_dir=kick_dir)

    # 站住并对准
    for _ in range(int(0.45 / DT)):
        sim.update(DT)
        if viz is not None:
            viz.render_simple(sim)

    # 距上次踢球不足 1s 则等待 (视觉上「一秒踢一次」)
    if last_kick_at is not None and kick_interval > 0:
        _pace_until(last_kick_at + kick_interval, sim, viz)

    kick_at = time.time()
    sim.queue_kick(0, power, kick_dir)

    deadline = settle_s
    t = 0.0
    result = "TIMEOUT"
    while t < deadline:
        sim.update(DT)
        t += DT
        if viz is not None:
            viz.render_simple(sim)
            # 近似实时播放, 避免画面飞速闪过
            time.sleep(DT * 0.5)

        stopped = sim.ball.speed < 0.05 and t > 0.4
        outcome = classify_outcome(
            sim.ball.x, sim.ball.y, match, stopped=stopped,
        )
        if outcome in ("GOAL", "MISS_WIDE", "MISS_SHORT"):
            result = outcome
            break

    # 球已停/进门后, 若整体仍不满 kick_interval, 继续等到间隔
    if kick_interval > 0:
        _pace_until(kick_at + kick_interval, sim, viz)

    trial = ShotTrial(
        index=index,
        angle_deg=math.degrees(kick_dir),
        angle_rad=kick_dir,
        target_y=target_y,
        power=power,
        result=result,
        ball_final=(sim.ball.x, sim.ball.y),
        settle_time=t,
    )
    return trial, kick_at


class _NullViz:
    def render_simple(self, sim: Simulator) -> None:
        return


class _MujocoShotViz:
    def __init__(self, sim: Simulator):
        from simulation.mujoco_visualizer import MuJoCoVisualizer

        self._viz = MuJoCoVisualizer(sim, title="Shoot Angle Lab — 1 T1")

    def render_simple(self, sim: Simulator) -> None:
        ws = WorldState(
            ball=sim.ball,
            teammates=list(sim.blue_robots.values()),
            opponents=[],
            our_goal=Goal(x=OUR_GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            opponent_goal=Goal(x=GOAL_X, y_min=-GOAL_WIDTH / 2, y_max=GOAL_WIDTH / 2),
            timestamp=sim.timestamp,
        )

        class _StubFSM:
            def get_state(self, _rid):
                class S:
                    value = "SHOOT"
                return S()

        self._viz.render(ws, _StubFSM())


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="单人多角度射门实验")
    p.add_argument("--angles", type=int, default=7, help="球门线上采样角度数")
    p.add_argument("--power", type=float, default=90.0, help="踢球力度 0-100")
    p.add_argument("--ball-x", type=float, default=3.0, help="发球点 X")
    p.add_argument("--ball-y", type=float, default=0.0, help="发球点 Y")
    p.add_argument("--settle", type=float, default=3.0, help="单次最多等待秒数")
    p.add_argument(
        "--kick-interval", type=float, default=1.0,
        help="相邻两次踢球的最小间隔 (秒, 默认 1.0)",
    )
    p.add_argument("--viz", choices=["none", "mujoco"], default="none")
    p.add_argument("--repeat", type=int, default=1, help="整组角度重复次数")
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    targets = sample_goal_targets(args.angles)

    if args.viz == "mujoco":
        from simulation.mujoco_simulator import MuJoCoSimulator

        sim = MuJoCoSimulator(num_blue=1, num_yellow=0)
        ws = create_shoot_angle_scenario(ball_x=args.ball_x, ball_y=args.ball_y)
        sim.load_world_state(ws)
        # 只显示 1 个 T1, 隐藏 XML 里其余机器人 + 传球线
        sim.set_visible_robots([0])
        viz = _MujocoShotViz(sim)
    else:
        sim = Simulator(num_blue=1, num_yellow=0)
        ws = create_shoot_angle_scenario(ball_x=args.ball_x, ball_y=args.ball_y)
        sim.ball = Ball(x=ws.ball.x, y=ws.ball.y)
        r = sim.blue_robots[0]
        r.x, r.y, r.theta = ws.teammates[0].x, ws.teammates[0].y, ws.teammates[0].theta
        viz = _NullViz()

    match = MatchController()
    trials: List[ShotTrial] = []
    last_kick_at: Optional[float] = None

    print("=" * 64)
    print("  Shoot Angle Lab — 1 T1 / 右侧球门 / 无对抗")
    print(
        f"  ball=({args.ball_x:.2f},{args.ball_y:.2f})  power={args.power}  "
        f"angles={args.angles}  kick_interval={args.kick_interval:.1f}s"
    )
    if args.viz == "mujoco":
        print("  MuJoCo:    仅显示 robot_0")
    print("=" * 64)

    for rep in range(args.repeat):
        for i, ty in enumerate(targets):
            trial, last_kick_at = run_trial(
                sim,
                match,
                index=rep * len(targets) + i,
                target_y=ty,
                ball_x=args.ball_x,
                ball_y=args.ball_y,
                power=args.power,
                settle_s=args.settle,
                kick_interval=args.kick_interval,
                last_kick_at=last_kick_at,
                viz=viz,
            )
            trials.append(trial)
            print(
                f"  [{trial.index:02d}] dir={trial.angle_deg:+6.1f}°  "
                f"target_y={trial.target_y:+.2f}  → {trial.result:11s}  "
                f"ball=({trial.ball_final[0]:+.2f},{trial.ball_final[1]:+.2f})  "
                f"t={trial.settle_time:.2f}s"
            )

    goals = sum(1 for t in trials if t.result == "GOAL")
    print("-" * 64)
    print(f"  进球 {goals}/{len(trials)}  ({100.0 * goals / max(len(trials), 1):.0f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
