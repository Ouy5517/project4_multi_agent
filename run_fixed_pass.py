#!/usr/bin/env python3
"""
固定坐标点模拟队内通信与传球 — 独立入口
=========================================
基于题目四实施手册第7节的固定点传球方案：
  R1(2.0,4.0) → P=(7.0,4.0) ← R2(5.0,1.5)

用法:
    python3 run_fixed_pass.py                      # 默认场景
    python3 run_fixed_pass.py --target-x 8.0 --target-y 3.0
    python3 run_fixed_pass.py --duration 20.0
    python3 run_fixed_pass.py --export-csv
"""

from __future__ import annotations

import argparse
import json
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common.models import Vec2
from decision.pass_fsm import PassConfig
from simulation.fixed_point_simulator import FixedPointSimulator, export_events_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="题目四：固定坐标点模拟队内通信与传球"
    )
    parser.add_argument("--target-x", type=float, default=7.0,
                        help="固定接球点 X 坐标 (默认: 7.0)")
    parser.add_argument("--target-y", type=float, default=4.0,
                        help="固定接球点 Y 坐标 (默认: 4.0)")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="仿真时长/秒 (默认: 15.0)")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="时间步长/秒 (默认: 0.1)")
    parser.add_argument("--export-csv", nargs="?", const="outputs/decision_log.csv",
                        help="导出 CSV；可省略路径使用默认值")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = PassConfig(fixed_target=Vec2(args.target_x, args.target_y))
    simulator = FixedPointSimulator(config)
    result = simulator.run(duration_s=args.duration, dt=args.dt)

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
        output_path = export_events_csv(result.events, args.export_csv)
        print(f"\nCSV 已生成：{output_path}")

    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
