#!/usr/bin/python3.10
"""
Pass-to-Kick end-to-end demo with optional real kick execution.

1. Build scenario → 2. PassStrategy → 3. ExecutionPlan → 4. Execute (dry-run or real)
5. If real kick: use ball_monitor output to verify ball displacement.

Default: dry-run. Use --execute-kick for real RPC calls.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

PROJECT = Path("/home/plon/Workspace/booster_soccer_project")
sys.path.insert(0, str(PROJECT))

from common.world_state import WorldState, Point, Ball, RobotState, OpponentState
from strategy.pass_strategy import PassConfig, PassStrategy
from integration import PassExecutionAdapter, DryRunRpcClient, Phase


def build_scenario() -> WorldState:
    return WorldState(
        timestamp=0.0,
        ball=Ball(x=0.05, y=0.0),
        robots=[
            RobotState(robot_id="T1_A", team="blue", x=0.0, y=0.0, theta=0.0,
                       role="carrier", has_ball=True),
            RobotState(robot_id="T1_B", team="blue", x=2.0, y=-1.0, theta=0.0,
                       role="support", has_ball=False),
            RobotState(robot_id="T1_C", team="blue", x=2.5, y=1.0, theta=0.0,
                       role="support", has_ball=False),
        ],
        opponents=[
            OpponentState(opponent_id="OPP_1", x=-2.0, y=0.0),
        ],
        our_goal=Point(x=-3.0, y=0.0),
        enemy_goal=Point(x=3.0, y=0.0),
        field_width=6.0, field_height=4.0,
        scenario_name="pass_to_kick_demo",
    )


class RealRpcClient:
    """Real ROS2 RPC client for --execute-kick mode."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from booster_interface.srv import RpcService

        rclpy.init(args=[])
        self.node = Node("pass_to_kick_client")
        self.cli = self.node.create_client(RpcService, "booster_rpc_service")
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("RPC service not available")
        self.calls = []

    def call(self, api_id: int, body: str) -> dict:
        from booster_interface.srv import RpcService
        req = RpcService.Request()
        req.msg.api_id = api_id
        req.msg.body = body
        f = self.cli.call_async(req)
        import rclpy as _rclpy
        _rclpy.spin_until_future_complete(self.node, f, timeout_sec=5.0)
        if f.done():
            resp = f.result()
            r = {
                "success": resp.msg.status == 0,
                "status_code": resp.msg.status,
                "status_name": "SUCCESS" if resp.msg.status == 0 else str(resp.msg.status),
                "response_body": resp.msg.body,
            }
        else:
            r = {"success": False, "status_code": -3, "status_name": "TIMEOUT"}
        self.calls.append(r)
        return r


def read_ball_displacement():
    """Read max ball displacement from ball_monitor output."""
    ball_path = PROJECT / "results" / "kick_ball_motion.jsonl"
    if not ball_path.exists():
        return None
    with open(ball_path) as f:
        records = [json.loads(l) for l in f if l.strip()]
    if not records:
        return None
    init = records[0].get("init_position", [0, 0, 0])
    last = records[-1]
    return {
        "init": init,
        "max_disp": last.get("max_disp", 0),
        "detected": last.get("detected", "?"),
        "kicked": last.get("max_disp", 0) > 0.05,
    }


def main():
    parser = argparse.ArgumentParser(description="Pass-to-Kick Demo")
    parser.add_argument("--execute-kick", action="store_true",
                        help="Execute real kick via ROS2 RPC (default: dry-run)")
    args = parser.parse_args()

    print("=" * 70)
    print("Booster T1 — Pass-to-Kick Demo")
    print(f"Mode: {'REAL KICK' if args.execute_kick else 'DRY-RUN'}")
    print(f"Time: {datetime.now().isoformat()}")
    print("=" * 70)

    # 1. Scenario
    world = build_scenario()
    print(f"\n[1] Scenario: {world.summary()}")

    # 2. Pass Strategy
    strategy = PassStrategy(PassConfig())
    carrier = world.ball_carrier()
    decision = strategy.decide_pass(world, carrier.robot_id)
    print(f"\n[2] Decision: should_pass={decision.should_pass}, "
          f"receiver={decision.receiver_id}, risk={decision.risk_level}")

    # 3. Candidates
    print("\n[3] Candidates:")
    for c in decision.component_scores:
        eliminated = " [ELIMINATED]" if c.eliminated else ""
        print(f"  {c.receiver_id}: total={c.total_score:.2f} risk={c.risk:.2f}{eliminated}")
        for r in c.elimination_reasons:
            print(f"    ⛔ {r}")

    # 4. Execution Plan
    kick_enabled = args.execute_kick
    rpc = RealRpcClient() if args.execute_kick else DryRunRpcClient()

    adapter = PassExecutionAdapter(
        rpc_client=rpc,
        mode="simulation" if args.execute_kick else "dry_run",
        kick_enabled=kick_enabled,
        kick_api="kVisualKick",
    )

    plan = adapter.build_plan(decision, robot_position=(carrier.x, carrier.y),
                              ball_position=(world.ball.x, world.ball.y))
    print(f"\n[4] Plan ({plan.mode}, kick={plan.can_kick}):")
    for s in plan.steps:
        print(f"  [{s.phase.value}] {s.description} [{s.status}]")

    # 5. Execute
    print(f"\n[5] Executing...")
    results = adapter.execute_plan(plan)
    ok = sum(1 for r in results if r.get("success"))
    print(f"  {ok}/{len(results)} steps OK")

    # 6. Ball check
    ball = read_ball_displacement()
    print(f"\n[6] Ball Displacement:")
    if ball:
        print(f"  Init: ({ball['init'][0]:.4f}, {ball['init'][1]:.4f}, {ball['init'][2]:.4f})")
        print(f"  Max horiz disp: {ball['max_disp']:.4f} m")
        print(f"  Detected: {ball['detected']}")
        print(f"  KICKED (>0.05m): {ball['kicked']}")
    else:
        print("  No ball monitor data (expected in dry-run)")

    # 7. Summary
    out = {
        "timestamp": datetime.now().isoformat(),
        "mode": "real" if args.execute_kick else "dry_run",
        "decision": decision.to_dict(),
        "plan": [s.phase.value for s in plan.steps],
        "results_ok": ok,
        "results_total": len(results),
        "ball": ball,
    }

    out_dir = PROJECT / "results"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "pass_to_kick_demo.json"
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\n  Output: {out_path}")

    if ball and ball["kicked"]:
        print("\n  RESULT: PASS_EXECUTION_SUCCESS — ball moved!")
    elif args.execute_kick:
        rpc_codes = [r.get("status_code") for r in results if isinstance(r, dict)]
        print(f"\n  RESULT: KICK_NOT_VERIFIED — RPC codes={rpc_codes}, ball did not move.")
        print(f"  Cause: Kick APIs return 502 (not implemented in this simulation mck build).")
    else:
        print("\n  RESULT: DRY_RUN_COMPLETE — use --execute-kick for real test.")

    print("=" * 70)


if __name__ == "__main__":
    main()
