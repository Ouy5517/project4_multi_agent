#!/usr/bin/python3.10
"""
End-to-end pass execution demo.

1. Construct game state from scenario JSON
2. Load pass strategy config
3. Run pass strategy
4. Show all candidate scores and eliminations
5. Generate execution plan
6. Dry-run: Rotate → Approach → Align → Kick → Stop
7. Explain decisions
"""

from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

PROJECT = Path("/home/plon/Workspace/booster_soccer_project")
sys.path.insert(0, str(PROJECT))

from common.world_state import WorldState, Point
from strategy.pass_strategy import PassConfig, PassStrategy
from integration import PassExecutionAdapter, DryRunRpcClient, Phase


def main():
    print("=" * 70)
    print("Booster T1 — Pass Execution End-to-End Demo")
    print(f"Time: {datetime.now().isoformat()}")
    print("=" * 70)

    # ── Load scenario ──
    scenario_path = PROJECT / "config" / "scenarios" / "pass_scenario_01.json"
    if scenario_path.exists():
        from common.world_state import load_world_state
        world = load_world_state(scenario_path)
        print(f"\n[1] Scenario loaded from: {scenario_path}")
    else:
        print(f"\n[1] Scenario file not found: {scenario_path}")
        print("    Using built-in scenario.")
        world = _build_inline_scenario()

    print(f"    {world.summary()}")

    # ── Config & Strategy ──
    config = PassConfig()
    strategy = PassStrategy(config)

    carrier = world.ball_carrier()
    if not carrier:
        print("\n[FAIL] No ball carrier in scenario.")
        return
    print(f"\n    Ball carrier: {carrier.robot_id} at ({carrier.x:.2f}, {carrier.y:.2f})")

    # ── Run strategy ──
    print("\n[2] Running pass strategy...")
    decision = strategy.decide_pass(world, carrier.robot_id)

    # ── Candidate scores ──
    print("\n[3] Candidate Scores:")
    for c in decision.component_scores:
        eliminated_mark = " [ELIMINATED]" if c.eliminated else ""
        print(f"  {c.receiver_id}: total={c.total_score:.2f} dist={c.distance_score:.2f} "
              f"safety={c.safety_score:.2f} space={c.space_score:.2f} "
              f"line={c.line_score:.2f} adv={c.advance_score:.2f} "
              f"attack={c.attack_score:.2f} risk={c.risk:.2f}{eliminated_mark}")
        for reason in c.elimination_reasons:
            print(f"    ⛔ {reason}")
        if c.target_point:
            print(f"    target=({c.target_point.x:.2f}, {c.target_point.y:.2f}) "
                  f"pass_time={c.pass_time:.2f}s clearance={c.min_line_clearance:.2f}m")

    # ── Decision ──
    tgt = decision.target_point
    print(f"\n[4] Decision:")
    print(f"  should_pass: {decision.should_pass}")
    print(f"  receiver: {decision.receiver_id}")
    print(f"  target: ({tgt.x:.2f}, {tgt.y:.2f})" if tgt else "  target: None")
    print(f"  pass_speed: {decision.pass_speed:.2f} m/s")
    print(f"  risk: {decision.risk_level}")
    print(f"  reason: {decision.reason}")

    # ── Elimination analysis ──
    print("\n[5] Elimination Analysis:")
    for c in decision.component_scores:
        if c.receiver_id == decision.receiver_id:
            print(f"  ✅ SELECTED: {c.receiver_id}")
        elif c.eliminated:
            reasons = c.elimination_reasons or ["lower total score"]
            print(f"  ❌ REJECTED: {c.receiver_id} — {'; '.join(reasons)}")
        else:
            print(f"  ➖ NOT SELECTED: {c.receiver_id} — lower score ({c.total_score:.2f})")

    # ── Build execution plan ──
    print("\n[6] Building Execution Plan...")
    adapter = PassExecutionAdapter(
        rpc_client=DryRunRpcClient(),
        mode="dry_run",
        kick_enabled=False,
    )

    carrier_pos = (carrier.x, carrier.y)
    ball_pos = (world.ball.x, world.ball.y) if world.ball else None

    plan = adapter.build_plan(decision, robot_position=carrier_pos, ball_position=ball_pos)

    print(f"    mode={plan.mode}, can_kick={plan.can_kick}")
    for i, step in enumerate(plan.steps):
        status = f" [{step.status}]" if step.status != "pending" else ""
        dur = f" ({step.duration:.1f}s)" if step.duration > 0 else ""
        print(f"  [{i}] {step.phase.value}{status}: {step.description}{dur}")

    # ── Dry-run execution ──
    print("\n[7] Dry-Run Execution:")
    results = adapter.execute_plan(plan)
    ok = sum(1 for r in results if r.get("success"))
    print(f"    {ok}/{len(results)} steps successful")

    # ── Save outputs ──
    out_dir = PROJECT / "results"
    out_dir.mkdir(exist_ok=True)

    output = {
        "timestamp": datetime.now().isoformat(),
        "scenario": world.summary(),
        "decision": decision.to_dict(),
        "plan": [{"phase": s.phase.value, "command": s.command,
                   "description": s.description, "status": s.status}
                  for s in plan.steps],
        "results_count": len(results),
        "success_count": ok,
    }

    json_path = out_dir / "pass_execution_demo.json"
    with open(json_path, "w") as f:
        json.dump(output, f, indent=2, default=str)
    print(f"\n  Full: {json_path}")

    jsonl_path = out_dir / "pass_execution_demo.jsonl"
    with open(jsonl_path, "a") as f:
        f.write(json.dumps(output, default=str) + "\n")
    print(f"  JSONL: {jsonl_path}")

    print("\n" + "=" * 70)
    print("Demo complete.")
    print("=" * 70)


def _build_inline_scenario() -> WorldState:
    """Build a simple passing scenario inline."""
    from common.world_state import Ball, RobotState, OpponentState

    return WorldState(
        timestamp=0.0,
        ball=Ball(x=0.05, y=0.0, vx=0.0, vy=0.0),
        robots=[
            RobotState(robot_id="T1_A", team="blue", x=0.0, y=0.0, theta=0.0,
                       role="carrier", has_ball=True, vx=0.0, vy=0.0),
            RobotState(robot_id="T1_B", team="blue", x=2.0, y=-0.5, theta=0.0,
                       role="support", has_ball=False, vx=0.0, vy=0.0),
            RobotState(robot_id="T1_C", team="blue", x=2.1, y=0.5, theta=0.0,
                       role="support", has_ball=False, vx=0.0, vy=0.0),
        ],
        opponents=[
            # Move opponents far away so at least one teammate is safe
            OpponentState(opponent_id="OPP_1", x=-1.5, y=-1.0, vx=0.0, vy=0.0),
            OpponentState(opponent_id="OPP_2", x=-1.0, y=1.0, vx=0.0, vy=0.0),
        ],
        our_goal=Point(x=-3.0, y=0.0),
        enemy_goal=Point(x=3.0, y=0.0),
        field_width=6.0,
        field_height=4.0,
        scenario_name="pass_demo_inline",
    )


if __name__ == "__main__":
    main()
