#!/usr/bin/env python3.10
"""Final mock 2v2 cooperation demo using real TeamStrategy decisions."""
from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT = Path("/home/plon/Workspace/booster_soccer_project")
sys.path.insert(0, str(PROJECT))

from common.robot_action import RobotAction
from common.world_state import WorldState
from strategy.team_strategy import TeamStrategy


def now() -> str:
    return datetime.now().isoformat()


class MockRobotActionAdapter:
    """Action layer only: smooth mock interpolation records intended motion."""

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        self.actions: list[dict] = []

    def execute(self, scenario: str, actions: list[RobotAction]) -> None:
        for action in actions:
            for step in range(1, 6):
                self.actions.append(
                    {
                        "time": now(),
                        "scenario": scenario,
                        "adapter": "MockRobotActionAdapter",
                        "robot_id": action.robot_id,
                        "strategy": action.action_type.value,
                        "interpolation_step": step,
                        "interpolation_steps": 5,
                        "target": action.target,
                    }
                )

    def save(self) -> None:
        with (self.run_dir / "actions.jsonl").open("w", encoding="utf-8") as handle:
            for row in self.actions:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def world_state(name: str, robots: list[dict], opponents: list[dict], ball: tuple[float, float]) -> WorldState:
    return WorldState.from_dict(
        {
            "scenario_name": name,
            "timestamp": time.time(),
            "ball": {"x": ball[0], "y": ball[1]},
            "robots": robots,
            "opponents": opponents,
            "our_goal": {"x": -3.3, "y": 0.0},
            "enemy_goal": {"x": 3.3, "y": 0.0},
            "field_width": 7.0,
            "field_height": 5.0,
        }
    )


def blue(robot_id: str, x: float, y: float, role: str, has_ball: bool = False) -> dict:
    return {"robot_id": robot_id, "team": "BLUE", "x": x, "y": y, "theta": 0.0, "role": role, "has_ball": has_ball}


def red(opponent_id: str, x: float, y: float) -> dict:
    return {"opponent_id": opponent_id, "x": x, "y": y}


def build_scenarios() -> dict[str, WorldState]:
    return {
        "A_PASS": world_state(
            "A_PASS",
            [blue("T1_BLUE_1", -1.0, 0.0, "BALL_HANDLER", True), blue("T1_BLUE_2", 0.8, 1.0, "SUPPORT")],
            [red("T1_RED_1", 1.8, -1.2), red("T1_RED_2", 2.2, 1.6)],
            (-1.0, 0.0),
        ),
        "B_DRIBBLE": world_state(
            "B_DRIBBLE",
            [blue("T1_BLUE_1", -1.0, 0.0, "BALL_HANDLER", True), blue("T1_BLUE_2", 1.0, 0.0, "SUPPORT")],
            [red("T1_RED_1", 2.0, -1.0), red("T1_RED_2", -0.1, 0.03)],
            (-1.0, 0.0),
        ),
        "C_SHOOT": world_state(
            "C_SHOOT",
            [blue("T1_BLUE_1", 2.2, 0.0, "BALL_HANDLER", True), blue("T1_BLUE_2", 1.2, 1.0, "SUPPORT")],
            [red("T1_RED_1", -1.5, -1.0), red("T1_RED_2", -1.2, 1.0)],
            (2.2, 0.0),
        ),
        "D_BLOCK": world_state(
            "D_BLOCK",
            [blue("T1_BLUE_1", -1.2, -0.6, "SUPPORT"), blue("T1_BLUE_2", -0.8, 0.6, "BLOCK")],
            [red("T1_RED_1", 0.2, 0.0), red("T1_RED_2", 1.0, -0.7)],
            (0.25, 0.0),
        ),
    }


def main() -> int:
    run_id = os.environ.get("RUN_ID", f"mock_{int(time.time())}")
    run_dir = Path(os.environ.get("FINAL_SUBMISSION_RUN_DIR", PROJECT / "results" / "final_submission" / f"mock_{run_id}"))
    run_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("MOCK 2v2 COOPERATION DEMO")
    print("Actions use MockRobotActionAdapter")
    print("Strategies use real TeamStrategy")
    print("=" * 72)

    strategy = TeamStrategy()
    adapter = MockRobotActionAdapter(run_dir)
    decisions: list[dict] = []
    world_rows: list[dict] = []
    actual: dict[str, str] = {}

    for scenario, state in build_scenarios().items():
        actions = strategy.decide(state)
        adapter.execute(scenario, actions)
        first = actions[0].action_type.value if actions else "HOLD"
        actual[scenario] = first
        row = {
            "time": now(),
            "scenario": scenario,
            "actual_strategy": first,
            "summary": strategy.last_decision_summary,
            "actions": [action.to_dict() for action in actions],
            "mock": True,
            "real_strategy_engine": True,
        }
        decisions.append(row)
        world_rows.append({"time": now(), "scenario": scenario, "world_state": state.to_dict()})
        print(f"{scenario}: {first} | {strategy.last_decision_summary}")

    adapter.save()
    with (run_dir / "decisions.jsonl").open("w", encoding="utf-8") as handle:
        for row in decisions:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    with (run_dir / "world_state.jsonl").open("w", encoding="utf-8") as handle:
        for row in world_rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")

    expected = {
        "A_PASS": "PASS",
        "B_DRIBBLE": "DRIBBLE",
        "C_SHOOT": "SHOOT",
        "D_BLOCK": "BLOCK",
    }
    scenarios = {
        key: {"expected": value, "actual": actual.get(key), "passed": actual.get(key) == value}
        for key, value in expected.items()
    }
    (run_dir / "scenarios.json").write_text(json.dumps(scenarios, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    summary = {
        "run_id": run_id,
        "mock": True,
        "real_strategy_engine": True,
        "mock_action_adapter": True,
        "scenario_a_strategy": actual.get("A_PASS"),
        "scenario_b_strategy": actual.get("B_DRIBBLE"),
        "scenario_c_strategy": actual.get("C_SHOOT"),
        "scenario_d_strategy": actual.get("D_BLOCK"),
        "all_scenarios_passed": all(item["passed"] for item in scenarios.values()),
        "gui_label_required": "MOCK 2v2 COOPERATION DEMO",
        "timestamp": now(),
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print("ACTION REQUIRED:")
    print("请使用 Win+Shift+S 截取当前Webots Mock 2v2画面并保存为：")
    print("outputs/screenshots/final_mock_2v2.png")
    time.sleep(30)
    return 0 if summary["all_scenarios_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
