from __future__ import annotations

import csv
import json
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
RESULTS_DIR = PROJECT_ROOT / "results"

from common.world_state import WorldState
from strategy.pass_strategy import PassConfig, PassDecision, PassStrategy


@dataclass(frozen=True)
class ExperimentCase:
    name: str
    world: WorldState
    config: PassConfig


def robot(robot_id: str, x: float, y: float, has_ball: bool = False, vx: float = 0.0, vy: float = 0.0) -> dict:
    return {
        "robot_id": robot_id,
        "team": "blue",
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "theta": 0.0,
        "role": "attacker" if has_ball else "support",
        "has_ball": has_ball,
    }


def opponent(opponent_id: str, x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> dict:
    return {"opponent_id": opponent_id, "x": x, "y": y, "vx": vx, "vy": vy}


def make_world(name: str, robots: list[dict], opponents: list[dict]) -> WorldState:
    return WorldState.from_dict(
        {
            "scenario_name": name,
            "timestamp": 0.0,
            "ball": {"x": robots[0]["x"], "y": robots[0]["y"]},
            "robots": robots,
            "opponents": opponents,
            "our_goal": {"x": -4.0, "y": 0.0},
            "enemy_goal": {"x": 4.0, "y": 0.0},
            "field_width": 8.0,
            "field_height": 5.0,
        }
    )


def build_cases() -> list[ExperimentCase]:
    default = PassConfig()
    loose = PassConfig(min_receive_clearance=0.65, hard_receive_clearance=0.4)
    strict = PassConfig(min_receive_clearance=1.15, hard_receive_clearance=0.75)
    attack_weight = PassConfig(weights={"distance": 0.05, "safety": 0.2, "space": 0.1, "line": 0.2, "advance": 0.25, "attack": 0.2})
    return [
        ExperimentCase("no_opponent", make_world("no_opponent", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 0.6, 0.8)], []), default),
        ExperimentCase("single_blocker", make_world("single_blocker", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 1.4, 0.0)], [opponent("OP_1", -0.2, 0.05)]), default),
        ExperimentCase("multi_blocker", make_world("multi_blocker", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 1.4, 0.0)], [opponent("OP_1", -1.0, 0.1), opponent("OP_2", -0.1, -0.1), opponent("OP_3", 0.8, 0.1)]), default),
        ExperimentCase("multi_teammate", make_world("multi_teammate", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", -0.5, 1.0), robot("T1_C", 1.5, -0.8)], [opponent("OP_1", -0.5, 1.3)]), default),
        ExperimentCase("fixed_point", make_world("fixed_point", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 1.0, 1.0)], [opponent("OP_1", 0.3, -0.8)]), default),
        ExperimentCase("dynamic_receive", make_world("dynamic_receive", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 0.6, 0.2, 0.0, 0.5, 0.3)], []), default),
        ExperimentCase("loose_safety", make_world("loose_safety", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 0.8, 0.6)], [opponent("OP_1", 0.9, 1.3)]), loose),
        ExperimentCase("strict_safety", make_world("strict_safety", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", 0.8, 0.6)], [opponent("OP_1", 0.9, 1.3)]), strict),
        ExperimentCase("attack_weight", make_world("attack_weight", [robot("T1_A", -2.0, 0.0, True), robot("T1_B", -0.4, 1.0), robot("T1_C", 1.7, -0.8)], []), attack_weight),
    ]


def nearest_teammate(world: WorldState) -> PassDecision:
    passer = world.ball_carrier()
    assert passer is not None
    cfg = PassConfig(allow_emergency_risky_pass=True, min_receive_clearance=0.01, hard_receive_clearance=0.0, line_static_clearance=0.01)
    return PassStrategy(cfg).decide_pass(world, passer.robot_id)


def soft_score_only(world: WorldState, cfg: PassConfig) -> PassDecision:
    passer = world.ball_carrier()
    assert passer is not None
    soft_cfg = PassConfig(**{**cfg.__dict__, "allow_emergency_risky_pass": True, "min_receive_clearance": 0.01, "hard_receive_clearance": 0.0, "line_static_clearance": 0.01})
    return PassStrategy(soft_cfg).decide_pass(world, passer.robot_id)


def full_strategy(world: WorldState, cfg: PassConfig) -> PassDecision:
    passer = world.ball_carrier()
    assert passer is not None
    return PassStrategy(cfg).decide_pass(world, passer.robot_id)


def row(case: ExperimentCase, strategy_name: str, decision: PassDecision, elapsed_ms: float) -> dict:
    risky = any(score.eliminated for score in decision.component_scores if score.receiver_id == decision.receiver_id)
    return {
        "case": case.name,
        "strategy": strategy_name,
        "should_pass": decision.should_pass,
        "receiver_id": decision.receiver_id or "",
        "risk_level": decision.risk_level,
        "total_score": f"{decision.total_score:.4f}",
        "potential_intercept": bool(risky or decision.risk_level == "HIGH"),
        "safe_target": bool(decision.should_pass and decision.risk_level != "HIGH" and not risky),
        "decision_time_ms": f"{elapsed_ms:.4f}",
        "reason": decision.reason,
    }


def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    cases = build_cases()
    rows: list[dict] = []
    details: list[dict] = []
    for case in cases:
        for name, fn in [
            ("nearest_teammate", lambda c: nearest_teammate(c.world)),
            ("soft_score_only", lambda c: soft_score_only(c.world, c.config)),
            ("full_safe_pass", lambda c: full_strategy(c.world, c.config)),
        ]:
            start = time.perf_counter()
            decision = fn(case)
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            rows.append(row(case, name, decision, elapsed_ms))
            details.append({"case": case.name, "strategy": name, "decision": decision.to_dict()})

    csv_path = RESULTS_DIR / "pass_experiment_results.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    log_path = RESULTS_DIR / "pass_experiment_log.jsonl"
    with log_path.open("w", encoding="utf-8") as file:
        for item in details:
            file.write(json.dumps(item, ensure_ascii=False) + "\n")

    summary_path = RESULTS_DIR / "pass_experiment_summary.md"
    with summary_path.open("w", encoding="utf-8") as file:
        file.write("# Pass Experiment Summary\n\n")
        file.write("| Strategy | Pass Rate | Safe Target Rate | Potential Intercept Rate | Avg Decision Time ms |\n")
        file.write("| --- | ---: | ---: | ---: | ---: |\n")
        for strategy in sorted({item["strategy"] for item in rows}):
            selected = [item for item in rows if item["strategy"] == strategy]
            pass_rate = sum(item["should_pass"] for item in selected) / len(selected)
            safe_rate = sum(item["safe_target"] for item in selected) / len(selected)
            intercept_rate = sum(item["potential_intercept"] for item in selected) / len(selected)
            avg_time = statistics.mean(float(item["decision_time_ms"]) for item in selected)
            file.write(f"| {strategy} | {pass_rate:.2f} | {safe_rate:.2f} | {intercept_rate:.2f} | {avg_time:.4f} |\n")

    print(f"CSV: {csv_path}")
    print(f"Log: {log_path}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
