from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from common.world_state import WorldState
from strategy.pass_strategy import PassConfig, PassStrategy


def build_demo_world() -> WorldState:
    return WorldState.from_dict(
        {
            "scenario_name": "pass_decision_demo",
            "timestamp": 0.0,
            "ball": {"x": -2.0, "y": 0.0},
            "robots": [
                {
                    "robot_id": "T1_A",
                    "team": "blue",
                    "x": -2.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "role": "attacker",
                    "has_ball": True,
                },
                {
                    "robot_id": "T1_B",
                    "team": "blue",
                    "x": -0.5,
                    "y": 1.0,
                    "vx": 0.2,
                    "vy": 0.0,
                    "theta": 0.0,
                    "role": "support",
                },
                {
                    "robot_id": "T1_C",
                    "team": "blue",
                    "x": 1.2,
                    "y": -0.8,
                    "vx": 0.5,
                    "vy": 0.15,
                    "theta": 0.0,
                    "role": "support",
                },
            ],
            "opponents": [
                {"opponent_id": "OP_1", "x": -0.4, "y": 1.2, "vx": 0.0, "vy": 0.0},
                {"opponent_id": "OP_2", "x": 0.2, "y": -2.0, "vx": 0.0, "vy": 0.1},
                {"opponent_id": "OP_3", "x": 2.5, "y": 1.7, "vx": -0.1, "vy": 0.0},
            ],
            "our_goal": {"x": -4.0, "y": 0.0},
            "enemy_goal": {"x": 4.0, "y": 0.0},
            "field_width": 8.0,
            "field_height": 5.0,
        }
    )


def main() -> None:
    config = PassConfig.from_yaml(PROJECT_ROOT / "config" / "pass_strategy.yaml")
    decision = PassStrategy(config).decide_pass(build_demo_world(), "T1_A")

    print("候选队友评分：")
    for score in decision.component_scores:
        reasons = ", ".join(score.elimination_reasons) or "通过"
        print(
            f"- {score.receiver_id}: distance={score.distance_score:.2f}, "
            f"safety={score.safety_score:.2f}, space={score.space_score:.2f}, "
            f"line={score.line_score:.2f}, advance={score.advance_score:.2f}, "
            f"attack={score.attack_score:.2f}, risk={score.risk:.2f}, "
            f"total={score.total_score:.2f}, reasons={reasons}"
        )

    print("\n最终决策：")
    print(f"should_pass={decision.should_pass}")
    print(f"receiver_id={decision.receiver_id}")
    if decision.target_point:
        print(f"target_point=({decision.target_point.x:.2f}, {decision.target_point.y:.2f})")
    print(f"pass_speed={decision.pass_speed:.2f}")
    print(f"risk_level={decision.risk_level}")
    print(f"reason={decision.reason}")


if __name__ == "__main__":
    main()
