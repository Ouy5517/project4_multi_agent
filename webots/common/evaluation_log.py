from __future__ import annotations

import json
from pathlib import Path

from common.robot_action import RobotAction
from common.world_state import WorldState


class EvaluationLogger:
    """将每次策略决策保存为 JSONL，并同步打印中文摘要。"""

    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

    def reset(self) -> None:
        self.log_path.write_text("", encoding="utf-8")

    def log(
        self,
        scenario_name: str,
        world_state: WorldState,
        actions: list[RobotAction],
        decision_summary: str,
    ) -> None:
        record = {
            "scenario_name": scenario_name,
            "world_state": world_state.to_dict(),
            "actions": [action.to_dict() for action in actions],
            "decision_summary": decision_summary,
        }
        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(record, ensure_ascii=False) + "\n")

        print("\n日志摘要：")
        print(f"- 场景：{scenario_name}")
        print(f"- 结论：{decision_summary}")
        for action in actions:
            print(
                f"- {action.robot_id} 选择 {action.action_type.value}："
                f"{action.reason}"
            )
