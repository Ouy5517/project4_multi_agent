from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, Tuple

from common.events import ActionEvent, DecisionEvent, OutcomeEvent


class RunRecorder:
    """Append-only recorder for events, trajectories, and run summary."""

    def __init__(self, output_dir: Path | str, scenario: str, seed: int):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.scenario = scenario
        self.seed = seed
        self._events_path = self.output_dir / "events.jsonl"
        self._trajectory_path = self.output_dir / "trajectory.csv"
        self._events = self._events_path.open("w", encoding="utf-8")
        self._trajectory = self._trajectory_path.open("w", encoding="utf-8", newline="")
        self._trajectory_writer = csv.DictWriter(
            self._trajectory,
            fieldnames=[
                "tick",
                "timestamp",
                "entity",
                "robot_id",
                "x",
                "y",
                "vx",
                "vy",
                "state",
                "role",
            ],
        )
        self._trajectory_writer.writeheader()

    def record_event(self, event: ActionEvent | DecisionEvent | OutcomeEvent) -> None:
        self._events.write(json.dumps(asdict(event), ensure_ascii=False, sort_keys=True) + "\n")
        self._events.flush()

    def record_trajectory(
        self,
        tick: int,
        timestamp: float,
        ball: Tuple[float, float, float, float],
        robots: Iterable[Dict[str, object]],
    ) -> None:
        for robot in robots:
            self._trajectory_writer.writerow(
                {
                    "tick": tick,
                    "timestamp": f"{timestamp:.6f}",
                    "entity": "robot",
                    "robot_id": robot["id"],
                    "x": f"{float(robot['x']):.6f}",
                    "y": f"{float(robot['y']):.6f}",
                    "vx": "",
                    "vy": "",
                    "state": robot.get("state", ""),
                    "role": robot.get("role", ""),
                }
            )
        bx, by, bvx, bvy = ball
        self._trajectory_writer.writerow(
            {
                "tick": tick,
                "timestamp": f"{timestamp:.6f}",
                "entity": "ball",
                "robot_id": "",
                "x": f"{bx:.6f}",
                "y": f"{by:.6f}",
                "vx": f"{bvx:.6f}",
                "vy": f"{bvy:.6f}",
                "state": "",
                "role": "",
            }
        )
        self._trajectory.flush()

    def finish(self, outcome: OutcomeEvent) -> None:
        self.record_event(outcome)
        summary = {
            "scenario": self.scenario,
            "seed": self.seed,
            "outcome": outcome.outcome,
            "success": outcome.success,
            "failure_code": outcome.failure_code,
            "metrics": outcome.metrics,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        self.close()

    def close(self) -> None:
        if not self._events.closed:
            self._events.close()
        if not self._trajectory.closed:
            self._trajectory.close()
