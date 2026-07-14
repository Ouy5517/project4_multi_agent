import csv
import json

from common.events import ActionEvent, DecisionEvent, OutcomeEvent
from telemetry.run_recorder import RunRecorder


def test_run_recorder_writes_events_trajectory_and_summary(tmp_path):
    recorder = RunRecorder(
        tmp_path,
        scenario="pass_fixed",
        seed=1001,
    )
    recorder.record_event(
        ActionEvent(
            event_id="evt-1",
            tick=1,
            timestamp=0.1,
            robot_id=0,
            action="move",
            params={"x": 1.0},
        )
    )
    recorder.record_event(
        DecisionEvent(
            tick=1,
            timestamp=0.1,
            robot_id=0,
            state="CHASE",
            role="ball_carrier",
            reason_code="ASSIGNED_ROLE",
            reason="closest to ball",
        )
    )
    recorder.record_trajectory(
        tick=1,
        timestamp=0.1,
        ball=(0.0, 0.0, 0.0, 0.0),
        robots=[{"id": 0, "x": -1.0, "y": 0.0, "state": "CHASE", "role": "ball_carrier"}],
    )
    recorder.finish(
        OutcomeEvent(
            tick=2,
            timestamp=0.2,
            outcome="pass_received",
            success=True,
            metrics={"elapsed_s": 0.2},
        )
    )

    events = (tmp_path / "events.jsonl").read_text(encoding="utf-8").splitlines()
    assert json.loads(events[0])["action"] == "move"
    assert json.loads(events[1])["reason_code"] == "ASSIGNED_ROLE"

    with (tmp_path / "trajectory.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["robot_id"] == "0"
    assert rows[0]["state"] == "CHASE"

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["scenario"] == "pass_fixed"
    assert summary["success"] is True
