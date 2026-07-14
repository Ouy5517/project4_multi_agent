from dataclasses import asdict

from common.events import ActionEvent, DecisionEvent, OutcomeEvent


def test_action_event_is_json_serializable_dict():
    event = ActionEvent(
        event_id="evt-1",
        tick=3,
        timestamp=0.1,
        robot_id=0,
        action="kick",
        params={"power": 40.0},
        accepted=True,
    )
    assert asdict(event) == {
        "event_id": "evt-1",
        "tick": 3,
        "timestamp": 0.1,
        "robot_id": 0,
        "action": "kick",
        "params": {"power": 40.0},
        "accepted": True,
        "reject_code": None,
    }


def test_decision_and_outcome_events_include_reason_and_failure_code():
    decision = DecisionEvent(
        tick=4,
        timestamp=0.2,
        robot_id=1,
        state="PASS",
        role="supporter",
        reason_code="PASS_AVAILABLE",
        reason="safe forward teammate",
    )
    outcome = OutcomeEvent(
        tick=5,
        timestamp=0.3,
        outcome="pass_received",
        success=True,
        metrics={"distance": 1.2},
    )
    assert asdict(decision)["reason_code"] == "PASS_AVAILABLE"
    assert asdict(outcome)["failure_code"] is None
