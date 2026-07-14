from __future__ import annotations

from pathlib import Path

from common.robot_action import ActionType
from common.world_state import WorldState, load_world_state
from strategy.team_strategy import TeamStrategy


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def action_types(actions: list) -> set[ActionType]:
    return {action.action_type for action in actions}


def test_pass_success_outputs_pass() -> None:
    world_state = load_world_state(PROJECT_ROOT / "scenarios" / "pass_success.json")
    actions = TeamStrategy().decide(world_state)

    assert ActionType.PASS in action_types(actions)
    assert ActionType.MOVE_TO_RECEIVE in action_types(actions)


def test_dribble_when_marked_outputs_dribble() -> None:
    world_state = load_world_state(
        PROJECT_ROOT / "scenarios" / "dribble_when_marked.json"
    )
    actions = TeamStrategy().decide(world_state)

    assert ActionType.DRIBBLE in action_types(actions)


def test_pass_receive_shoot_finally_shoots() -> None:
    world_state = load_world_state(
        PROJECT_ROOT / "scenarios" / "pass_receive_shoot.json"
    )
    strategy = TeamStrategy()
    first_actions = strategy.decide(world_state)
    second_state = WorldState.from_dict(world_state.next_states[0])
    second_actions = strategy.decide(second_state)

    assert ActionType.PASS in action_types(first_actions)
    assert ActionType.MOVE_TO_RECEIVE in action_types(first_actions)
    assert ActionType.SHOOT in action_types(second_actions)


def test_no_carrier_nearest_robot_chases_ball() -> None:
    world_state = WorldState.from_dict(
        {
            "scenario_name": "no_carrier",
            "timestamp": 5.0,
            "ball": {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0},
            "robots": [
                {
                    "robot_id": "T1_A",
                    "team": "blue",
                    "x": -2.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "role": "attacker",
                    "has_ball": False
                },
                {
                    "robot_id": "T1_B",
                    "team": "blue",
                    "x": 0.4,
                    "y": 0.1,
                    "theta": 0.0,
                    "role": "support",
                    "has_ball": False
                }
            ],
            "opponents": [],
            "our_goal": {"x": -3.0, "y": 0.0},
            "enemy_goal": {"x": 3.0, "y": 0.0},
            "field_width": 6.0,
            "field_height": 4.0
        }
    )

    actions = TeamStrategy().decide(world_state)

    assert len(actions) == 1
    assert actions[0].robot_id == "T1_B"
    assert actions[0].action_type == ActionType.CHASE_BALL
