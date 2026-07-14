from __future__ import annotations

from common.world_state import Point, WorldState
from strategy.pass_strategy import PassConfig, PassStrategy


def world(
    robots: list[dict],
    opponents: list[dict],
    field_width: float = 8.0,
    field_height: float = 5.0,
) -> WorldState:
    return WorldState.from_dict(
        {
            "scenario_name": "unit",
            "timestamp": 0.0,
            "ball": {"x": robots[0]["x"], "y": robots[0]["y"]},
            "robots": robots,
            "opponents": opponents,
            "our_goal": {"x": -4.0, "y": 0.0},
            "enemy_goal": {"x": 4.0, "y": 0.0},
            "field_width": field_width,
            "field_height": field_height,
        }
    )


def passer() -> dict:
    return {
        "robot_id": "T1_A",
        "team": "blue",
        "x": -2.0,
        "y": 0.0,
        "theta": 0.0,
        "role": "attacker",
        "has_ball": True,
    }


def mate(robot_id: str, x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> dict:
    return {
        "robot_id": robot_id,
        "team": "blue",
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "theta": 0.0,
        "role": "support",
    }


def opponent(opponent_id: str, x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> dict:
    return {"opponent_id": opponent_id, "x": x, "y": y, "vx": vx, "vy": vy}


def test_teammate_completely_open() -> None:
    decision = PassStrategy(PassConfig(pass_time_margin=0.1)).decide_pass(
        world([passer(), mate("T1_B", 0.5, 1.0)], []),
        "T1_A",
    )

    assert decision.should_pass
    assert decision.receiver_id == "T1_B"
    assert decision.component_scores[0].elimination_reasons == []


def test_receive_point_near_opponent_is_hard_rejected() -> None:
    decision = PassStrategy().decide_pass(
        world([passer(), mate("T1_B", 0.5, 1.0)], [opponent("OP_1", 0.8, 0.95)]),
        "T1_A",
    )

    assert not decision.should_pass
    assert "接球点硬安全距离不足" in decision.component_scores[0].elimination_reasons


def test_static_opponent_blocks_pass_line() -> None:
    decision = PassStrategy().decide_pass(
        world([passer(), mate("T1_B", 1.5, 0.0)], [opponent("OP_1", -0.2, 0.05)]),
        "T1_A",
    )

    assert not decision.should_pass
    assert "传球线路存在可拦截对手" in decision.component_scores[0].elimination_reasons


def test_fast_opponent_cutting_into_line_blocks() -> None:
    cfg = PassConfig(opponent_max_speed=2.4)
    decision = PassStrategy(cfg).decide_pass(
        world([passer(), mate("T1_B", 1.5, 0.0)], [opponent("OP_1", -0.3, 0.8, 0.0, -2.4)]),
        "T1_A",
    )

    assert not decision.should_pass
    assert decision.component_scores[0].line_score < 1.0


def test_fixed_point_pass_checks_line() -> None:
    decision = PassStrategy().fixed_point_pass(
        world([passer(), mate("T1_B", 0.5, 1.0)], [opponent("OP_1", -0.5, 0.02)]),
        "T1_A",
        Point(1.0, 0.0),
    )

    assert not decision.should_pass
    assert "传球线路存在可拦截对手" in decision.component_scores[0].elimination_reasons


def test_multiple_teammates_compete() -> None:
    decision = PassStrategy(PassConfig(pass_time_margin=0.1)).decide_pass(
        world(
            [passer(), mate("T1_B", -0.8, 1.0), mate("T1_C", 1.4, 0.8)],
            [opponent("OP_1", -0.6, 1.45)],
        ),
        "T1_A",
    )

    assert decision.should_pass
    assert decision.receiver_id == "T1_C"
    assert len(decision.component_scores) == 2


def test_no_safe_candidate() -> None:
    decision = PassStrategy().decide_pass(
        world(
            [passer(), mate("T1_B", 1.0, 0.5), mate("T1_C", 1.0, -0.5)],
            [opponent("OP_1", 0.9, 0.5), opponent("OP_2", 0.9, -0.5)],
        ),
        "T1_A",
    )

    assert not decision.should_pass


def test_receive_point_clamped_inside_field() -> None:
    decision = PassStrategy(PassConfig(max_pass_distance=8.0)).decide_pass(
        world([passer(), mate("T1_B", 3.9, 2.4, 1.0, 1.0)], [], field_width=8.0, field_height=5.0),
        "T1_A",
    )

    assert decision.target_point is not None
    assert decision.target_point.x <= 3.85
    assert decision.target_point.y <= 2.35


def test_target_too_close_rejected() -> None:
    decision = PassStrategy().decide_pass(
        world([passer(), mate("T1_B", -1.7, 0.0)], []),
        "T1_A",
    )

    assert not decision.should_pass
    assert "传球距离过近" in decision.component_scores[0].elimination_reasons


def test_target_too_far_rejected() -> None:
    decision = PassStrategy().decide_pass(
        world([passer(), mate("T1_B", 4.0, 0.0)], [], field_width=12.0),
        "T1_A",
    )

    assert not decision.should_pass
    assert "传球距离过远" in decision.component_scores[0].elimination_reasons


def test_dynamic_teammate_receive_point_uses_velocity() -> None:
    decision = PassStrategy().decide_pass(
        world([passer(), mate("T1_B", 0.5, 0.0, 0.6, 0.3)], []),
        "T1_A",
    )

    assert decision.target_point is not None
    assert decision.target_point.x > 0.5
    assert decision.target_point.y > 0.0


def test_emergency_degraded_risky_pass() -> None:
    cfg = PassConfig(allow_emergency_risky_pass=True)
    decision = PassStrategy(cfg).decide_pass(
        world([passer(), mate("T1_B", 1.0, 0.5)], [opponent("OP_1", 1.0, 0.5)]),
        "T1_A",
    )

    assert decision.should_pass
    assert decision.risk_level == "HIGH"
    assert "紧急降级" in decision.reason


def test_multiple_opponents_form_blockade() -> None:
    decision = PassStrategy().decide_pass(
        world(
            [passer(), mate("T1_B", 1.5, 0.0)],
            [
                opponent("OP_1", -1.0, 0.1),
                opponent("OP_2", -0.1, -0.1),
                opponent("OP_3", 0.8, 0.1),
            ],
        ),
        "T1_A",
    )

    assert not decision.should_pass


def test_weight_change_selects_different_receiver() -> None:
    base_world = world(
        [passer(), mate("T1_B", -0.4, 1.1), mate("T1_C", 1.7, 1.0)],
        [],
    )
    near_cfg = PassConfig(weights={"distance": 0.8, "safety": 0.05, "space": 0.05, "line": 0.05, "advance": 0.03, "attack": 0.02})
    attack_cfg = PassConfig(weights={"distance": 0.02, "safety": 0.05, "space": 0.05, "line": 0.05, "advance": 0.4, "attack": 0.43})

    near_decision = PassStrategy(near_cfg).decide_pass(base_world, "T1_A")
    attack_decision = PassStrategy(attack_cfg).decide_pass(base_world, "T1_A")

    assert near_decision.receiver_id != attack_decision.receiver_id
