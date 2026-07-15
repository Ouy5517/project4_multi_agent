from __future__ import annotations

import ast
import json
from pathlib import Path

from mujoco_soccer.multi_agent.action_arbitrator import ActionArbitrator
from mujoco_soccer.multi_agent.concurrent_match import ConcurrentMatch


ROBOTS = {"T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"}


def test_concurrent_modules_exist_and_no_active_robot_gate() -> None:
    base = Path("mujoco_soccer/multi_agent")
    for name in [
        "__init__.py",
        "shared_world_state.py",
        "robot_agent.py",
        "team_coordinator.py",
        "role_allocator.py",
        "possession_manager.py",
        "action_arbitrator.py",
        "behavior_planner.py",
        "local_avoidance.py",
        "intercept_predictor.py",
        "concurrent_match.py",
        "multi_agent_logger.py",
    ]:
        assert (base / name).exists()
    text = "\n".join(path.read_text() for path in base.glob("*.py"))
    for forbidden in ("active_robot", "current_active", "only_active", "park_non_active", "single_actor", "turn_owner"):
        assert forbidden not in text


def test_concurrent_match_loop_steps_outside_agent_decision_loop() -> None:
    tree = ast.parse(Path("mujoco_soccer/multi_agent/concurrent_match.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.For):
            source = ast.get_source_segment(Path("mujoco_soccer/multi_agent/concurrent_match.py").read_text(), node) or ""
            if "for robot in ROBOTS" in source or "for agent" in source:
                assert "mj_step" not in source


def test_action_arbitrator_allows_one_same_team_kick_owner() -> None:
    from mujoco_soccer.control.planar_base_controller import BaseTarget
    from mujoco_soccer.multi_agent.robot_agent import AgentCommand

    arb = ActionArbitrator()
    commands = {
        "T1_BLUE_1": AgentCommand("T1_BLUE_1", "PASS", "BALL_HANDLER", BaseTarget(0, 0, 0), "pass", (1, 0)),
        "T1_BLUE_2": AgentCommand("T1_BLUE_2", "SHOOT", "SUPPORT", BaseTarget(0, 0, 0), "shoot", (1, 0)),
        "T1_RED_1": AgentCommand("T1_RED_1", "PRESS_BALL", "PRESSER", BaseTarget(0, 0, 0)),
        "T1_RED_2": AgentCommand("T1_RED_2", "BLOCK_LINE", "COVER", BaseTarget(0, 0, 0)),
    }
    out = arb.arbitrate(1.0, commands)
    active_blue = [cmd for key, cmd in out.items() if "BLUE" in key and cmd.kick_action]
    assert len(active_blue) == 1
    assert arb.conflicts == 1


def test_concurrent_match_short_run_logs_four_agent_decisions() -> None:
    summary = ConcurrentMatch(run_id="unit_concurrent_short", duration=1.0, seed=42, no_render=True).run()
    assert summary["finished"] is True
    run_dir = Path(summary["log_dir"])
    first = json.loads((run_dir / "agent_decisions.jsonl").read_text().splitlines()[0])
    assert set(first["decisions"]) == ROBOTS
    assert first["snapshot_id"] == first["decision_tick"]
    assert summary["decision_counts"] == {robot: 20 for robot in ROBOTS}
    assert summary["ball_mutation_detected"] is False
