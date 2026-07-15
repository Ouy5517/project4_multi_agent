from __future__ import annotations

from mujoco_soccer.multi_agent.robot_agent import AgentCommand


def behavior_score(command: AgentCommand) -> dict[str, float]:
    base = {
        "SHOOT": 0.95,
        "PASS": 0.86,
        "DRIBBLE": 0.72,
        "CLEAR": 0.82,
        "COUNTER_ATTACK": 0.80,
        "PRESS_BALL": 0.68,
        "BLOCK_LINE": 0.62,
        "OPEN_FOR_PASS": 0.60,
        "RECEIVE_PASS": 0.78,
    }.get(command.behavior, 0.35)
    return {"selected": base, "confidence": command.confidence}

