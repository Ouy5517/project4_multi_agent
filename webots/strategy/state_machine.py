from __future__ import annotations

from enum import Enum

from common.robot_action import ActionType, RobotAction


class StrategyState(str, Enum):
    SEARCH_BALL = "SEARCH_BALL"
    ATTACK_PASS = "ATTACK_PASS"
    ATTACK_DRIBBLE = "ATTACK_DRIBBLE"
    ATTACK_SHOOT = "ATTACK_SHOOT"
    DEFEND_MARK = "DEFEND_MARK"
    IDLE = "IDLE"


class StrategyStateMachine:
    """记录当前团队策略状态，便于日志展示和后续扩展。"""

    def __init__(self) -> None:
        self.current_state = StrategyState.IDLE

    def update(self, actions: list[RobotAction]) -> StrategyState:
        action_types = {action.action_type for action in actions}
        if ActionType.SHOOT in action_types:
            self.current_state = StrategyState.ATTACK_SHOOT
        elif ActionType.PASS in action_types:
            self.current_state = StrategyState.ATTACK_PASS
        elif ActionType.DRIBBLE in action_types:
            self.current_state = StrategyState.ATTACK_DRIBBLE
        elif ActionType.CHASE_BALL in action_types:
            self.current_state = StrategyState.SEARCH_BALL
        elif ActionType.MARK_OPPONENT in action_types or ActionType.BLOCK in action_types:
            self.current_state = StrategyState.DEFEND_MARK
        else:
            self.current_state = StrategyState.IDLE
        return self.current_state
