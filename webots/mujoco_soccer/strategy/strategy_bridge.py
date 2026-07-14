from __future__ import annotations

from common.robot_action import ActionType, RobotAction
from common.world_state import WorldState
from strategy.team_strategy import TeamStrategy


class StrategyBridge:
    def __init__(self) -> None:
        self.blue_strategy = TeamStrategy(shoot_distance=1.25, pass_safe_distance=0.7)

    def decide_blue(self, world: WorldState) -> list[RobotAction]:
        return self.blue_strategy.decide(world)

    @staticmethod
    def has_action(actions: list[RobotAction], action: ActionType) -> bool:
        return any(item.action_type == action for item in actions)

