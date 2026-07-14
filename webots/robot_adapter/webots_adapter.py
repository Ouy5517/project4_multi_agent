from __future__ import annotations

from common.robot_action import ActionType, RobotAction
from robot_adapter.booster_action_map import get_booster_action_mapping


class WebotsAdapter:
    """Webots 示例适配器：只打印执行建议，不控制机器人或调用 SDK。"""

    def execute(self, actions: list[RobotAction]) -> None:
        print("\nWebots / Booster T1 动作建议：")
        for action in actions:
            mapping = get_booster_action_mapping(action.action_type)
            suggested_input = self._suggest_input(action)
            print(f"- robot_id: {action.robot_id}")
            print(f"  action_type: {action.action_type.value}")
            print(f"  target: {action.target}")
            print(f"  建议输入命令: {suggested_input}")
            print(f"  动作解释: {mapping.description} {mapping.example_action}")

    def _suggest_input(self, action: RobotAction) -> str:
        mapping = get_booster_action_mapping(action.action_type)
        if action.action_type in {
            ActionType.MOVE_TO_RECEIVE,
            ActionType.MOVE_TO_SUPPORT,
            ActionType.MARK_OPPONENT,
            ActionType.CHASE_BALL,
        }:
            return self._directional_input(action) or mapping.suggested_input
        return mapping.suggested_input

    def _directional_input(self, action: RobotAction) -> str | None:
        if action.vx or action.vy:
            x_direction = action.vx
            y_direction = action.vy
        else:
            target_x = action.target.get("x")
            target_y = action.target.get("y")
            if not isinstance(target_x, (int, float)) or not isinstance(
                target_y, (int, float)
            ):
                return None
            x_direction = float(target_x)
            y_direction = float(target_y)

        if abs(x_direction) >= abs(y_direction):
            return "w" if x_direction >= 0 else "s"
        return "a" if y_direction >= 0 else "d"
