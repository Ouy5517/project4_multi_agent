from __future__ import annotations

from common.robot_action import RobotAction


class MockRobotAdapter:
    """Mock 执行器：不控制真实机器人，只输出动作意图。"""

    def execute(self, actions: list[RobotAction]) -> None:
        print("\nMock 执行：")
        for action in actions:
            print(
                f"- {action.robot_id} 执行 {action.action_type.value}，"
                f"目标={action.target}"
            )
