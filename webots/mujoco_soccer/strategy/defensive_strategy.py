from __future__ import annotations

from dataclasses import dataclass

from common.robot_action import ActionType, RobotAction
from common.world_state import WorldState


@dataclass
class DefensiveStrategy:
    def decide(self, world: WorldState, defender_id: str) -> RobotAction:
        defender = next((robot for robot in world.robots if robot.robot_id == defender_id), None)
        if defender is None:
            return RobotAction("TEAM", ActionType.HOLD, {}, "未找到防守机器人。", confidence=0.3)
        if defender.has_ball:
            return RobotAction(
                robot_id=defender_id,
                action_type=ActionType.CLEAR,
                target={"x": world.enemy_goal.x, "y": world.enemy_goal.y},
                reason="防守者获得球权，执行解围。",
                confidence=0.92,
            )
        ball_distance = defender.point.distance_to(world.ball)
        goal_distance = world.ball.distance_to(world.our_goal)
        if ball_distance < 0.45 and goal_distance < 2.5:
            return RobotAction(defender_id, ActionType.CLEAR, {"x": world.enemy_goal.x, "y": world.enemy_goal.y}, "防守方接近球且球在危险区，执行解围。", confidence=0.9)
        if goal_distance < 2.8:
            return RobotAction(defender_id, ActionType.BLOCK, {"x": world.ball.x, "y": world.ball.y}, "射门线路进入防守区，执行封堵。", confidence=0.84)
        return RobotAction(defender_id, ActionType.HOLD, {}, "保持防守阵型。", confidence=0.6)
