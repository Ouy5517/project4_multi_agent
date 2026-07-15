from __future__ import annotations

from common.world_state import Ball, OpponentState, Point, RobotState, WorldState


ROLES = {
    "BLUE_1": "BALL_HANDLER",
    "BLUE_2": "SUPPORT",
    "RED_1": "MARK",
    "RED_2": "BLOCK",
}


class WorldStateReader:
    def __init__(self, supervisor, robot_nodes: dict[str, object], ball_node: object) -> None:
        self.supervisor = supervisor
        self.robot_nodes = robot_nodes
        self.ball_node = ball_node

    def read_blue_attack(self, carrier: str | None, scenario_name: str) -> WorldState:
        ball_pos = self.ball_node.getPosition()
        velocity = self.ball_node.getVelocity()
        robots = []
        opponents = []
        for name, node in self.robot_nodes.items():
            pos = node.getPosition()
            if name.startswith("BLUE"):
                robots.append(
                    RobotState(
                        robot_id=name,
                        team="BLUE",
                        x=pos[0],
                        y=pos[1],
                        theta=0.0,
                        role=ROLES[name],
                        has_ball=name == carrier,
                    )
                )
            else:
                opponents.append(OpponentState(opponent_id=name, x=pos[0], y=pos[1]))
        return WorldState(
            timestamp=self.supervisor.getTime(),
            ball=Ball(x=ball_pos[0], y=ball_pos[1], vx=velocity[0], vy=velocity[1]),
            robots=robots,
            opponents=opponents,
            our_goal=Point(-3.3, 0.0),
            enemy_goal=Point(3.3, 0.0),
            field_width=7.0,
            field_height=5.0,
            scenario_name=scenario_name,
        )

    def read_red_attack(self, carrier: str | None, scenario_name: str) -> WorldState:
        ball_pos = self.ball_node.getPosition()
        velocity = self.ball_node.getVelocity()
        robots = []
        opponents = []
        for name, node in self.robot_nodes.items():
            pos = node.getPosition()
            if name.startswith("RED"):
                robots.append(
                    RobotState(
                        robot_id=name,
                        team="RED",
                        x=pos[0],
                        y=pos[1],
                        theta=0.0,
                        role=ROLES[name],
                        has_ball=name == carrier,
                    )
                )
            else:
                opponents.append(OpponentState(opponent_id=name, x=pos[0], y=pos[1]))
        return WorldState(
            timestamp=self.supervisor.getTime(),
            ball=Ball(x=ball_pos[0], y=ball_pos[1], vx=velocity[0], vy=velocity[1]),
            robots=robots,
            opponents=opponents,
            our_goal=Point(3.3, 0.0),
            enemy_goal=Point(-3.3, 0.0),
            field_width=7.0,
            field_height=5.0,
            scenario_name=scenario_name,
        )
