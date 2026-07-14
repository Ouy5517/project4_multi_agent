from __future__ import annotations

import mujoco

from common.world_state import Ball, OpponentState, Point, RobotState, WorldState


ROBOTS = ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"]


class WorldStateAdapter:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        self.robot_bodies = {
            robot: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base")
            for robot in ROBOTS
        }
        self.yaw_joints = {
            robot: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_yaw")
            for robot in ROBOTS
        }

    def build(self, data: mujoco.MjData, team: str, carrier: str | None, stage: str) -> WorldState:
        ball_pos = data.xpos[self.ball_body]
        ball_vel = data.cvel[self.ball_body]
        blue = []
        red = []
        for robot in ROBOTS:
            body = self.robot_bodies[robot]
            pos = data.xpos[body]
            qadr = self.model.jnt_qposadr[self.yaw_joints[robot]]
            state = {
                "robot_id": robot,
                "team": "blue" if "BLUE" in robot else "red",
                "x": float(pos[0]),
                "y": float(pos[1]),
                "theta": float(data.qpos[qadr]),
                "role": "attacker" if robot.endswith("_1") else "support",
                "has_ball": robot == carrier,
            }
            if "BLUE" in robot:
                blue.append(state)
            else:
                red.append(state)
        if team == "blue":
            robots = [RobotState.from_dict(item) for item in blue]
            opponents = [OpponentState(opponent_id=item["robot_id"], x=item["x"], y=item["y"]) for item in red]
            our_goal = Point(-3.35, 0.0)
            enemy_goal = Point(3.35, 0.0)
        else:
            robots = [RobotState.from_dict(item) for item in red]
            opponents = [OpponentState(opponent_id=item["robot_id"], x=item["x"], y=item["y"]) for item in blue]
            our_goal = Point(3.35, 0.0)
            enemy_goal = Point(-3.35, 0.0)
        return WorldState(
            timestamp=float(data.time),
            ball=Ball(float(ball_pos[0]), float(ball_pos[1]), float(ball_vel[3]), float(ball_vel[4])),
            robots=robots,
            opponents=opponents,
            our_goal=our_goal,
            enemy_goal=enemy_goal,
            field_width=7.0,
            field_height=5.0,
            scenario_name=stage,
        )

