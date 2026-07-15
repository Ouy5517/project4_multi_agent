from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco

from mujoco_soccer.strategy.world_state_adapter import ROBOTS


TEAM_OF = {
    "T1_BLUE_1": "blue",
    "T1_BLUE_2": "blue",
    "T1_RED_1": "red",
    "T1_RED_2": "red",
}


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    team: str
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    role: str
    behavior: str
    distance_to_ball: float
    arrival_time: float


@dataclass(frozen=True)
class SharedWorldState:
    snapshot_id: int
    decision_tick: int
    sim_time: float
    ball_xy: tuple[float, float]
    ball_velocity: tuple[float, float]
    ball_prediction: list[tuple[float, float]]
    robots: dict[str, RobotSnapshot]
    blue_goal: tuple[float, float] = (-3.35, 0.0)
    red_goal: tuple[float, float] = (3.35, 0.0)
    possession: str = "FREE"
    possession_confidence: float = 0.0
    last_contact_robot: str | None = None
    last_contact_team: str | None = None
    pass_line: tuple[tuple[float, float], tuple[float, float]] | None = None
    current_contacts: list[dict[str, object]] = field(default_factory=list)


class SharedWorldStateBuilder:
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
        self._last_xy: dict[str, tuple[float, float]] = {}
        self._last_time = 0.0

    def build(
        self,
        data: mujoco.MjData,
        snapshot_id: int,
        decision_tick: int,
        roles: dict[str, str],
        behaviors: dict[str, str],
        possession: str,
        possession_confidence: float,
        last_contact_robot: str | None,
        last_contact_team: str | None,
        contacts: list[dict[str, object]],
    ) -> SharedWorldState:
        ball_pos = data.xpos[self.ball_body]
        ball_vel = data.cvel[self.ball_body]
        ball_xy = (float(ball_pos[0]), float(ball_pos[1]))
        ball_v = (float(ball_vel[3]), float(ball_vel[4]))
        dt = max(1e-6, float(data.time) - self._last_time)
        robots: dict[str, RobotSnapshot] = {}
        for robot in ROBOTS:
            body = self.robot_bodies[robot]
            pos = data.xpos[body]
            xy = (float(pos[0]), float(pos[1]))
            last = self._last_xy.get(robot, xy)
            vx, vy = (xy[0] - last[0]) / dt, (xy[1] - last[1]) / dt
            qadr = self.model.jnt_qposadr[self.yaw_joints[robot]]
            dist = math.hypot(xy[0] - ball_xy[0], xy[1] - ball_xy[1])
            robots[robot] = RobotSnapshot(
                robot_id=robot,
                team=TEAM_OF[robot],
                x=xy[0],
                y=xy[1],
                yaw=float(data.qpos[qadr]),
                vx=vx,
                vy=vy,
                role=roles.get(robot, "UNASSIGNED"),
                behavior=behaviors.get(robot, "HOLD_POSITION"),
                distance_to_ball=dist,
                arrival_time=dist / 0.55,
            )
            self._last_xy[robot] = xy
        self._last_time = float(data.time)
        prediction = [(ball_xy[0] + ball_v[0] * t, ball_xy[1] + ball_v[1] * t) for t in (0.2, 0.4, 0.8)]
        return SharedWorldState(
            snapshot_id=snapshot_id,
            decision_tick=decision_tick,
            sim_time=float(data.time),
            ball_xy=ball_xy,
            ball_velocity=ball_v,
            ball_prediction=prediction,
            robots=robots,
            possession=possession,
            possession_confidence=possession_confidence,
            last_contact_robot=last_contact_robot,
            last_contact_team=last_contact_team,
            current_contacts=contacts,
        )

