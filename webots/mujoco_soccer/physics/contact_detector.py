from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import mujoco
import numpy as np


FOOT_TO_ROBOT = {
    "BLUE1": "T1_BLUE_1",
    "BLUE2": "T1_BLUE_2",
    "RED1": "T1_RED_1",
    "RED2": "T1_RED_2",
}


@dataclass
class ContactEvent:
    event: str
    robot: str
    foot: str
    stage: str
    sim_time: float
    contact_force: float
    ball_speed_before: float
    ball_speed_after: float
    ball_displacement: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class ContactDetector:
    def __init__(self, model: mujoco.MjModel, force_threshold: float = 0.4) -> None:
        self.model = model
        self.force_threshold = force_threshold
        self.ball_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "soccer_ball_geom")
        self.last_ball_xy: tuple[float, float] | None = None
        self.last_ball_speed = 0.0
        self._last_event_time: dict[str, float] = {}

    def ball_xy(self, data: mujoco.MjData) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        pos = data.xpos[body]
        return float(pos[0]), float(pos[1])

    def update(self, data: mujoco.MjData, stage: str, sim_time: float) -> list[ContactEvent]:
        ball_xy = self.ball_xy(data)
        ball_vel = self._ball_velocity(data)
        ball_speed = math.hypot(ball_vel[0], ball_vel[1])
        if self.last_ball_xy is None:
            self.last_ball_xy = ball_xy
            self.last_ball_speed = ball_speed
            return []
        displacement = math.hypot(ball_xy[0] - self.last_ball_xy[0], ball_xy[1] - self.last_ball_xy[1])
        events: list[ContactEvent] = []
        for idx in range(data.ncon):
            contact = data.contact[idx]
            if self.ball_geom not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == self.ball_geom else contact.geom1
            other_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other) or ""
            if not other_name.endswith("_FOOT_BALL_PROXY"):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, data, idx, force)
            normal_force = float(abs(force[0]))
            if normal_force < self.force_threshold:
                continue
            prefix = other_name.split("_", 1)[0]
            robot = FOOT_TO_ROBOT.get(prefix, "UNKNOWN")
            key = f"{robot}:{other_name}"
            if sim_time - self._last_event_time.get(key, -99.0) < 0.28:
                continue
            self._last_event_time[key] = sim_time
            foot = "RIGHT" if "_RIGHT_" in other_name else "LEFT"
            events.append(
                ContactEvent(
                    event="FOOT_BALL_CONTACT_CONFIRMED",
                    robot=robot,
                    foot=foot,
                    stage=stage,
                    sim_time=sim_time,
                    contact_force=normal_force,
                    ball_speed_before=self.last_ball_speed,
                    ball_speed_after=ball_speed,
                    ball_displacement=displacement,
                )
            )
        self.last_ball_xy = ball_xy
        self.last_ball_speed = ball_speed
        return events

    def _ball_velocity(self, data: mujoco.MjData) -> tuple[float, float]:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        vel = data.cvel[body]
        return float(vel[3]), float(vel[4])


def contact_pair_names(model: mujoco.MjModel, data: mujoco.MjData) -> list[tuple[str, str]]:
    pairs = []
    for idx in range(data.ncon):
        contact = data.contact[idx]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or ""
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or ""
        pairs.append((g1, g2))
    return pairs

