"""
MuJoCo 3D 仿真器 (最小传球 Demo)
================================
复用 field_simulator 的 2D 物理与决策逻辑,
将状态同步到 MuJoCo 场景进行 3D 渲染。

设计: 决策层仍读 WorldState / MockRobotAction, 无需修改 strategy/decision。
"""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

import mujoco

from common.config import DT
from common.world_state import Ball, Robot, Team, RobotRole, WorldState
from simulation.field_simulator import Simulator


def _yaw_to_quat(theta: float) -> Tuple[float, float, float, float]:
    """绕 Z 轴 yaw → MuJoCo 四元数 (w, x, y, z)"""
    half = theta * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))


class MuJoCoSimulator(Simulator):
    """
    2D 逻辑仿真 + MuJoCo 3D 可视化。

    与 Simulator 接口兼容, 可直接配合 WorldStateProvider / MockRobotAction。
    """

    ROBOT_Z = 0.15
    BALL_Z = 0.05

    def __init__(self, xml_path: Optional[str] = None,
                 num_blue: int = 2, num_yellow: int = 0):
        super().__init__(num_blue=num_blue, num_yellow=num_yellow)

        if xml_path is None:
            root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            xml_path = os.path.join(root, "assets", "soccer_minimal.xml")

        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"MuJoCo model not found: {xml_path}")

        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self._ball_qposadr = self._joint_qposadr("ball_joint")
        self._ball_qveladr = self._joint_qveladr("ball_joint")
        self._robot_mocap_ids: Dict[int, int] = {}
        for rid in self.blue_robots:
            body_name = f"robot_{rid}"
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            mocap_id = self.model.body_mocapid[body_id]
            if mocap_id < 0:
                raise RuntimeError(f"Body {body_name} is not mocap")
            self._robot_mocap_ids[rid] = mocap_id

        self.sync_to_mujoco()

    def _joint_qposadr(self, name: str) -> int:
        jnt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[jnt])

    def _joint_qveladr(self, name: str) -> int:
        jnt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_dofadr[jnt])

    def load_world_state(self, ws: WorldState):
        """从 WorldState 初始化球和机器人位置 (用于传球场景)"""
        self.ball = Ball(
            x=ws.ball.x, y=ws.ball.y, z=self.BALL_Z,
            vx=ws.ball.vx, vy=ws.ball.vy, vz=0.0,
        )
        for robot in ws.teammates:
            if robot.id in self.blue_robots:
                r = self.blue_robots[robot.id]
                r.x, r.y, r.theta = robot.x, robot.y, robot.theta
                r.role = robot.role
                r.kick_cooldown = robot.kick_cooldown
                self._init_positions[robot.id] = (robot.x, robot.y, robot.theta)

        for robot in ws.opponents:
            if robot.id in self.yellow_robots:
                r = self.yellow_robots[robot.id]
                r.x, r.y, r.theta = robot.x, robot.y, robot.theta

        self._move_targets.clear()
        self._turn_targets.clear()
        self._kick_queue.clear()
        self.sync_to_mujoco()

    def update(self, dt: float = DT):
        """2D 物理更新 + 同步到 MuJoCo"""
        super().update(dt)
        self.sync_to_mujoco()

    def sync_to_mujoco(self):
        """将当前 2D 状态写入 MuJoCo data"""
        # 足球
        adr = self._ball_qposadr
        self.data.qpos[adr:adr + 3] = [self.ball.x, self.ball.y, self.BALL_Z]
        self.data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
        vadr = self._ball_qveladr
        self.data.qvel[vadr:vadr + 3] = [self.ball.vx, self.ball.vy, 0]
        self.data.qvel[vadr + 3:vadr + 6] = 0

        # 己方圆柱机器人 (mocap)
        for rid, mocap_id in self._robot_mocap_ids.items():
            robot = self.blue_robots.get(rid)
            if robot is None:
                continue
            self.data.mocap_pos[mocap_id] = [robot.x, robot.y, self.ROBOT_Z]
            quat = _yaw_to_quat(robot.theta)
            self.data.mocap_quat[mocap_id] = list(quat)

        # 必须调用 forward, viewer 才会刷新 mocap / qpos 到画面
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        super().reset()
        self.sync_to_mujoco()
