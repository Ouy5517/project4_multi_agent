from __future__ import annotations

import math
import os
import platform
from typing import Iterable, MutableMapping


def configure_wsl_opengl(environment: MutableMapping[str, str], kernel_release: str) -> None:
    """WSLg 外接显卡不稳定时默认使用低负载的软件 OpenGL。"""
    if "microsoft" in kernel_release.lower():
        environment.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


configure_wsl_opengl(os.environ, platform.release())

import mujoco

from common.config import FIELD_HEIGHT, FIELD_WIDTH, GOAL_WIDTH
from common.world_state import WorldState


def _robot_body_xml(robot_id: int, team: str) -> str:
    color = "0.08 0.38 0.95 1" if team == "blue" else "1.00 0.72 0.05 1"
    marker = "0.75 0.88 1.00 1" if team == "blue" else "1.00 0.94 0.62 1"
    return f"""
    <body name="{team}_{robot_id}">
      <freejoint name="{team}_{robot_id}_joint"/>
      <geom name="{team}_{robot_id}_body" type="cylinder" size="0.18 0.15"
            pos="0 0 0.15" rgba="{color}" contype="0" conaffinity="0"/>
      <geom name="{team}_{robot_id}_heading" type="box" size="0.12 0.025 0.018"
            pos="0.10 0 0.318" rgba="{marker}" contype="0" conaffinity="0"/>
    </body>"""


def build_scene_xml(blue_ids: Iterable[int], yellow_ids: Iterable[int]) -> str:
    robot_bodies = "\n".join(
        [_robot_body_xml(robot_id, "blue") for robot_id in blue_ids]
        + [_robot_body_xml(robot_id, "yellow") for robot_id in yellow_ids]
    )
    half_width = FIELD_WIDTH / 2
    half_height = FIELD_HEIGHT / 2
    goal_half = GOAL_WIDTH / 2
    return f"""
<mujoco model="booster_t1_2d_soccer">
  <compiler angle="radian"/>
  <option timestep="0.0333333333" gravity="0 0 -9.81"/>
  <statistic center="0 0 0" extent="5.5"/>
  <visual>
    <global offwidth="1280" offheight="768"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="50"/>
    <rgba haze="0.05 0.08 0.16 1"/>
  </visual>
  <asset>
    <texture name="field_checker" type="2d" builtin="checker"
             rgb1="0.08 0.43 0.10" rgb2="0.12 0.62 0.16"
             mark="edge" markrgb="0.18 0.72 0.22" width="512" height="512"/>
    <material name="field_material" texture="field_checker" texrepeat="9 6"
              texuniform="true" reflectance="0.06"/>
    <material name="line_material" rgba="0.94 0.98 0.94 1"/>
    <material name="goal_material" rgba="0.95 0.88 0.18 1"/>
  </asset>
  <worldbody>
    <light name="key" pos="0 -3 8" dir="0 0 -1" diffuse="0.95 0.95 0.95"/>
    <light name="fill" pos="-4 3 5" dir="0 0 -1" diffuse="0.35 0.38 0.45"/>
    <geom name="field" type="plane" size="{half_width + 0.35} {half_height + 0.35} 0.1"
          material="field_material" contype="0" conaffinity="0"/>

    <geom name="touchline_top" type="box" pos="0 {half_height} 0.012"
          size="{half_width} 0.025 0.012" material="line_material"/>
    <geom name="touchline_bottom" type="box" pos="0 {-half_height} 0.012"
          size="{half_width} 0.025 0.012" material="line_material"/>
    <geom name="goal_line_left" type="box" pos="{-half_width} 0 0.012"
          size="0.025 {half_height} 0.012" material="line_material"/>
    <geom name="goal_line_right" type="box" pos="{half_width} 0 0.012"
          size="0.025 {half_height} 0.012" material="line_material"/>
    <geom name="halfway_line" type="box" pos="0 0 0.012"
          size="0.018 {half_height} 0.012" material="line_material"/>

    <geom name="left_goal_back" type="box" pos="{-half_width - 0.28} 0 0.12"
          size="0.025 {goal_half} 0.12" material="goal_material"/>
    <geom name="left_goal_top" type="box" pos="{-half_width - 0.14} {goal_half} 0.12"
          size="0.14 0.025 0.12" material="goal_material"/>
    <geom name="left_goal_bottom" type="box" pos="{-half_width - 0.14} {-goal_half} 0.12"
          size="0.14 0.025 0.12" material="goal_material"/>
    <geom name="right_goal_back" type="box" pos="{half_width + 0.28} 0 0.12"
          size="0.025 {goal_half} 0.12" material="goal_material"/>
    <geom name="right_goal_top" type="box" pos="{half_width + 0.14} {goal_half} 0.12"
          size="0.14 0.025 0.12" material="goal_material"/>
    <geom name="right_goal_bottom" type="box" pos="{half_width + 0.14} {-goal_half} 0.12"
          size="0.14 0.025 0.12" material="goal_material"/>

    <body name="ball">
      <freejoint name="ball_joint"/>
      <geom name="ball_geom" type="sphere" size="0.075" rgba="0.97 0.97 0.94 1"
            contype="0" conaffinity="0"/>
    </body>
{robot_bodies}
  </worldbody>
</mujoco>
"""


class MujocoSoccerViewer:
    """轻量 MuJoCo 2.5D 窗口；运动状态仍由项目内 2D 决策仿真提供。"""

    def __init__(self, world_state: WorldState, launch_window: bool = True):
        blue_ids = [robot.id for robot in world_state.teammates]
        yellow_ids = [robot.id for robot in world_state.opponents]
        self._robot_joint_names = {
            *(f"blue_{robot_id}_joint" for robot_id in blue_ids),
            *(f"yellow_{robot_id}_joint" for robot_id in yellow_ids),
        }
        self.phase_label = ""
        self.model = mujoco.MjModel.from_xml_string(build_scene_xml(blue_ids, yellow_ids))
        self.data = mujoco.MjData(self.model)
        self._viewer = None
        self._closed = False
        self.render(world_state, fsm=None)

        if launch_window:
            from mujoco import viewer as mj_viewer

            self._viewer = mj_viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=True,
                show_right_ui=False,
            )
            self._viewer.cam.lookat[:] = (0.0, 0.0, 0.0)
            self._viewer.cam.distance = 10.5
            self._viewer.cam.azimuth = 90.0
            self._viewer.cam.elevation = -58.0
            self._viewer.sync()

    def _set_free_joint_pose(self, joint_name: str, x: float, y: float, z: float, theta: float = 0.0) -> None:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            return
        qpos_address = self.model.jnt_qposadr[joint_id]
        half_theta = theta / 2
        self.data.qpos[qpos_address : qpos_address + 7] = (
            x,
            y,
            z,
            math.cos(half_theta),
            0.0,
            0.0,
            math.sin(half_theta),
        )

    def render(self, world_state: WorldState, fsm=None) -> None:
        if self._closed:
            return

        def update_scene() -> None:
            self._set_free_joint_pose("ball_joint", world_state.ball.x, world_state.ball.y, 0.075)
            active_joint_names = set()
            for robot in world_state.teammates:
                active_joint_names.add(f"blue_{robot.id}_joint")
                self._set_free_joint_pose(
                    f"blue_{robot.id}_joint", robot.x, robot.y, 0.0, robot.theta
                )
            for robot in world_state.opponents:
                active_joint_names.add(f"yellow_{robot.id}_joint")
                self._set_free_joint_pose(
                    f"yellow_{robot.id}_joint", robot.x, robot.y, 0.0, robot.theta
                )
            for joint_name in self._robot_joint_names - active_joint_names:
                self._set_free_joint_pose(joint_name, 0.0, 0.0, -5.0)
            mujoco.mj_forward(self.model, self.data)

        if self._viewer is None:
            update_scene()
            return

        with self._viewer.lock():
            update_scene()
        state_lines = []
        if fsm is not None:
            for robot in world_state.teammates:
                try:
                    state = fsm.get_state(robot.id).value
                except (KeyError, AttributeError):
                    state = robot.role.value.upper()
                state_lines.append(f"B{robot.id}: {state}")
        self._viewer.set_texts(
            [
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                    "综合演示",
                    self.phase_label or "实时决策",
                ),
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                    "蓝方状态",
                    "\n".join(state_lines),
                ),
            ]
        )
        self._viewer.sync()

    def set_phase(self, phase_label: str) -> None:
        self.phase_label = phase_label

    def is_running(self) -> bool:
        if self._closed:
            return False
        return self._viewer is None or self._viewer.is_running()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._viewer is not None:
            self._viewer.close()
