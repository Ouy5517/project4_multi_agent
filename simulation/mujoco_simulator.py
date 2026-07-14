"""
MuJoCo 3D 仿真器 (完整版)
=========================
复用 field_simulator 的 2D 物理与决策逻辑,
将状态同步到 MuJoCo 场景进行 3D 渲染。

设计: 决策层仍读 WorldState / MockRobotAction, 无需修改 strategy/decision。

支持:
- 3v3 完整场景 (蓝方 + 黄方 共 6 机器人)
- 铰接人形机器人 (髋/膝/肩可动)
- 踢球肢体动画 (蓄力→摆腿→触球→收回)
- 决策状态光圈 (底部颜色环)
- 传球连线可视化
- 自由视角交互
"""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

import mujoco

from common.config import DT, NUM_ROBOTS_PER_TEAM
from common.world_state import Ball, Robot, Team, RobotRole, WorldState
from simulation.field_simulator import Simulator
from simulation.limb_animator import LimbAnimator, JointPose, style_from_power


# ================================================================
# 决策状态 → 光圈颜色映射
# ================================================================
STATE_RING_COLORS = {
    "IDLE":      (0.53, 0.53, 0.53, 0.7),   # 灰色
    "CHASE":     (0.13, 0.59, 0.95, 0.85),  # 蓝色
    "DRIBBLE":   (1.0,  0.60, 0.0,  0.85),  # 橙色
    "PASS":      (0.30, 0.80, 0.20, 0.85),  # 绿色
    "SHOOT":     (0.91, 0.12, 0.39, 0.9),   # 红色
    "BLOCK":     (0.61, 0.15, 0.69, 0.85),  # 紫色
}
DEFAULT_RING_COLOR = (0.5, 0.5, 0.5, 0.7)


def _yaw_to_quat(theta: float) -> Tuple[float, float, float, float]:
    """绕 Z 轴 yaw → MuJoCo 四元数 (w, x, y, z)"""
    half = theta * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _find_xml(xml_name: str) -> str:
    """在 assets/ 目录查找 MuJoCo XML 文件"""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "assets", xml_name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"MuJoCo model not found: {path}")
    return path


# 每个机器人的肢体关节名后缀 → JointPose 属性
_LIMB_JOINT_SUFFIXES = (
    ("r_hip", "r_hip"),
    ("r_knee", "r_knee"),
    ("l_hip", "l_hip"),
    ("l_knee", "l_knee"),
    ("r_shoulder", "r_shoulder"),
    ("l_shoulder", "l_shoulder"),
)


class MuJoCoSimulator(Simulator):
    """
    2D 逻辑仿真 + MuJoCo 3D 可视化。

    与 Simulator 接口兼容, 可直接配合 WorldStateProvider / MockRobotAction。
    支持完整的 3v3 场景, 包括机器人状态光圈、传球连线和踢球肢体动画。
    """

    # 机器人 mocap 原点离地高度 (T1 proxy 已在 XML 内 GROUND_BIAS 校正)
    ROBOT_Z = 0.0

    # 球离地高度
    BALL_Z = 0.055

    # 可选跟随关节 (短名驱动之外)
    _FOLLOW_JOINT_SUFFIXES = (
        ("Right_Ankle_Pitch", "r_hip", -0.25),
        ("Left_Ankle_Pitch", "l_hip", -0.25),
        ("Right_Elbow_Pitch", "r_shoulder", 0.35),
        ("Left_Elbow_Pitch", "l_shoulder", 0.35),
        ("Waist", "r_hip", -0.08),
        ("Head_pitch", None, 0.06),
    )

    def __init__(self, xml_path: Optional[str] = None,
                 num_blue: int = NUM_ROBOTS_PER_TEAM,
                 num_yellow: int = NUM_ROBOTS_PER_TEAM):
        super().__init__(num_blue=num_blue, num_yellow=num_yellow)

        # 自动选择 XML (优先 soccer_full.xml, 回退 soccer_minimal.xml)
        if xml_path is None:
            try:
                xml_path = _find_xml("soccer_full.xml")
            except FileNotFoundError:
                xml_path = _find_xml("soccer_minimal.xml")

        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 缓存 geom ID 用于快速更新颜色
        self._ring_geom_ids: Dict[int, int] = {}
        self._pass_line_mocap_ids: List[int] = []
        self._pass_line_geom_ids: List[int] = []

        # 初始化 ball joint 地址
        self._ball_qposadr = self._joint_qposadr("ball_joint")
        self._ball_qveladr = self._joint_qveladr("ball_joint")

        # 初始化机器人 mocap ID 映射
        self._robot_mocap_ids: Dict[int, int] = {}
        # robot_id → {pose_attr → qposadr}
        self._limb_qpos: Dict[int, Dict[str, int]] = {}
        self._limb_animator = LimbAnimator()
        self._limb_follow_qpos: Dict[int, Dict[str, int]] = {}
        self._init_robot_mappings()

        # 初始化传球线 mocap ID 和 geom ID
        self._init_pass_lines()

        self.sync_to_mujoco()

    # ================================================================
    # 初始化
    # ================================================================

    def _init_robot_mappings(self):
        """建立 robot_id → mocap_id / ring_geom_id / 肢体关节 的映射"""
        for rid in list(self.blue_robots.keys()) + list(self.yellow_robots.keys()):
            body_name = f"robot_{rid}"
            try:
                body_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                mocap_id = self.model.body_mocapid[body_id]
                if mocap_id < 0:
                    print(f"  警告: Body {body_name} 不是 mocap, 跳过")
                    continue
                self._robot_mocap_ids[rid] = mocap_id
            except Exception:
                print(f"  警告: 未找到 body '{body_name}', 跳过")
                continue

            # 查找对应的 ring geom
            ring_name = f"robot_{rid}_ring"
            try:
                ring_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, ring_name)
                self._ring_geom_ids[rid] = ring_id
            except Exception:
                pass  # ring geom 可选

            # 肢体关节 qpos 地址 (短名 + 跟随关节)
            joint_map: Dict[str, int] = {}
            for suffix, attr in _LIMB_JOINT_SUFFIXES:
                jname = f"robot_{rid}_{suffix}"
                try:
                    joint_map[attr] = self._joint_qposadr(jname)
                except Exception:
                    pass
            follow_map: Dict[str, int] = {}
            for suffix, _src, _scale in self._FOLLOW_JOINT_SUFFIXES:
                jname = f"robot_{rid}_{suffix}"
                try:
                    follow_map[suffix] = self._joint_qposadr(jname)
                except Exception:
                    pass
            if joint_map:
                self._limb_qpos[rid] = joint_map
            if follow_map:
                if not hasattr(self, "_limb_follow_qpos"):
                    self._limb_follow_qpos: Dict[int, Dict[str, int]] = {}
                self._limb_follow_qpos[rid] = follow_map

    def _init_pass_lines(self):
        """查找传球线 mocap body 和 geom"""
        for i in range(3):
            body_name = f"pass_line_{i}"
            geom_name = f"pass_line_{i}_geom"
            try:
                body_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                mocap_id = self.model.body_mocapid[body_id]
                self._pass_line_mocap_ids.append(mocap_id)
            except Exception:
                self._pass_line_mocap_ids.append(-1)

            try:
                geom_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
                self._pass_line_geom_ids.append(geom_id)
            except Exception:
                self._pass_line_geom_ids.append(-1)

    # ================================================================
    # MuJoCo 工具
    # ================================================================

    def _joint_qposadr(self, name: str) -> int:
        jnt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[jnt])

    def _joint_qveladr(self, name: str) -> int:
        jnt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_dofadr[jnt])

    def _set_geom_rgba(self, geom_id: int, rgba: Tuple[float, float, float, float]):
        """设置 geom 颜色 (通过 model.geom_rgba 数组)"""
        if geom_id < 0:
            return
        # model.geom_rgba shape: (ngeom, 4)
        self.model.geom_rgba[geom_id] = rgba

    # ================================================================
    # 状态同步
    # ================================================================

    def load_world_state(self, ws: WorldState):
        """从 WorldState 初始化球和机器人位置"""
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
                self._init_positions[robot.id] = (robot.x, robot.y, robot.theta)

        self._move_targets.clear()
        self._turn_targets.clear()
        self._kick_queue.clear()
        self._limb_animator.reset()
        self.sync_to_mujoco()

    def queue_kick(self, robot_id: int, power: float, direction: float):
        """排队踢球: 先播肢体动画, 触球帧再施加冲量 (低力度=带球轻触)"""
        self._limb_animator.start_kick(
            robot_id, power, direction, style=style_from_power(power))

    def update(self, dt: float = DT):
        """2D 物理更新 + 肢体动画 + 同步到 MuJoCo"""
        moving_ids = set(self._move_targets.keys())
        turning = self._collect_turning()
        braking_ids = self._collect_braking()
        self._limb_animator.step(
            dt,
            moving_ids=moving_ids,
            turning=turning,
            braking_ids=braking_ids,
        )
        for rid, power, direction in self._limb_animator.pop_ready_impulses():
            self._kick_queue.append((rid, power, direction))

        super().update(dt)
        self.sync_to_mujoco()

    def _collect_turning(self) -> Dict[int, float]:
        """robot_id → 有符号剩余转角 (用于转向肢体姿态)"""
        result: Dict[int, float] = {}
        all_robots = {**self.blue_robots, **self.yellow_robots}
        for rid, target_theta in self._turn_targets.items():
            robot = all_robots.get(rid)
            if robot is None:
                continue
            diff = self._angle_diff(target_theta, robot.theta)
            if abs(diff) > 0.08:
                result[rid] = diff
        return result

    def _collect_braking(self) -> set:
        """接近移动目标时进入刹车姿态"""
        braking = set()
        all_robots = {**self.blue_robots, **self.yellow_robots}
        for rid, (tx, ty) in self._move_targets.items():
            robot = all_robots.get(rid)
            if robot is None:
                continue
            dist = math.sqrt((tx - robot.x) ** 2 + (ty - robot.y) ** 2)
            if 0.05 < dist < 0.45:
                braking.add(rid)
        return braking
    def sync_to_mujoco(self):
        """将当前 2D 状态与肢体关节写入 MuJoCo data"""

        # --- 足球 ---
        adr = self._ball_qposadr
        self.data.qpos[adr:adr + 3] = [self.ball.x, self.ball.y, self.BALL_Z]
        self.data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
        vadr = self._ball_qveladr
        self.data.qvel[vadr:vadr + 3] = [self.ball.vx, self.ball.vy, 0]
        self.data.qvel[vadr + 3:vadr + 6] = 0

        # --- 机器人 (mocap) ---
        all_robots = {**self.blue_robots, **self.yellow_robots}
        for rid, mocap_id in self._robot_mocap_ids.items():
            robot = all_robots.get(rid)
            if robot is None:
                continue
            self.data.mocap_pos[mocap_id] = [robot.x, robot.y, self.ROBOT_Z]
            quat = _yaw_to_quat(robot.theta)
            self.data.mocap_quat[mocap_id] = list(quat)

        # --- 肢体关节 ---
        self._apply_limb_poses()

        mujoco.mj_forward(self.model, self.data)

    def _apply_limb_poses(self):
        """把 LimbAnimator 的姿态写入短名关节, 并轻度驱动踝/肘/腰跟随"""
        for rid, joint_map in self._limb_qpos.items():
            pose: JointPose = self._limb_animator.get_pose(rid)
            for attr, qadr in joint_map.items():
                self.data.qpos[qadr] = getattr(pose, attr, 0.0)

            follow = self._limb_follow_qpos.get(rid, {})
            for suffix, src_attr, scale in self._FOLLOW_JOINT_SUFFIXES:
                qadr = follow.get(suffix)
                if qadr is None:
                    continue
                if src_attr is None:
                    self.data.qpos[qadr] = scale
                else:
                    self.data.qpos[qadr] = getattr(pose, src_attr, 0.0) * scale

    # ================================================================
    # 状态光圈 (根据 FSM 决策状态更新)
    # ================================================================

    def update_status_rings(self, fsm):
        """
        根据 DecisionFSM 的状态更新每个机器人的底部光圈颜色。

        Args:
            fsm: DecisionFSM 实例, 需要支持 get_state(robot_id) → DecisionState
        """
        all_robots = {**self.blue_robots, **self.yellow_robots}
        for rid in all_robots:
            ring_id = self._ring_geom_ids.get(rid, -1)
            if ring_id < 0:
                continue

            # 只对蓝方机器人显示决策状态颜色
            if rid in self.blue_robots:
                try:
                    state = fsm.get_state(rid)
                    state_name = state.value if hasattr(state, 'value') else str(state)
                except Exception:
                    state_name = "IDLE"
                color = STATE_RING_COLORS.get(state_name, DEFAULT_RING_COLOR)
            else:
                # 黄方(对手): 固定暗色光圈
                color = (0.7, 0.6, 0.1, 0.5)

            self._set_geom_rgba(ring_id, color)

    # ================================================================
    # 传球连线
    # ================================================================

    def update_pass_lines(self, ws: WorldState, fsm):
        """
        根据当前 PASS 状态绘制传球连线。

        在 PASS 状态机器人和接球目标之间绘制绿色虚线。
        """
        active_lines = []

        for robot in ws.teammates:
            try:
                state = fsm.get_state(robot.id)
                state_name = state.value if hasattr(state, 'value') else str(state)
            except Exception:
                continue

            if state_name != "PASS":
                continue

            target_id = fsm.get_pass_target_id(robot.id)
            if target_id is None:
                continue

            receiver = ws.get_robot_by_id(target_id)
            if receiver is None:
                continue

            active_lines.append((robot, receiver))

        # 更新每条传球线
        for i in range(3):
            mocap_id = self._pass_line_mocap_ids[i] if i < len(self._pass_line_mocap_ids) else -1
            geom_id = self._pass_line_geom_ids[i] if i < len(self._pass_line_geom_ids) else -1

            if i < len(active_lines) and mocap_id >= 0:
                passer, receiver = active_lines[i]

                # 计算中点位置和方向
                mid_x = (passer.x + receiver.x) / 2
                mid_y = (passer.y + receiver.y) / 2
                dx = receiver.x - passer.x
                dy = receiver.y - passer.y
                dist = math.sqrt(dx**2 + dy**2)
                angle = math.atan2(dy, dx)

                # 更新 mocap 位置 (中点)
                self.data.mocap_pos[mocap_id] = [mid_x, mid_y, 0.04]
                # 旋转到连线方向
                quat = _yaw_to_quat(angle)
                self.data.mocap_quat[mocap_id] = list(quat)

                # 更新 geom 大小 (半长为距离的一半)
                if geom_id >= 0:
                    self.model.geom_size[geom_id] = [dist / 2, 0.015, 0.005]

                # 显示
                if geom_id >= 0:
                    self._set_geom_rgba(geom_id, (0.3, 1.0, 0.15, 0.85))
            else:
                # 隐藏未使用的传球线
                if geom_id >= 0:
                    self._set_geom_rgba(geom_id, (0.3, 1.0, 0.15, 0.0))

    # ================================================================
    # 重置
    # ================================================================

    def reset(self):
        super().reset()
        self._limb_animator.reset()
        # 重置所有光圈为默认色
        for rid in self._ring_geom_ids:
            self._set_geom_rgba(self._ring_geom_ids[rid], DEFAULT_RING_COLOR)
        # 隐藏所有传球线
        for geom_id in self._pass_line_geom_ids:
            if geom_id >= 0:
                self._set_geom_rgba(geom_id, (0.3, 1.0, 0.15, 0.0))
        self.sync_to_mujoco()
