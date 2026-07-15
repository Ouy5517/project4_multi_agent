"""
机器人肢体动画器
================
为 MuJoCo mocap 人形驱动关节角度。

动作优先级 (高→低):
  kick / dribble 剪辑 → 转向 → 刹车 → 行走 → 站立

关节约定 (robot 局部坐标):
  hip > 0 大腿后摆, hip < 0 大腿前踢  (hinge +Y)
  knee > 0 小腿弯曲
  shoulder: 手臂前后摆 (Shoulder_Pitch)
  elbow < 0 屈肘 (T1 Elbow_Pitch 常用负向弯曲)
  shoulder_roll: 手臂收拢/张开 (Shoulder_Roll)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


KICK_DURATION = 0.55
DRIBBLE_DURATION = 0.32
KICK_CONTACT_RATIO = 0.48
DRIBBLE_CONTACT_RATIO = 0.42
DRIBBLE_POWER_THRESHOLD = 25.0

# 跑步基础屈肘 (弧度, 负值=自然弯曲)
ELBOW_RUN_BASE = -0.55
# T1 下垂: 左肩滚转为负、右肩为正 (零位是横伸 T 字)
SHOULDER_ROLL_HANG_L = -1.25
SHOULDER_ROLL_HANG_R = 1.25


@dataclass
class JointPose:
    """单机全身关节角 (弧度)"""
    r_hip: float = 0.0
    r_knee: float = 0.0
    l_hip: float = 0.0
    l_knee: float = 0.0
    r_shoulder: float = 0.0
    l_shoulder: float = 0.0
    r_elbow: float = 0.0
    l_elbow: float = 0.0
    r_shoulder_roll: float = 0.0
    l_shoulder_roll: float = 0.0

    def scaled(self, s: float) -> "JointPose":
        return JointPose(
            r_hip=self.r_hip * s,
            r_knee=self.r_knee * s,
            l_hip=self.l_hip * s,
            l_knee=self.l_knee * s,
            r_shoulder=self.r_shoulder * s,
            l_shoulder=self.l_shoulder * s,
            r_elbow=self.r_elbow * s,
            l_elbow=self.l_elbow * s,
            r_shoulder_roll=self.r_shoulder_roll * s,
            l_shoulder_roll=self.l_shoulder_roll * s,
        )

    def blended(self, other: "JointPose", t: float) -> "JointPose":
        t = max(0.0, min(1.0, t))
        return JointPose(
            r_hip=_lerp(self.r_hip, other.r_hip, t),
            r_knee=_lerp(self.r_knee, other.r_knee, t),
            l_hip=_lerp(self.l_hip, other.l_hip, t),
            l_knee=_lerp(self.l_knee, other.l_knee, t),
            r_shoulder=_lerp(self.r_shoulder, other.r_shoulder, t),
            l_shoulder=_lerp(self.l_shoulder, other.l_shoulder, t),
            r_elbow=_lerp(self.r_elbow, other.r_elbow, t),
            l_elbow=_lerp(self.l_elbow, other.l_elbow, t),
            r_shoulder_roll=_lerp(self.r_shoulder_roll, other.r_shoulder_roll, t),
            l_shoulder_roll=_lerp(self.l_shoulder_roll, other.l_shoulder_roll, t),
        )


@dataclass
class KickClip:
    """一次踢球/带球剪辑"""
    robot_id: int
    power: float
    direction: float
    style: str = "kick"
    elapsed: float = 0.0
    duration: float = KICK_DURATION
    impulse_fired: bool = False

    @property
    def contact_ratio(self) -> float:
        return DRIBBLE_CONTACT_RATIO if self.style == "dribble" else KICK_CONTACT_RATIO

    @property
    def progress(self) -> float:
        if self.duration <= 0:
            return 1.0
        return min(1.0, self.elapsed / self.duration)

    @property
    def done(self) -> bool:
        return self.elapsed >= self.duration

    @property
    def at_contact(self) -> bool:
        return (not self.impulse_fired) and self.progress >= self.contact_ratio


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _arm_balance(l_shoulder: float, r_shoulder: float,
                 elbow_base: float = ELBOW_RUN_BASE,
                 roll_l: float = SHOULDER_ROLL_HANG_L,
                 roll_r: float = SHOULDER_ROLL_HANG_R) -> dict:
    """由肩摆推导肘关节; 滚转保持下垂基姿。"""
    return {
        "l_elbow": elbow_base - 0.22 * max(0.0, l_shoulder),
        "r_elbow": elbow_base - 0.22 * max(0.0, r_shoulder),
        "l_shoulder_roll": roll_l,
        "r_shoulder_roll": roll_r,
    }


def _kick_pose(progress: float) -> JointPose:
    """大力踢球/传球: 蓄力 → 摆腿 → 触球 → 收回, 手臂大幅对侧平衡"""
    pose = JointPose()
    if progress < 0.25:
        t = _smoothstep(progress / 0.25)
        pose.r_hip = _lerp(0.0, 0.55, t)
        pose.r_knee = _lerp(0.0, 1.35, t)
        pose.l_hip = _lerp(0.0, -0.08, t)
        pose.r_shoulder = _lerp(0.0, -0.55, t)
        pose.l_shoulder = _lerp(0.0, 0.85, t)
    elif progress < 0.48:
        t = _smoothstep((progress - 0.25) / 0.23)
        pose.r_hip = _lerp(0.55, -1.05, t)
        pose.r_knee = _lerp(1.35, 0.25, t)
        pose.l_hip = -0.08
        pose.r_shoulder = _lerp(-0.55, 0.75, t)
        pose.l_shoulder = _lerp(0.85, -0.45, t)
    elif progress < 0.70:
        t = _smoothstep((progress - 0.48) / 0.22)
        pose.r_hip = _lerp(-1.05, -0.85, t)
        pose.r_knee = _lerp(0.25, 0.35, t)
        pose.l_hip = _lerp(-0.08, 0.0, t)
        pose.r_shoulder = _lerp(0.75, 0.35, t)
        pose.l_shoulder = _lerp(-0.45, 0.10, t)
    else:
        t = _smoothstep((progress - 0.70) / 0.30)
        pose.r_hip = _lerp(-0.85, 0.0, t)
        pose.r_knee = _lerp(0.35, 0.0, t)
        pose.r_shoulder = _lerp(0.35, 0.0, t)
        pose.l_shoulder = _lerp(0.10, 0.0, t)

    arms = _arm_balance(pose.l_shoulder, pose.r_shoulder,
                        elbow_base=-0.65)
    pose.l_elbow = arms["l_elbow"]
    pose.r_elbow = arms["r_elbow"]
    pose.l_shoulder_roll = arms["l_shoulder_roll"]
    pose.r_shoulder_roll = arms["r_shoulder_roll"]
    return pose


def _dribble_pose(progress: float) -> JointPose:
    """带球轻触: 短促前踢 + 轻量摆臂"""
    pose = JointPose()
    if progress < 0.30:
        t = _smoothstep(progress / 0.30)
        pose.r_hip = _lerp(0.0, 0.18, t)
        pose.r_knee = _lerp(0.15, 0.55, t)
        pose.l_hip = -0.04
        pose.r_shoulder = _lerp(0.0, -0.28, t)
        pose.l_shoulder = _lerp(0.0, 0.40, t)
    elif progress < 0.55:
        t = _smoothstep((progress - 0.30) / 0.25)
        pose.r_hip = _lerp(0.18, -0.45, t)
        pose.r_knee = _lerp(0.55, 0.20, t)
        pose.l_hip = -0.04
        pose.r_shoulder = _lerp(-0.28, 0.40, t)
        pose.l_shoulder = _lerp(0.40, -0.22, t)
    else:
        t = _smoothstep((progress - 0.55) / 0.45)
        pose.r_hip = _lerp(-0.45, 0.0, t)
        pose.r_knee = _lerp(0.20, 0.10, t)
        pose.l_hip = _lerp(-0.04, 0.0, t)
        pose.r_shoulder = _lerp(0.40, 0.0, t)
        pose.l_shoulder = _lerp(-0.22, 0.0, t)

    arms = _arm_balance(pose.l_shoulder, pose.r_shoulder,
                        elbow_base=-0.55)
    pose.l_elbow = arms["l_elbow"]
    pose.r_elbow = arms["r_elbow"]
    pose.l_shoulder_roll = arms["l_shoulder_roll"]
    pose.r_shoulder_roll = arms["r_shoulder_roll"]
    return pose


def _walk_pose(phase: float, amp: float = 0.42) -> JointPose:
    """
    对角跑步态: 腿与对侧手臂反相大幅摆动。
    phase ∈ ℝ; amp 控制步幅。
    """
    # 手臂振幅: 下垂姿下的前后摆, 不宜过大
    arm_amp = amp * 1.05
    l_sh = math.sin(phase) * arm_amp
    r_sh = math.sin(phase + math.pi) * arm_amp
    pump_l = 0.18 * max(0.0, math.sin(phase))
    pump_r = 0.18 * max(0.0, math.sin(phase + math.pi))
    arms = _arm_balance(l_sh, r_sh, elbow_base=ELBOW_RUN_BASE)
    return JointPose(
        r_hip=math.sin(phase) * amp,
        r_knee=max(0.0, math.sin(phase) * amp * 0.95),
        l_hip=math.sin(phase + math.pi) * amp,
        l_knee=max(0.0, math.sin(phase + math.pi) * amp * 0.95),
        r_shoulder=r_sh,
        l_shoulder=l_sh,
        r_elbow=arms["r_elbow"] - pump_r,
        l_elbow=arms["l_elbow"] - pump_l,
        r_shoulder_roll=arms["r_shoulder_roll"],
        l_shoulder_roll=arms["l_shoulder_roll"],
    )


def _turn_pose(turn_dir: float) -> JointPose:
    """转向: 外侧臂后拉、内侧臂前探引导转向"""
    s = 1.0 if turn_dir >= 0 else -1.0
    l_sh = -0.55 * s
    r_sh = 0.55 * s
    arms = _arm_balance(l_sh, r_sh, elbow_base=-0.55)
    return JointPose(
        r_hip=-0.12 * s,
        r_knee=0.35 if s < 0 else 0.18,
        l_hip=0.12 * s,
        l_knee=0.35 if s > 0 else 0.18,
        r_shoulder=r_sh,
        l_shoulder=l_sh,
        r_elbow=arms["r_elbow"],
        l_elbow=arms["l_elbow"],
        r_shoulder_roll=arms["r_shoulder_roll"],
        l_shoulder_roll=arms["l_shoulder_roll"],
    )


def _brake_pose() -> JointPose:
    """刹车: 双臂略前探但仍下垂"""
    return JointPose(
        r_hip=0.18,
        r_knee=0.55,
        l_hip=0.18,
        l_knee=0.55,
        r_shoulder=0.30,
        l_shoulder=0.30,
        r_elbow=-0.40,
        l_elbow=-0.40,
        r_shoulder_roll=SHOULDER_ROLL_HANG_R - 0.15,
        l_shoulder_roll=SHOULDER_ROLL_HANG_L + 0.15,
    )


def _idle_pose() -> JointPose:
    """站立: 双臂自然下垂"""
    return JointPose(
        r_elbow=ELBOW_RUN_BASE,
        l_elbow=ELBOW_RUN_BASE,
        r_shoulder_roll=SHOULDER_ROLL_HANG_R,
        l_shoulder_roll=SHOULDER_ROLL_HANG_L,
    )


def style_from_power(power: float) -> str:
    return "dribble" if power <= DRIBBLE_POWER_THRESHOLD else "kick"


def kick_swing_joint_offsets(
    progress: float,
    style: str = "kick",
) -> Dict[str, float]:
    """mocap 姿态 → mujoco_soccer T1 关节偏移"""
    pose = _dribble_pose(progress) if style == "dribble" else _kick_pose(progress)
    scale = 0.55 if style == "dribble" else 0.85
    return {
        "Right_Hip_Pitch": -pose.r_hip * scale,
        "Right_Knee_Pitch": pose.r_knee * scale * 0.7,
        "Right_Ankle_Pitch": pose.r_hip * 0.25 * scale,
        "Left_Hip_Pitch": -pose.l_hip * scale * 0.5,
        "Left_Knee_Pitch": pose.l_knee * scale * 0.4,
        "Right_Shoulder_Pitch": pose.r_shoulder * scale,
        "Left_Shoulder_Pitch": pose.l_shoulder * scale,
        "Right_Elbow_Pitch": pose.r_elbow * scale,
        "Left_Elbow_Pitch": pose.l_elbow * scale,
        "Right_Shoulder_Roll": pose.r_shoulder_roll * scale,
        "Left_Shoulder_Roll": pose.l_shoulder_roll * scale,
        "Waist": (-pose.r_hip) * 0.08 * scale,
        "Head_pitch": 0.06 if style == "kick" else 0.03,
    }


@dataclass
class LimbAnimator:
    """管理所有机器人的肢体姿态"""

    kicks: Dict[int, KickClip] = field(default_factory=dict)
    walk_phase: Dict[int, float] = field(default_factory=dict)
    poses: Dict[int, JointPose] = field(default_factory=dict)
    turn_fade: Dict[int, float] = field(default_factory=dict)
    brake_fade: Dict[int, float] = field(default_factory=dict)
    _ready_impulses: List[Tuple[int, float, float]] = field(default_factory=list)

    def start_kick(
        self,
        robot_id: int,
        power: float,
        direction: float,
        style: Optional[str] = None,
        duration: Optional[float] = None,
    ) -> None:
        resolved = style or style_from_power(power)
        if duration is None:
            duration = DRIBBLE_DURATION if resolved == "dribble" else KICK_DURATION
        self.kicks[robot_id] = KickClip(
            robot_id=robot_id,
            power=power,
            direction=direction,
            style=resolved,
            duration=duration,
        )

    def is_kicking(self, robot_id: int) -> bool:
        return robot_id in self.kicks

    def pop_ready_impulses(self) -> List[Tuple[int, float, float]]:
        ready = list(self._ready_impulses)
        self._ready_impulses.clear()
        return ready

    def step(
        self,
        dt: float,
        moving_ids: Optional[Set[int]] = None,
        turning: Optional[Dict[int, float]] = None,
        braking_ids: Optional[Set[int]] = None,
    ) -> None:
        moving_ids = moving_ids or set()
        turning = turning or {}
        braking_ids = braking_ids or set()
        self._ready_impulses.clear()

        finished: List[int] = []
        for rid, clip in self.kicks.items():
            clip.elapsed += dt
            if clip.at_contact:
                clip.impulse_fired = True
                self._ready_impulses.append(
                    (clip.robot_id, clip.power, clip.direction))
            if clip.style == "dribble":
                self.poses[rid] = _dribble_pose(clip.progress)
            else:
                self.poses[rid] = _kick_pose(clip.progress)
            if clip.done:
                finished.append(rid)

        for rid in finished:
            del self.kicks[rid]
            self.poses[rid] = _idle_pose()

        active_ids = set(moving_ids) | set(turning) | set(braking_ids) | set(self.poses)
        for rid in active_ids:
            if rid in self.kicks:
                continue

            want_turn = rid in turning and abs(turning[rid]) > 1e-3
            self.turn_fade[rid] = _fade(
                self.turn_fade.get(rid, 0.0), want_turn, dt, up=8.0, down=4.0)
            want_brake = rid in braking_ids
            self.brake_fade[rid] = _fade(
                self.brake_fade.get(rid, 0.0), want_brake, dt, up=6.0, down=3.0)

            base = JointPose()
            if rid in moving_ids and self.brake_fade.get(rid, 0.0) < 0.85:
                # 跑步频率略提高, 摆臂更明显
                phase = self.walk_phase.get(rid, 0.0) + dt * 8.5
                self.walk_phase[rid] = phase
                amp = 0.42 * (1.0 - 0.50 * self.brake_fade.get(rid, 0.0))
                base = _walk_pose(phase, amp=amp)
            else:
                self.walk_phase.pop(rid, None)

            tf = self.turn_fade.get(rid, 0.0)
            if tf > 1e-3 and rid in turning:
                base = base.blended(_turn_pose(turning[rid]), tf)

            bf = self.brake_fade.get(rid, 0.0)
            if bf > 1e-3:
                base = base.blended(_brake_pose(), bf * 0.85)

            if rid not in moving_ids and tf < 1e-3 and bf < 1e-3:
                prev = self.poses.get(rid, _idle_pose())
                blend = min(1.0, dt * 6.0)
                self.poses[rid] = prev.blended(_idle_pose(), blend)
            else:
                self.poses[rid] = base

        for store in (self.turn_fade, self.brake_fade):
            for rid in list(store):
                if store[rid] < 1e-4 and rid not in turning and rid not in braking_ids:
                    del store[rid]

    def get_pose(self, robot_id: int) -> JointPose:
        return self.poses.get(robot_id, JointPose())

    def reset(self) -> None:
        self.kicks.clear()
        self.walk_phase.clear()
        self.poses.clear()
        self.turn_fade.clear()
        self.brake_fade.clear()
        self._ready_impulses.clear()


def _fade(current: float, active: bool, dt: float, up: float, down: float) -> float:
    if active:
        return min(1.0, current + dt * up)
    return max(0.0, current - dt * down)
