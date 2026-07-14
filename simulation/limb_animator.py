"""
机器人肢体动画器
================
为 MuJoCo mocap 人形驱动关节角度。

动作优先级 (高→低):
  kick / dribble 剪辑 → 转向 → 刹车 → 行走 → 站立

关节约定 (robot 局部坐标, hinge axis = +Y):
  hip > 0 大腿后摆, hip < 0 大腿前踢
  knee > 0 小腿弯曲
  shoulder: 手臂前后摆
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


KICK_DURATION = 0.55
DRIBBLE_DURATION = 0.32
KICK_CONTACT_RATIO = 0.48
DRIBBLE_CONTACT_RATIO = 0.42
# power <= 此值视为带球轻触 (与 strategy_dribble.power=15 对齐)
DRIBBLE_POWER_THRESHOLD = 25.0


@dataclass
class JointPose:
    """单机全身关节角 (弧度)"""
    r_hip: float = 0.0
    r_knee: float = 0.0
    l_hip: float = 0.0
    l_knee: float = 0.0
    r_shoulder: float = 0.0
    l_shoulder: float = 0.0

    def scaled(self, s: float) -> "JointPose":
        return JointPose(
            r_hip=self.r_hip * s,
            r_knee=self.r_knee * s,
            l_hip=self.l_hip * s,
            l_knee=self.l_knee * s,
            r_shoulder=self.r_shoulder * s,
            l_shoulder=self.l_shoulder * s,
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
        )


@dataclass
class KickClip:
    """一次踢球/带球剪辑"""
    robot_id: int
    power: float
    direction: float
    style: str = "kick"  # "kick" | "dribble"
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


def _kick_pose(progress: float) -> JointPose:
    """大力踢球: 蓄力 → 摆腿 → 触球 → 收回"""
    pose = JointPose()
    if progress < 0.25:
        t = _smoothstep(progress / 0.25)
        pose.r_hip = _lerp(0.0, 0.55, t)
        pose.r_knee = _lerp(0.0, 1.35, t)
        pose.l_hip = _lerp(0.0, -0.08, t)
        pose.r_shoulder = _lerp(0.0, -0.35, t)
        pose.l_shoulder = _lerp(0.0, 0.45, t)
    elif progress < 0.48:
        t = _smoothstep((progress - 0.25) / 0.23)
        pose.r_hip = _lerp(0.55, -1.05, t)
        pose.r_knee = _lerp(1.35, 0.25, t)
        pose.l_hip = -0.08
        pose.r_shoulder = _lerp(-0.35, 0.55, t)
        pose.l_shoulder = _lerp(0.45, -0.25, t)
    elif progress < 0.70:
        t = _smoothstep((progress - 0.48) / 0.22)
        pose.r_hip = _lerp(-1.05, -0.85, t)
        pose.r_knee = _lerp(0.25, 0.35, t)
        pose.l_hip = _lerp(-0.08, 0.0, t)
        pose.r_shoulder = _lerp(0.55, 0.25, t)
        pose.l_shoulder = _lerp(-0.25, 0.0, t)
    else:
        t = _smoothstep((progress - 0.70) / 0.30)
        pose.r_hip = _lerp(-0.85, 0.0, t)
        pose.r_knee = _lerp(0.35, 0.0, t)
        pose.r_shoulder = _lerp(0.25, 0.0, t)
    return pose


def _dribble_pose(progress: float) -> JointPose:
    """
    带球轻触: 短促前摆、幅度小, 几乎不后摆蓄力。
    适合反复垫球推进。
    """
    pose = JointPose()
    if progress < 0.30:
        t = _smoothstep(progress / 0.30)
        pose.r_hip = _lerp(0.0, 0.18, t)      # 轻微后收
        pose.r_knee = _lerp(0.15, 0.55, t)
        pose.l_hip = -0.04
        pose.r_shoulder = _lerp(0.0, -0.12, t)
        pose.l_shoulder = _lerp(0.0, 0.18, t)
    elif progress < 0.55:
        t = _smoothstep((progress - 0.30) / 0.25)
        pose.r_hip = _lerp(0.18, -0.45, t)    # 短促前踢
        pose.r_knee = _lerp(0.55, 0.20, t)
        pose.l_hip = -0.04
        pose.r_shoulder = _lerp(-0.12, 0.22, t)
        pose.l_shoulder = _lerp(0.18, -0.08, t)
    else:
        t = _smoothstep((progress - 0.55) / 0.45)
        pose.r_hip = _lerp(-0.45, 0.0, t)
        pose.r_knee = _lerp(0.20, 0.10, t)
        pose.l_hip = _lerp(-0.04, 0.0, t)
        pose.r_shoulder = _lerp(0.22, 0.0, t)
        pose.l_shoulder = _lerp(-0.08, 0.0, t)
    return pose


def _walk_pose(phase: float, amp: float = 0.35) -> JointPose:
    """简易对角步态, phase ∈ ℝ"""
    return JointPose(
        r_hip=math.sin(phase) * amp,
        r_knee=max(0.0, math.sin(phase) * amp * 0.9),
        l_hip=math.sin(phase + math.pi) * amp,
        l_knee=max(0.0, math.sin(phase + math.pi) * amp * 0.9),
        r_shoulder=math.sin(phase + math.pi) * amp * 0.6,
        l_shoulder=math.sin(phase) * amp * 0.6,
    )


def _turn_pose(turn_dir: float) -> JointPose:
    """
    转向姿态: turn_dir > 0 左转 (CCW), < 0 右转。
    内侧腿微屈承重, 外侧肩后拉, 呈现躯干准备转向。
    """
    s = 1.0 if turn_dir >= 0 else -1.0
    return JointPose(
        r_hip=-0.12 * s,
        r_knee=0.35 if s < 0 else 0.18,
        l_hip=0.12 * s,
        l_knee=0.35 if s > 0 else 0.18,
        r_shoulder=0.40 * s,
        l_shoulder=-0.40 * s,
    )


def _brake_pose() -> JointPose:
    """刹车下蹲: 双膝弯曲、髋略前倾、手臂前伸保持平衡"""
    return JointPose(
        r_hip=0.18,
        r_knee=0.55,
        l_hip=0.18,
        l_knee=0.55,
        r_shoulder=0.35,
        l_shoulder=0.35,
    )


def style_from_power(power: float) -> str:
    return "dribble" if power <= DRIBBLE_POWER_THRESHOLD else "kick"


def kick_swing_joint_offsets(
    progress: float,
    style: str = "kick",
) -> Dict[str, float]:
    """
    将 mocap 肢体姿态映射为 mujoco_soccer T1 关节偏移。
    供 VisibleGaitController 叠加使用 (右腿踢球约定)。
    """
    pose = _dribble_pose(progress) if style == "dribble" else _kick_pose(progress)
    # T1 Hip_Pitch: 正值偏向前抬腿; 我们的负 hip 前踢 → 取反
    # Knee: 正值屈膝, 与我们的约定一致
    scale = 0.55 if style == "dribble" else 0.85
    return {
        "Right_Hip_Pitch": -pose.r_hip * scale,
        "Right_Knee_Pitch": pose.r_knee * scale * 0.7,
        "Right_Ankle_Pitch": pose.r_hip * 0.25 * scale,
        "Left_Hip_Pitch": -pose.l_hip * scale * 0.5,
        "Left_Knee_Pitch": pose.l_knee * scale * 0.4,
        "Right_Shoulder_Pitch": pose.r_shoulder * scale,
        "Left_Shoulder_Pitch": pose.l_shoulder * scale,
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
        """开始一次踢球/带球动画"""
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
        """
        推进动画并更新 poses。

        Args:
            moving_ids: 正在移动的机器人
            turning: robot_id → 剩余转角差 (有符号, >0 左转)
            braking_ids: 正在减速接近目标的机器人
        """
        moving_ids = moving_ids or set()
        turning = turning or {}
        braking_ids = braking_ids or set()
        self._ready_impulses.clear()

        # --- 踢球 / 带球剪辑 (最高优先) ---
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
            self.poses[rid] = JointPose()

        # --- 更新转向 / 刹车 fade ---
        active_ids = set(moving_ids) | set(turning) | set(braking_ids) | set(self.poses)
        for rid in active_ids:
            if rid in self.kicks:
                continue

            # fade in/out
            want_turn = rid in turning and abs(turning[rid]) > 1e-3
            self.turn_fade[rid] = _fade(
                self.turn_fade.get(rid, 0.0), want_turn, dt, up=8.0, down=4.0)
            want_brake = rid in braking_ids
            self.brake_fade[rid] = _fade(
                self.brake_fade.get(rid, 0.0), want_brake, dt, up=6.0, down=3.0)

            base = JointPose()
            if rid in moving_ids and self.brake_fade.get(rid, 0.0) < 0.85:
                phase = self.walk_phase.get(rid, 0.0) + dt * 7.0
                self.walk_phase[rid] = phase
                amp = 0.35 * (1.0 - 0.55 * self.brake_fade.get(rid, 0.0))
                base = _walk_pose(phase, amp=amp)
            else:
                self.walk_phase.pop(rid, None)

            tf = self.turn_fade.get(rid, 0.0)
            if tf > 1e-3 and rid in turning:
                base = base.blended(_turn_pose(turning[rid]), tf)

            bf = self.brake_fade.get(rid, 0.0)
            if bf > 1e-3:
                base = base.blended(_brake_pose(), bf * 0.85)

            # 无运动时平滑收立
            if rid not in moving_ids and tf < 1e-3 and bf < 1e-3:
                prev = self.poses.get(rid, JointPose())
                blend = min(1.0, dt * 6.0)
                self.poses[rid] = prev.blended(JointPose(), blend)
            else:
                self.poses[rid] = base

        # 清理无用 fade
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
