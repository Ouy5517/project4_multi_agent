"""肢体动画器单测 — kick / dribble / turn / brake / arm swing"""
import math

from simulation.limb_animator import (
    LimbAnimator,
    KICK_CONTACT_RATIO,
    DRIBBLE_POWER_THRESHOLD,
    _kick_pose,
    _dribble_pose,
    _turn_pose,
    _brake_pose,
    style_from_power,
)


def test_kick_pose_windup_pulls_leg_back():
    pose = _kick_pose(0.12)
    assert pose.r_hip > 0.2
    assert pose.r_knee > 0.5


def test_kick_pose_contact_swings_forward():
    pose = _kick_pose(KICK_CONTACT_RATIO)
    assert pose.r_hip < -0.5


def test_kick_pose_recovers_to_stance():
    pose = _kick_pose(1.0)
    assert abs(pose.r_hip) < 0.05
    assert abs(pose.r_knee) < 0.05


def test_dribble_pose_is_shallower_than_kick():
    kick = _kick_pose(0.48)
    dribble = _dribble_pose(0.42)
    assert abs(dribble.r_hip) < abs(kick.r_hip)
    assert dribble.r_knee < kick.r_knee + 0.2


def test_style_from_power():
    assert style_from_power(15.0) == "dribble"
    assert style_from_power(DRIBBLE_POWER_THRESHOLD) == "dribble"
    assert style_from_power(80.0) == "kick"


def test_turn_pose_asymmetric():
    left = _turn_pose(0.5)
    right = _turn_pose(-0.5)
    assert left.r_shoulder > 0
    assert right.r_shoulder < 0
    assert left.l_knee > left.r_knee


def test_walk_pose_has_contralateral_arm_swing():
    from simulation.limb_animator import _walk_pose
    pose = _walk_pose(math.pi / 2, amp=0.42)
    assert pose.l_shoulder > 0.3
    assert pose.r_shoulder < -0.3
    assert pose.r_hip * pose.r_shoulder < 0
    assert pose.l_elbow < -0.4
    assert pose.r_elbow < -0.4
    # 下垂: 左负右正
    assert pose.l_shoulder_roll < -1.0
    assert pose.r_shoulder_roll > 1.0


def test_kick_pose_arms_counterbalance():
    pose = _kick_pose(0.48)
    assert abs(pose.l_shoulder) > 0.3 or abs(pose.r_shoulder) > 0.3
    assert pose.l_elbow < -0.4
    assert pose.r_elbow < -0.4
    assert pose.l_shoulder_roll < -1.0
    assert pose.r_shoulder_roll > 1.0


def test_animator_moving_swings_arms():
    anim = LimbAnimator()
    dt = 1.0 / 30.0
    peak = 0.0
    for _ in range(40):
        anim.step(dt, moving_ids={0})
        pose = anim.get_pose(0)
        peak = max(peak, abs(pose.l_shoulder), abs(pose.r_shoulder))
    assert peak > 0.35


def test_animator_fires_impulse_at_contact():
    anim = LimbAnimator()
    anim.start_kick(0, power=80.0, direction=0.0)
    dt = 1.0 / 30.0
    saw_impulse = False
    hip_min = 0.0
    for _ in range(40):
        anim.step(dt, moving_ids=set())
        impulses = anim.pop_ready_impulses()
        pose = anim.get_pose(0)
        hip_min = min(hip_min, pose.r_hip)
        if impulses:
            assert impulses == [(0, 80.0, 0.0)]
            saw_impulse = True
            assert pose.r_hip < -0.3
    assert saw_impulse
    assert hip_min < -0.8
    assert not anim.is_kicking(0)


def test_animator_dribble_shorter_and_fires():
    anim = LimbAnimator()
    anim.start_kick(1, power=15.0, direction=0.5)
    assert anim.kicks[1].style == "dribble"
    dt = 1.0 / 30.0
    saw = False
    for _ in range(25):
        anim.step(dt)
        if anim.pop_ready_impulses():
            saw = True
            break
    assert saw


def test_animator_turn_and_brake_pose():
    anim = LimbAnimator()
    dt = 1.0 / 30.0
    for _ in range(10):
        anim.step(dt, moving_ids={0}, turning={0: 0.6}, braking_ids={0})
    pose = anim.get_pose(0)
    assert pose.l_knee > 0.15  # 支撑/刹车腿屈曲
    assert abs(pose.r_shoulder) > 0.05 or abs(pose.l_shoulder) > 0.05


def test_mujoco_kick_delays_ball_impulse():
    from simulation.mujoco_simulator import MuJoCoSimulator
    from common.config import DT

    sim = MuJoCoSimulator(num_blue=2, num_yellow=0)
    r0 = sim.blue_robots[0]
    sim.ball.x = r0.x + 0.2
    sim.ball.y = r0.y
    sim.ball.vx = 0.0
    sim.ball.vy = 0.0

    sim.queue_kick(0, power=50.0, direction=0.0)
    sim.update(DT)
    assert abs(sim.ball.vx) < 1e-6

    kicked = False
    for _ in range(30):
        sim.update(DT)
        if abs(sim.ball.vx) > 0.1:
            kicked = True
            break
    assert kicked
    assert 0 in sim._limb_qpos
    assert len(sim._limb_qpos[0]) == 6


def test_mujoco_dribble_style_on_low_power():
    from simulation.mujoco_simulator import MuJoCoSimulator

    sim = MuJoCoSimulator(num_blue=1, num_yellow=0)
    sim.queue_kick(0, power=15.0, direction=0.0)
    assert sim._limb_animator.kicks[0].style == "dribble"
