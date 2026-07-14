#!/usr/bin/env python3
"""Booster T1 walk + kick demo with cyclic walking gait."""

import os, sys, math, time
os.chdir('/home/l/project4_multi_agent')
sys.path.insert(0, '.')
import mujoco, mujoco.viewer
import numpy as np
from common.config import DT, FPS
from common.world_state import WorldStateProvider, create_pass_and_shoot_scenario
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM, DecisionState

# ---- Model ----
m = mujoco.MjModel.from_xml_path('assets/soccer_humanoid.xml')
d = mujoco.MjData(m)
HOME_CTRL = m.key_ctrl[0].copy()
d.ctrl[:] = HOME_CTRL
mujoco.mj_forward(m, d)

t1_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'Trunk')
T1_QPOS = int(m.jnt_qposadr[m.body_jntadr[t1_body]])
ball_jnt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'ball_joint')
BALL_QPOS = int(m.jnt_qposadr[ball_jnt])

# Actuator indices for legs
# Left:  hip_pitch=11, knee=14, ankle=15
# Right: hip_pitch=17, knee=20, ankle=21
L_HIP=11; L_KNEE=14; L_ANKLE=15
R_HIP=17; R_KNEE=20; R_ANKLE=21

# ---- Walking gait ----
# Home angles
L_HIP_HOME=-0.2; L_KNEE_HOME=0.4; L_ANKLE_HOME=-0.2
R_HIP_HOME=-0.2; R_KNEE_HOME=0.4; R_ANKLE_HOME=-0.2

# Swing amplitudes
HIP_SWING = 0.5   # hip forward/back range
KNEE_LIFT = 0.6   # knee bend during swing
ANKLE_TILT = 0.15

WALK_PERIOD = 0.8  # seconds per full cycle (both legs)

def walk_legs(t):
    """Compute leg joint angles for walking at phase t (0..1 = one cycle)."""
    t = t % 1.0
    # Two half-cycles: 0-0.5 right stance/left swing, 0.5-1.0 left stance/right swing
    if t < 0.5:
        phase = t * 2  # 0..1 for this half
        # Left leg swings forward, right leg stance
        l_hip = L_HIP_HOME + math.sin(phase * math.pi) * HIP_SWING
        l_knee = L_KNEE_HOME + math.sin(phase * math.pi) * KNEE_LIFT
        l_ankle = L_ANKLE_HOME
        r_hip = R_HIP_HOME - math.sin(phase * math.pi) * HIP_SWING * 0.3
        r_knee = R_KNEE_HOME
        r_ankle = R_ANKLE_HOME
    else:
        phase = (t - 0.5) * 2
        # Right leg swings forward, left leg stance
        l_hip = L_HIP_HOME - math.sin(phase * math.pi) * HIP_SWING * 0.3
        l_knee = L_KNEE_HOME
        l_ankle = L_ANKLE_HOME
        r_hip = R_HIP_HOME + math.sin(phase * math.pi) * HIP_SWING
        r_knee = R_KNEE_HOME + math.sin(phase * math.pi) * KNEE_LIFT
        r_ankle = R_ANKLE_HOME

    return {L_HIP: l_hip, L_KNEE: l_knee, L_ANKLE: l_ankle,
            R_HIP: r_hip, R_KNEE: r_knee, R_ANKLE: r_ankle}

# ---- Kick keyframes ----
PREP_TIME=0.3; SWING_TIME=0.1; RECOVER_TIME=0.4

# ---- 2D sim ----
sim = Simulator(num_blue=2, num_yellow=1)
provider = WorldStateProvider(sim)
provider.set_mock(create_pass_and_shoot_scenario())
action = MockRobotAction(sim)
fsm = DecisionFSM(provider.get(), action, 2)

kick_phase = 'idle'
kick_timer = 0.0
ball_kicked = False
prev_shoot = False
t1_x, t1_y, t1_h = -1.5, 1.0, 0.3
walk_clock = 0.0

print("=== Booster T1 Walk + Kick ===")
print("  Legs cycle while T1 moves, kick on SHOOT state")

with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.lookat[:] = [0, 0, 0.7]
    v.cam.distance = 8
    v.cam.elevation = -20
    v.cam.azimuth = 90

    for tick in range(int(30 / DT)):
        loop_start = time.time()
        if not v.is_running(): break

        sim.update(DT); ws = provider.get(); fsm.update(ws, DT)
        s0 = fsm.get_state(0); s1 = fsm.get_state(1)
        shooting = (s0 == DecisionState.SHOOT or s1 == DecisionState.SHOOT)

        # Track R1 position
        speed = 0.0
        if ws.teammates:
            r = ws.teammates[0]
            dx = r.x - t1_x; dy = r.y - t1_y
            speed = math.sqrt(dx*dx + dy*dy)
            t1_x += dx * min(speed * 3.0, 1.0) * DT
            t1_y += dy * min(speed * 3.0, 1.0) * DT
            t1_h = r.theta

        # Default: lock all joints to home
        d.ctrl[:] = HOME_CTRL

        # ---- Walking animation (when not kicking) ----
        if kick_phase == 'idle' and speed > 0.05:
            walk_clock += DT
            legs = walk_legs(walk_clock / WALK_PERIOD)
            for idx, val in legs.items():
                d.ctrl[idx] = val
        else:
            walk_clock = 0.0

        # ---- Kick animation ----
        if shooting and not prev_shoot and kick_phase == 'idle':
            kick_phase = 'prep'; kick_timer = 0; ball_kicked = False
            print("  [t=%.1fs] KICK!" % ws.timestamp)

        if kick_phase != 'idle':
            kick_timer += DT
            walk_clock = 0.0

        def lerp(a,b,t): return a+(b-a)*max(0,min(1,t))

        if kick_phase == 'prep':
            t = min(kick_timer/PREP_TIME, 1.0)
            d.ctrl[R_HIP]=lerp(R_HIP_HOME,0.7,t); d.ctrl[R_KNEE]=lerp(R_KNEE_HOME,1.3,t)
            d.ctrl[R_ANKLE]=lerp(R_ANKLE_HOME,0.0,t)
            if t>=1.0: kick_phase='swing'; kick_timer=0

        elif kick_phase == 'swing':
            t = min(kick_timer/SWING_TIME, 1.0)
            d.ctrl[R_HIP]=lerp(0.7,-1.2,t); d.ctrl[R_KNEE]=lerp(1.3,0.15,t)
            d.ctrl[R_ANKLE]=lerp(0.0,-0.2,t)
            if t>=0.7 and not ball_kicked:
                ball_kicked=True
                d.qvel[BALL_QPOS+0]=7.5; d.qvel[BALL_QPOS+1]=-0.3; d.qvel[BALL_QPOS+2]=1.0
            if t>=1.0: kick_phase='recover'; kick_timer=0

        elif kick_phase == 'recover':
            t = min(kick_timer/RECOVER_TIME, 1.0)
            d.ctrl[R_HIP]=lerp(-1.2,R_HIP_HOME,t); d.ctrl[R_KNEE]=lerp(0.15,R_KNEE_HOME,t)
            d.ctrl[R_ANKLE]=lerp(-0.2,R_ANKLE_HOME,t)
            if t>=1.0: kick_phase='idle'; print("  [t=%.1fs] done" % ws.timestamp)

        prev_shoot=shooting

        # Sync ball when idle
        if kick_phase == 'idle':
            d.qpos[BALL_QPOS+0]=ws.ball.x; d.qpos[BALL_QPOS+1]=ws.ball.y; d.qpos[BALL_QPOS+2]=0.05
            d.qpos[BALL_QPOS+3]=1.0; d.qpos[BALL_QPOS+4]=0.0; d.qpos[BALL_QPOS+5]=0.0; d.qpos[BALL_QPOS+6]=0.0
            d.qvel[BALL_QPOS+0]=ws.ball.vx; d.qvel[BALL_QPOS+1]=ws.ball.vy; d.qvel[BALL_QPOS+2]=0.0

        # Sync T1 position
        d.qpos[T1_QPOS+0]=t1_x; d.qpos[T1_QPOS+1]=t1_y; d.qpos[T1_QPOS+2]=0.665
        h=t1_h/2; d.qpos[T1_QPOS+3]=math.cos(h); d.qpos[T1_QPOS+4]=0.0
        d.qpos[T1_QPOS+5]=0.0; d.qpos[T1_QPOS+6]=math.sin(h)

        mujoco.mj_step(m, d); v.sync()

        if tick%(FPS*5)==0:
            print("  t=%ds R0=%s R1=%s walk=%.1f kick=%s" % (
                int(ws.timestamp), s0.value if s0 else '-', s1.value if s1 else '-',
                speed, kick_phase))

        time.sleep(max(0.0, DT-(time.time()-loop_start)))

os._exit(0)
