#!/usr/bin/env python3
"""Booster T1 kick demo — uses ctrl (position actuators) to stay standing."""

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

# ============================================================
# Kick keyframes (right leg only — actuator indices 17,20,21)
# ============================================================
# Home ctrl values: RHipPitch=-0.2  RKnee=0.4  RAnkle=-0.2
HOME_RHIP = -0.2; HOME_RKNEE = 0.4; HOME_RANKLE = -0.2

IDX_RHIP   = 17
IDX_RKNEE  = 20
IDX_RANKLE = 21

# Kick phases
PREP_TIME   = 0.3
SWING_TIME  = 0.10
RECOVER_TIME = 0.4

def lerp_f(a, b, t):
    t = max(0, min(1, t))
    return a + (b - a) * t

# ============================================================
# Load model
# ============================================================
print("Loading model...")
m = mujoco.MjModel.from_xml_path('assets/soccer_humanoid.xml')
d = mujoco.MjData(m)

# Store home ctrl (all 23 actuators at home position)
HOME_CTRL = m.key_ctrl[0].copy() if m.nkey > 0 else np.zeros(m.nu)
d.ctrl[:] = HOME_CTRL
mujoco.mj_forward(m, d)

# Find freejoints by body name
t1_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'Trunk')
t1_jnt = m.body_jntadr[t1_body_id]  # first joint of this body
T1_QPOS = int(m.jnt_qposadr[t1_jnt])
ball_jnt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'ball_joint')
BALL_QPOS = int(m.jnt_qposadr[ball_jnt])

print("Model: %d bodies, %d ctrl, T1_qpos=%d" % (m.nbody, m.nu, T1_QPOS))

# ============================================================
# 2D simulation
# ============================================================
sim = Simulator(num_blue=2, num_yellow=1)
provider = WorldStateProvider(sim)
provider.set_mock(create_pass_and_shoot_scenario())
action = MockRobotAction(sim)
fsm = DecisionFSM(provider.get(), action, 2)

# ============================================================
# Kick state machine
# ============================================================
kick_phase = 'idle'  # idle | prep | swing | recover
kick_timer = 0.0
ball_kicked = False
prev_shoot = False
t1_x, t1_y, t1_h = -1.5, 1.0, 0.3

print("\n=== Booster T1 Kicking (Position Control) ===")

# ============================================================
# Viewer loop
# ============================================================
with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.lookat[:] = [0, 0, 0.7]
    v.cam.distance = 8
    v.cam.elevation = -20
    v.cam.azimuth = 90

    total = int(30 / DT)
    for tick in range(total):
        loop_start = time.time()
        if not v.is_running():
            break

        # ---- 2D decision ----
        sim.update(DT)
        ws = provider.get()
        fsm.update(ws, DT)
        s0 = fsm.get_state(0); s1 = fsm.get_state(1)
        shooting = (s0 == DecisionState.SHOOT or s1 == DecisionState.SHOOT)

        # ---- Track R1 position ----
        if ws.teammates and kick_phase == 'idle':
            t1_x, t1_y = ws.teammates[0].x, ws.teammates[0].y
            t1_h = ws.teammates[0].theta

        # ---- Lock all joints to home by default ----
        d.ctrl[:] = HOME_CTRL

        # ---- Kick animation ----
        if shooting and not prev_shoot and kick_phase == 'idle':
            kick_phase = 'prep'; kick_timer = 0; ball_kicked = False
            print("  [t=%.1fs] KICK!" % ws.timestamp)

        if kick_phase != 'idle':
            kick_timer += DT

        if kick_phase == 'prep':
            t = min(kick_timer / PREP_TIME, 1.0)
            d.ctrl[IDX_RHIP]  = lerp_f(HOME_RHIP, 0.7, t)
            d.ctrl[IDX_RKNEE] = lerp_f(HOME_RKNEE, 1.3, t)
            d.ctrl[IDX_RANKLE] = lerp_f(HOME_RANKLE, 0.0, t)
            if t >= 1.0: kick_phase = 'swing'; kick_timer = 0

        elif kick_phase == 'swing':
            t = min(kick_timer / SWING_TIME, 1.0)
            d.ctrl[IDX_RHIP]  = lerp_f(0.7, -1.2, t)
            d.ctrl[IDX_RKNEE] = lerp_f(1.3, 0.15, t)
            d.ctrl[IDX_RANKLE] = lerp_f(0.0, -0.2, t)
            if t >= 0.7 and not ball_kicked:
                ball_kicked = True
                d.qvel[BALL_QPOS + 0] = 7.5
                d.qvel[BALL_QPOS + 1] = -0.3
                d.qvel[BALL_QPOS + 2] = 1.0
            if t >= 1.0: kick_phase = 'recover'; kick_timer = 0

        elif kick_phase == 'recover':
            t = min(kick_timer / RECOVER_TIME, 1.0)
            d.ctrl[IDX_RHIP]  = lerp_f(-1.2, HOME_RHIP, t)
            d.ctrl[IDX_RKNEE] = lerp_f(0.15, HOME_RKNEE, t)
            d.ctrl[IDX_RANKLE] = lerp_f(-0.2, HOME_RANKLE, t)
            if t >= 1.0:
                kick_phase = 'idle'
                print("  [t=%.1fs] recovery done" % ws.timestamp)

        prev_shoot = shooting

        # ---- Clear ball ctrl/kf during kick ----
        if kick_phase != 'idle' and ball_kicked:
            # Let MuJoCo physics handle the ball after kick
            pass

        # ---- Sync ball position when not kicking ----
        if kick_phase == 'idle':
            d.qpos[BALL_QPOS + 0] = ws.ball.x
            d.qpos[BALL_QPOS + 1] = ws.ball.y
            d.qpos[BALL_QPOS + 2] = 0.05
            d.qpos[BALL_QPOS + 3] = 1.0
            d.qpos[BALL_QPOS + 4] = 0.0
            d.qpos[BALL_QPOS + 5] = 0.0
            d.qpos[BALL_QPOS + 6] = 0.0
            d.qvel[BALL_QPOS + 0] = ws.ball.vx
            d.qvel[BALL_QPOS + 1] = ws.ball.vy
            d.qvel[BALL_QPOS + 2] = 0.0

        # ---- Sync T1 position ----
        d.qpos[T1_QPOS + 0] = t1_x
        d.qpos[T1_QPOS + 1] = t1_y
        d.qpos[T1_QPOS + 2] = 0.665
        half = t1_h / 2
        d.qpos[T1_QPOS + 3] = math.cos(half)
        d.qpos[T1_QPOS + 4] = 0.0
        d.qpos[T1_QPOS + 5] = 0.0
        d.qpos[T1_QPOS + 6] = math.sin(half)

        mujoco.mj_step(m, d)
        v.sync()

        if tick % (FPS * 5) == 0:
            print("  t=%ds  R0=%s R1=%s kick=%s" % (
                int(ws.timestamp), s0.value if s0 else '-',
                s1.value if s1 else '-', kick_phase))

        time.sleep(max(0.0, DT - (time.time() - loop_start)))

os._exit(0)
