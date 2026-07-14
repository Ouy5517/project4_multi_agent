#!/usr/bin/env python3
"""Booster T1 — kick animation on soccer field with decision FSM."""

import os, sys, math, time
os.chdir('/home/l/project4_multi_agent')
sys.path.insert(0, '.')
import mujoco, mujoco.viewer
import numpy as np
from common.config import DT, FPS, FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH
from common.world_state import WorldStateProvider, create_pass_and_shoot_scenario
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM, DecisionState

# ---- Kick animation keyframes (right leg angles) ----
# Home:    RHipPitch=-0.2  RKnee=0.4  RAnkle=-0.2
# Retract: hip back, knee bent
# Kick:    hip forward fast, knee extends
HOME    = {'rhip': -0.2, 'rknee': 0.4, 'rankle': -0.2}
RETRACT = {'rhip':  0.7, 'rknee': 1.3, 'rankle':  0.0}
KICK    = {'rhip': -1.3, 'rknee': 0.15,'rankle': -0.3}

def lerp(a, b, t):
    t = max(0, min(1, t))
    return {'rhip': a['rhip']+(b['rhip']-a['rhip'])*t,
            'rknee': a['rknee']+(b['rknee']-a['rknee'])*t,
            'rankle': a['rankle']+(b['rankle']-a['rankle'])*t}

# Joint indices in combined model
# ball_joint=qpos[0:7], T1_freejoint=qpos[7:14], then hinge joints:
R_HIP_PITCH  = 31
R_KNEE_PITCH = 34
R_ANKLE_PITCH = 35

# ---- Load model ----
print("Loading T1 + soccer field...")
m = mujoco.MjModel.from_xml_path('assets/soccer_humanoid.xml')
d = mujoco.MjData(m)
d.qpos[:] = m.key_qpos[0]  # standing pose
mujoco.mj_forward(m, d)
print("Model: %d bodies, %d qpos" % (m.nbody, m.nq))

# ---- 2D decision simulation ----
sim = Simulator(num_blue=2, num_yellow=1)
provider = WorldStateProvider(sim)
provider.set_mock(create_pass_and_shoot_scenario())
action = MockRobotAction(sim)
fsm = DecisionFSM(provider.get(), action, 2)

# ---- Kick state ----
kick_phase = 'idle'  # idle | prep | swing | recover
kick_timer = 0.0
PREP_TIME = 0.35
SWING_TIME = 0.12
RECOVER_TIME = 0.45
ball_kicked = False
prev_shoot = False
t1_x, t1_y, t1_h = -1.5, 1.0, 0.3

print("\n=== Booster T1 Kick Demo ===")
print("  T1 tracks R1 position, kicks when FSM enters SHOOT")
print("  Right leg: retract -> swing -> contact -> recover")

# ---- Viewer loop ----
with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.lookat[:] = [0, 0, 0.7]
    v.cam.distance = 10
    v.cam.elevation = -25
    v.cam.azimuth = 90

    total = int(30 / DT)
    for tick in range(total):
        loop_start = time.time()
        if not v.is_running():
            break

        # 2D decision
        sim.update(DT)
        ws = provider.get()
        fsm.update(ws, DT)

        s0 = fsm.get_state(0); s1 = fsm.get_state(1)
        shooting = (s0 == DecisionState.SHOOT or s1 == DecisionState.SHOOT)

        # Track R1 when idle
        if ws.teammates and kick_phase == 'idle':
            t1_x, t1_y = ws.teammates[0].x, ws.teammates[0].y
            t1_h = ws.teammates[0].theta

        # ---- Kick animation FSM ----
        if shooting and not prev_shoot and kick_phase == 'idle':
            kick_phase = 'prep'; kick_timer = 0; ball_kicked = False
            print("  [t=%.1fs] KICK!" % ws.timestamp)

        if kick_phase != 'idle':
            kick_timer += DT

        if kick_phase == 'prep':
            t = min(kick_timer / PREP_TIME, 1.0)
            p = lerp(HOME, RETRACT, t)
            if t >= 1.0: kick_phase = 'swing'; kick_timer = 0

        elif kick_phase == 'swing':
            t = min(kick_timer / SWING_TIME, 1.0)
            p = lerp(RETRACT, KICK, t)
            # Ball contact at 70% of swing
            if t >= 0.65 and not ball_kicked:
                ball_kicked = True
                # Impulse toward opponent goal (+X direction)
                speed, ang = 7.0, 0.0 + (0.5 if t1_y < 0 else -0.3)
                d.qvel[0:3] = [math.cos(ang)*speed, math.sin(ang)*speed, 0.5]
            if t >= 1.0: kick_phase = 'recover'; kick_timer = 0

        elif kick_phase == 'recover':
            t = min(kick_timer / RECOVER_TIME, 1.0)
            p = lerp(KICK, HOME, t)
            if t >= 1.0:
                kick_phase = 'idle'
                print("  [t=%.1fs] recovery done" % ws.timestamp)

        # Apply leg pose
        if kick_phase != 'idle':
            d.qpos[R_HIP_PITCH]  = p['rhip']
            d.qpos[R_KNEE_PITCH] = p['rknee']
            d.qpos[R_ANKLE_PITCH] = p['rankle']

        prev_shoot = shooting

        # Sync ball when not kicking
        if kick_phase == 'idle':
            d.qpos[0:3] = [ws.ball.x, ws.ball.y, 0.05]
            d.qpos[3:7] = [1, 0, 0, 0]
            d.qvel[0:3] = [ws.ball.vx, ws.ball.vy, 0]

        # Sync T1 position
        d.qpos[7:10] = [t1_x, t1_y, 0.665]
        half = t1_h / 2
        d.qpos[10:14] = [math.cos(half), 0, 0, math.sin(half)]

        mujoco.mj_forward(m, d)
        v.sync()

        # Log
        if tick % (FPS * 5) == 0:
            print("  t=%ds  R0=%s R1=%s kick=%s ball=(%.1f,%.1f)" % (
                int(ws.timestamp), s0.value, s1.value, kick_phase,
                ws.ball.x, ws.ball.y))

        time.sleep(max(0.0, DT - (time.time() - loop_start)))

os._exit(0)
