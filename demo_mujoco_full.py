#!/usr/bin/env python3
"""MuJoCo 3D demo for any scenario (pass, 2v2, interference, etc.)"""

import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco.viewer
from common.config import DT, FPS, MAX_TICKS, NUM_ROBOTS_PER_TEAM
from common.world_state import WorldStateProvider, SCENARIOS
from common.robot_action import MockRobotAction
from simulation.mujoco_simulator import MuJoCoSimulator
from decision.decision_fsm import DecisionFSM


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', default='pass', choices=list(SCENARIOS.keys()))
    p.add_argument('--duration', type=int, default=30)
    p.add_argument('--num-blue', type=int, default=3)
    p.add_argument('--num-yellow', type=int, default=3)
    return p.parse_args()


def main():
    args = parse_args()
    num_blue = min(args.num_blue, 3)
    num_yellow = min(args.num_yellow, 3)

    print('=' * 60)
    print(f'  MuJoCo 3D Demo: {args.scenario} ({num_blue}v{num_yellow})')
    print('=' * 60)
    print(f'  Duration: {args.duration}s  |  Blue: {num_blue}  Yellow: {num_yellow}')
    print('  Controls: mouse drag=rotate, scroll=zoom, right-drag=pan')
    print('=' * 60)

    sim = MuJoCoSimulator(num_blue=num_blue, num_yellow=num_yellow)
    ws = SCENARIOS[args.scenario]()
    sim.load_world_state(ws)

    provider = WorldStateProvider(sim)
    action = MockRobotAction(sim)
    fsm = DecisionFSM(provider.get(), action, num_robots=num_blue)

    total_ticks = min(int(args.duration / DT), MAX_TICKS)
    print(f'\n  Steps: {total_ticks}\n')

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0]
        viewer.cam.distance = 12
        viewer.cam.elevation = -35
        viewer.cam.azimuth = 90

        start = time.time()
        for tick in range(total_ticks):
            if not viewer.is_running():
                break

            loop_start = time.time()
            ws = provider.get()
            fsm.update(ws, DT)
            sim.update(DT)
            viewer.sync()

            elapsed = time.time() - loop_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

            if tick > 0 and tick % (FPS * 5) == 0:
                summary = fsm.get_decision_summary()
                states = summary.get('state_distribution', {})
                print(f'  [t={ws.timestamp:.0f}s] states={states}')

    elapsed_total = time.time() - start
    summary = fsm.get_decision_summary()
    print(f'\n{"=" * 60}')
    print(f'  Done  |  Wall time: {elapsed_total:.1f}s')
    print(f'  States: {summary.get("state_distribution", {})}')
    print(f'{"=" * 60}\n')
    import os as _os
    _os._exit(0)


if __name__ == '__main__':
    main()
