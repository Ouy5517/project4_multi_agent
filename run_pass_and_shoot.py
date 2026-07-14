#!/usr/bin/env python3
'''
============================================================
  Booster T1 -- Pass -> Receive -> Shoot Continuous Demo
============================================================

Scenario: 2v1
  R1 (passer): midfield, evaluates then passes to R2
  R2 (receiver/shooter): forward position, receives, dribbles, shoots

Expected chain: R1:CHASE->PASS -> R2:CHASE->DRIBBLE->SHOOT

Usage:
    python3 run_pass_and_shoot.py
    python3 run_pass_and_shoot.py --duration 20 --export-csv
'''

import argparse, math, os, sys, time, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common.config import (DT, FPS, MAX_TICKS, FIELD_WIDTH, FIELD_HEIGHT,
                           GOAL_WIDTH, GOAL_X, OUR_GOAL_X, NUM_ROBOTS_PER_TEAM)
from common.world_state import (WorldStateProvider, Ball, Robot, Team, Goal,
                                create_pass_and_shoot_scenario)
from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator
from decision.decision_fsm import DecisionFSM, DecisionState


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=20)
    p.add_argument('--export-csv', action='store_true')
    p.add_argument('--log-dir', default='outputs')
    return p.parse_args()


class EventTracker:
    def __init__(self):
        self.events = []
    def add(self, t, phase, detail):
        self.events.append({'t': round(t, 2), 'phase': phase, 'detail': detail})
    def print_timeline(self):
        print()
        print('=' * 70)
        print('  Event Timeline')
        print('=' * 70)
        for e in self.events:
            print(f"  [{e['t']:06.2f}s] {e['phase']:<16} {e['detail']}")
        print('=' * 70)


def check_goal(ball):
    if ball.x >= FIELD_WIDTH / 2 and -GOAL_WIDTH / 2 <= ball.y <= GOAL_WIDTH / 2:
        return 'BLUE'
    if ball.x <= -FIELD_WIDTH / 2 and -GOAL_WIDTH / 2 <= ball.y <= GOAL_WIDTH / 2:
        return 'YELLOW'
    return None


def main():
    args = parse_args()

    print('=' * 70)
    print('  Booster T1 -- Pass -> Receive -> Shoot Continuous Demo')
    print('=' * 70)
    print(f'  Field: {FIELD_WIDTH}m x {FIELD_HEIGHT}m  |  Goal: {GOAL_WIDTH}m')
    print(f'  Duration: {args.duration}s  |  Expected: PASS -> RECEIVE -> DRIBBLE -> SHOOT -> GOAL!')
    print('=' * 70)

    sim = Simulator()
    provider = WorldStateProvider(sim)
    provider.set_mock(create_pass_and_shoot_scenario())
    ws = provider.get()
    action = MockRobotAction(sim)
    fsm = DecisionFSM(ws, action, NUM_ROBOTS_PER_TEAM)
    tracker = EventTracker()

    prev_carrier = None
    prev_s0, prev_s1 = None, None
    pass_done, shoot_done, goal_scored = False, False, None

    total_ticks = min(int(args.duration / DT), MAX_TICKS)
    print(f'\n  Total ticks: {total_ticks}\n')

    start = time.time()
    for tick in range(total_ticks):
        sim.update(DT)
        ws = provider.get()
        fsm.update(ws, DT)

        carrier = fsm._ball_carrier_id
        s0, s1 = fsm.get_state(0), fsm.get_state(1)

        # --- Event detection ---
        if carrier != prev_carrier:
            if carrier is not None:
                r = ws.get_robot_by_id(carrier)
                pos = f'({r.x:.1f},{r.y:.1f})' if r else '?'
                tracker.add(ws.timestamp, 'POSSESSION',
                            f'R{carrier} has ball at {pos}')
            prev_carrier = carrier

        if s0 != prev_s0 and s0 is not None:
            old = prev_s0.value if prev_s0 else '-'
            tracker.add(ws.timestamp, 'R1_STATE', f'{old} -> {s0.value}')
        if s1 != prev_s1 and s1 is not None:
            old = prev_s1.value if prev_s1 else '-'
            tracker.add(ws.timestamp, 'R2_STATE', f'{old} -> {s1.value}')
        prev_s0, prev_s1 = s0, s1

        if not pass_done and s0 == DecisionState.PASS:
            pass_done = True
            tracker.add(ws.timestamp, 'PASS_START',
                        f'R1 kicks ball toward R2 | ball=({ws.ball.x:.1f},{ws.ball.y:.1f})')

        if not shoot_done and s1 == DecisionState.SHOOT:
            shoot_done = True
            tracker.add(ws.timestamp, 'SHOOT_START',
                        f'R2 shoots at goal! | ball=({ws.ball.x:.1f},{ws.ball.y:.1f})')

        if goal_scored is None:
            scorer = check_goal(ws.ball)
            if scorer:
                goal_scored = scorer
                tracker.add(ws.timestamp, 'GOAL!!!',
                            f'{scorer} SCORES! ball=({ws.ball.x:.2f},{ws.ball.y:.2f})')

        # Progress
        if tick > 0 and tick % (FPS * 5) == 0:
            c = f'R{carrier}' if carrier is not None else '-'
            print(f'  [{tick:4d}/{total_ticks}] t={ws.timestamp:5.1f}s '
                  f'| ball=({ws.ball.x:+5.1f},{ws.ball.y:+5.1f}) '
                  f'| carrier={c} '
                  f'| R1={s0.value if s0 else "-":<8} '
                  f'| R2={s1.value if s1 else "-":<8}')

        if goal_scored:
            break

    elapsed = time.time() - start
    print(f'\n=== Simulation complete ===')
    print(f'Steps: {tick+1}  |  Sim time: {ws.timestamp:.1f}s  |  Wall time: {elapsed:.1f}s')

    tracker.print_timeline()

    # Checklist
    summary = fsm.get_decision_summary()
    print(f'\nState dist: {summary.get("state_distribution", {})}')

    checks = [
        ('R1 passed', pass_done),
        ('R2 shot', shoot_done),
        ('Goal scored', goal_scored is not None),
    ]
    print(f'\n=== Checklist ===')
    all_ok = True
    for name, ok in checks:
        mark = 'PASS' if ok else 'FAIL'
        if not ok:
            all_ok = False
        print(f'  [{mark}] {name}')
    print(f'\n  Result: {"ALL PASSED" if all_ok else "NOT ALL PASSED"}')

    if args.export_csv:
        os.makedirs(args.log_dir, exist_ok=True)
        p = os.path.join(args.log_dir, 'pass_shoot_log.csv')
        fsm.export_csv(p)
        print(f'\n  CSV exported: {p}')

    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
