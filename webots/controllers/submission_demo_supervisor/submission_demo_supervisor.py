#!/usr/bin/python3
"""Submission Demo Supervisor — labels robots, tracks state, mock control."""
import json, math, sys, os, time
from datetime import datetime

try:
    from controller import Supervisor
except ImportError:
    print("ERROR: Webots controller module not found.", file=sys.stderr)
    sys.exit(1)

ROBOT_DEFS = ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"]
ROLES = {
    "T1_BLUE_1": "BALL_HANDLER",
    "T1_BLUE_2": "SUPPORT",
    "T1_RED_1": "MARK",
    "T1_RED_2": "BLOCK",
}

def main():
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep()) or 100

    # Get nodes
    robots = {}
    for name in ROBOT_DEFS:
        node = sup.getFromDef(name)
        if node:
            robots[name] = node

    ball = sup.getFromDef("SOCCER_BALL")

    mode = os.environ.get("DEMO_MODE", "REAL")
    mode_label = "REAL DRIBBLE" if mode == "real" else "MOCK 2v2 COOPERATION DEMO"

    seq = 0
    while sup.step(ts) != -1:
        seq += 1
        if seq % 50 != 0:
            continue

        st = sup.getTime()

        # Update labels on robots. Labels belong to the Supervisor API, not Node.
        for name, node in robots.items():
            role = ROLES.get(name, "UNKNOWN")
            pos = node.getPosition()
            label = f"{name} — {role}\nz={pos[2]:.2f}"
            sup.setLabel(10 + ROBOT_DEFS.index(name), label, 0.01, 0.15 + 0.05 * ROBOT_DEFS.index(name), 0.08, 0xFFFFFF, 0.0, "Arial")

        # Top-level display
        if ball:
            bp = ball.getPosition()
            ball_owner = "NONE"
            for name, node in robots.items():
                rp = node.getPosition()
                d = math.hypot(rp[0] - bp[0], rp[1] - bp[1])
                if d < 0.3:
                    ball_owner = name
                    break

            # Check pass line BLUE_1 -> BLUE_2
            b1 = robots.get("T1_BLUE_1")
            b2 = robots.get("T1_BLUE_2")
            r2 = robots.get("T1_RED_2")
            pass_clear = "CLEAR"
            if b1 and b2 and r2:
                p1 = b1.getPosition()
                p2 = b2.getPosition()
                pr = r2.getPosition()
                # Simple line distance check
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dl = math.hypot(dx, dy)
                if dl > 0.01:
                    dist = abs(dy * pr[0] - dx * pr[1] + p2[0]*p1[1] - p2[1]*p1[0]) / dl
                    if dist < 0.5:
                        pass_clear = "BLOCKED"

            info = (
                f"{mode_label}\n"
                f"Actions use {'Move API' if mode == 'real' else 'MockRobotActionAdapter'}\n"
                f"Strategies use real TeamStrategy\n"
                f"Sim Time: {st:.1f}s\n"
                f"Ball Owner: {ball_owner}\n"
                f"Pass Line: {pass_clear}\n"
                f"Active mck: {1 if mode == 'real' else 0}\n"
                f"Strategy: {'DRIBBLE' if pass_clear == 'BLOCKED' else 'PASS'}"
            )
            sup.setLabel(1, info, 0.01, 0.01, 0.1, 0x00FF00, 0.07, "Lucida Console")

    print("[submission_demo_supervisor] Done")


if __name__ == "__main__":
    main()
