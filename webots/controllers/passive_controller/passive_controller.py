#!/usr/bin/python3
"""Passive controller — does nothing except step the robot, unblocking Webots simulation."""

import sys
try:
    from controller import Robot
except ImportError:
    print("FATAL: Webots controller module not found.", file=sys.stderr)
    sys.exit(1)

robot = Robot()
timestep = int(robot.getBasicTimeStep()) or 100
print(f"[passive_controller] Started, timestep={timestep}ms", flush=True)

while robot.step(timestep) != -1:
    pass  # Do nothing — just unblock Webots simulation
