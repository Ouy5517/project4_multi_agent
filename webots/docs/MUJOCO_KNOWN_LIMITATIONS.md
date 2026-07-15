# MuJoCo Known Limitations

The final run `full_final_acceptance` reached `demo_success=true`.

Previous misses fixed:

- `STAGE_06_BLUE1_PASS` increased from `0.236m` to `0.392m`.
- `BLUE_1` effective path reduced from `4.629m` to `1.793m`.
- Rendering full video in this headless EGL environment is slow, so the saved artifact is a PNG screenshot, not MP4.

Successful observed pieces:

- MuJoCo model loads.
- Four robots move with visible native joint gait.
- Ball has a freejoint and no actuator.
- BallGuard source scan passes.
- Foot-ball contacts are confirmed from MuJoCo contact pairs and contact force.
- DRIBBLE, PASS, SHOOT, and CLEAR/BLOCK defensive strategy paths all return from real strategy code.
