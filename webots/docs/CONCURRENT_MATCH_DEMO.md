# Concurrent Match Demo

The concurrent match demo is launched with:

```bash
./scripts/start_mujoco_concurrent_match.sh --view --seed 42
./scripts/start_mujoco_concurrent_match.sh --record --seed 42
./scripts/start_mujoco_concurrent_match.sh --no-render --seed 42
```

Use `--duration <seconds>` for short checks.

This mode is not a stage script. Four agents decide together at 20Hz, team coordination runs at 10Hz, and low-level controllers keep updating before each MuJoCo step. It is intended to show simultaneous support, pressing, blocking, passing, shooting, clearing, contested possession, and possession changes.

The original deterministic Visual V3 demo remains:

```bash
./scripts/start_mujoco_visual_soccer_demo_v3.sh --view
./scripts/start_mujoco_visual_soccer_demo_v3.sh --record
```

