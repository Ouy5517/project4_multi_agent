# MuJoCo Demo Script

Commands:

```bash
python -m mujoco_soccer.run_demo --mode model-check
python -m mujoco_soccer.run_demo --mode gait-check
python -m mujoco_soccer.run_demo --mode contact-check
python -m mujoco_soccer.run_demo --mode blue-dribble
python -m mujoco_soccer.run_demo --mode pass-receive
python -m mujoco_soccer.run_demo --mode defense-check
python -m mujoco_soccer.run_demo --mode pass-only --no-render --run-id pass_final_check
python -m mujoco_soccer.run_demo --mode full-demo
```

Final acceptance command:

```bash
python -m mujoco_soccer.run_demo --mode full-demo --no-render --run-id full_final_acceptance
```

Convenience scripts:

```bash
./scripts/start_mujoco_four_robot_demo.sh
./scripts/check_mujoco_four_robot_demo.sh
./scripts/package_mujoco_submission.sh
```

Visual V2 presentation script:

```bash
./scripts/start_mujoco_visual_soccer_demo_v2.sh --normal
```

Other playback modes:

```bash
./scripts/start_mujoco_visual_soccer_demo_v2.sh --slow
./scripts/start_mujoco_visual_soccer_demo_v2.sh --fast
./scripts/start_mujoco_visual_soccer_demo_v2.sh --no-record
```

Visual V2 uses `mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml`, a non-official NAO-inspired primitive proxy. It preserves Assisted Planar Locomotion, native joint actuator gait, and physical foot-ball collision. It does not start Webots, `mck`, or RPC.

Concurrent four-agent match:

```bash
./scripts/start_mujoco_concurrent_match.sh --view --seed 42
./scripts/start_mujoco_concurrent_match.sh --record --seed 42
./scripts/start_mujoco_concurrent_match.sh --no-render --seed 42
```

Equivalent module command:

```bash
python -m mujoco_soccer.run_demo --mode concurrent-match --no-render --duration 60 --seed 42
```

The concurrent mode uses the accepted Visual V3 MJCF as its visual/physics base, but it is a separate execution path. It records four-agent decision bundles, shared world snapshots, team roles, possession, commands, contacts, events, ball motion, final frame, summary, and acceptance metrics under `results/mujoco_concurrent_match/<run_id>/`.

Smooth concurrent frontend:

```bash
./scripts/start_mujoco_concurrent_match_smooth.sh --view --seed 42
./scripts/start_mujoco_concurrent_match_smooth.sh --benchmark --duration 20 --seed 42
./scripts/start_mujoco_concurrent_match_smooth.sh --record --seed 42
./scripts/start_mujoco_concurrent_match_smooth.sh --replay
```

VIEW mode uses the MuJoCo passive viewer only and does not start the video writer. RECORD mode does not open the realtime viewer; it runs physical simulation logging and generates a 60 FPS trajectory replay. Smooth mode adds async logging, frame pacing metrics, path-coupled gait, command smoothing, and `frontend_smoothness_acceptance.json`.

Final unified launcher:

```bash
./scripts/start_final_soccer_demo.sh --match
./scripts/start_final_soccer_demo.sh --showcase
./scripts/start_final_soccer_demo.sh --record
./scripts/start_final_soccer_demo.sh --replay
./scripts/start_final_soccer_demo.sh --benchmark --duration 15 --seed 42
./scripts/start_final_soccer_demo.sh --acceptance
```

`--record` generates a 60 FPS trajectory replay from physical simulation logs and writes `video_provenance.json`. It is not a direct realtime screen recording.
