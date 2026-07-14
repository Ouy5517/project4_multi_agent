# MuJoCo Concurrent Multi-Agent Soccer

`concurrent-match` is an independent MuJoCo 2v2 mode where all four robot agents observe the same immutable world snapshot, decide on the same decision tick, and then apply their commands before a single `mj_step`.

It does not replace the deterministic Visual V3 acceptance demo. The deterministic demo remains available for guaranteed DRIBBLE/PASS/SHOOT/CLEAR/COUNTER presentation. The concurrent match is for live multi-agent cooperation, contesting, role switching, and possession changes.

Run:

```bash
python -m mujoco_soccer.run_demo --mode concurrent-match --no-render --duration 60 --seed 42
./scripts/start_mujoco_concurrent_match.sh --view --seed 42
./scripts/start_mujoco_concurrent_match.sh --record --seed 42
```

The mode keeps Assisted Planar Locomotion, native actuator gait, the Visual V3 MJCF, and physical foot-ball collision. It does not write ball `qpos`, ball `qvel`, apply hidden forces, or add ball actuators.

Logs are written to:

```text
results/mujoco_concurrent_match/<run_id>/
```

Key files include `shared_world_state.jsonl`, `agent_decisions.jsonl`, `team_roles.jsonl`, `possession.jsonl`, `robot_commands.jsonl`, `contacts.jsonl`, `concurrency_acceptance.json`, and `summary.json`.

Final release entry:

```bash
./scripts/start_final_soccer_demo.sh --match
```

Final metrics distinguish behavior ticks, action starts, successful physical-contact actions, contact samples, and unique contact events. The 60 FPS video path is a trajectory replay generated from physical simulation logs and is documented by `video_provenance.json`.
