# Demo Script — Booster T1 2v2 Soccer (3-4 minutes)

## Pre-flight (before recording)

1. WSL restart (`wsl --shutdown` in PowerShell)
2. `cd /home/plon/Workspace/booster_soccer_project`
3. `./scripts/stop_final_submission_demo.sh`
4. Clean FastDDS: `rm -f /dev/shm/fast_* /dev/shm/fastdds_*`
5. Clean LCM: `rm -f /home/plon/Workspace/booster_t1_webots/runner_extracted/record_*.lcm`

## REAL MODE: `./scripts/start_final_submission_demo.sh real`

### Scene 1: Field & Setup (30s)
- Green 2v2 field, 4 robots visible
- Labels: BLUE 1—BALL_HANDLER, BLUE 2—SUPPORT, RED 1—MARK, RED 2—BLOCK
- On-screen info: Mode REAL DRIBBLE, Active mck: 1
- Ball positioned in front of BLUE_1

### Scene 2: BLUE_1 Active Control (45s)
- Show mck ready message in terminal
- BLUE_1 enters Walking mode
- BLUE_1 approaches ball in 3 small steps
- Show RPC responses (code=0 for all)

### Scene 3: Dribble Push (30s)
- BLUE_1 uses Move API (vx=0.04-0.08 m/s)
- Short forward push contacts ball
- Ball moves by physics collision
- Show ball displacement in terminal output

### Scene 4: Results (15s)
- Show `results/final_submission/real_dribble_summary.json`
- Ball displacement value
- All RPC commands succeeded

## MOCK MODE: `./scripts/start_final_submission_demo.sh mock`
(Run separately, or show output from pre-run)

### Scene 5: Strategy Demo (45s)
- Show "MOCK 2v2 COOPERATION DEMO" clearly on screen
- 4 scenarios animated:
  - A: PASS available (no blocker)
  - B: DRIBBLE forced (RED_2 blocks line)
  - C: SHOOT opportunity (near goal)
  - D: HOLD (too many opponents)
- Each scenario shows REAL strategy decision

### Scene 6: Test Results (20s)
- `74 passed` unit tests
- Show test output
- Show generated log files

### Scene 7: Known Limitations (15s)
- Single active mck (binary limitation)
- 3 passive/mock robots
- Strategy engine is REAL
- Multi-mck requires DDS isolation

## Output Files After Recording

```
results/final_submission/
  real_dribble_summary.json
  real_dribble_actions.jsonl
  real_dribble_decisions.jsonl
  real_dribble_ball_motion.jsonl
  mock_2v2_summary.json
  mock_2v2_decisions.jsonl

outputs/screenshots/
  final_real_dribble.png
  final_mock_2v2.png
```
# Final Demo Script - 2026-07-12

1. Start mock demo:
   `./scripts/start_final_submission_demo.sh mock`
2. Show the Webots GUI label:
   `MOCK 2v2 COOPERATION DEMO`
3. Explain that actions use `MockRobotActionAdapter` and strategies use real `TeamStrategy`.
4. Show decisions PASS, DRIBBLE, SHOOT, BLOCK from `summary.json`.
5. For real mode, show `results/final_submission/20260712_152137_6c4b6c/real_summary.json`.
6. State clearly: the final real run did not complete physical dribble because mck crashed after recording reentrancy warnings.

Native physical control:

1. Run unassisted: `./scripts/start_native_physical_kick.sh kick`.
2. Run assisted only after unassisted failure: `./scripts/start_native_physical_kick.sh assisted-kick`.
3. Show `docs/NATIVE_PHYSICAL_CONTROL.md` and the native summaries.
4. State clearly: final native path is N3; no verified foot-ball displacement was achieved.
