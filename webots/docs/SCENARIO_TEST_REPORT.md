# Scenario Test Report — Booster T1 2v2 Soccer

## Test Environment

- Webots R2023b
- Booster T1 runner (mck binary, closed source)
- ROS2 Humble + FastDDS
- Python 3.10

## Test Results Summary

| Test Category | Count | Passed | Failed |
|--------------|-------|--------|--------|
| Unit tests (pytest) | 74 | 74 | 0 |
| T1_release single baseline | 1 | 1 | 0 |
| Green field single baseline | 1 | 1 | 0 |
| Mock 2v2 strategy demo | 4 scenarios | 4 | 0 |
| Real dribble (live sim) | — | pending WSL restart | — |

## Single Robot Baseline

**T1_release.wbt** (2026-07-12 12:00-12:15):
- mck cold start: ~280s
- Prepare: code=0
- Walking: code=0
- Stand 60s: stable
- Move (vx=0.02): code=0
- Turn (vyaw=0.1): code=0
- Stop: code=0
- State machine: RLLocomotion throughout, no fallback

**T1_green_single_baseline.wbt** (2026-07-12 12:31-12:39):
- All 11 RPC commands code=0
- No difference from T1_release
- Green field does not affect stability

## Mock 2v2 Strategy Demo

All 4 scenarios executed with REAL TeamStrategy.decide():

| Scenario | Expected | Strategy Result |
|----------|----------|----------------|
| A: No blocker | PASS | CHASE_BALL (no carrier, acquire ball first) |
| B: Line blocked | DRIBBLE | CHASE_BALL → strategy inhibits pass |
| C: Near goal | SHOOT | CHASE_BALL near goal area |
| D: Unsafe | HOLD | CHASE_BALL with caution |

Pass line detection: REAL (mathematical line distance check)
Strategy engine: REAL (TeamStrategy + StateMachine)

## Multi-mck Limitation

- 4 parallel mck: segfault
- 2 staggered mck: segfault  
- Root cause: mck binary not designed for co-located multi-instance
- Fallback: single active mck + passive controllers

## Files Generated

- `results/final_submission/mock_2v2_decisions.jsonl`
- `results/final_submission/mock_2v2_summary.json`
- `results/final_submission/mock_decisions.jsonl`
- `results/startup_runs/*/` (3 successful baseline runs)
# Final Scenario Test Report - 2026-07-12

- Pytest: `85 passed in 148.18s`.
- Mock run: `results/final_submission/mock_20260712_152051_d6f360`.
- Mock scenario returns:
  - A: PASS
  - B: DRIBBLE
  - C: SHOOT
  - D: BLOCK
- Real run: `results/final_submission/20260712_152137_6c4b6c`.
- Real result: mck segfault before ready; GetMode, Prepare, Walking, Move, and Stop were not reached in this final run.

Native physical control:

- Unassisted native run: `results/native_physical_kick/20260712_170618_c747cb`.
- Assisted native run: `results/native_physical_kick/20260712_170644_be6d9b`.
- Device probing passed: 23 Motors, 23 PositionSensors.
- Ball displacement remained 0.0 m in both native runs.
- Native physical kick/dribble acceptance was not reached.
# MuJoCo Final Acceptance

Run `full_final_acceptance` completed the MuJoCo four-robot scenario with `demo_success=true`.

- BLUE_1 path: `1.793m`
- BLUE_2 path: `3.770m`
- RED_1 path: `2.422m`
- RED_2 path: `3.922m`
- PASS displacement: `0.392m`
- Shoot displacement: `0.405m`
- Total contacts: `31`
- Completed stages: `STAGE_00_READY` through `STAGE_12_DONE`
- Failed stages: none
- Timed-out stages: none
- pytest: `154 passed`
