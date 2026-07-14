# Known Limitations

## 1. Single Active mck

Only ONE mck instance can run at a time. Multiple mck processes segfault.
Root cause: mck binary (closed-source) not designed for co-located multi-instance.
Fallback: 1 active robot + 3 passive/mock robots.

## 2. GetStatus(2018) Returns 502

Cannot query body_control or current_actions via RPC.
Physical state must be inferred from GetMode(2017) + mck logs.

## 3. Real Dribble

Requires WSL restart to clean DDS/LCM state.
After restart: `./scripts/start_final_submission_demo.sh real`
Ball moved by physics collision via Move API (not FancyKick/VisualKick).

## 4. Four-Robot Control

Mock mode uses submission_demo_supervisor for position animation.
Strategy decisions use REAL TeamStrategy.decide() — not hardcoded.
Pass line detection uses mathematical line-distance calculation.
# Final Known Limitations - 2026-07-12

- Four real mck 2v2 control is not implemented and was not retried.
- The final real single-mck run segfaulted during initialization after repeated recording reentrancy warnings.
- `record_manager.record_backends = {}` is not the only backend control path; `lcm_matlab_backend.cpp` and `socket_backend.cpp` still appeared in the final mck log.
- Real physical dribble was not completed in the final run.
- Mock 2v2 uses `MockRobotActionAdapter` for movement and must not be described as four real mck robots.
- Native Webots Motor control has device probing and bounded interpolation, but the final native runs did not produce a verified foot-ball collision or ball displacement greater than 0.05 m.
- Assisted native mode is implemented and visibly labeled, but the single assisted run timed out before executing the kick phase.
