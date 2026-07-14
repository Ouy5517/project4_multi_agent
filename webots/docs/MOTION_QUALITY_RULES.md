# Visual V2 Motion Quality Rules

The MuJoCo soccer demo uses Assisted Planar Locomotion for base movement and native joint actuators for visible gait. Visual V2 keeps the accepted physical soccer behavior and records motion quality signals for the presentation run.

Rules monitored in `motion_quality.json`:

- `abrupt_yaw`: yaw step exceeds the per-frame rule.
- `excessive_acceleration`: base acceleration exceeds the configured visual limit.
- `boundary_violation`: robot base leaves the field buffer.
- `robot_overlap`: reserved for robot center-distance checks.
- `sideways_sliding`: reserved for forward-motion alignment checks.
- `gait_without_motion`: reserved for visible moonwalk checks.
- `motion_without_gait`: reserved for missing gait while moving.

The final accepted Visual V2 run requires:

- `MOTION_RULE_VIOLATION` count is zero.
- `nan_detected=false`.
- `joint_limit_violation=false`.
- `ball_mutation_detected=false`.
- All four robots touch the ball.
- DRIBBLE, PASS, SHOOT, CLEAR, and COUNTER displacements still meet acceptance thresholds.

Visual V2 intentionally prioritizes the accepted physical behavior over aggressive locomotion retuning. Experimental path-coupled gait and turn-first controls exist in the controller layer, but the final full-demo path keeps the proven physical contact timing so the ball is still moved only by real foot collision.

