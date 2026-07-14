# Metric Definitions

- `pass_behavior_ticks`: decision ticks where PASS was selected.
- `pass_action_starts`: transition count into PASS.
- `pass_successes`: PASS action with physical contact evidence.
- `shoot_behavior_ticks`: decision ticks where SHOOT was selected.
- `shoot_action_starts`: transition count into SHOOT.
- `shoot_successes`: SHOOT action with physical contact evidence.
- `intercept_behavior_ticks`: decision ticks where PRESS/INTERCEPT was selected.
- `intercept_action_starts`: transition count into INTERCEPT.
- `intercept_successes`: INTERCEPT/PRESS action with physical contact evidence.
- `clear_behavior_ticks`: decision ticks where CLEAR was selected.
- `clear_action_starts`: transition count into CLEAR.
- `clear_successes`: CLEAR action with physical contact evidence.
- `contact_samples`: raw MuJoCo contact samples from the detector.
- `unique_contact_events`: contact samples grouped by robot and time window.
- `actual_present_hz`: actual viewer presents after `viewer.sync`.
- `render_state_change_hz`: presented states whose robot/ball/joint vector changed.
- `effective_motion_frame_ratio`: changing visual frames divided by comparable presented frames.

Behavior ticks are not described as successful actions.
