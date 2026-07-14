# Video Provenance

The release distinguishes realtime viewing from generated video.

Realtime viewer:

- MuJoCo native passive viewer.
- Approximately 60 Hz actual present rate.
- No `VideoWriter`, offscreen renderer, MP4 encoder, or contact sheet generation in VIEW mode.

60 FPS video:

- Generated from `robot_states.jsonl` and `ball_motion.jsonl`.
- Uses the completed physical simulation trajectory.
- Interpolates trajectory samples to 60 FPS.
- Does not alter match events, possession, contacts, or ball physics.
- Is not a direct realtime screen recording.

Every generated video writes `video_provenance.json`.
