# MuJoCo Four Robot 2v2 Demo

This branch adds a MuJoCo-only 2v2 soccer demonstration under `mujoco_soccer/`.

Key facts:

- Engine: MuJoCo.
- Route: M2, `T1_MUJOCO_VISUAL_PROXY`.
- Official T1 MJCF found locally, but not used for the four-robot run because the assisted planar base and four-instance namespace conversion were higher risk than a deterministic proxy.
- Locomotion: assisted planar locomotion through MuJoCo position actuators on `base_x`, `base_y`, and `base_yaw`.
- Joint gait: native MuJoCo actuator targets on visible leg and arm joints.
- Ball motion: dynamic freejoint soccer ball, moved by foot proxy collision only.
- Not used: Webots Supervisor, mck, RPC, ROS controllers.

Latest real run:

- `results/mujoco_four_robot_demo/full_no_render_final_candidate/summary.json`
- `results/mujoco_four_robot_demo/full_no_render_final_candidate/final_frame.png`

Final accepted run:

- `run_id`: `full_final_acceptance`
- `demo_success`: `true`
- BLUE_1 path: `1.793m`
- PASS displacement: `0.392m`
- Total contacts: `31`
- Simulation time: `61.53s`
- Screenshot: `results/mujoco_four_robot_demo/full_final_acceptance/final_frame.png`

The final run keeps the M2 proxy-model route and does not claim free-dynamics humanoid walking.
