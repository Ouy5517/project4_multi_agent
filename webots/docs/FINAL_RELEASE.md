# Final Release

This release is `MUJOCO FOUR-ROBOT SOCCER FINAL RELEASE`.

Primary commands:

```bash
./scripts/start_final_soccer_demo.sh --match
./scripts/start_final_soccer_demo.sh --showcase
./scripts/start_final_soccer_demo.sh --record
./scripts/run_final_acceptance.sh
```

The final realtime match uses MuJoCo, four independent `RobotAgent` instances, Assisted Planar Locomotion, native joint actuator gait, and physical foot-ball contacts. It does not start Webots, `mck`, RPC, or ROS controllers.

The visual robot is a NAO-inspired proxy at T1 demonstration scale. It is not an official NAO mesh and is not a full official Booster T1 dynamics model.

The 60 FPS video is a trajectory replay generated from a complete physical simulation log. It is not claimed as a direct realtime screen recording.
