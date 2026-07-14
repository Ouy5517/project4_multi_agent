# MuJoCo Visual V2 Demo

Visual V2 is the final presentation-oriented MuJoCo view for the accepted four-robot soccer demo.

It keeps the successful physics baseline intact:

- Baseline run: `results/mujoco_four_robot_demo/full_final_acceptance`
- Baseline model: `mujoco_soccer/models/t1_2v2_soccer.xml`
- Visual V2 model: `mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml`
- Soccer ball motion remains physical foot-ball contact only.
- No ball `qpos`, `qvel`, `mj_applyFT`, hidden pusher, or ball actuator is used.
- The demo still uses Assisted Planar Locomotion plus native joint actuator gait.

Run the final visual demo with:

```bash
./scripts/start_mujoco_visual_soccer_demo_v2.sh --normal
```

Options:

```bash
./scripts/start_mujoco_visual_soccer_demo_v2.sh --slow
./scripts/start_mujoco_visual_soccer_demo_v2.sh --normal
./scripts/start_mujoco_visual_soccer_demo_v2.sh --fast
./scripts/start_mujoco_visual_soccer_demo_v2.sh --no-record
```

The script checks Python, MuJoCo, the Visual V2 model, and `DISPLAY`. It starts the clean OpenCV viewer when available and falls back to offscreen clean recording or MuJoCo passive viewing as needed. It never starts Webots, `mck`, RPC, or ROS controllers.

Visual V2 output includes:

```text
results/mujoco_four_robot_demo/<run_id>/
├── demo_visual_v2.mp4
├── final_frame_visual_v2.png
├── opening_frame.png
├── dribble_frame.png
├── pass_frame.png
├── shoot_frame.png
├── clear_frame.png
├── counter_frame.png
├── contact_sheet_visual_v2.png
├── visual_acceptance.json
├── motion_quality.json
└── summary.json
```

The recorder targets 1280x720, 30fps, H.264 video. If the raw recording exceeds the 55 second display budget, the recorder compresses presentation playback time while preserving the same simulated physics result.

Final release note: the accepted concurrent match uses the Visual V3 model derived from this NAO-inspired primitive proxy family. It remains a non-official visual proxy and does not claim to be an official NAO mesh or full Booster T1 dynamics model.
