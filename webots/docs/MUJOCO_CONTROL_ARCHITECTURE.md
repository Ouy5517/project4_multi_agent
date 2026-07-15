# MuJoCo Control Architecture

The demo uses assisted planar locomotion.

- `planar_base_controller.py` writes only `data.ctrl` targets for base position actuators.
- `visible_gait_controller.py` writes native joint actuator targets for visible arms and legs.
- `foot_push_controller.py` computes deterministic behind-ball poses and push targets.
- `contact_detector.py` reads MuJoCo contacts and `mj_contactForce`.
- `ball_guard.py` scans the MuJoCo package for direct ball state writes.
- `strategy_bridge.py` reuses the existing `TeamStrategy`.
- `defensive_strategy.py` returns defensive `BLOCK` or `CLEAR` style actions.

Runtime code does not directly write soccer ball qpos or qvel.

