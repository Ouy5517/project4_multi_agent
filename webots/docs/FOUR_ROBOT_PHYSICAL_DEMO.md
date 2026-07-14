# Assisted 2v2 Physical Soccer Demo

Final world:

- `worlds/T1_2v2_assisted_physical_soccer.wbt`
- Runtime copy: `/home/plon/Workspace/booster_t1_webots/simulation/webots_simulation/worlds/T1_2v2_assisted_physical_soccer.wbt`

Mode:

- `ASSISTED_2V2_PHYSICAL`
- Robot root locomotion is Supervisor-assisted.
- Leg gait and arm swing are native Webots Motor commands.
- Ball motion is from Webots physics collision only.
- Supervisor ball manipulation is off.
- `mck` and `rpc_service_node` are not used.

One-key scripts:

- Start: `./scripts/start_four_robot_physical_demo.sh`
- Stop: `./scripts/stop_four_robot_physical_demo.sh`
- Check: `./scripts/check_four_robot_physical_demo.sh`

Current verified result:

- Four full T1 robots load with unique foot DEFs and visible gait commands.
- BLUE_1 achieved one confirmed physical foot contact in the latest formal run.
- The latest formal run stalled inside Webots during `RED_2_LEAVE_PASS_LINE`, so full 2v2 acceptance was not reached.

Do not describe this as four autonomous balanced robots or official mck locomotion. It is an assisted physical demo.
