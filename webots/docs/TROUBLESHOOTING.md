# Troubleshooting

Runner issue:

- `mck` executes missing `../src/models/main_app/copy_robot_config.sh`.
- `mck` then tries `/opt/booster/configs/system_settings_config.yaml`.
- It also needs `/opt/booster/configs/robot_config.yaml`.
- These files were not found in the five local official packages.

SDK issue:

- SDK builds successfully.
- Runtime high-level client aborts when Runner/RPC is absent.
- Use `booster_t1_webots/scripts/test_locomotion.sh`; it refuses to send motion commands unless `mck` and `rpc_service_node` are running.

pytest issue:

- ROS pytest plugins are compatible with pytest 8.2.2 here.
- `requirements.txt` pins pytest to 8.2.2.
