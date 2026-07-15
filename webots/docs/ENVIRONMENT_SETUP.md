# Environment Setup

- OS: Ubuntu 22.04 on WSL2.
- Workspace: `/home/plon/Workspace`.
- Webots: `/home/plon/Workspace/webots_updated/webots`, R2023b.
- T1 Webots project: `/home/plon/Workspace/booster_t1_webots`.
- SDK: `/home/plon/Workspace/booster_robotics_sdk`.
- Strategy project: `/home/plon/Workspace/booster_soccer_project`.

Python strategy environment:

```bash
cd /home/plon/Workspace/booster_soccer_project
source coop_env/bin/activate
pip install -r requirements.txt
pytest -q
```

Current blocker for Webots Runner:

- `/opt/booster/configs/robot_config.yaml` is missing.
- `/opt/booster/configs/system_settings_config.yaml` is missing.
- Local official packages did not contain `copy_robot_config.sh` or the two YAML files.
