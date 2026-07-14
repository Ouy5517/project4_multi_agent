# User Guide

Run tests:

```bash
cd /home/plon/Workspace/booster_soccer_project
source coop_env/bin/activate
pytest -q
```

Run pass demo:

```bash
python demo/pass_decision_demo.py
```

Run experiments:

```bash
python demo/run_pass_experiments.py
```

Run scenario driver:

```bash
python main.py --adapter mock
```

Check Webots Runner status:

```bash
/home/plon/Workspace/booster_t1_webots/scripts/check_status.sh
```
