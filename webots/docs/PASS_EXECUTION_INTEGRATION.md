# Booster T1 Pass Execution Integration

## Architecture

```
PassDecision  ──►  PassExecutionAdapter  ──►  ExecutionPlan  ──►  ROS2 RPC
                                                                    │
                                                                    ▼
                                                             rpc_service_node
                                                                    │
                                                                    ▼
                                                                  mck
                                                                    │
                                                                    ▼
                                                               Webots T1
```

## Components

### 1. ROS2 Control Client (`tools/t1_ros2_control_client.py`)

Direct RPC client for `booster_rpc_service`. Supports:
- `status` — Get robot status (GetStatus, api_id=2018)
- `mode` — Get current mode (GetMode, api_id=2017)
- `prepare` — Change to Prepare mode (mode=1)
- `stand` — Alias for prepare
- `move` — Move forward at 0.05 m/s for 0.5s then stop
- `stop` — Zero velocity
- `turn` — Turn at 0.1 rad/s for 0.5s then stop
- `safe-demo` — Full sequence: Prepare → Walking → Forward(0.5s) → Stop(3s) → Turn(0.5s) → Stop

### 2. Pass Execution Adapter (`integration/__init__.py`)

Converts strategy decisions to robot execution plans.

#### Modes

| Mode | kick_enabled | RPC Client | Use Case |
|------|-------------|------------|----------|
| dry_run | False | DryRunRpcClient | Testing, demos |
| simulation | True | Real ROS2 client | Webots simulation |

#### Execution Phases

1. **ROTATE_TO_TARGET** — Turn toward receiver
2. **APPROACH_BALL** — Move toward ball if far (optional)
3. **ALIGN_FOR_PASS** — Fine orientation adjustment
4. **EXECUTE_KICK** — Kick if enabled, else NOT_IMPLEMENTED
5. **STOP** — Zero velocity
6. **VERIFY** — Check final mode

#### Safety Limits

| Parameter | Limit |
|-----------|-------|
| Max linear speed | 0.2 m/s |
| Max angular speed | 0.5 rad/s |
| Max turn duration | 2.0 s |
| Max move duration | 2.0 s |

### 3. Kick APIs

| API | api_id | Request Body | Mode |
|-----|--------|-------------|------|
| kShoot | 2024 | `""` | Walking (2) or Soccer (4) |
| kVisualKick | 2038 | `{"start": true, "version": 0}` | Walking (2) or Soccer (4) |
| kVisualKickV2 | 2038 | `{"start": true, "version": 1}` | Walking (2) or Soccer (4) |

### 4. ROS2 Service Interface

| Field | Value |
|-------|-------|
| Node | `rpc_service_node` |
| Service | `booster_rpc_service` |
| Type | `booster_interface/srv/RpcService` |
| Request | `int64 api_id`, `string body` (JSON) |
| Response | `int64 status`, `string body` (JSON) |

### 5. Robot Modes

| Mode | Value | Description |
|------|-------|-------------|
| kDamping | 0 | Motors in damping |
| kPrepare | 1 | Standing on both feet |
| kWalking | 2 | Can move, rotate, kick |
| kCustom | 3 | Custom actions |
| kSoccer | 4 | Soccer mode |

## Usage

### One-Click Startup
```bash
cd /home/plon/Workspace/booster_t1_webots
./scripts/start_all.sh
```

### Status Check
```bash
cd /home/plon/Workspace/booster_t1_webots
./scripts/check_status.sh
```

### Motion Control
```bash
cd /home/plon/Workspace/booster_soccer_project
./scripts/run_t1_control.sh mode
./scripts/run_t1_control.sh prepare
./scripts/run_t1_control.sh safe-demo
```

### Pass Execution Demo
```bash
cd /home/plon/Workspace/booster_soccer_project
source coop_env/bin/activate
python3 demos/pass_execution_demo.py
```

### Run All Tests
```bash
cd /home/plon/Workspace/booster_soccer_project
source coop_env/bin/activate
python3 -m pytest tests/ -v
```

## Environment

| Variable | Value |
|----------|-------|
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `runner_extracted/fastdds_profile.xml` |
| `ROS_LOCALHOST_ONLY` | `0` |
| `WEBOTS_HOME` | `/home/plon/Workspace/webots_updated` |
| Python | `/usr/bin/python3.10` (must NOT be miniconda python3.13) |

## Known Limitations

1. **Kick execution not tested**: Ball model may be absent from `T1_release.wbt`.
2. **Visible motion**: Requires human confirmation in Webots GUI.
3. **WSL DDS**: `ros2 node list` hangs but rclpy direct clients work.
4. **C++ SDK**: `b1_loco_example_client` is for real robot DDS, incompatible with simulation ROS2.
5. **mck cold start**: ~4-5 minutes due to socket_backend timeout.

## Files

| Path | Purpose |
|------|---------|
| `tools/t1_ros2_control_client.py` | ROS2 control client |
| `scripts/run_t1_control.sh` | Control client wrapper |
| `integration/__init__.py` | Pass execution adapter |
| `demos/pass_execution_demo.py` | End-to-end demo |
| `tests/test_pass_execution.py` | 35 integration tests |
| `results/ros2_control_test.jsonl` | RPC test results |
| `results/pass_execution_demo.json` | Demo output |
