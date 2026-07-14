# Booster T1 Webots Simulation — Test Results

## 2026-07-11 Integration Test

### Environment
- **Webots**: R2023b (`/home/plon/Workspace/webots_updated`)
- **World**: `T1_release.wbt` (robot: T1_release, controller: `<extern>`, timestep: 1ms)
- **Configs**: `/opt/booster/configs/` (T1 2.3.4 official)
- **Runner**: `/home/plon/Workspace/booster_t1_webots/runner_extracted`
- **Launch**: `webots-controller --protocol=tcp --ip-address=127.0.0.1 --port=1234 --robot-name=T1_release ./mck configs/config.lua`

### Process Status

| Component | PID | State | Notes |
|-----------|-----|-------|-------|
| webots-bin | 59639 | Running | T1_release.wbt loaded |
| mck | 59711 | Running | WBC tasks generating, joystick polling |
| rpc_service_node | 59698 | Running | ROS2 RPC service |

### Connection

| Check | Result |
|-------|--------|
| Port 1234 LISTEN | ✅ |
| Port 1234 ESTAB | ✅ bidirectional (mck:45390 ↔ webots-bin:1234) |
| WEBOTS_CONTROLLER_URL | `tcp://127.0.0.1:1234/T1_release` |

### Config Loading

| File | Size | Status |
|------|------|--------|
| robot_config.yaml | 11139 bytes | ✅ Parsed successfully (`RobotConfigParser::Parse()`) |
| system_settings_config.yaml | 156 bytes | ✅ Loaded (no missing error) |
| security_config.yaml | 1029 bytes | ✅ Present |
| motor_calib.yaml | 227K | ✅ Present |

### Critical Errors

| Error | Status |
|-------|--------|
| `Load robot config file failed` | **GONE** |
| Segmentation fault | **GONE** |
| `socket_backend.cpp:24` Failed to initialize socket | ⚠️ Non-critical (LCM record server not needed for sim) |
| `hand_config not found` | ⚠️ Non-critical (simulation has no hands) |
| `Joystick type not found` | ⚠️ Non-critical (no physical joystick) |
| `robot_cycle_config not found` | ⚠️ Non-critical (using default 2000) |

### Webots GUI

| Check | Result |
|-------|--------|
| "Waiting for connection" message | **GONE** |
| Simulation time advancing | ✅ (user confirmed) |

### One-Click Scripts

| Script | Status |
|--------|--------|
| start_all.sh | ✅ All 3 components start, ESTAB after ~275s |
| stop_all.sh | ✅ Clean shutdown, always exit 0 |
| check_status.sh | ✅ Accurate process/connection/error reporting |

### SDK Compatibility

| SDK Component | Compatible | Reason |
|---------------|------------|--------|
| b1_loco_example_client (C++) | ❌ | Uses raw FastDDS ChannelFactory; needs real robot network |
| b1_low_sdk_example (C++) | ❌ | Low-level hardware interface |
| ROS2 RpcService | ✅ | Internal communication works; CLI discovery blocked by WSL |
| ROS2 topics (LowState, MotorState, etc.) | ✅ | Published internally |

### Pass Strategy

| Test | Result |
|------|--------|
| pytest (18 tests) | ✅ 18 passed in 0.05s |
| pass_decision_demo.py | ✅ Full decision pipeline |
| run_pass_experiments.py | ✅ CSV + JSONL + summary generated |

### Known Limitations

1. **WSL DDS multicast** prevents `ros2 node list` from discovering nodes.
   Internal ROS2 communication works correctly via the FastDDS loopback profile.
2. **C++ SDK** is designed for real robot hardware (raw DDS network interface).
   Not compatible with simulation without significant rework.
3. **mck cold start** takes ~275s due to socket_backend initialization timeout (~140s)
   and module loading (~10s). This is normal for simulation.

### Test Logs

- `booster_t1_webots/logs/mck_webots_20260711_203452.log` — mck via webots-controller
- `booster_t1_webots/logs/startup_20260711_204738.log` — start_all.sh output
- `booster_soccer_project/results/pass_experiment_results.csv`
- `booster_soccer_project/results/pass_experiment_log.jsonl`

---

## 2026-07-11 ROS2 Control Client Test

### RPC Discovery

| Check | Result |
|-------|--------|
| DDS UDP sockets active | ✅ 127.0.0.1:7400,7410,7411 |
| ros2 node list | ❌ Daemon discovery issue (DDS transport works) |
| rclpy direct client | ✅ Communicates via same loopback transport |

### Service Interface

| Field | Value |
|-------|-------|
| Node | `rpc_service_node` |
| Service | `booster_rpc_service` |
| Type | `booster_interface/srv/RpcService` |

### Command Tests

| # | Command | api_id | Status | Result |
|---|---------|--------|--------|--------|
| 1 | GetStatus | 2018 | 502 | STATE_TRANSITION_FAILED (sim limitation) |
| 2 | GetMode | 2017 | 0 | `{"mode":1}` — Prepare ✅ |
| 3 | ChangeMode→Prepare | 2000 | 0 | SUCCESS ✅ |
| 4 | ChangeMode→Walking | 2000 | 0 | SUCCESS ✅ |
| 5 | Move(vx=0.05) | 2001 | 0 | SUCCESS ✅ |
| 6 | Stop | 2001 | 0 | SUCCESS ✅ |
| 7 | Move(vyaw=0.1) | 2001 | 0 | SUCCESS ✅ |
| 8 | Stop | 2001 | 0 | SUCCESS ✅ |

### Safe-Demo: All 6 steps passed

```
Prepare → Walking → Forward(0.5s) → Stop → Turn(0.5s) → Stop
```

### Post-Test Stability

| Check | Status |
|-------|--------|
| webots-bin | ✅ Alive |
| mck | ✅ Alive |
| rpc_service_node | ✅ Alive |
| 1234 ESTAB | ✅ |
| Segfault | 0 |
| Config errors | 0 |

### Key Findings

1. `ros2 node list` failure is a ROS2 daemon discovery issue, NOT a DDS problem.
   rclpy clients using the same DDS profile communicate correctly.
2. Must use Python 3.10 (system), not Python 3.13 (miniconda) for ROS2 Humble.
3. GetStatus returns 502 in simulation — use GetMode instead.
4. All locomotion commands (ChangeMode, Move, Stop) work end-to-end.
