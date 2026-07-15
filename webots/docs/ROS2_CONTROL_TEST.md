# Booster T1 ROS2 Control Client — Test Results

## 2026-07-11 21:10 UTC

### Environment
- **Python**: 3.10.12 (system, ROS2 Humble compatible)
- **ROS2 Distro**: Humble
- **DDS**: FastDDS with `fastdds_profile.xml` (loopback transport)
- **RPC Process**: PID 59700, `rpc_service_node`
- **ROS_LOCALHOST_ONLY**: 0

### ROS2 CLI Discovery

| Test | Result | Detail |
|------|--------|--------|
| `ros2 node list` | ❌ Timeout | ROS2 daemon can't join existing DDS participants |
| DDS UDP sockets | ✅ Active | 127.0.0.1:7400, 7410, 7411, 45034 |
| `rclpy` direct client | ✅ Works | Creates own DDS participant on same loopback transport |

**Root cause**: `ros2 node list` uses the ROS2 daemon which starts a separate DDS
participant. The daemon's participant can't discover existing participants because
the `fastdds_profile.xml` whitelists specific transports. Python `rclpy` nodes
using the same profile CAN communicate because they share the loopback transport.

### RPC Service Interface

| Field | Value |
|-------|-------|
| Node name | `rpc_service_node` |
| Service name | `booster_rpc_service` |
| Service type | `booster_interface/srv/RpcService` |
| Request | `BoosterApiReqMsg {int64 api_id, string body}` |
| Response | `BoosterApiRespMsg {int64 status, string body}` |

### API Commands Tested

| # | Command | api_id | Request | Status | Response |
|---|---------|--------|---------|--------|----------|
| 1 | GetStatus | 2018 | `""` | 502 | `{}` — STATE_TRANSITION_FAILED (sim limitation) |
| 2 | GetMode | 2017 | `""` | 0 | `{"mode": 1}` — Prepare mode |
| 3 | ChangeMode→Prepare | 2000 | `{"mode": 1}` | 0 | `{}` |
| 4 | ChangeMode→Prepare | 2000 | `{"mode": 1}` | 0 | `{}` |
| 5 | ChangeMode→Walking | 2000 | `{"mode": 2}` | 0 | `{}` |
| 6 | Move forward | 2001 | `{"vx":0.05,"vy":0.0,"vyaw":0.0}` | 0 | `{}` |
| 7 | Stop | 2001 | `{"vx":0.0,"vy":0.0,"vyaw":0.0}` | 0 | `{}` |
| 8 | Turn right | 2001 | `{"vx":0.0,"vy":0.0,"vyaw":0.1}` | 0 | `{}` |
| 9 | Stop | 2001 | `{"vx":0.0,"vy":0.0,"vyaw":0.0}` | 0 | `{}` |

### Safe-Demo Sequence (all SUCCESS)

```
[21:10:48] ChangeMode→Prepare     code=0 SUCCESS
[21:10:51] ChangeMode→Walking     code=0 SUCCESS
[21:10:52] Move(vx=0.05)          code=0 SUCCESS  (forward 0.05 m/s)
[21:10:52] Stop                   code=0 SUCCESS  (after 0.5s)
[21:10:55] Move(vyaw=0.1)         code=0 SUCCESS  (turn 0.1 rad/s)
[21:10:56] Stop                   code=0 SUCCESS  (after 0.5s)
```

### Simulation Stability

| Check | Before Test | After Test |
|-------|-------------|------------|
| webots-bin | ✅ PID 59639 | ✅ PID 59639 |
| mck | ✅ PID 59711 | ✅ PID 59711 |
| rpc_service_node | ✅ PID 59698 | ✅ PID 59698 |
| Port 1234 ESTAB | ✅ | ✅ |
| Segfault | 0 | 0 |
| Config errors | 0 | 0 |

### Status Codes Reference

| Code | Name | Meaning |
|------|------|---------|
| -1 | INVALID | Default, request not sent |
| 0 | SUCCESS | Command accepted |
| 100 | TIMEOUT | No response within 3s |
| 400 | BAD_REQUEST | Invalid parameters |
| 409 | CONFLICT | State conflict |
| 429 | TOO_FREQUENT | Rate limited |
| 500 | INTERNAL_ERROR | Server error |
| 501 | SERVER_REFUSED | Server refused |
| 502 | STATE_TRANSITION_FAILED | Invalid state transition |

### Known Issues

1. **GetStatus returns 502**: Simulation robot state machine may not support
   full status query. GetMode works correctly.
2. **ros2 CLI discovery**: Works only when using the exact same environment
   as the running process. Python rclpy clients work reliably.
3. **Python version**: Must use Python 3.10 (system), not miniconda Python 3.13.

### Test Artifacts

- `results/ros2_control_test.jsonl` — All test calls with timestamps and responses
