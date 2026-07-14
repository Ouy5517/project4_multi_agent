# Booster T1 Kick Physics Test — Final Results

## 2026-07-11 21:47~22:02 UTC

### Test Environment

| Parameter | Value |
|-----------|-------|
| World | `T1_soccer_test.wbt` (T1_release + SOCCER_BALL + ball_monitor) |
| Ball | Sphere r=0.11m, mass=0.43kg, init=(0.3, -0.005, 0.11) |
| Runner config | `configs/config.lua` (default) and `configs/config_soccer.lua` |

### Kick Module Inventory

| Component | Found | Detail |
|-----------|-------|--------|
| `librl_locomotion.so` | ✅ | Contains VMPFancyKick, VMPSideKick, VMPForwardKick, VMPFancyForwardKick |
| `lib/T1_vmp_fancy_kick` (model) | ✅ | 10 MB |
| `lib/T1_vmp_side_kick` (model) | ✅ | 4.3 MB |
| `lib/T1_vmp_forward_kick` (model) | ✅ | 4.3 MB |
| `lib/T1_vmp_fancy_forward_kick` (model) | ✅ | exists |
| Lua module configs | ✅ | `common_module_options.lua` lines 2248-2410 |
| Graph connections | ✅ | `common_graph_define.lua` 134 references |

### Kick Enable Flags

#### Default config (config.lua → common_module_options.lua)
```
enable_robocup_locomotion: 0
enable_fancy_kick:         0  (joystick_service internal, no Lua key)
```

#### Soccer config (config_soccer.lua → common_module_options_soccer.lua)
```
enable_robocup_locomotion: 1  ✅ Changed by Lua
enable_face_down_get_up:   1  ✅ Changed by Lua
enable_face_up_get_up:     1  ✅ Changed by Lua
enable_fancy_kick:         0  ❌ STILL 0 — controlled by joystick hardware only
```

### Root Cause

`enable_fancy_kick` is set exclusively by `joystick_service.cpp:556` based on
physical joystick button state. When no joystick is connected (simulation mode),
it is always 0. **No Lua configuration key exists for `enable_fancy_kick`.**

The five other enable flags (`enable_robocup_locomotion`, `enable_face_down_get_up`,
etc.) ARE Lua-configurable and responded correctly to `config_soccer.lua`.

### API Test Results

| API | api_id | Mode | Config | Result |
|-----|--------|------|--------|--------|
| ChangeMode→Soccer | 2000 | Prepare | default | **502** Invalid mode |
| ChangeMode→Soccer | 2000 | Prepare | soccer | 502 (unverified, same root cause) |
| kShoot | 2024 | Walking | default | **502** STATE_TRANSITION_FAILED |
| kVisualKick V0 | 2038 | Walking | default | **502** STATE_TRANSITION_FAILED |
| kVisualKick V0 | 2038 | Walking | soccer | 502 (conf, same root cause) |

All kick APIs rejected because `command_manager_v2` requires `enable_fancy_kick=1`
which is only set by `joystick_service` when a physical joystick button mapped
to kick is pressed.

### Mode Validation

`loco_api_service.cpp:59`: only modes 0-3 are valid (kDamping=0, kPrepare=1,
kWalking=2, kCustom=3). Soccer mode (4) is explicitly rejected with "Invalid mode: 4".

### Ball Displacement (all tests)

| Metric | Value |
|--------|-------|
| Initial position | (0.3000, -0.0050, 0.1100) |
| Max horizontal displacement | **0.0000 m** |
| Detected state | BALL_STATIC |
| Threshold (>0.05m) | **NOT REACHED** |

### Conclusion

**Physical kick is NOT available in this simulation mck build.** The binary has
all kick modules and models compiled in, but `joystick_service.cpp` gates
`enable_fancy_kick` behind physical joystick input. Without a joystick, the
state machine rejects all kick API calls with code 502.

Move/Stop/Turn are fully functional via RPC.

### Required to Enable Kick

1. Connect a physical joystick to the simulation host, OR
2. Obtain an mck build with joystick-independent kick gating (modified source), OR
3. Use a different mck binary that maps kick to a non-joystick API path

### Files Created/Modified

| File | Purpose |
|------|---------|
| `configs/config_soccer.lua` | Soccer Lua config (robocup enabled, kick still blocked) |
| `configs/common_module_options_soccer.lua` | Copied from default, enable flags changed |
| `worlds/T1_soccer_test.wbt` | Soccer world with ball + ball_monitor Supervisor |
| `controllers/ball_monitor/ball_monitor.py` | Ball displacement tracker |
| `results/kick_ball_motion.jsonl` | Ball position timeseries |
