#!/usr/bin/python3
"""Match state monitor — reads robot/ball/goal positions from 2v2 world.

v2: Per-run isolation with unique run_id, metadata, events, and validation.
"""

import json
import math
import os
import sys
import time
import uuid
from datetime import datetime, timezone

try:
    from controller import Supervisor
except ImportError:
    print("ERROR: Webots controller module not found.", file=sys.stderr)
    sys.exit(1)

# ── Per-run isolation ──
BASE_OUT_DIR = "/home/plon/Workspace/booster_soccer_project/results"
STARTUP_RUNS_DIR = os.path.join(BASE_OUT_DIR, "startup_runs")

ROBOT_NAMES = ["T1_BLUE_1", "T1_BLUE_2", "T1_RED_1", "T1_RED_2"]

# ── Validation thresholds ──
MAX_ABS_COORD = 100.0     # metres — beyond this is INVALID_STATE
MAX_LINEAR_SPEED = 50.0   # m/s — heuristic for explosion detection
ASSISTED_ROOT_MOTION = os.environ.get("FOUR_ROBOT_DEMO_MODE") in {"full", "single-contact", "motion-check"}
# We track position deltas between consecutive samples to detect speed spikes.


def _generate_run_id() -> str:
    """YYYYMMDD_HHMMSS_<short_random>"""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    short = uuid.uuid4().hex[:6]
    return f"{ts}_{short}"


def _sanitize(val):
    """Return None for NaN/Inf, else the value."""
    if val is None:
        return None
    try:
        f = float(val)
    except (ValueError, TypeError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def _validate_position(pos, label=""):
    """Return (is_valid, issues_list)."""
    issues = []
    if pos is None or len(pos) < 3:
        issues.append(f"{label}_pos_missing")
        return False, issues
    x, y, z = _sanitize(pos[0]), _sanitize(pos[1]), _sanitize(pos[2])
    if x is None or y is None or z is None:
        issues.append(f"{label}_nan_inf")
    else:
        if abs(x) > MAX_ABS_COORD:
            issues.append(f"{label}_x_exceed:{x:.1f}")
        if abs(y) > MAX_ABS_COORD:
            issues.append(f"{label}_y_exceed:{y:.1f}")
        if abs(z) > MAX_ABS_COORD:
            issues.append(f"{label}_z_exceed:{z:.1f}")
    return len(issues) == 0, issues


def _validate_speed(prev_pos, curr_pos, dt, label=""):
    """Check speed between consecutive samples."""
    if prev_pos is None or curr_pos is None:
        return True, []
    if dt <= 0:
        return True, []
    try:
        dx = _sanitize(curr_pos[0]) - _sanitize(prev_pos[0])
        dy = _sanitize(curr_pos[1]) - _sanitize(prev_pos[1])
        dz = _sanitize(curr_pos[2]) - _sanitize(prev_pos[2])
        if any(v is None for v in (dx, dy, dz)):
            return True, []
        speed = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
        if speed > MAX_LINEAR_SPEED:
            return False, [f"{label}_speed_exceed:{speed:.1f}_m/s"]
    except Exception:
        pass
    return True, []


def _find_robot_by_name(supervisor, name):
    """Search root children for a Robot with given name."""
    root = supervisor.getRoot()
    n_children = root.getNumberOfFields()
    for i in range(n_children):
        field = root.getFieldByIndex(i)
        if field.getType() == 0x2A:  # SF_NODE
            node = field.getSFNode()
            if node:
                try:
                    name_field = node.getField("name")
                    if name_field and name_field.getSFString() == name:
                        return node
                except Exception:
                    pass
        try:
            if field.getType() in (0x2B, 0x2C):  # MF_NODE
                for j in range(field.getCount()):
                    node = field.getMFNode(j)
                    try:
                        name_field = node.getField("name")
                        if name_field and name_field.getSFString() == name:
                            return node
                    except Exception:
                        pass
        except Exception:
            pass
    return None


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep()) or 100

    # ── Generate run_id and output directory ──
    run_id = os.environ.get("MATCH_STATE_RUN_ID") or _generate_run_id()
    run_dir = os.environ.get("FINAL_SUBMISSION_RUN_DIR") or os.path.join(STARTUP_RUNS_DIR, run_id)
    os.makedirs(run_dir, exist_ok=True)

    out_file = os.path.join(run_dir, "match_state.jsonl")
    events_file = os.path.join(run_dir, "events.jsonl")
    metadata_file = os.path.join(run_dir, "metadata.json")

    # Detect world file from environment or supervisor
    world_file = os.environ.get("WEBOTS_WORLD", "unknown.wbt")

    # ── Write metadata ──
    metadata = {
        "run_id": run_id,
        "world_file": world_file,
        "basicTimeStep": timestep,
        "start_wall_time": datetime.now(timezone.utc).isoformat(),
        "robot_names": ROBOT_NAMES,
        "controller": "match_state_monitor",
        "version": 2,
    }
    with open(metadata_file, "w") as mf:
        json.dump(metadata, mf, indent=2)
        mf.write("\n")

    # ── Find ball ──
    ball = supervisor.getFromDef("SOCCER_BALL")
    if ball is None:
        print("[match_monitor] WARNING: SOCCER_BALL not found, ball data will be absent.",
              file=sys.stderr)

    # ── Find goals ──
    blue_goal = None
    red_goal = None
    bg_node = supervisor.getFromDef("BLUE_GOAL")
    if bg_node is not None:
        blue_goal = bg_node
    rg_node = supervisor.getFromDef("RED_GOAL")
    if rg_node is not None:
        red_goal = rg_node

    # ── Find robots ──
    robots = {}
    for name in ROBOT_NAMES:
        node = supervisor.getFromDef(name)
        if node is None:
            node = _find_robot_by_name(supervisor, name)
        if node:
            robots[name] = node
            print(f"[match_monitor] Found {name}")
        else:
            print(f"[match_monitor] WARNING: {name} not found")

    metadata["robots_found"] = list(robots.keys())
    # Rewrite metadata with robots_found
    with open(metadata_file, "w") as mf:
        json.dump(metadata, mf, indent=2)
        mf.write("\n")

    # ── Write events helper ──
    def emit_event(event_type: str, **kwargs):
        ev = {
            "run_id": run_id,
            "event": event_type,
            "sim_time": round(supervisor.getTime(), 6),
            "wall_time": datetime.now(timezone.utc).isoformat(),
        }
        ev.update(kwargs)
        with open(events_file, "a") as ef:
            ef.write(json.dumps(ev, default=str) + "\n")
            ef.flush()

    emit_event("MONITOR_START", robots_found=list(robots.keys()))

    # ── State variables ──
    sequence_id = 0
    prev_sim_time = -1.0
    prev_sample_time = -1.0
    prev_positions = {}  # robot_name -> [x, y, z]
    log_interval = 10      # every 10ms for first 2s (100Hz at 1ms timestep)
    normal_interval = 100  # every 100ms after 2s

    with open(out_file, "w") as f:  # "w" NOT "a" — never append to old runs
        # Write header record
        f.write(json.dumps({
            "event": "RUN_START",
            "run_id": run_id,
            "sequence_id": sequence_id,
            "sim_time": round(supervisor.getTime(), 6),
            "wall_time": datetime.now(timezone.utc).isoformat(),
            "robots_found": list(robots.keys()),
        }) + "\n")
        f.flush()

        while supervisor.step(timestep) != -1:
            sequence_id += 1
            sim_time = supervisor.getTime()

            # ── Detect sim_time regression ──
            if sim_time < prev_sim_time - 0.0005:  # small tolerance for float rounding
                emit_event("WORLD_RELOAD",
                           prev_sim_time=round(prev_sim_time, 6),
                           new_sim_time=round(sim_time, 6))
            # ── Sampling rate ──
            if sim_time < 2.0:
                if sequence_id % log_interval != 0:
                    continue
            else:
                if sequence_id % normal_interval != 0:
                    continue
            sample_dt = sim_time - prev_sample_time if prev_sample_time >= 0 else timestep / 1000.0
            prev_sample_time = sim_time
            prev_sim_time = sim_time

            state = {
                "run_id": run_id,
                "sequence_id": sequence_id,
                "event": "STATE",
                "sim_time": round(sim_time, 6),
                "wall_time": datetime.now(timezone.utc).isoformat(),
            }

            any_invalid = False

            # ── Ball ──
            if ball is not None:
                bp = ball.getPosition()
                valid_b, issues_b = _validate_position(bp, "ball")
                state["ball"] = {
                    "x": round(_sanitize(bp[0]), 4) if _sanitize(bp[0]) is not None else None,
                    "y": round(_sanitize(bp[1]), 4) if _sanitize(bp[1]) is not None else None,
                    "z": round(_sanitize(bp[2]), 4) if _sanitize(bp[2]) is not None else None,
                }
                if not valid_b:
                    any_invalid = True
                    for issue in issues_b:
                        emit_event("INVALID_STATE", detail=issue, target="ball")
                # Ball out of bounds
                bx = _sanitize(bp[0])
                by = _sanitize(bp[1])
                if bx is not None and by is not None:
                    state["ball_out"] = abs(bx) > 3.5 or abs(by) > 2.5
            else:
                state["ball"] = None

            # ── Robots ──
            state["robots"] = {}
            for name, node in robots.items():
                pos = node.getPosition()
                orient = node.getOrientation()

                # Validate position
                valid_p, issues_p = _validate_position(pos, name)
                if not valid_p:
                    any_invalid = True
                    for issue in issues_p:
                        emit_event("INVALID_STATE", detail=issue, target=name)

                # Validate speed vs previous
                if name in prev_positions and not ASSISTED_ROOT_MOTION:
                    valid_s, issues_s = _validate_speed(prev_positions[name], pos, max(sample_dt, 0.001), name)
                    if not valid_s:
                        any_invalid = True
                        for issue in issues_s:
                            emit_event("INVALID_STATE", detail=issue, target=name)

                prev_positions[name] = [pos[0], pos[1], pos[2]]

                yaw = math.atan2(orient[3], orient[0]) if len(orient) >= 4 else 0.0

                robot_state = {
                    "x": round(_sanitize(pos[0]), 4) if _sanitize(pos[0]) is not None else None,
                    "y": round(_sanitize(pos[1]), 4) if _sanitize(pos[1]) is not None else None,
                    "z": round(_sanitize(pos[2]), 4) if _sanitize(pos[2]) is not None else None,
                    "yaw": round(yaw, 4),
                }
                state["robots"][name] = robot_state

            # ── Ball carrier ──
            if ball is not None:
                bp = ball.getPosition()
                ball_xy = (_sanitize(bp[0]), _sanitize(bp[1]))
                closest = None
                closest_dist = 0.3
                if ball_xy[0] is not None and ball_xy[1] is not None:
                    for name, node in robots.items():
                        rp = node.getPosition()
                        rx = _sanitize(rp[0])
                        ry = _sanitize(rp[1])
                        if rx is not None and ry is not None:
                            d = math.hypot(rx - ball_xy[0], ry - ball_xy[1])
                            if d < closest_dist:
                                closest_dist = d
                                closest = name
                state["ball_carrier"] = closest

            # ── Goals ──
            if blue_goal:
                bg = blue_goal.getPosition()
                state["blue_goal"] = {
                    "x": round(_sanitize(bg[0]), 4) if _sanitize(bg[0]) is not None else None,
                    "y": round(_sanitize(bg[1]), 4) if _sanitize(bg[1]) is not None else None,
                }
            if red_goal:
                rg = red_goal.getPosition()
                state["red_goal"] = {
                    "x": round(_sanitize(rg[0]), 4) if _sanitize(rg[0]) is not None else None,
                    "y": round(_sanitize(rg[1]), 4) if _sanitize(rg[1]) is not None else None,
                }

            if any_invalid:
                state["invalid"] = True

            f.write(json.dumps(state, default=str) + "\n")
            f.flush()

    emit_event("MONITOR_STOP")
    print(f"[match_monitor] Run {run_id} complete — {sequence_id} steps, "
          f"output: {out_file}")


if __name__ == "__main__":
    main()
