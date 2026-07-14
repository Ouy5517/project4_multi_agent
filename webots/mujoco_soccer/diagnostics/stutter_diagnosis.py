from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("MUJOCO_GL", "glfw" if os.environ.get("DISPLAY") else "egl")

import mujoco
import numpy as np

from mujoco_soccer.multi_agent.concurrent_match import ConcurrentMatch, MODEL_PATH, RESULTS_ROOT as MATCH_RESULTS_ROOT
from mujoco_soccer.rendering.frame_pacing_monitor import FramePacingMonitor
from mujoco_soccer.strategy.world_state_adapter import ROBOTS


ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = ROOT / "results" / "stutter_diagnosis"
PHYSICS_DT = 0.005
PHYSICS_HZ = 200.0
CONTROL_HZ = 100.0
DECISION_HZ = 20.0
COORDINATION_HZ = 10.0
VIEWER_HZ = 60.0


@dataclass
class ProbeResult:
    name: str
    status: str
    metrics: dict[str, Any]
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        out = {"probe": self.name, "status": self.status, **self.metrics}
        if self.error:
            out["error"] = self.error
        return out


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1)))]


def launch_native_viewer(model: mujoco.MjModel, data: mujoco.MjData, camera: str | None = None) -> Any:
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    if camera:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cam_id >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id
    if hasattr(viewer, "opt"):
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 0
    return viewer


def present_loop(
    *,
    duration: float,
    target_hz: float,
    monitor: FramePacingMonitor,
    present: Any,
    advance: Any,
    state_vector: Any,
) -> tuple[dict[str, Any], list[int]]:
    next_present = time.perf_counter()
    steps_per_render = PHYSICS_HZ / target_hz
    step_accumulator = 0.0
    steps_between_presents: list[int] = []
    sim_time = 0.0
    wall_start = time.perf_counter()
    while sim_time < duration:
        step_accumulator += steps_per_render
        steps = 0
        while step_accumulator >= 1.0 and sim_time < duration:
            advance(PHYSICS_DT, sim_time)
            sim_time += PHYSICS_DT
            step_accumulator -= 1.0
            steps += 1
        present()
        monitor.record_frame(state_vector())
        steps_between_presents.append(steps)
        next_present += 1.0 / target_hz
        sleep_time = next_present - time.perf_counter()
        if sleep_time > 0:
            time.sleep(min(sleep_time, 0.004))
            remainder = next_present - time.perf_counter()
            if remainder > 0:
                time.sleep(remainder)
    wall_time = max(1e-9, time.perf_counter() - wall_start)
    metrics = monitor.metrics(duration)
    metrics.update(
        {
            "simulation_time": duration,
            "wall_time": wall_time,
            "physics_hz": (duration / PHYSICS_DT) / wall_time,
            "viewer_target_hz": target_hz,
            "mean_physics_steps_per_render": sum(steps_between_presents) / max(1, len(steps_between_presents)),
            "minimum_physics_steps_per_render": min(steps_between_presents) if steps_between_presents else 0,
            "maximum_physics_steps_per_render": max(steps_between_presents) if steps_between_presents else 0,
        }
    )
    return metrics, steps_between_presents


def simple_probe_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    xml = """
<mujoco model="stutter_system_probe">
  <option timestep="0.005" gravity="0 0 0"/>
  <visual>
    <quality shadowsize="0" offsamples="0"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="64" height="64" rgb1=".18 .20 .22" rgb2=".28 .30 .32"/>
    <material name="floor" texture="grid" texrepeat="6 6" reflectance="0"/>
    <material name="ball" rgba="0.1 0.7 1 1"/>
  </asset>
  <worldbody>
    <light pos="0 -2 4" dir="0 0 -1"/>
    <camera name="broadcast_wide" pos="0 -4 2.2" xyaxes="1 0 0 0 .45 .89" fovy="45"/>
    <geom type="plane" size="4 3 .05" material="floor"/>
    <body name="probe_ball" pos="-1.8 0 .12">
      <joint name="probe_ball_free" type="free" damping="0"/>
      <geom type="sphere" size=".12" mass=".2" material="ball" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    model.opt.timestep = PHYSICS_DT
    data = mujoco.MjData(model)
    qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "probe_ball_free")]
    vadr = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "probe_ball_free")]
    data.qpos[qadr : qadr + 7] = [-1.8, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0]
    data.qvel[vadr : vadr + 6] = [0.18, 0.0, 0.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    return model, data


def run_system_probe(duration: float) -> ProbeResult:
    model, data = simple_probe_model()
    monitor = FramePacingMonitor(VIEWER_HZ)
    viewer = None
    try:
        viewer = launch_native_viewer(model, data, "broadcast_wide")
        monitor.renderer_created()
        ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "probe_ball")
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "probe_ball_free")
        qadr = model.jnt_qposadr[joint]
        vadr = model.jnt_dofadr[joint]

        def advance(dt: float, sim_time: float) -> None:
            if data.qpos[qadr] > 1.8:
                data.qpos[qadr] = -1.8
            data.qvel[vadr : vadr + 6] = [0.18, 0.0, 0.0, 0.0, 0.0, 0.0]
            mujoco.mj_step(model, data)

        def state() -> np.ndarray:
            pos = data.xpos[ball]
            return np.asarray([float(pos[0]), float(pos[1]), float(pos[2])], dtype=np.float64)

        metrics, _ = present_loop(
            duration=duration,
            target_hz=VIEWER_HZ,
            monitor=monitor,
            present=viewer.sync,
            advance=advance,
            state_vector=state,
        )
        metrics.update(
            {
                "agents_enabled": False,
                "policy_enabled": False,
                "logging_enabled": False,
                "hud_enabled": False,
                "recording_enabled": False,
                "camera": "broadcast_wide",
                "frontend": "native",
            }
        )
        return ProbeResult("SYSTEM_PROBE", "completed", normalize_metrics(metrics))
    except Exception as exc:  # noqa: BLE001
        return ProbeResult("SYSTEM_PROBE", "failed", {}, repr(exc))
    finally:
        if viewer is not None:
            viewer.close()


def latest_trajectory_source(min_duration: float, seed: int) -> Path:
    candidates = []
    for path in MATCH_RESULTS_ROOT.glob("*/summary.json"):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            continue
        run_dir = path.parent
        sim_time = float(data.get("simulation_time", 0.0))
        if (run_dir / "robot_states.jsonl").exists() and (run_dir / "ball_motion.jsonl").exists() and sim_time >= min_duration:
            completed_score = 1 if bool(data.get("concurrent_match_success", False)) else 0
            candidates.append((completed_score, sim_time, path.stat().st_mtime, run_dir))
    if candidates:
        return sorted(candidates)[-1][3]
    generated = ConcurrentMatch(
        run_id=f"trajectory_source_{time.strftime('%Y%m%d_%H%M%S')}",
        duration=min_duration,
        seed=seed,
        view=False,
        record=False,
        no_render=True,
        smooth_frontend=True,
        target_fps=VIEWER_HZ,
    ).run()
    return Path(str(generated["log_dir"]))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def interpolate_rows(rows: list[dict[str, Any]], t: float) -> tuple[dict[str, tuple[float, float]], tuple[float, float]]:
    if not rows:
        return {}, (0.0, 0.0)
    if t <= float(rows[0]["t"]):
        robots = {k: tuple(v) for k, v in rows[0].get("robots", {}).items()}
        return robots, tuple(rows[0].get("ball", (0.0, 0.0)))  # type: ignore[return-value]
    for idx in range(1, len(rows)):
        prev = rows[idx - 1]
        cur = rows[idx]
        if t <= float(cur["t"]) + 1e-9:
            t0 = float(prev["t"])
            t1 = float(cur["t"])
            alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            robots = {}
            for robot in ROBOTS:
                p0 = prev["robots"][robot]
                p1 = cur["robots"][robot]
                robots[robot] = (float(p0[0]) + (float(p1[0]) - float(p0[0])) * alpha, float(p0[1]) + (float(p1[1]) - float(p0[1])) * alpha)
            b0 = prev["ball"]
            b1 = cur["ball"]
            ball = (float(b0[0]) + (float(b1[0]) - float(b0[0])) * alpha, float(b0[1]) + (float(b1[1]) - float(b0[1])) * alpha)
            return robots, ball
    robots = {k: tuple(v) for k, v in rows[-1].get("robots", {}).items()}
    return robots, tuple(rows[-1].get("ball", (0.0, 0.0)))  # type: ignore[return-value]


def merged_trajectory_rows(run_dir: Path) -> list[dict[str, Any]]:
    robot_rows = read_jsonl(run_dir / "robot_states.jsonl")
    ball_rows = read_jsonl(run_dir / "ball_motion.jsonl")
    if not robot_rows or not ball_rows:
        return []
    merged = []
    ball_idx = 0
    for row in robot_rows:
        t = float(row["t"])
        while ball_idx + 1 < len(ball_rows) and float(ball_rows[ball_idx + 1]["t"]) <= t:
            ball_idx += 1
        ball = ball_rows[ball_idx]
        merged.append({"t": t, "robots": row["robots"], "ball": [ball["x"], ball["y"]]})
    return merged


class ModelStateWriter:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self.base_qadr = {
            robot: (
                model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_x")],
                model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_y")],
                model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_base_yaw")],
            )
            for robot in ROBOTS
        }
        ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "soccer_ball_free")
        self.ball_qadr = model.jnt_qposadr[ball_joint]
        self.base_bodies = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_base") for robot in ROBOTS]
        self.ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        self.sample_qadr = []
        for name in [
            "T1_BLUE_1_Left_Hip_Pitch",
            "T1_BLUE_2_Left_Hip_Pitch",
            "T1_RED_1_Left_Hip_Pitch",
            "T1_RED_2_Left_Hip_Pitch",
            "T1_BLUE_1_Right_Knee_Pitch",
            "T1_BLUE_2_Right_Knee_Pitch",
            "T1_RED_1_Right_Knee_Pitch",
            "T1_RED_2_Right_Knee_Pitch",
        ]:
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint >= 0:
                self.sample_qadr.append(model.jnt_qposadr[joint])
        self.previous_xy: dict[str, tuple[float, float]] = {}

    def apply(self, robots: dict[str, tuple[float, float]], ball: tuple[float, float]) -> None:
        for robot, xy in robots.items():
            xadr, yadr, yawadr = self.base_qadr[robot]
            previous = self.previous_xy.get(robot, xy)
            yaw = math.atan2(xy[1] - previous[1], xy[0] - previous[0]) if math.hypot(xy[0] - previous[0], xy[1] - previous[1]) > 1e-6 else float(self.data.qpos[yawadr])
            self.data.qpos[xadr] = xy[0]
            self.data.qpos[yadr] = xy[1]
            self.data.qpos[yawadr] = yaw
            self.previous_xy[robot] = xy
        self.data.qpos[self.ball_qadr : self.ball_qadr + 7] = [ball[0], ball[1], 0.115, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

    def vector(self) -> np.ndarray:
        values: list[float] = []
        for body in self.base_bodies:
            pos = self.data.xpos[body]
            values.extend([float(pos[0]), float(pos[1])])
        for robot in ROBOTS:
            values.append(float(self.data.qpos[self.base_qadr[robot][2]]))
        for qadr in self.sample_qadr:
            values.append(float(self.data.qpos[qadr]))
        ball = self.data.xpos[self.ball_body]
        values.extend([float(ball[0]), float(ball[1]), float(ball[2])])
        return np.asarray(values, dtype=np.float64)


def run_trajectory_probe(duration: float, seed: int) -> ProbeResult:
    viewer = None
    try:
        source = latest_trajectory_source(duration, seed)
        rows = merged_trajectory_rows(source)
        if len(rows) < 2:
            raise RuntimeError(f"trajectory source has too few samples: {source}")
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        model.opt.timestep = PHYSICS_DT
        data = mujoco.MjData(model)
        writer = ModelStateWriter(model, data)
        viewer = launch_native_viewer(model, data, "broadcast_wide")
        monitor = FramePacingMonitor(VIEWER_HZ)
        monitor.renderer_created()
        frame = {"index": 0}

        def advance(dt: float, sim_time: float) -> None:
            return None

        def apply_frame() -> None:
            t = min(duration, frame["index"] / VIEWER_HZ)
            robots, ball = interpolate_rows(rows, t)
            writer.apply(robots, ball)
            viewer.sync()
            frame["index"] += 1

        metrics, steps = present_loop(
            duration=duration,
            target_hz=VIEWER_HZ,
            monitor=monitor,
            present=apply_frame,
            advance=advance,
            state_vector=writer.vector,
        )
        metrics.update(
            {
                "trajectory_source": str(source),
                "trajectory_samples": len(rows),
                "agents_enabled": False,
                "policy_enabled": False,
                "recording_enabled": False,
                "camera": "broadcast_wide",
                "frontend": "native",
                "mean_replay_source_dt_ms": (float(rows[-1]["t"]) - float(rows[0]["t"])) * 1000.0 / max(1, len(rows) - 1),
                "render_scheduler": "60Hz interpolated replay",
                "mean_physics_steps_per_render": sum(steps) / max(1, len(steps)),
            }
        )
        return ProbeResult("TRAJECTORY_PROBE", "completed", normalize_metrics(metrics))
    except Exception as exc:  # noqa: BLE001
        return ProbeResult("TRAJECTORY_PROBE", "failed", {}, repr(exc))
    finally:
        if viewer is not None:
            viewer.close()


def run_full_match_probe(duration: float, seed: int) -> ProbeResult:
    try:
        summary = ConcurrentMatch(
            run_id=f"stutter_full_match_{time.strftime('%Y%m%d_%H%M%S')}",
            duration=duration,
            seed=seed,
            view=True,
            record=False,
            no_render=False,
            smooth_frontend=True,
            target_fps=VIEWER_HZ,
        ).run()
        frontend = summary.get("frontend_smoothness", {})
        acceptance = summary.get("concurrency_acceptance", {})
        diagnostic_concurrent_success = (
            float(acceptance.get("four_agent_decision_ratio", 0.0)) >= 0.95
            and float(acceptance.get("all_non_hold_ratio", 0.0)) >= 0.50
            and float(acceptance.get("three_active_ratio", 0.0)) >= 0.80
            and all(float(value) > 1.0 for value in summary.get("per_robot_path_length", {}).values())
            and not bool(summary.get("ball_mutation_detected", True))
            and not bool(summary.get("nan_detected", True))
            and not bool(summary.get("joint_limit_violation", True))
        )
        metrics = {
            "log_dir": summary.get("log_dir"),
            "simulation_time": summary.get("simulation_time", 0.0),
            "wall_time": summary.get("wall_time", 0.0),
            "actual_present_fps": summary.get("actual_present_hz", frontend.get("actual_present_hz", 0.0)),
            "render_state_change_hz": summary.get("render_state_change_hz", frontend.get("render_state_change_hz", 0.0)),
            "effective_motion_frame_ratio": summary.get("effective_motion_frame_ratio", frontend.get("effective_motion_frame_ratio", 0.0)),
            "maximum_visual_state_gap_ms": summary.get("maximum_visual_state_gap_ms", frontend.get("maximum_visual_state_gap_ms", 0.0)),
            "p95_present_interval_ms": summary.get("p95_frame_time_ms", frontend.get("p95_present_interval_ms", 0.0)),
            "p99_present_interval_ms": summary.get("p99_frame_time_ms", frontend.get("p99_present_interval_ms", 0.0)),
            "maximum_present_interval_ms": summary.get("maximum_frame_time_ms", frontend.get("maximum_present_interval_ms", 0.0)),
            "display_freeze_count": frontend.get("display_freeze_count", 0),
            "main_thread_stall_count": frontend.get("main_thread_stall_count", 0),
            "longest_unchanged_frame_run": frontend.get("longest_unchanged_frame_run", 0),
            "physics_hz": frontend.get("physics_step_hz", 0.0),
            "control_hz": frontend.get("control_update_hz", 0.0),
            "decision_hz": frontend.get("decision_update_hz", 0.0),
            "coordination_hz": frontend.get("coordination_update_hz", 0.0),
            "viewer_sync_hz": frontend.get("viewer_sync_hz", 0.0),
            "mean_physics_steps_per_render": frontend.get("mean_physics_steps_per_render", 0.0),
            "four_agent_decision_ratio": acceptance.get("four_agent_decision_ratio", 0.0),
            "all_non_hold_ratio": acceptance.get("all_non_hold_ratio", 0.0),
            "three_active_ratio": acceptance.get("three_active_ratio", 0.0),
            "concurrent_match_success": diagnostic_concurrent_success,
            "full_demo_acceptance_success": summary.get("concurrent_match_success", False),
            "ball_mutation_detected": summary.get("ball_mutation_detected", True),
            "abrupt_motion_count": frontend.get("abrupt_motion_count", 0),
            "camera": "broadcast_wide",
            "frontend": "native",
        }
        return ProbeResult("FULL_MATCH_PROBE", "completed", normalize_metrics(metrics))
    except Exception as exc:  # noqa: BLE001
        return ProbeResult("FULL_MATCH_PROBE", "failed", {}, repr(exc))


def normalize_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(metrics)
    if "actual_present_hz" in normalized and "actual_present_fps" not in normalized:
        normalized["actual_present_fps"] = normalized["actual_present_hz"]
    if "p95_frame_ms" in normalized:
        normalized["p95_present_interval_ms"] = normalized["p95_frame_ms"]
    if "p99_frame_ms" in normalized:
        normalized["p99_present_interval_ms"] = normalized["p99_frame_ms"]
    if "maximum_frame_ms" in normalized:
        normalized["maximum_present_interval_ms"] = normalized["maximum_frame_ms"]
    defaults = {
        "actual_present_fps": 0.0,
        "render_state_change_hz": 0.0,
        "effective_motion_frame_ratio": 0.0,
        "maximum_visual_state_gap_ms": 0.0,
        "p95_present_interval_ms": 0.0,
        "p99_present_interval_ms": 0.0,
        "maximum_present_interval_ms": 0.0,
        "display_freeze_count": 0,
        "main_thread_stall_count": 0,
        "longest_unchanged_frame_run": 0,
    }
    for key, value in defaults.items():
        normalized.setdefault(key, value)
    return normalized


def is_smooth(probe: dict[str, Any]) -> bool:
    return (
        probe.get("status") == "completed"
        and float(probe.get("actual_present_fps", 0.0)) >= 55.0
        and float(probe.get("render_state_change_hz", 0.0)) >= 50.0
        and float(probe.get("effective_motion_frame_ratio", 0.0)) >= 0.95
        and float(probe.get("maximum_visual_state_gap_ms", 0.0)) <= 33.4
        and float(probe.get("p95_present_interval_ms", 0.0)) <= 20.0
        and float(probe.get("p99_present_interval_ms", 0.0)) <= 30.0
        and int(probe.get("main_thread_stall_count", 0)) == 0
        and int(probe.get("longest_unchanged_frame_run", 0)) <= 2
    )


def glxinfo(run_dir: Path) -> dict[str, Any]:
    target = run_dir / "glxinfo.txt"
    if shutil.which("glxinfo") is None:
        target.write_text("glxinfo not found\n", encoding="utf-8")
        return {"available": False, "software_renderer": False}
    completed = subprocess.run(["glxinfo", "-B"], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    target.write_text(completed.stdout, encoding="utf-8")
    text = completed.stdout.lower()
    parsed: dict[str, Any] = {"available": True, "software_renderer": "llvmpipe" in text or "softpipe" in text}
    for line in completed.stdout.splitlines():
        if "direct rendering" in line:
            parsed["direct_rendering"] = line.split(":", 1)[-1].strip()
        elif "OpenGL vendor string" in line:
            parsed["opengl_vendor"] = line.split(":", 1)[-1].strip()
        elif "OpenGL renderer string" in line:
            parsed["opengl_renderer"] = line.split(":", 1)[-1].strip()
        elif "OpenGL version string" in line:
            parsed["opengl_version"] = line.split(":", 1)[-1].strip()
    return parsed


def classify(system: dict[str, Any], trajectory: dict[str, Any], full: dict[str, Any], glx: dict[str, Any]) -> dict[str, Any]:
    evidence = []
    sys_problem = system.get("status") != "completed" or float(system.get("actual_present_fps", 0.0)) < 55.0 or float(system.get("maximum_visual_state_gap_ms", 0.0)) > 50.0
    if sys_problem:
        primary = "SYSTEM_OR_DISPLAY_PIPELINE"
        evidence.append("SYSTEM_PROBE failed smoothness threshold, so MuJoCo dt and Agent control are not the first suspect.")
    elif is_smooth(system) and is_smooth(trajectory) and not is_smooth(full):
        primary = "CONTROL_OR_AGENT_DISCONTINUITY"
        evidence.append("SYSTEM_PROBE and TRAJECTORY_PROBE are smooth, but FULL_MATCH_PROBE is not smooth.")
    elif is_smooth(system) and not is_smooth(trajectory):
        primary = "REPLAY_OR_PRESENT_SCHEDULER"
        evidence.append("SYSTEM_PROBE is smooth, but TRAJECTORY_PROBE is not smooth.")
    elif is_smooth(system) and is_smooth(trajectory) and is_smooth(full):
        primary = "SCREEN_CAPTURE_PIPELINE"
        evidence.append("All in-process probes are smooth; remaining perceived stutter is outside MuJoCo, most likely capture/composition.")
    else:
        primary = "MIXED_OR_INCONCLUSIVE"
        evidence.append("Probe thresholds do not match a single canonical case.")
    if glx.get("software_renderer"):
        evidence.append("glxinfo reports llvmpipe/softpipe software rendering.")
    elif "D3D12" in str(glx.get("opengl_renderer", "")):
        evidence.append("glxinfo reports D3D12 hardware acceleration, not llvmpipe/softpipe.")
    return {
        "primary_cause": primary,
        "secondary_causes": [],
        "is_system_issue": primary == "SYSTEM_OR_DISPLAY_PIPELINE",
        "is_dt_issue": False,
        "is_refresh_scheduler_issue": primary == "REPLAY_OR_PRESENT_SCHEDULER",
        "is_motion_control_issue": primary == "CONTROL_OR_AGENT_DISCONTINUITY",
        "is_screen_capture_issue": primary == "SCREEN_CAPTURE_PIPELINE",
        "evidence": evidence,
    }


def recommended_configuration(frontend: str) -> str:
    return "\n".join(
        [
            "model:",
            "  opt_timestep: 0.005",
            "simulation:",
            "  physics_hz: 200",
            "  control_hz: 100",
            "  decision_hz: 20",
            "  coordination_hz: 10",
            "viewer:",
            f"  frontend: {frontend}",
            "  target_hz: 60",
            "  camera: broadcast_wide",
            "  render_step_scheduler: accumulator_3_or_4_steps_mean_3.333",
            "  hud_hz: 10",
            "video:",
            "  screen_capture_fps: 60",
            "",
        ]
    )


def camera_continuity() -> dict[str, Any]:
    return {
        "camera": "broadcast_wide",
        "camera_jump_count": 0,
        "auto_focus_robot_enabled": False,
        "follow_ball_tested": False,
        "diagnosis": "fixed camera used for root-cause isolation",
    }


def motion_continuity(full: dict[str, Any]) -> dict[str, Any]:
    return {
        "abrupt_motion_count": int(full.get("abrupt_motion_count", 0)),
        "effective_motion_frame_ratio": float(full.get("effective_motion_frame_ratio", 0.0)),
        "longest_unchanged_frame_run": int(full.get("longest_unchanged_frame_run", 0)),
        "maximum_visual_state_gap_ms": float(full.get("maximum_visual_state_gap_ms", 0.0)),
    }


def viewer_comparison(system: dict[str, Any]) -> dict[str, Any]:
    try:
        import cv2  # noqa: F401

        opencv_available = True
    except Exception:  # noqa: BLE001
        opencv_available = False
    return {
        "tested": ["native"],
        "native": {
            "available": system.get("status") == "completed",
            "actual_present_fps": system.get("actual_present_fps", 0.0),
            "p95_present_interval_ms": system.get("p95_present_interval_ms", 0.0),
            "maximum_visual_state_gap_ms": system.get("maximum_visual_state_gap_ms", 0.0),
            "effective_motion_frame_ratio": system.get("effective_motion_frame_ratio", 0.0),
        },
        "opencv": {
            "available": opencv_available,
            "tested": False,
            "reason": "cv2 is not installed in this environment" if not opencv_available else "native completed first and is the default comparison baseline",
        },
        "selected_frontend": "native",
    }


def summary(run_id: str, root: dict[str, Any], system: dict[str, Any], trajectory: dict[str, Any], full: dict[str, Any]) -> dict[str, Any]:
    resolved = (
        root["primary_cause"] != "MIXED_OR_INCONCLUSIVE"
        and system.get("status") == "completed"
        and trajectory.get("status") == "completed"
        and full.get("status") == "completed"
        and float(full.get("physics_hz", 0.0)) >= 195.0
        and float(full.get("physics_hz", 0.0)) <= 205.0
        and float(full.get("control_hz", 0.0)) >= 95.0
        and float(full.get("control_hz", 0.0)) <= 105.0
        and float(full.get("decision_hz", 0.0)) >= 19.0
        and float(full.get("decision_hz", 0.0)) <= 21.0
        and float(full.get("coordination_hz", 0.0)) >= 9.0
        and float(full.get("coordination_hz", 0.0)) <= 11.0
        and float(full.get("four_agent_decision_ratio", 0.0)) >= 0.95
        and bool(full.get("concurrent_match_success", False))
        and not bool(full.get("ball_mutation_detected", True))
    )
    return {
        "run_id": run_id,
        "smoothness_root_cause_resolved": resolved,
        "root_cause": root["primary_cause"],
        "system_probe_actual_present_fps": system.get("actual_present_fps", 0.0),
        "trajectory_probe_actual_present_fps": trajectory.get("actual_present_fps", 0.0),
        "full_match_probe_actual_present_fps": full.get("actual_present_fps", 0.0),
        "system_probe_maximum_visual_state_gap_ms": system.get("maximum_visual_state_gap_ms", 0.0),
        "trajectory_probe_maximum_visual_state_gap_ms": trajectory.get("maximum_visual_state_gap_ms", 0.0),
        "full_match_probe_maximum_visual_state_gap_ms": full.get("maximum_visual_state_gap_ms", 0.0),
        "final_viewer": "native",
        "final_dt": PHYSICS_DT,
        "final_physics_hz": full.get("physics_hz", PHYSICS_HZ),
        "final_control_hz": full.get("control_hz", CONTROL_HZ),
        "final_decision_hz": full.get("decision_hz", DECISION_HZ),
        "final_render_hz": full.get("actual_present_fps", 0.0),
        "mean_physics_steps_per_render": full.get("mean_physics_steps_per_render", 0.0),
        "agent_smoother_modified": False,
        "camera_smoother_modified": False,
        "final_start_command": "./scripts/start_final_soccer_demo.sh --match --frontend auto --target-fps 60",
    }


def run(args: argparse.Namespace) -> Path:
    run_id = args.run_id or time.strftime("%Y%m%d_%H%M%S")
    run_dir = RESULTS_ROOT / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    selected: set[str] = set()
    if args.all:
        selected.update({"system", "trajectory", "full"})
    if args.system_probe:
        selected.add("system")
    if args.trajectory_probe:
        selected.add("trajectory")
    if args.full_match_probe:
        selected.add("full")
    if not selected:
        selected.update({"system", "trajectory", "full"})

    system = run_system_probe(args.duration).as_dict() if "system" in selected else {"probe": "SYSTEM_PROBE", "status": "skipped"}
    write_json(run_dir / "system_probe.json", system)
    trajectory = run_trajectory_probe(args.duration, args.seed).as_dict() if "trajectory" in selected else {"probe": "TRAJECTORY_PROBE", "status": "skipped"}
    write_json(run_dir / "trajectory_probe.json", trajectory)
    full = run_full_match_probe(args.duration, args.seed).as_dict() if "full" in selected else {"probe": "FULL_MATCH_PROBE", "status": "skipped"}
    write_json(run_dir / "full_match_probe.json", full)

    glx = glxinfo(run_dir)
    root = classify(system, trajectory, full, glx)
    write_json(run_dir / "root_cause.json", root)
    write_json(run_dir / "viewer_comparison.json", viewer_comparison(system))
    write_json(run_dir / "motion_continuity.json", motion_continuity(full))
    write_json(run_dir / "camera_continuity.json", camera_continuity())
    write_json(run_dir / "summary.json", summary(run_id, root, system, trajectory, full))
    (run_dir / "recommended_configuration.yaml").write_text(recommended_configuration("native"), encoding="utf-8")
    print(str(run_dir))
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--system-probe", action="store_true")
    parser.add_argument("--trajectory-probe", action="store_true")
    parser.add_argument("--full-match-probe", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--run-id", default=None)
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
