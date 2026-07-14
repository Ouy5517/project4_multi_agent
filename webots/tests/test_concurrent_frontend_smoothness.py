from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import mujoco

from mujoco_soccer.control.motion_command_smoother import MotionCommandSmoother
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.logging.async_run_logger import AsyncRunLogger
from mujoco_soccer.multi_agent.concurrent_match import ConcurrentMatch, MODEL_PATH
from mujoco_soccer.rendering.concurrent_fast_viewer import ConcurrentFastViewer
from mujoco_soccer.rendering.frame_interpolator import RenderPose, interpolate_pose


def test_frame_interpolator_does_not_write_back_to_qpos() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    before = data.qpos.copy()
    pose = interpolate_pose(RenderPose(0.0, 0.0, 0.0, 3.0), RenderPose(1.0, 1.0, 1.0, -3.0), 0.5)
    assert pose.x == 0.5
    assert abs(pose.yaw) <= math.pi
    assert (data.qpos == before).all()


def test_smooth_viewer_does_not_start_video_writer_and_targets_60fps() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    viewer = ConcurrentFastViewer(model, data, target_fps=60)
    assert viewer.render_interval_steps <= 4
    assert viewer.metrics()["view_started_video_writer"] is False


def test_async_run_logger_uses_queue_and_flushes() -> None:
    run_dir = Path("results/mujoco_concurrent_match/unit_async_logger")
    if run_dir.exists():
        shutil.rmtree(run_dir)
    logger = AsyncRunLogger(run_dir)
    logger.append_jsonl("events.jsonl", {"event": "PASS"}, priority="HIGH")
    logger.close()
    assert json.loads((run_dir / "events.jsonl").read_text().strip())["event"] == "PASS"
    assert logger.metrics()["async_logging"] is True
    assert logger.metrics()["synchronous_flush_on_main_thread"] is False


def test_motion_command_smoother_limits_speed_and_tracks_stats() -> None:
    smoother = MotionCommandSmoother()
    first = smoother.smooth(BaseTarget(2.0, 0.0, 0.0, max_speed=0.6, max_yaw_rate=math.radians(120)), 0.05, 0.0, "PRESS_BALL", None)
    second = smoother.smooth(BaseTarget(-2.0, 0.0, math.pi, max_speed=0.6, max_yaw_rate=math.radians(120)), 0.05, 0.05, "PRESS_BALL", None)
    assert first.max_speed <= 0.20
    assert second.max_speed <= 0.20
    assert second.max_yaw_rate <= math.radians(90)
    assert smoother.stats.maximum_jerk <= 4.01


def test_smooth_concurrent_short_run_writes_frontend_summary() -> None:
    summary = ConcurrentMatch(run_id="unit_smooth_short", duration=1.0, seed=42, no_render=True, smooth_frontend=True).run()
    run_dir = Path(summary["log_dir"])
    assert (run_dir / "frontend_smoothness_acceptance.json").exists()
    smoothness = json.loads((run_dir / "frontend_smoothness_acceptance.json").read_text())
    assert smoothness["matplotlib_used"] is False
    assert smoothness["view_mode_started_video_writer"] is False
    assert summary["decision_counts"]["T1_BLUE_1"] == 20


def test_smooth_launcher_exists() -> None:
    assert Path("scripts/start_mujoco_concurrent_match_smooth.sh").exists()
