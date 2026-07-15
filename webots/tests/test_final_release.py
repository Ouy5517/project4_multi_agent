from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path


def test_final_release_config_and_scripts_exist() -> None:
    config = Path("mujoco_soccer/config/final_release.yaml")
    text = config.read_text()
    for item in ["physics_hz: 200", "target_fps: 60", "output_fps: 60", "source: physical_trajectory_log"]:
        assert item in text
    for script in [
        "scripts/start_final_soccer_demo.sh",
        "scripts/run_final_acceptance.sh",
        "scripts/package_final_release.sh",
        "scripts/render_concurrent_log_video_60fps.py",
        "scripts/analyze_video_smoothness.py",
    ]:
        assert Path(script).exists()


def test_final_model_has_complete_noncolliding_goal_nets() -> None:
    tree = ET.parse("mujoco_soccer/models/t1_2v2_soccer_visual_v3.xml")
    geoms = {geom.attrib["name"]: geom.attrib for geom in tree.findall(".//geom") if "name" in geom.attrib}
    required = [
        "BLUE_GOAL_left_post",
        "BLUE_GOAL_right_post",
        "BLUE_GOAL_crossbar",
        "BLUE_GOAL_back_left_post",
        "BLUE_GOAL_top_net",
        "BLUE_GOAL_back_net",
        "RED_GOAL_left_post",
        "RED_GOAL_right_post",
        "RED_GOAL_crossbar",
        "RED_GOAL_back_left_post",
        "RED_GOAL_top_net",
        "RED_GOAL_back_net",
    ]
    assert all(name in geoms for name in required)
    for name, attrs in geoms.items():
        if "_net" in name:
            assert attrs.get("contype") == "0"
            assert attrs.get("conaffinity") == "0"


def test_final_summary_metric_fields_are_present() -> None:
    candidates = sorted(Path("results/mujoco_concurrent_match").glob("concurrent_*/summary.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    assert candidates
    summary = json.loads(candidates[0].read_text())
    required = [
        "pass_behavior_ticks",
        "pass_action_starts",
        "pass_successes",
        "shoot_behavior_ticks",
        "shoot_action_starts",
        "shoot_successes",
        "intercept_behavior_ticks",
        "intercept_action_starts",
        "intercept_successes",
        "clear_behavior_ticks",
        "clear_action_starts",
        "clear_successes",
        "contact_samples",
        "unique_contact_events",
    ]
    for key in required:
        assert key in summary or key in Path("mujoco_soccer/multi_agent/concurrent_match.py").read_text()


def test_video_provenance_schema_in_renderer_script() -> None:
    text = Path("scripts/render_concurrent_log_video_60fps.py").read_text()
    for key in [
        '"video_type": "trajectory_replay"',
        '"direct_realtime_screen_recording": False',
        '"source_is_physical_simulation": True',
        '"trajectory_interpolation": True',
    ]:
        assert key in text


def test_final_launcher_does_not_start_webots_mck_or_rpc() -> None:
    text = Path("scripts/start_final_soccer_demo.sh").read_text()
    assert "Webots/mck/RPC are not started" in text
    assert "webots" not in text.lower().replace("webots/mck/rpc", "")
    assert "rpc_service_node" not in text
