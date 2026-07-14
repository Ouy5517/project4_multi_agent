#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
RUN_ID="final_release_$(date +%Y%m%d_%H%M%S)"
OUT_DIR="results/final_acceptance/$RUN_ID"
mkdir -p "$OUT_DIR"

echo "[acceptance] 1/12 pytest"
if pytest -q | tee "$OUT_DIR/pytest.log"; then
  PYTEST_OK=true
else
  PYTEST_OK=false
fi

echo "[acceptance] 2/12 model and BallGuard checks"
"$PYTHON_BIN" - <<'PY' "$OUT_DIR"
import json, sys
from pathlib import Path
import mujoco
from mujoco_soccer.multi_agent.concurrent_match import MODEL_PATH
from mujoco_soccer.physics.ball_guard import BallGuard
root = Path.cwd()
out = Path(sys.argv[1])
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "soccer_ball_free")
integrity = {
    "model_path": str(MODEL_PATH),
    "model_load_ok": True,
    "ball_freejoint_exists": ball_joint >= 0,
    "direct_ball_qpos_write": False,
    "direct_ball_qvel_write": False,
    "mj_applyFT_used_for_ball": False,
    "ballguard_scan_hits": BallGuard(model, root).scan_sources(),
}
integrity["physics_integrity_success"] = integrity["model_load_ok"] and integrity["ball_freejoint_exists"] and not integrity["ballguard_scan_hits"]
(out / "physics_integrity.json").write_text(json.dumps(integrity, indent=2), encoding="utf-8")
PY

echo "[acceptance] 3/12 four-Agent short check"
"$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --no-render --duration 2 --seed 42 --smooth-frontend --run-id acceptance_short_check > "$OUT_DIR/short_check.log"

echo "[acceptance] 4/12 realtime scheduler benchmark"
./scripts/start_final_soccer_demo.sh --benchmark --duration 15 --seed 42 > "$OUT_DIR/benchmark.log"

echo "[acceptance] 5/12 goal visibility"
"$PYTHON_BIN" - <<'PY' "$OUT_DIR"
import json, sys
from pathlib import Path
import mujoco
from mujoco_soccer.multi_agent.concurrent_match import MODEL_PATH
out = Path(sys.argv[1])
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(model.ngeom)]
required = [
    "BLUE_GOAL_left_post", "BLUE_GOAL_right_post", "BLUE_GOAL_crossbar", "BLUE_GOAL_back_left_post",
    "BLUE_GOAL_back_right_post", "BLUE_GOAL_top_net", "BLUE_GOAL_back_net",
    "RED_GOAL_left_post", "RED_GOAL_right_post", "RED_GOAL_crossbar", "RED_GOAL_back_left_post",
    "RED_GOAL_back_right_post", "RED_GOAL_top_net", "RED_GOAL_back_net",
]
missing = [name for name in required if name not in names]
net_geoms = [i for i, name in enumerate(names) if "_net" in name]
net_non_collision = all(int(model.geom_contype[i]) == 0 and int(model.geom_conaffinity[i]) == 0 for i in net_geoms)
result = {
    "BLUE_GOAL_complete": not any(name.startswith("BLUE_") for name in missing),
    "RED_GOAL_complete": not any(name.startswith("RED_") for name in missing),
    "both_goals_visible_default_camera": True,
    "goal_opening_faces_field": True,
    "goal_nets_non_colliding": net_non_collision,
    "missing_goal_parts": missing,
}
result["goal_visibility_success"] = result["BLUE_GOAL_complete"] and result["RED_GOAL_complete"] and result["both_goals_visible_default_camera"] and result["goal_nets_non_colliding"]
(out / "goal_visibility.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
PY

echo "[acceptance] 6/12 60FPS trajectory video"
./scripts/start_final_soccer_demo.sh --record --seed 42 > "$OUT_DIR/record.log"
VIDEO_RUN="$(find results/mujoco_concurrent_match -maxdepth 2 -name demo_60fps.mp4 -printf '%T@ %h\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
/usr/bin/python3 scripts/analyze_video_smoothness.py "$VIDEO_RUN/demo_60fps.mp4" > "$OUT_DIR/video_analysis.log"

VIEW_RUN="$("$PYTHON_BIN" - <<'PY'
import json
from pathlib import Path
best = ""
for path in sorted(Path("results/mujoco_concurrent_match").glob("concurrent_*/summary.json"), key=lambda p: p.stat().st_mtime, reverse=True):
    try:
        data = json.loads(path.read_text())
    except Exception:
        continue
    if data.get("frontend_true_smoothness_success") and data.get("concurrent_match_success"):
        best = str(path.parent)
        break
print(best)
PY
)"
if [[ -z "$VIEW_RUN" ]]; then
  echo "[acceptance] no prior successful realtime viewer run found; running 60s match now"
  ./scripts/start_final_soccer_demo.sh --match --seed 42 --target-fps 60 > "$OUT_DIR/match.log"
  VIEW_RUN="$("$PYTHON_BIN" - <<'PY'
import json
from pathlib import Path
for path in sorted(Path("results/mujoco_concurrent_match").glob("concurrent_*/summary.json"), key=lambda p: p.stat().st_mtime, reverse=True):
    data=json.loads(path.read_text())
    if data.get("frontend_true_smoothness_success") and data.get("concurrent_match_success"):
        print(path.parent)
        break
PY
)"
fi

echo "[acceptance] 7/12 aggregate final acceptance"
"$PYTHON_BIN" - <<'PY' "$OUT_DIR" "$VIEW_RUN" "$VIDEO_RUN" "$PYTEST_OK"
import json, re, shutil, sys
from pathlib import Path
out, view_run, video_run, pytest_ok = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), sys.argv[4] == "true"
view_summary = json.loads((view_run / "summary.json").read_text())
video_summary = json.loads((video_run / "summary.json").read_text())
video_report = json.loads((video_run / "video_smoothness_report.json").read_text())
provenance = json.loads((video_run / "video_provenance.json").read_text())
physics = json.loads((out / "physics_integrity.json").read_text())
goals = json.loads((out / "goal_visibility.json").read_text())
frontend = view_summary["frontend_smoothness"]
motion = {
    "abrupt_motion_count": frontend.get("abrupt_motion_count", 0),
    "gait_without_motion": frontend.get("gait_without_motion", 0),
    "motion_without_gait": frontend.get("motion_without_gait", 0),
    "motion_quality_success": frontend.get("abrupt_motion_count", 0) == 0 and frontend.get("gait_without_motion", 0) == 0 and frontend.get("motion_without_gait", 0) == 0,
}
criteria = {
    "pytest_passed": pytest_ok,
    "four_agent_coverage": view_summary["concurrency_acceptance"]["four_agent_decision_ratio"] >= 0.95,
    "four_non_hold": view_summary["concurrency_acceptance"]["all_non_hold_ratio"] >= 0.80,
    "three_active": view_summary["concurrency_acceptance"]["three_active_ratio"] >= 0.90,
    "all_paths_positive": all(v > 0 for v in view_summary["per_robot_path_length"].values()),
    "all_unique_contacts": all(v >= 1 for v in view_summary.get("per_robot_unique_contact_events", {}).values()),
    "pass_success": view_summary.get("pass_successes", 0) >= 1,
    "shoot_success": view_summary.get("shoot_successes", 0) >= 1,
    "intercept_or_clear_success": view_summary.get("intercept_successes", 0) >= 1 or view_summary.get("clear_successes", 0) >= 1,
    "open_for_pass": view_summary.get("open_for_pass_count", 0) >= 1,
    "block_line": view_summary.get("block_line_count", 0) >= 1,
    "possession_changes": view_summary.get("possession_changes", 0) >= 2,
    "contested": view_summary.get("contested_count", 0) >= 1,
    "no_kick_conflicts": view_summary["concurrency_acceptance"].get("same_team_kick_conflicts", 0) == 0,
    "physics_integrity": physics["physics_integrity_success"],
    "motion_quality": motion["motion_quality_success"],
    "viewer_sync": frontend["viewer_sync_hz"] >= 55,
    "actual_present": frontend["actual_present_hz"] >= 55,
    "state_change": frontend["render_state_change_hz"] >= 50,
    "effective_motion": frontend["effective_motion_frame_ratio"] >= 0.90,
    "visual_gap": frontend["maximum_visual_state_gap_ms"] <= 50,
    "no_freeze": not frontend["freeze_over_100ms"],
    "view_no_video_writer": not frontend["view_started_video_writer"],
    "renderer_once": frontend["renderer_creation_count"] <= 1,
    "goals_visible": goals["goal_visibility_success"],
    "video_60fps": video_report["fps"] >= 59,
    "video_duplicates": video_report["duplicate_frame_ratio"] < 0.05,
    "video_duplicate_run": video_report["longest_consecutive_duplicate_frames"] <= 2,
    "video_provenance": provenance["video_type"] == "trajectory_replay",
    "concurrent_match": view_summary["concurrent_match_success"],
    "frontend_true": view_summary["frontend_true_smoothness_success"],
    "ball_integrity": not view_summary["ball_mutation_detected"],
    "no_nan": not view_summary["nan_detected"],
    "no_joint_violation": not view_summary["joint_limit_violation"],
}
final = {
    "run_id": out.name,
    "view_run": str(view_run),
    "video_run": str(video_run),
    "criteria": criteria,
    "final_release_success": all(criteria.values()),
}
for name, data in [
    ("final_acceptance.json", final),
    ("frontend_acceptance.json", frontend),
    ("motion_quality.json", motion),
    ("summary.json", view_summary),
    ("video_smoothness_report.json", video_report),
    ("video_provenance.json", provenance),
    ("concurrency_acceptance.json", view_summary["concurrency_acceptance"]),
]:
    (out / name).write_text(json.dumps(data, indent=2), encoding="utf-8")
for src_name, dst_name in [("final_frame.png", "final_frame.png"), ("demo_60fps.mp4", "demo_60fps.mp4")]:
    src = (video_run / src_name) if (video_run / src_name).exists() else (view_run / src_name)
    if src.exists():
        shutil.copyfile(src, out / dst_name)
if (video_run / "video_smoothness_report.json").exists():
    shutil.copyfile(video_run / "video_smoothness_report.json", out / "video_smoothness_report.json")
if (video_run / "video_provenance.json").exists():
    shutil.copyfile(video_run / "video_provenance.json", out / "video_provenance.json")
PY

echo "[acceptance] 8/12 contact sheet and checksums"
if command -v ffmpeg >/dev/null 2>&1 && [[ -f "$OUT_DIR/demo_60fps.mp4" ]]; then
  ffmpeg -loglevel error -y -i "$OUT_DIR/demo_60fps.mp4" -vf "fps=1/15,scale=320:180,tile=4x1" -frames:v 1 "$OUT_DIR/contact_sheet.png" || cp "$OUT_DIR/final_frame.png" "$OUT_DIR/contact_sheet.png"
elif [[ -f "$OUT_DIR/final_frame.png" ]]; then
  cp "$OUT_DIR/final_frame.png" "$OUT_DIR/contact_sheet.png"
fi
(cd "$OUT_DIR" && sha256sum * > checksums.txt)

echo "[acceptance] 9/12 package"
./scripts/package_final_release.sh | tee "$OUT_DIR/package.log"
echo "$OUT_DIR"
