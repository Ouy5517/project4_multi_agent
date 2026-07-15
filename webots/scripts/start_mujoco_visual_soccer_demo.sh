#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/coop_env/bin/python"
if [[ ! -x "$PY" ]]; then
  PY="python3"
fi

cd "$ROOT"
RUN_ID="full_visual_demo_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$ROOT/results/mujoco_four_robot_demo/$RUN_ID"
MODEL="$ROOT/mujoco_soccer/models/t1_2v2_soccer.xml"

echo "[visual-demo] Python: $("$PY" --version 2>&1)"
"$PY" - <<'PY'
import mujoco
print(f"[visual-demo] MuJoCo: {mujoco.__version__}")
PY

if [[ ! -f "$MODEL" ]]; then
  echo "[visual-demo] Model missing, regenerating: $MODEL"
  "$PY" -m mujoco_soccer.tools_generate_proxy_model >/dev/null
fi

echo "[visual-demo] DISPLAY=${DISPLAY:-<unset>} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>}"
echo "[visual-demo] Starting MuJoCo GUI viewer. Close the viewer window to stop early."

export MUJOCO_GL="${MUJOCO_GL:-glfw}"
"$PY" -m mujoco_soccer.run_demo \
  --mode visual-full-demo \
  --visual \
  --run-id "$RUN_ID" 2>&1 | tee "$ROOT/visual_demo_${RUN_ID}.log"

if [[ ! -f "$RUN_DIR/summary.json" ]]; then
  echo "[visual-demo] Missing summary.json; visual run failed before completion." >&2
  exit 1
fi

if [[ ! -f "$RUN_DIR/final_frame.png" ]]; then
  echo "[visual-demo] final_frame.png was not produced by recorder; writing fallback screenshot."
  MUJOCO_GL=egl "$PY" - <<PY
from pathlib import Path
import json
import imageio.v2 as imageio
import mujoco
run_dir = Path("$RUN_DIR")
model = mujoco.MjModel.from_xml_path("$MODEL")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=720, width=1280)
renderer.update_scene(data, camera="overview")
out = run_dir / "final_frame.png"
imageio.imwrite(out, renderer.render())
summary_path = run_dir / "summary.json"
summary = json.loads(summary_path.read_text())
summary["final_frame"] = str(out)
summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
PY
fi

echo "[visual-demo] Result directory: $RUN_DIR"
"$PY" - <<PY
import json
from pathlib import Path
summary = json.loads((Path("$RUN_DIR") / "summary.json").read_text())
print("[visual-demo] demo_success:", summary.get("demo_success"))
print("[visual-demo] final_frame:", summary.get("final_frame"))
print("[visual-demo] video_or_frames:", summary.get("video_path"))
print("[visual-demo] failure_reason:", summary.get("failure_reason"))
PY
