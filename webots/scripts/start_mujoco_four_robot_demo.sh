#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/coop_env/bin/python"
if [[ ! -x "$PY" ]]; then
  PY="python3"
fi

cd "$ROOT"
RUN_ID="mujoco_four_robot_$(date +%Y%m%d_%H%M%S)"
"$PY" -m mujoco_soccer.tools_generate_proxy_model >/dev/null
"$PY" -m pytest tests/test_mujoco_four_robot_demo.py -q
"$PY" -m mujoco_soccer.run_demo --mode full-demo --run-id "$RUN_ID" --no-render
MUJOCO_GL=egl "$PY" - <<PY
from pathlib import Path
import imageio.v2 as imageio
import mujoco
run_dir = Path("results/mujoco_four_robot_demo") / "$RUN_ID"
model = mujoco.MjModel.from_xml_path("mujoco_soccer/models/t1_2v2_soccer.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=720, width=1280)
renderer.update_scene(data, camera="overview")
imageio.imwrite(run_dir / "final_frame.png", renderer.render())
PY
echo "MuJoCo run directory: $ROOT/results/mujoco_four_robot_demo/$RUN_ID"

