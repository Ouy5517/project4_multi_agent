#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${ROBOCUP_VENV:-$HOME/.venvs/robocup-p4}"

if ! grep -q 'Ubuntu 22.04' /etc/os-release; then
  echo "ERROR: Ubuntu 22.04 is required." >&2
  exit 1
fi

python3 - <<'PY'
import sys
if sys.version_info < (3, 10):
    raise SystemExit("ERROR: Python 3.10+ is required.")
PY

if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "ERROR: python3-venv is missing. Run: sudo apt install -y python3-venv python3-pip ffmpeg" >&2
  exit 1
fi

python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r "$ROOT/requirements-dev.txt"
mkdir -p "$ROOT/outputs"

echo "WSL development environment ready: $VENV"
