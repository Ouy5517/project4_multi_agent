#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

cmd="${1:-demo}"
shift || true

case "$cmd" in
  demo)
    python main.py --scenario pass_fixed --headless --fast --strict "$@"
    ;;
  test)
    python -m pytest -q "$@"
    ;;
  acceptance)
    bash scripts/run_acceptance.sh "$@"
    ;;
  record)
    python main.py --scenario pass_fixed --headless --fast --strict "$@"
    ;;
  view)
    python main.py --scenario showcase --viewer mujoco --duration 70 "$@"
    ;;
  *)
    python main.py "$cmd" "$@"
    ;;
esac
