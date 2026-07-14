#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python main.py --scenario pass_fixed --headless --fast --strict
python main.py --scenario dribble_open --headless --fast --strict
python main.py --scenario position_block --headless --fast --strict
