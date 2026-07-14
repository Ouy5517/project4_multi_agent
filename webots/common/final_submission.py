from __future__ import annotations

import math
import os
import re
import secrets
from datetime import datetime
from pathlib import Path
from typing import Any


RUN_ID_RE = re.compile(r"^\d{8}_\d{6}_[0-9a-f]{6}$")


def make_run_id() -> str:
    return f"{datetime.now():%Y%m%d_%H%M%S}_{secrets.token_hex(3)}"


def horizontal_displacement(
    initial: list[float] | tuple[float, float] | None,
    final: list[float] | tuple[float, float] | None,
) -> float | None:
    if not initial or not final or len(initial) < 2 or len(final) < 2:
        return None
    return math.hypot(float(final[0]) - float(initial[0]), float(final[1]) - float(initial[1]))


def direction_error(
    initial: list[float] | tuple[float, float] | None,
    final: list[float] | tuple[float, float] | None,
    target: list[float] | tuple[float, float] | None,
) -> float | None:
    if not initial or not final or not target:
        return None
    move = (float(final[0]) - float(initial[0]), float(final[1]) - float(initial[1]))
    aim = (float(target[0]) - float(initial[0]), float(target[1]) - float(initial[1]))
    if math.hypot(*move) < 1e-9 or math.hypot(*aim) < 1e-9:
        return None
    return abs(math.atan2(math.sin(math.atan2(move[1], move[0]) - math.atan2(aim[1], aim[0])),
                          math.cos(math.atan2(move[1], move[0]) - math.atan2(aim[1], aim[0]))))


def compute_dribble_success(summary: dict[str, Any], threshold: float = 0.05) -> dict[str, Any]:
    initial = summary.get("ball_initial_position")
    final = summary.get("ball_final_position")
    disp = horizontal_displacement(initial, final)
    summary["ball_displacement"] = disp
    summary["dribble_success"] = bool(disp is not None and disp > threshold)
    if disp is None:
        summary["failure_reason"] = "missing ball coordinates"
    elif disp <= threshold:
        summary["failure_reason"] = f"ball displacement {disp:.4f}m <= {threshold:.2f}m"
    else:
        summary.setdefault("failure_reason", None)
    return summary


def contains_supervisor_ball_move(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    forbidden = (
        "SOCCER_BALL" in text
        and (
            ".getField(\"translation\").set" in text
            or ".getField('translation').set" in text
            or "setSFVec3f" in text
        )
    )
    return forbidden


def newest_jsonl_state(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    latest = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                item = __import__("json").loads(line)
            except Exception:
                continue
            if item.get("event") == "STATE":
                latest = item
    return latest


def env_flag(name: str, default: str = "") -> str:
    return os.environ.get(name, default)
