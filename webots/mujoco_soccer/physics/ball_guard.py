from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco


FORBIDDEN_PATTERNS = [
    "data.qpos[ball",
    "data.qvel[ball",
    "mj_applyFT",
    "xfrc_applied",
]


@dataclass
class BallGuard:
    model: mujoco.MjModel
    project_root: Path
    mutation_detected: bool = False

    def __post_init__(self) -> None:
        ball_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "soccer_ball_free")
        self.ball_qposadr = int(self.model.jnt_qposadr[ball_joint])
        self.ball_dofadr = int(self.model.jnt_dofadr[ball_joint])

    def scan_sources(self) -> list[str]:
        hits: list[str] = []
        for path in (self.project_root / "mujoco_soccer").rglob("*.py"):
            if path.name == "ball_guard.py":
                continue
            text = path.read_text(encoding="utf-8")
            for pattern in FORBIDDEN_PATTERNS:
                if pattern in text:
                    hits.append(f"{path}:{pattern}")
        return hits

    def update(self, contact_seen: bool, ball_speed: float) -> None:
        if not contact_seen and ball_speed > 4.5:
            self.mutation_detected = True

