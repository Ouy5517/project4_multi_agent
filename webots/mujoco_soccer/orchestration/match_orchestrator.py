from __future__ import annotations

from mujoco_soccer.orchestration.stage_machine import STAGES


def full_stage_list() -> list[str]:
    return list(STAGES)
