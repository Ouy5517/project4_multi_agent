from __future__ import annotations

from dataclasses import dataclass


STAGES = [
    "STAGE_00_READY",
    "STAGE_01_VISIBLE_ALL_RUN",
    "STAGE_02_RED2_BLOCK_PASS",
    "STAGE_03_BLUE1_DRIBBLE_1",
    "STAGE_04_BLUE1_DRIBBLE_2",
    "STAGE_05_RED2_LEAVE_LINE",
    "STAGE_06_BLUE1_PASS",
    "STAGE_07_BLUE2_RECEIVE",
    "STAGE_08_BLUE2_SHOOT",
    "STAGE_09_RED1_INTERCEPT_CLEAR",
    "STAGE_10_RED2_COUNTER",
    "STAGE_11_FINAL_FORMATION",
    "STAGE_12_DONE",
]


@dataclass
class StageStatus:
    name: str
    timeout: float
    max_retries: int = 1
    path_budget: float = 4.0


def default_stage_statuses() -> dict[str, StageStatus]:
    return {
        stage: StageStatus(stage, 10.0 if "RUN" in stage else 8.0, max_retries=1)
        for stage in STAGES
    }

