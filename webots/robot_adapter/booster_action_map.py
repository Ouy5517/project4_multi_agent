from __future__ import annotations

from dataclasses import dataclass

from common.robot_action import ActionType


@dataclass(frozen=True)
class BoosterActionMapping:
    """策略动作到 Webots / Booster T1 高层动作的说明性映射。"""

    high_level_action: str
    description: str
    example_action: str
    suggested_input: str


BOOSTER_ACTION_MAP: dict[ActionType, BoosterActionMapping] = {
    ActionType.PASS: BoosterActionMapping(
        high_level_action="PASS",
        description="面向队友方向，短距离推球或踢球。",
        example_action="先转向目标队友，再低速前进或执行 kick 踢球动作。",
        suggested_input="a/d 转向目标，然后 w 低速推进或 kick",
    ),
    ActionType.MOVE_TO_RECEIVE: BoosterActionMapping(
        high_level_action="MOVE_TO_RECEIVE",
        description="移动到接球区域。",
        example_action="根据目标点方向选择 w/a/s/d 移动到接球点。",
        suggested_input="w/a/s/d",
    ),
    ActionType.MOVE_TO_SUPPORT: BoosterActionMapping(
        high_level_action="MOVE_TO_SUPPORT",
        description="移动到支援区域，等待下一次传接窗口。",
        example_action="根据支援点方向选择 w/a/s/d 移动。",
        suggested_input="w/a/s/d",
    ),
    ActionType.DRIBBLE: BoosterActionMapping(
        high_level_action="DRIBBLE",
        description="带球向敌方球门方向推进。",
        example_action="保持面向推进方向，低速 w 前进。",
        suggested_input="w",
    ),
    ActionType.SHOOT: BoosterActionMapping(
        high_level_action="SHOOT",
        description="面向球门后射门。",
        example_action="先转向球门，再执行 kick 或 Soccer Agent 踢球动作。",
        suggested_input="a/d 转向球门，然后 kick",
    ),
    ActionType.MARK_OPPONENT: BoosterActionMapping(
        high_level_action="MARK_OPPONENT",
        description="移动到对手和我方球门之间进行卡位。",
        example_action="移动到防守站位，保持在对手射门线路上。",
        suggested_input="w/a/s/d",
    ),
    ActionType.CHASE_BALL: BoosterActionMapping(
        high_level_action="CHASE_BALL",
        description="追球。",
        example_action="朝球的位置移动。",
        suggested_input="w/a/s/d",
    ),
    ActionType.STOP: BoosterActionMapping(
        high_level_action="STOP",
        description="停止。",
        example_action="停止移动。",
        suggested_input="l",
    ),
}


def get_booster_action_mapping(action_type: ActionType) -> BoosterActionMapping:
    """返回动作的说明性映射；不会调用真实 Booster SDK。"""
    return BOOSTER_ACTION_MAP[action_type]


def map_to_booster_command(action_type: ActionType) -> str:
    """预留 Booster SDK 动作映射入口，当前不调用真实 SDK。"""
    return BOOSTER_ACTION_MAP[action_type].high_level_action
