"""Tactic target selection in a pure-function style, composed from split submodules.

This package only outputs :class:`Pose2D` targets and never dispatches commands.
BT leaves later pass these targets to :class:`MotionController` to produce :class:`RobotCommand`.

Module split:

stateless field predicates and player scoring
sideline recovery targets
chase/kick decisions and pass scoring
support positioning and teammate spacing
avoidance targets during opponent restarts

External callers use the thin :class:`Targeting` facade. Submodule dependencies:

predicates -> (TeamFieldFrame, geometry)
recovery   -> predicates
attack     -> predicates + recovery
support    -> predicates + attack (PlayerAllowed)
restart    -> predicates + recovery (BaseReadyTarget)

For a first change, start with :mod:`attack`: changing ``select_kick_target``
immediately changes attacking strategy without touching BT structure.
"""

from __future__ import annotations

from ...soccer_framework import (
    BallState,
    GameControlState,
    Pose2D,
    ReadySlot,
    SoccerConfig,
    PlayContext,
)
from ..navigation import Obstacle, ObstacleCollector
from ..geometry import TeamFieldFrame
from . import attack, predicates, recovery, restart, support
from .attack import PlayerAllowed
from .recovery import BaseReadyTarget


__all__ = [
    "BaseReadyTarget",
    "PlayerAllowed",
    "Targeting",
]


class Targeting:
    """Target-selection facade that forwards methods to pure functions in ``targeting.*`` submodules.

    Benefits of keeping this as a class:
    External code can keep calling ``team.targeting.xxx(...)`` with no API churn.
    ``config``
    Shared ``config`` / ``field`` / ``obstacles`` objects are captured once at construction.

    Submodules are pure functions and can be imported directly. Custom Playbooks
    usually need only one or two of them.
    """

    def __init__(
        self,
        config: SoccerConfig,
        field: TeamFieldFrame,
        obstacles: ObstacleCollector,
    ):
        self.config = config
        self.field = field
        self.obstacles = obstacles

    # Field predicates and player scoring

    def ball_in_own_defensive_area(self, ball: BallState) -> bool:
        return predicates.ball_in_own_defensive_area(self.config, ball)

    def ball_beyond_goal_line(self, ball: BallState) -> bool:
        return predicates.ball_beyond_goal_line(self.config, ball)

    def ball_beyond_own_goal_line(self, ball: BallState) -> bool:
        return predicates.ball_beyond_own_goal_line(self.config, ball)

    def ball_near_sideline(self, ball: BallState) -> bool:
        return predicates.ball_near_sideline(self.config, ball)

    def ball_is_in_midfield_or_own_half(self, ball: BallState) -> bool:
        return predicates.ball_is_in_midfield_or_own_half(self.config, ball)

    @staticmethod
    def sideline_sign(y: float) -> float:
        return predicates.sideline_sign(y)

    def ball_claim_score(
        self,
        slot: ReadySlot,
        pose: Pose2D,
        ball: BallState,
    ) -> float:
        return predicates.ball_claim_score(self.config, slot, pose, ball)

    def pose_for_slot(
        self,
        context: PlayContext,
        slot: ReadySlot,
    ) -> Pose2D | None:
        return predicates.pose_for_slot(self.config, context, slot)

    def side_should_challenge(
        self,
        context: PlayContext,
    ) -> bool:
        return predicates.side_should_challenge(self.config, context)

    # Sideline recovery

    def sideline_recovery_target(self, ball: BallState) -> Pose2D:
        return recovery.sideline_recovery_target(self.config, self.field, ball)

    # Chase/kick decisions and pass scoring

    def select_kick_target(
        self,
        player_id: int,
        context: PlayContext,
        is_player_allowed: PlayerAllowed,
    ) -> Pose2D:
        return attack.select_kick_target(
            self.config, self.field, self.obstacles,
            player_id, context, is_player_allowed,
        )

    def select_clear_or_pass_target(
        self,
        player_id: int,
        context: PlayContext,
        is_player_allowed: PlayerAllowed,
    ) -> Pose2D:
        return attack.select_clear_or_pass_target(
            self.config, self.field, self.obstacles,
            player_id, context, is_player_allowed,
        )

    def should_make_restart_touch(self, game: GameControlState) -> bool:
        return attack.should_make_restart_touch(self.config, game)

    def best_pass_target(
        self,
        player_id: int,
        context: PlayContext,
        is_player_allowed: PlayerAllowed,
    ) -> Pose2D | None:
        return attack.best_pass_target(
            self.config, self.obstacles,
            player_id, context, is_player_allowed,
        )

    def shot_lane_is_clear(
        self,
        context: PlayContext,
    ) -> bool:
        return attack.shot_lane_is_clear(
            self.config, self.field, self.obstacles, context,
        )

    def lane_clear_score(
        self,
        start_x: float,
        start_y: float,
        target_x: float,
        target_y: float,
        obstacles: tuple[Obstacle, ...],
    ) -> float:
        return attack.lane_clear_score(
            self.config, start_x, start_y, target_x, target_y, obstacles,
        )

    def dribble_target(self, ball: BallState) -> Pose2D:
        return attack.dribble_target(self.config, self.field, ball)

    def kick_reason(
        self,
        target: Pose2D,
        default: str = "chaser kick",
    ) -> str:
        return attack.kick_reason(self.config, target, default=default)

    # Support positioning

    def support_target(
        self,
        player_id: int,
        context: PlayContext,
        is_player_allowed: PlayerAllowed,
    ) -> Pose2D:
        return support.support_target(
            self.config, self.field, player_id, context,
            is_player_allowed,
        )

    # Restart avoidance

    def opponent_restart_target(
        self,
        player_id: int,
        slot: ReadySlot,
        context: PlayContext,
        base_ready_target: BaseReadyTarget,
    ) -> Pose2D:
        return restart.opponent_restart_target(
            self.config, self.field, player_id, slot, context,
            base_ready_target,
        )

    def opponent_restart_hold_vyaw(
        self,
        player_id: int,
        game: GameControlState,
    ) -> float:
        return restart.opponent_restart_hold_vyaw(self.config, player_id, game)
