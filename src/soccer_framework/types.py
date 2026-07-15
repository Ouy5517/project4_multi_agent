"""Soccer data types.

BallState / RobotState / PlayContext /
GameControlState
Pure data-contract layer: enums, dataclasses such as Pose2D / BallState /
RobotState / PlayContext / GameControlState / Intent / RobotCommand, plus the
PlayContextProvider abstract base class.

This module intentionally avoids ROS and boosteros dependencies so it can be
syntax-checked and unit-tested alone. See ``config.py`` for config, ``game_state.py``
for GameController JSON codec, and ``tactics/geometry.py`` for Pose2D geometry helpers.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum


__all__ = [
    "ADULT_FIELD_DIMENSIONS",
    "GAME_CONTROLLER_STATE_VERSION",
    "KICKING_TEAM_NONE",
    "MAX_NUM_PLAYERS",
    "BallState",
    "CompetitionType",
    "FieldDimensions",
    "FieldMarker",
    "GameControlState",
    "GamePhase",
    "GameState",
    "KickIntent",
    "MoveIntent",
    "NoopIntent",
    "Penalty",
    "PlayerState",
    "Pose2D",
    "RobotCommand",
    "RobotIntent",
    "RobotRuntimeStatus",
    "RobotState",
    "ReadySlot",
    "SetPlay",
    "StopIntent",
    "TeamState",
    "PlayContext",
    "PlayContextProvider",
]


GAME_CONTROLLER_STATE_VERSION = 19
MAX_NUM_PLAYERS = 20
KICKING_TEAM_NONE = 255


class ReadySlot(str, Enum):
    CENTER = "center"
    SIDE = "side"
    KEEPER = "keeper"


@dataclass(frozen=True)
class FieldMarker:
    marker_type: str
    x: float
    y: float


@dataclass(frozen=True)
class FieldDimensions:
    length: float
    width: float
    penalty_dist: float
    goal_width: float
    circle_radius: float
    penalty_area_length: float
    penalty_area_width: float
    goal_area_length: float
    goal_area_width: float

    def markers(self) -> tuple[FieldMarker, ...]:
        markers: list[FieldMarker] = []
        half_length = self.length / 2.0
        half_width = self.width / 2.0

        markers.append(FieldMarker("X", 0.0, -self.circle_radius))
        markers.append(FieldMarker("X", 0.0, self.circle_radius))
        markers.append(FieldMarker("P", half_length - self.penalty_dist, 0.0))
        markers.append(FieldMarker("P", -half_length + self.penalty_dist, 0.0))
        markers.append(FieldMarker("T", 0.0, half_width))
        markers.append(FieldMarker("T", 0.0, -half_width))

        for sign_x in (1.0, -1.0):
            for sign_y in (1.0, -1.0):
                markers.append(
                    FieldMarker(
                        "L",
                        sign_x * (half_length - self.penalty_area_length),
                        sign_y * self.penalty_area_width / 2.0,
                    )
                )
                markers.append(
                    FieldMarker(
                        "T",
                        sign_x * half_length,
                        sign_y * self.penalty_area_width / 2.0,
                    )
                )
                markers.append(
                    FieldMarker(
                        "L",
                        sign_x * (half_length - self.goal_area_length),
                        sign_y * self.goal_area_width / 2.0,
                    )
                )
                markers.append(
                    FieldMarker(
                        "T",
                        sign_x * half_length,
                        sign_y * self.goal_area_width / 2.0,
                    )
                )

        markers.append(FieldMarker("L", half_length, half_width))
        markers.append(FieldMarker("L", half_length, -half_width))
        markers.append(FieldMarker("L", -half_length, half_width))
        markers.append(FieldMarker("L", -half_length, -half_width))
        return tuple(markers)


ADULT_FIELD_DIMENSIONS = FieldDimensions(
    length=14.0,
    width=9.0,
    penalty_dist=2.1,
    goal_width=2.6,
    circle_radius=1.5,
    penalty_area_length=3.0,
    penalty_area_width=6.0,
    goal_area_length=1.0,
    goal_area_width=4.0,
)


class CompetitionType(str, Enum):
    SMALL = "SMALL"
    MIDDLE = "MIDDLE"
    LARGE = "LARGE"


class GamePhase(str, Enum):
    NORMAL = "NORMAL"
    PENALTY_SHOOT_OUT = "PENALTY_SHOOT_OUT"
    EXTRA_TIME = "EXTRA_TIME"
    TIMEOUT = "TIMEOUT"


class GameState(str, Enum):
    INITIAL = "INITIAL"
    READY = "READY"
    SET = "SET"
    PLAYING = "PLAYING"
    FINISHED = "FINISHED"


class SetPlay(str, Enum):
    NONE = "NONE"
    DIRECT_FREE_KICK = "DIRECT_FREE_KICK"
    INDIRECT_FREE_KICK = "INDIRECT_FREE_KICK"
    PENALTY_KICK = "PENALTY_KICK"
    THROW_IN = "THROW_IN"
    GOAL_KICK = "GOAL_KICK"
    CORNER_KICK = "CORNER_KICK"


class Penalty(str, Enum):
    NONE = "NONE"
    ILLEGAL_POSITIONING = "ILLEGAL_POSITIONING"
    MOTION_IN_SET = "MOTION_IN_SET"
    LOCAL_GAME_STUCK = "LOCAL_GAME_STUCK"
    INCAPABLE_ROBOT = "INCAPABLE_ROBOT"
    PICKED_UP = "PICKED_UP"
    BALL_HOLDING = "BALL_HOLDING"
    LEAVING_THE_FIELD = "LEAVING_THE_FIELD"
    PLAYING_WITH_ARMS_HANDS = "PLAYING_WITH_ARMS_HANDS"
    PUSHING = "PUSHING"
    SENT_OFF = "SENT_OFF"
    SUBSTITUTE = "SUBSTITUTE"


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class BallState:
    x: float = 0.0
    y: float = 0.0
    last_seen_at: float = 0.0
    confidence: float = 1.0

    def is_recent(self, now: float, max_age_sec: float = 1.5) -> bool:
        return self.last_seen_at > 0.0 and now - self.last_seen_at <= max_age_sec


@dataclass
class PlayerState:
    penalty: Penalty = Penalty.NONE
    secs_till_unpenalised: int = 0
    warnings: int = 0
    cautions: int = 0


@dataclass
class TeamState:
    team_number: int = 1
    field_player_colour: int = 0
    goalkeeper_colour: int = 0
    goalkeeper: int = 0
    score: int = 0
    penalty_shot: int = 0
    single_shots: int = 0
    message_budget: int = 0
    players: list[PlayerState] = field(
        default_factory=lambda: [PlayerState() for _ in range(MAX_NUM_PLAYERS)]
    )


@dataclass
class GameControlState:
    version: int = GAME_CONTROLLER_STATE_VERSION
    packet_number: int = 0
    players_per_team: int = 0
    competition_type: CompetitionType = CompetitionType.MIDDLE
    stopped: bool = False
    game_phase: GamePhase = GamePhase.NORMAL
    state: GameState = GameState.INITIAL
    set_play: SetPlay = SetPlay.NONE
    first_half: bool = True
    kicking_team: int = KICKING_TEAM_NONE
    secs_remaining: int = 0
    secondary_time: int = 0
    teams: list[TeamState] = field(
        default_factory=lambda: [TeamState(team_number=1), TeamState(team_number=2)]
    )
    last_seen_at: float = 0.0

    def is_recent(self, now: float, max_age_sec: float = 2.0) -> bool:
        """Return whether this state was received within ``max_age_sec``.

        ``last_seen_at == 0.0`` means it was never written by a topic callback,
        often in tests. Return True to match the old ``last_topic_at <= 0.0`` behavior,
        even though this differs from ``BallState.is_recent``.
        """

        if self.last_seen_at <= 0.0:
            return True
        return now - self.last_seen_at <= max_age_sec

    def has_kicking_team(self) -> bool:
        return self.kicking_team != KICKING_TEAM_NONE

    def is_kickoff_for_team(self, team_id: int) -> bool:
        return self.set_play == SetPlay.NONE and self.kicking_team == team_id

    def is_restart_for_team(self, team_id: int) -> bool:
        return self.has_kicking_team() and self.kicking_team == team_id

    def get_team_state(self, team_id: int) -> TeamState | None:
        for team in self.teams:
            if team.team_number == team_id:
                return team
        return None

    def get_player_state(self, team_id: int, player_id: int) -> PlayerState | None:
        team = self.get_team_state(team_id)
        if team is None or player_id < 1 or player_id > len(team.players):
            return None
        return team.players[player_id - 1]

    def is_active_player(self, team_id: int, player_id: int) -> bool:
        player = self.get_player_state(team_id, player_id)
        return (
            player is not None
            and player.penalty == Penalty.NONE
            and player.secs_till_unpenalised <= 0
        )


@dataclass
class RobotState:
    player_id: int
    pose: Pose2D | None = None
    last_seen_at: float = 0.0

    def is_recent(self, now: float, max_age_sec: float = 2.0) -> bool:
        """Return whether this robot's pose was seen within ``max_age_sec``."""

        return self.last_seen_at > 0.0 and now - self.last_seen_at <= max_age_sec


@dataclass
class RobotRuntimeStatus:
    mode: str | None = None
    fall_down_state: str | None = None
    fall_down_recoverable: bool = False
    updated_at: float = 0.0

    @property
    def is_fall_down_normal(self) -> bool:
        return self.fall_down_state in {None, "normal"}

    @property
    def has_fallen(self) -> bool:
        return self.fall_down_state == "fallen"


@dataclass
class PlayContext:
    """Externally observed match context containing only things that were seen.

    A robot's own hardware state, such as mode or fall_down, is not part of the
    observed context. The BT DataLayer pulls it directly from :class:`TeamRobotManager`
    and stores it on the blackboard instead.
    """

    game_state: GameControlState | None = None
    teammates: dict[int, RobotState] = field(default_factory=dict)
    opponents: dict[int, RobotState] = field(default_factory=dict)
    ball: BallState | None = None

    @property
    def game(self) -> GameControlState | None:
        """Current GameController state, None if stale or filtered."""

        return self.game_state

    @property
    def known_game(self) -> GameControlState:
        """Return ``game_state`` after narrowing it to a present value."""

        if self.game_state is None:
            raise ValueError("PlayContext.game_state is required for this decision")
        return self.game_state

    @property
    def known_ball(self) -> BallState:
        """Return ``ball`` after narrowing it to a present value."""

        if self.ball is None:
            raise ValueError("PlayContext.ball is required for this decision")
        return self.ball


@dataclass(frozen=True)
class MoveIntent:
    """Chassis velocity intent ``(vx, vy, vyaw)`` for this tick.

    The robot is a biped, so ``vx + vyaw`` is the stable combination; navigation forces lateral ``vy`` to 0.
    """

    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


@dataclass(frozen=True)
class KickIntent:
    """Kick intent forwarded by :class:`PlayerKickStateMachine` to SoccerKickManager.

    ``ball_x``
    ``ball_x`` and ``ball_y`` are the ball position in robot-body coordinates and are required.
    """

    direction: float
    power: float
    ball_x: float
    ball_y: float


@dataclass(frozen=True)
class StopIntent:
    """Stop intent: no chassis motion or kick this tick; ``reason`` explains why."""


@dataclass(frozen=True)
class NoopIntent:
    """No-op intent: do not touch this robot's hardware interface this tick."""


RobotIntent = MoveIntent | KickIntent | StopIntent | NoopIntent


@dataclass(frozen=True)
class RobotCommand:
    """Final per-player command produced by one BT tick; ``intent`` selects the execution channel.

    ``reason`` is shared explanatory text across intents, mainly for logs and telemetry.

    Dispatch with checks such as ``isinstance(command.intent, KickIntent)``. BT
    leaves construct intent dataclasses directly, and types enforce required kick ``ball_x/y``.
    """

    intent: RobotIntent = StopIntent()
    reason: str = "stop"

    @classmethod
    def stop(cls, reason: str) -> "RobotCommand":
        return cls(intent=StopIntent(), reason=reason)

    @classmethod
    def noop(cls, reason: str) -> "RobotCommand":
        return cls(intent=NoopIntent(), reason=reason)


class PlayContextProvider(ABC):
    @abstractmethod
    def start(self) -> None:
        """Start provider resources."""

    @abstractmethod
    def stop(self) -> None:
        """Stop provider resources."""

    @abstractmethod
    def get_snapshot(self) -> PlayContext:
        """Return a consistent snapshot for strategy code."""

    @abstractmethod
    def set_game_state(self, game_state: GameControlState) -> None:
        """Update the provider with the latest GameController state."""
