"""Public framework API for SoccerSim strategy developers.

Strategy code can usually import only from here:

- data classes such as :class:`PlayContext`, :class:`BallState`,
  :class:`RobotState`, :class:`Pose2D`, and :class:`GameControlState`
- command classes such as :class:`RobotCommand`, :class:`MoveIntent`,
  :class:`KickIntent`, :class:`StopIntent`, and :class:`NoopIntent`
- configuration classes such as :class:`SoccerConfig`,
  :class:`SoccerStrategyTuning`, :class:`SoccerDebugConfig`, and
  :class:`FieldDimensions`
- structured logging helpers such as :func:`create_soccer_logger` and
  :func:`create_soccer_telemetry`

Pure coordinate geometry helpers live in :mod:`src.tactics.geometry`. Use
submodules for ROS adapters, GameController JSON codec, and protocol constants.
"""

from __future__ import annotations

from .config import (
    SoccerConfig,
    SoccerDebugConfig,
    SoccerStrategyTuning,
)
from .types import (
    ADULT_FIELD_DIMENSIONS,
    BallState,
    CompetitionType,
    FieldDimensions,
    FieldMarker,
    GameControlState,
    GamePhase,
    GameState,
    KickIntent,
    MoveIntent,
    NoopIntent,
    Penalty,
    PlayerState,
    Pose2D,
    ReadySlot,
    RobotCommand,
    RobotIntent,
    RobotRuntimeStatus,
    RobotState,
    SetPlay,
    StopIntent,
    TeamState,
    PlayContext,
    PlayContextProvider,
)
from .telemetry import (
    JsonlTelemetry,
    NullTelemetry,
    SoccerLogger,
    TelemetrySink,
    StructuredLogPlugin,
    create_soccer_logger,
    create_soccer_telemetry,
)


__all__ = [
    "ADULT_FIELD_DIMENSIONS",
    "BallState",
    "CompetitionType",
    "FieldDimensions",
    "FieldMarker",
    "GameControlState",
    "GamePhase",
    "GameState",
    "JsonlTelemetry",
    "KickIntent",
    "MoveIntent",
    "NoopIntent",
    "NullTelemetry",
    "Penalty",
    "PlayerState",
    "Pose2D",
    "ReadySlot",
    "RobotCommand",
    "RobotIntent",
    "RobotRuntimeStatus",
    "RobotState",
    "SetPlay",
    "SoccerConfig",
    "SoccerDebugConfig",
    "SoccerLogger",
    "SoccerStrategyTuning",
    "StopIntent",
    "StructuredLogPlugin",
    "TeamState",
    "TelemetrySink",
    "PlayContext",
    "PlayContextProvider",
    "create_soccer_logger",
    "create_soccer_telemetry",
]
