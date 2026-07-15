"""GameController state JSON codec.

Converts between the GC topic JSON representation and :class:`GameControlState`.
The logic has no ROS or boosteros dependency; it is only field mapping plus robust parsing.
"""

from __future__ import annotations

import json
from collections.abc import Mapping

from .types import (
    GAME_CONTROLLER_STATE_VERSION,
    KICKING_TEAM_NONE,
    CompetitionType,
    GameControlState,
    GamePhase,
    GameState,
    Penalty,
    PlayerState,
    SetPlay,
    TeamState,
)


__all__ = [
    "game_control_state_from_dict",
    "game_control_state_from_json",
    "game_control_state_to_dict",
    "game_control_state_to_json",
    "player_state_from_dict",
    "player_state_to_dict",
    "team_state_from_dict",
    "team_state_to_dict",
]


def game_control_state_to_json(game_state: GameControlState) -> str:
    return json.dumps(
        game_control_state_to_dict(game_state),
        separators=(",", ":"),
        sort_keys=True,
    )


def game_control_state_from_json(data: str) -> GameControlState:
    try:
        payload = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid GameControlState JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("GameControlState JSON must be an object")
    return game_control_state_from_dict(payload)


def game_control_state_to_dict(game_state: GameControlState) -> dict[str, object]:
    return {
        "version": game_state.version,
        "packetNumber": game_state.packet_number,
        "playersPerTeam": game_state.players_per_team,
        "competitionType": game_state.competition_type.value,
        "stopped": game_state.stopped,
        "gamePhase": game_state.game_phase.value,
        "state": game_state.state.value,
        "setPlay": game_state.set_play.value,
        "firstHalf": game_state.first_half,
        "kickingTeam": game_state.kicking_team,
        "secsRemaining": game_state.secs_remaining,
        "secondaryTime": game_state.secondary_time,
        "teams": [team_state_to_dict(team) for team in game_state.teams],
    }


def game_control_state_from_dict(payload: Mapping[str, object]) -> GameControlState:
    teams_payload = payload.get("teams", [])
    if not isinstance(teams_payload, list):
        raise ValueError("GameControlState.teams must be a list")
    return GameControlState(
        version=_json_int(payload, "version", GAME_CONTROLLER_STATE_VERSION),
        packet_number=_json_int(payload, "packetNumber", 0),
        players_per_team=_json_int(payload, "playersPerTeam", 0),
        competition_type=CompetitionType(
            _json_str(payload, "competitionType", CompetitionType.MIDDLE.value)
        ),
        stopped=_json_bool(payload, "stopped", False),
        game_phase=GamePhase(_json_str(payload, "gamePhase", GamePhase.NORMAL.value)),
        state=GameState(_json_str(payload, "state", GameState.INITIAL.value)),
        set_play=SetPlay(_json_str(payload, "setPlay", SetPlay.NONE.value)),
        first_half=_json_bool(payload, "firstHalf", True),
        kicking_team=_json_int(payload, "kickingTeam", KICKING_TEAM_NONE),
        secs_remaining=_json_int(payload, "secsRemaining", 0),
        secondary_time=_json_int(payload, "secondaryTime", 0),
        teams=[team_state_from_dict(team) for team in teams_payload],
    )


def team_state_to_dict(team: TeamState) -> dict[str, object]:
    return {
        "teamNumber": team.team_number,
        "fieldPlayerColour": team.field_player_colour,
        "goalkeeperColour": team.goalkeeper_colour,
        "goalkeeper": team.goalkeeper,
        "score": team.score,
        "penaltyShot": team.penalty_shot,
        "singleShots": team.single_shots,
        "messageBudget": team.message_budget,
        "players": [player_state_to_dict(player) for player in team.players],
    }


def team_state_from_dict(payload: object) -> TeamState:
    if not isinstance(payload, Mapping):
        raise ValueError("TeamState JSON item must be an object")
    players_payload = payload.get("players", [])
    if not isinstance(players_payload, list):
        raise ValueError("TeamState.players must be a list")
    return TeamState(
        team_number=_json_int(payload, "teamNumber", 1),
        field_player_colour=_json_int(payload, "fieldPlayerColour", 0),
        goalkeeper_colour=_json_int(payload, "goalkeeperColour", 0),
        goalkeeper=_json_int(payload, "goalkeeper", 0),
        score=_json_int(payload, "score", 0),
        penalty_shot=_json_int(payload, "penaltyShot", 0),
        single_shots=_json_int(payload, "singleShots", 0),
        message_budget=_json_int(payload, "messageBudget", 0),
        players=[player_state_from_dict(player) for player in players_payload],
    )


def player_state_to_dict(player: PlayerState) -> dict[str, object]:
    return {
        "penalty": player.penalty.value,
        "secsTillUnpenalised": player.secs_till_unpenalised,
        "warnings": player.warnings,
        "cautions": player.cautions,
    }


def player_state_from_dict(payload: object) -> PlayerState:
    if not isinstance(payload, Mapping):
        raise ValueError("PlayerState JSON item must be an object")
    return PlayerState(
        penalty=Penalty(_json_str(payload, "penalty", Penalty.NONE.value)),
        secs_till_unpenalised=_json_int(payload, "secsTillUnpenalised", 0),
        warnings=_json_int(payload, "warnings", 0),
        cautions=_json_int(payload, "cautions", 0),
    )


def _json_int(payload: Mapping[str, object], key: str, default: int) -> int:
    value = payload.get(key, default)
    if isinstance(value, (int, float, str)):
        return int(value)
    raise ValueError(f"{key} must be an int-compatible value")


def _json_str(payload: Mapping[str, object], key: str, default: str) -> str:
    value = payload.get(key, default)
    if isinstance(value, (str, int, float, bool)):
        return str(value)
    raise ValueError(f"{key} must be a string-compatible value")


def _json_bool(payload: Mapping[str, object], key: str, default: bool) -> bool:
    value = payload.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    raise ValueError(f"{key} must be a bool-compatible value")
