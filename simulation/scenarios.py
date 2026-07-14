"""JSON scenario loading for the live 2D simulator."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, List

from common.config import FIELD_HEIGHT, FIELD_WIDTH
from common.world_state import Ball, Robot, Team


ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = ROOT / "scenarios"


class ScenarioValidationError(ValueError):
    """Raised when a scenario JSON file violates the project contract."""


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    duration_s: float
    seed: int
    ball: Ball
    blue: List[Robot]
    yellow: List[Robot]
    expect: Dict[str, Any]


def load_scenario(path_or_name: str) -> ScenarioConfig:
    path = _resolve_path(path_or_name)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ScenarioValidationError(f"$: invalid JSON: {exc}") from exc

    config = _parse(raw)
    validate_scenario(config)
    return config


def validate_scenario(config: ScenarioConfig) -> None:
    _require(config.name, "name", str)
    if not config.name:
        raise ScenarioValidationError("name: must not be empty")
    if not math.isfinite(config.duration_s) or config.duration_s <= 0:
        raise ScenarioValidationError("duration_s: must be > 0")
    if not isinstance(config.seed, int):
        raise ScenarioValidationError("seed: must be an integer")

    all_ids = [robot.id for robot in config.blue + config.yellow]
    if len(all_ids) != len(set(all_ids)):
        raise ScenarioValidationError("robots.id: duplicate robot id")

    for index, robot in enumerate(config.blue):
        if robot.id < 0 or robot.id > 9:
            raise ScenarioValidationError(f"blue[{index}].id: expected 0..9")
        _validate_robot(robot, f"blue[{index}]")
    for index, robot in enumerate(config.yellow):
        if robot.id < 10:
            raise ScenarioValidationError(f"yellow[{index}].id: expected >= 10")
        _validate_robot(robot, f"yellow[{index}]")
    _validate_ball(config.ball, "ball")

    for key in ("passer_id", "receiver_id"):
        if key in config.expect and config.expect[key] not in all_ids:
            raise ScenarioValidationError(f"expect.{key}: robot id does not exist")


def load_scenario_into_simulator(simulator, path_or_name: str) -> ScenarioConfig:
    config = load_scenario(path_or_name)
    simulator.load_state(config.ball, config.blue, config.yellow)
    return config


def _resolve_path(path_or_name: str) -> Path:
    path = Path(path_or_name)
    if path.is_file():
        return path
    name = path_or_name
    if not name.endswith(".json"):
        name = f"{name}.json"
    candidate = SCENARIO_DIR / name
    if candidate.is_file():
        return candidate
    raise ScenarioValidationError(f"scenario: not found: {path_or_name}")


def _parse(raw: Dict[str, Any]) -> ScenarioConfig:
    if not isinstance(raw, dict):
        raise ScenarioValidationError("$: expected object")
    try:
        ball = _parse_ball(raw["ball"], "ball")
        blue = [_parse_robot(item, Team.BLUE, f"blue[{i}]") for i, item in enumerate(raw["blue"])]
        yellow = [_parse_robot(item, Team.YELLOW, f"yellow[{i}]") for i, item in enumerate(raw["yellow"])]
        return ScenarioConfig(
            name=raw["name"],
            duration_s=float(raw["duration_s"]),
            seed=int(raw["seed"]),
            ball=ball,
            blue=blue,
            yellow=yellow,
            expect=dict(raw.get("expect", {})),
        )
    except KeyError as exc:
        raise ScenarioValidationError(f"{exc.args[0]}: missing required field") from exc
    except (TypeError, ValueError) as exc:
        raise ScenarioValidationError(f"$: invalid field type: {exc}") from exc


def _parse_ball(raw: Dict[str, Any], path: str) -> Ball:
    _require(raw, path, dict)
    return Ball(
        x=float(raw["x"]),
        y=float(raw["y"]),
        vx=float(raw.get("vx", 0.0)),
        vy=float(raw.get("vy", 0.0)),
    )


def _parse_robot(raw: Dict[str, Any], team: Team, path: str) -> Robot:
    _require(raw, path, dict)
    return Robot(
        id=int(raw["id"]),
        team=team,
        x=float(raw["x"]),
        y=float(raw["y"]),
        theta=float(raw.get("theta", 0.0)),
    )


def _validate_ball(ball: Ball, path: str) -> None:
    for attr in ("x", "y", "vx", "vy"):
        _finite(getattr(ball, attr), f"{path}.{attr}")
    _in_field(ball.x, ball.y, path)


def _validate_robot(robot: Robot, path: str) -> None:
    for attr in ("x", "y", "theta"):
        _finite(getattr(robot, attr), f"{path}.{attr}")
    _in_field(robot.x, robot.y, path)


def _in_field(x: float, y: float, path: str) -> None:
    if not (-FIELD_WIDTH / 2 <= x <= FIELD_WIDTH / 2):
        raise ScenarioValidationError(f"{path}.x: out of field")
    if not (-FIELD_HEIGHT / 2 <= y <= FIELD_HEIGHT / 2):
        raise ScenarioValidationError(f"{path}.y: out of field")


def _finite(value: float, path: str) -> None:
    if not math.isfinite(value):
        raise ScenarioValidationError(f"{path}: must be finite")


def _require(value: Any, path: str, expected_type: type) -> None:
    if not isinstance(value, expected_type):
        raise ScenarioValidationError(f"{path}: expected {expected_type.__name__}")
