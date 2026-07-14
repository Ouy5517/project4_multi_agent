from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

from common.config import (
    FIELD_HEIGHT,
    FIELD_WIDTH,
    ROBOT_KICK_COOLDOWN,
    ROBOT_KICK_RANGE,
)
from common.world_state import Team
from simulation.scenarios import load_scenario_into_simulator


@dataclass(frozen=True)
class ShowcasePhase:
    key: str
    label: str
    scenario: str
    duration_s: float


SHOWCASE_PHASES: Tuple[ShowcasePhase, ...] = (
    ShowcasePhase("PASS_RECEIVE", "1/5 传球与接球", "pass_fixed", 10.0),
    ShowcasePhase("DRIBBLE", "2/5 带球推进", "dribble_open", 12.0),
    ShowcasePhase("POSITION", "3/5 无球跑位", "dribble_open", 10.0),
    ShowcasePhase("BLOCK", "4/5 卡位防守", "position_block", 12.0),
    ShowcasePhase("ATTACK_DEFENSE", "5/5 2v2 简单攻防", "2v2_attack_defense", 20.0),
)


class ShowcaseDirector:
    """在同一窗口中循环加载可复现的协作与攻防场景。"""

    def __init__(self, simulator):
        self._phase_index = 0
        self._elapsed_s = 0.0
        self._load_phase(simulator)

    @property
    def phase(self) -> ShowcasePhase:
        return SHOWCASE_PHASES[self._phase_index]

    @property
    def total_duration_s(self) -> float:
        return sum(phase.duration_s for phase in SHOWCASE_PHASES)

    def next_phase(self, simulator) -> ShowcasePhase:
        self._phase_index = (self._phase_index + 1) % len(SHOWCASE_PHASES)
        self._elapsed_s = 0.0
        self._load_phase(simulator)
        return self.phase

    def advance(self, simulator, dt: float) -> bool:
        self._elapsed_s += dt
        if self._elapsed_s + 1e-9 < self.phase.duration_s:
            return False
        self.next_phase(simulator)
        return True

    def control_opponents(self, simulator) -> None:
        if self.phase.key not in {"BLOCK", "ATTACK_DEFENSE"}:
            return

        opponents = simulator.get_robots(Team.YELLOW)
        if not opponents:
            return

        ball = simulator.get_ball()
        attacker = min(
            opponents,
            key=lambda robot: math.hypot(robot.x - ball.x, robot.y - ball.y),
        )
        distance = math.hypot(attacker.x - ball.x, attacker.y - ball.y)
        attack_direction = math.atan2(-ball.y, -FIELD_WIDTH / 2 - ball.x)

        if distance <= ROBOT_KICK_RANGE * 1.1 and attacker.kick_cooldown <= 0.0:
            simulator.set_turn_target(attacker.id, attack_direction)
            simulator.queue_kick(attacker.id, 18.0, attack_direction)
            attacker.kick_cooldown = ROBOT_KICK_COOLDOWN
        else:
            behind_x = ball.x - math.cos(attack_direction) * 0.20
            behind_y = ball.y - math.sin(attack_direction) * 0.20
            simulator.set_move_target(attacker.id, behind_x, behind_y)
            simulator.set_turn_target(attacker.id, attack_direction)

        supporters = [robot for robot in opponents if robot.id != attacker.id]
        for index, robot in enumerate(supporters):
            side = 1.0 if index % 2 == 0 else -1.0
            target_x = max(-FIELD_WIDTH / 2 + 0.4, min(FIELD_WIDTH / 2 - 0.4, ball.x + 0.9))
            target_y = max(-FIELD_HEIGHT / 2 + 0.4, min(FIELD_HEIGHT / 2 - 0.4, ball.y + side * 1.0))
            simulator.set_move_target(robot.id, target_x, target_y)
            simulator.set_turn_target(robot.id, attack_direction)

    def _load_phase(self, simulator) -> None:
        load_scenario_into_simulator(simulator, self.phase.scenario)

