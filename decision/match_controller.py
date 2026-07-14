"""
比赛计分与定点球 (开球 / 任意球) 管理
====================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, List, Optional, Tuple

from common.config import (
    FIELD_HEIGHT,
    FIELD_WIDTH,
    GOAL_WIDTH,
    GOAL_X,
    KICKOFF_BALL_MOVE_THRESH,
    OUR_GOAL_X,
    SET_PIECE_HOLD,
)
from common.world_state import Team
from strategy.set_piece import freekick_formation, kickoff_formation

if TYPE_CHECKING:
    from common.robot_action import RobotActionInterface
    from decision.decision_fsm import DecisionFSM
    from simulation.field_simulator import Simulator


class MatchPhase(Enum):
    PLAY = "play"
    KICKOFF = "kickoff"
    FREE_KICK = "free_kick"


@dataclass
class MatchController:
    blue_score: int = 0
    yellow_score: int = 0
    goals: List[dict] = field(default_factory=list)
    phase: MatchPhase = MatchPhase.PLAY
    kicking_team: Optional[Team] = None
    _cooldown: float = 0.0
    kickoff_cooldown: float = SET_PIECE_HOLD
    _ball_anchor: Tuple[float, float] = (0.0, 0.0)
    blue_gk_id: int = 2
    yellow_gk_id: int = 12
    blue_ids: List[int] = field(default_factory=lambda: [0, 1, 2])
    yellow_ids: List[int] = field(default_factory=lambda: [10, 11, 12])

    def update_cooldown(self, dt: float) -> None:
        if self._cooldown > 0:
            self._cooldown = max(0.0, self._cooldown - dt)

    @property
    def frozen(self) -> bool:
        return self.phase != MatchPhase.PLAY and self._cooldown > 0

    def detect_goal(self, ball_x: float, ball_y: float) -> Optional[Team]:
        if self.phase != MatchPhase.PLAY:
            return None
        if ball_x >= GOAL_X - 0.02 and abs(ball_y) <= GOAL_WIDTH / 2:
            return Team.BLUE
        if ball_x <= OUR_GOAL_X + 0.02 and abs(ball_y) <= GOAL_WIDTH / 2:
            return Team.YELLOW
        return None

    def detect_out_of_play(self, ball_x: float, ball_y: float) -> Optional[str]:
        if self.phase != MatchPhase.PLAY:
            return None
        if abs(ball_y) >= FIELD_HEIGHT / 2 - 0.02:
            return "touchline"
        if abs(ball_x) >= FIELD_WIDTH / 2 - 0.02 and abs(ball_y) > GOAL_WIDTH / 2:
            return "goalline"
        return None

    def handle_goal(
        self,
        scorer: Team,
        simulator: "Simulator",
        blue_fsm: "DecisionFSM",
        yellow_fsm: "DecisionFSM",
        action: Optional["RobotActionInterface"] = None,
        timestamp: float = 0.0,
    ) -> None:
        if scorer == Team.BLUE:
            self.blue_score += 1
            kick_team = Team.YELLOW
        else:
            self.yellow_score += 1
            kick_team = Team.BLUE
        self.goals.append({
            "team": scorer.value,
            "blue": self.blue_score,
            "yellow": self.yellow_score,
            "t": timestamp,
        })
        print(
            f"  ⚽ {scorer.value.upper()} 进球! "
            f"比分 蓝 {self.blue_score} : {self.yellow_score} 黄 → "
            f"{kick_team.value} 开球"
        )
        self.begin_kickoff(simulator, blue_fsm, yellow_fsm, kicking_team=kick_team)

    def begin_kickoff(
        self,
        simulator: "Simulator",
        blue_fsm: "DecisionFSM",
        yellow_fsm: "DecisionFSM",
        kicking_team: Team,
    ) -> None:
        self.phase = MatchPhase.KICKOFF
        self.kicking_team = kicking_team
        self._cooldown = self.kickoff_cooldown
        simulator.restart_kickoff(reset_clock=False)
        self._apply_formation(simulator, kind="kickoff")
        blue_fsm.reset_round()
        yellow_fsm.reset_round()
        self._ball_anchor = (simulator.ball.x, simulator.ball.y)
        print(f"  🏁 开球站位就绪 ({kicking_team.value})")

    def begin_freekick(
        self,
        simulator: "Simulator",
        blue_fsm: "DecisionFSM",
        yellow_fsm: "DecisionFSM",
        attacking_team: Team,
        ball_x: Optional[float] = None,
        ball_y: Optional[float] = None,
    ) -> None:
        bx = simulator.ball.x if ball_x is None else ball_x
        by = simulator.ball.y if ball_y is None else ball_y
        bx = max(-FIELD_WIDTH / 2 + 0.4, min(FIELD_WIDTH / 2 - 0.4, bx))
        by = max(-FIELD_HEIGHT / 2 + 0.4, min(FIELD_HEIGHT / 2 - 0.4, by))
        simulator.ball.x, simulator.ball.y = bx, by
        simulator.ball.vx = simulator.ball.vy = 0.0
        self.phase = MatchPhase.FREE_KICK
        self.kicking_team = attacking_team
        self._cooldown = self.kickoff_cooldown
        self._apply_formation(simulator, kind="freekick", ball_x=bx, ball_y=by)
        blue_fsm.reset_round()
        yellow_fsm.reset_round()
        self._ball_anchor = (bx, by)
        print(f"  🧭 任意球: {attacking_team.value} 主罚 @ ({bx:.1f},{by:.1f})")

    def tick_set_piece(self, simulator: "Simulator") -> None:
        if self.phase == MatchPhase.PLAY:
            return
        if self._cooldown > 0:
            self._apply_formation(
                simulator,
                kind="kickoff" if self.phase == MatchPhase.KICKOFF else "freekick",
                ball_x=simulator.ball.x,
                ball_y=simulator.ball.y,
            )
            simulator._move_targets.clear()
            simulator._kick_queue.clear()
            return
        ax, ay = self._ball_anchor
        moved = ((simulator.ball.x - ax) ** 2 + (simulator.ball.y - ay) ** 2) ** 0.5
        if moved >= KICKOFF_BALL_MOVE_THRESH or self._cooldown <= 0:
            if self.phase != MatchPhase.PLAY:
                print(f"  ▶ 比赛继续 ({self.phase.value} → play)")
            self.phase = MatchPhase.PLAY
            self.kicking_team = None

    def _apply_formation(
        self,
        simulator: "Simulator",
        *,
        kind: str,
        ball_x: float = 0.0,
        ball_y: float = 0.0,
    ) -> None:
        kick = self.kicking_team or Team.BLUE
        if kind == "kickoff":
            blue = kickoff_formation(
                Team.BLUE, robot_ids=self.blue_ids, goalkeeper_id=self.blue_gk_id,
                is_kicking_team=(kick == Team.BLUE),
            )
            yellow = kickoff_formation(
                Team.YELLOW, robot_ids=self.yellow_ids, goalkeeper_id=self.yellow_gk_id,
                is_kicking_team=(kick == Team.YELLOW),
            )
            simulator.ball.x = simulator.ball.y = 0.0
            simulator.ball.vx = simulator.ball.vy = 0.0
        else:
            blue = freekick_formation(
                Team.BLUE, robot_ids=self.blue_ids, goalkeeper_id=self.blue_gk_id,
                ball_x=ball_x, ball_y=ball_y, is_attacking=(kick == Team.BLUE),
            )
            yellow = freekick_formation(
                Team.YELLOW, robot_ids=self.yellow_ids, goalkeeper_id=self.yellow_gk_id,
                ball_x=ball_x, ball_y=ball_y, is_attacking=(kick == Team.YELLOW),
            )

        for formation in (blue, yellow):
            for rid, (x, y, theta) in formation.items():
                robot = simulator.get_robot_by_id(rid)
                if robot is None:
                    continue
                robot.x, robot.y, robot.theta = x, y, theta
                robot.kick_cooldown = 0.0

        simulator._move_targets.clear()
        simulator._turn_targets.clear()
        simulator._kick_queue.clear()

    def scoreboard(self) -> str:
        return f"蓝 {self.blue_score} : {self.yellow_score} 黄"
