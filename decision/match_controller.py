"""
比赛计分与开球回合管理
======================
进球 → 积分 → 重置到开球阵型进入下一回合。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Optional

from common.config import FIELD_WIDTH, GOAL_WIDTH, GOAL_X, OUR_GOAL_X
from common.world_state import Team

if TYPE_CHECKING:
    from common.robot_action import RobotActionInterface
    from decision.decision_fsm import DecisionFSM
    from simulation.field_simulator import Simulator


@dataclass
class MatchController:
    blue_score: int = 0
    yellow_score: int = 0
    goals: List[dict] = field(default_factory=list)
    _cooldown: float = 0.0
    kickoff_cooldown: float = 1.5  # 进球后短暂冻结再开球

    def update_cooldown(self, dt: float) -> None:
        if self._cooldown > 0:
            self._cooldown = max(0.0, self._cooldown - dt)

    @property
    def frozen(self) -> bool:
        return self._cooldown > 0

    def detect_goal(self, ball_x: float, ball_y: float) -> Optional[Team]:
        """检测球是否越过球门线 (门宽内)。"""
        if self.frozen:
            return None
        if ball_x >= GOAL_X - 0.02 and abs(ball_y) <= GOAL_WIDTH / 2:
            return Team.BLUE
        if ball_x <= OUR_GOAL_X + 0.02 and abs(ball_y) <= GOAL_WIDTH / 2:
            return Team.YELLOW
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
        else:
            self.yellow_score += 1
        self.goals.append({
            "team": scorer.value,
            "blue": self.blue_score,
            "yellow": self.yellow_score,
            "t": timestamp,
        })
        print(
            f"  ⚽ {scorer.value.upper()} 进球! "
            f"比分 蓝 {self.blue_score} : {self.yellow_score} 黄 → 下一回合开球"
        )
        self.kickoff_round(simulator, blue_fsm, yellow_fsm, action)
        self._cooldown = self.kickoff_cooldown

    def kickoff_round(
        self,
        simulator: "Simulator",
        blue_fsm: "DecisionFSM",
        yellow_fsm: "DecisionFSM",
        action: Optional["RobotActionInterface"] = None,
    ) -> None:
        """重置机器人/球到开球位, 清空 FSM 状态。"""
        simulator.restart_kickoff(reset_clock=False)
        blue_fsm.reset_round()
        yellow_fsm.reset_round()
        simulator._move_targets.clear()
        simulator._turn_targets.clear()
        simulator._kick_queue.clear()

    def scoreboard(self) -> str:
        return f"蓝 {self.blue_score} : {self.yellow_score} 黄"
