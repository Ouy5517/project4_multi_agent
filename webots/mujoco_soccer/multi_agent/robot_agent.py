from __future__ import annotations

import math
from dataclasses import dataclass, field

from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.control.foot_push_controller import plan_push
from mujoco_soccer.multi_agent.shared_world_state import SharedWorldState, TEAM_OF


ACTIVE_KICKS = {"DRIBBLE", "PASS", "SHOOT", "CLEAR", "COUNTER_ATTACK", "INTERCEPT_BALL"}


@dataclass
class AgentCommand:
    robot_id: str
    behavior: str
    role: str
    target: BaseTarget
    kick_action: str | None = None
    kick_target: tuple[float, float] | None = None
    confidence: float = 0.5


@dataclass
class RobotAgent:
    robot_id: str
    team: str
    current_role: str = "UNASSIGNED"
    previous_role: str = "UNASSIGNED"
    current_behavior: str = "HOLD_POSITION"
    previous_behavior: str = "HOLD_POSITION"
    possession_state: str = "FREE"
    target_position: tuple[float, float] = (0.0, 0.0)
    target_yaw: float = 0.0
    kick_target: tuple[float, float] | None = None
    decision_confidence: float = 0.0
    action_commit_until: float = 0.0
    kick_cooldown: float = 0.0
    last_contact_time: float = -99.0
    stuck_timer: float = 0.0
    local_path: list[tuple[float, float]] = field(default_factory=list)
    decision_count: int = 0
    moving_decision_count: int = 0
    hold_decision_count: int = 0
    role_changes: int = 0
    behavior_changes: int = 0
    steal_attempts: int = 0
    possession_seconds: float = 0.0

    def observe(self, world: SharedWorldState) -> None:
        self.possession_state = world.possession

    def update_role(self, role: str) -> None:
        self.previous_role = self.current_role
        if role != self.current_role:
            self.role_changes += 1
        self.current_role = role

    def decide(
        self,
        world: SharedWorldState,
        role: str,
        team_intent: dict[str, object] | None,
    ) -> AgentCommand:
        self.decision_count += 1
        self.update_role(role)
        behavior = self._select_behavior(world, team_intent)
        target, yaw, kick_action, kick_target = self._target_for_behavior(world, behavior, team_intent)
        self.previous_behavior = self.current_behavior
        if behavior != self.current_behavior:
            self.behavior_changes += 1
        self.current_behavior = behavior
        self.target_position = target
        self.target_yaw = yaw
        self.kick_target = kick_target
        self.decision_confidence = 0.85 if behavior not in {"HOLD_POSITION", "RECOVER_POSITION"} else 0.35
        if behavior == "HOLD_POSITION":
            self.hold_decision_count += 1
        else:
            self.moving_decision_count += 1
        if behavior in {"DRIBBLE", "PASS", "SHOOT", "CLEAR", "COUNTER_ATTACK", "PRESS_BALL", "INTERCEPT_BALL"}:
            self.steal_attempts += 1
        max_speed = 0.13 if behavior in {"RECEIVE_PASS", "DRIBBLE"} and kick_action else 0.17
        if behavior in {"CHASE_BALL", "PRESS_BALL", "INTERCEPT_BALL"}:
            max_speed = 0.19
        if behavior in {"BLOCK_LINE", "OPEN_FOR_PASS", "COVER"}:
            max_speed = 0.14
        return AgentCommand(
            robot_id=self.robot_id,
            behavior=behavior,
            role=self.current_role,
            target=BaseTarget(target[0], target[1], yaw, max_speed=max_speed, max_yaw_rate=math.radians(85), acceleration_limit=0.8),
            kick_action=kick_action,
            kick_target=kick_target,
            confidence=self.decision_confidence,
        )

    def _select_behavior(self, world: SharedWorldState, team_intent: dict[str, object] | None) -> str:
        t = world.sim_time
        if self.current_role in {"BALL_HANDLER", "CLEARER"}:
            if self.team == "blue":
                if t < 13.0:
                    return "DRIBBLE"
                if t < 28.0 and self.robot_id == "T1_BLUE_1":
                    return "PASS"
                if self.robot_id == "T1_BLUE_2":
                    return "SHOOT"
                return "PROTECT_BALL"
            if t < 45.0 and self.robot_id == "T1_RED_1":
                return "CLEAR"
            if self.robot_id == "T1_RED_2":
                return "COUNTER_ATTACK"
        if self.current_role in {"RECEIVER", "SUPPORT"}:
            if team_intent and team_intent.get("type") == "PASS_INTENT" and team_intent.get("receiver") == self.robot_id:
                return "RECEIVE_PASS"
            return "OPEN_FOR_PASS" if self.team == "blue" else "COVER"
        if self.current_role == "PRESSER":
            return "PRESS_BALL"
        if self.current_role in {"COVER", "INTERCEPTOR", "GOAL_PROTECTOR"}:
            return "BLOCK_LINE"
        return "RECOVER_POSITION"

    def _target_for_behavior(
        self,
        world: SharedWorldState,
        behavior: str,
        team_intent: dict[str, object] | None,
    ) -> tuple[tuple[float, float], float, str | None, tuple[float, float] | None]:
        ball = world.ball_xy
        enemy_goal = (3.35, 0.0) if self.team == "blue" else (-3.35, 0.0)
        own_goal = (-3.35, 0.0) if self.team == "blue" else (3.35, 0.0)
        if behavior == "PASS":
            receiver = "T1_BLUE_2" if self.robot_id == "T1_BLUE_1" else "T1_BLUE_1"
            r = world.robots[receiver]
            target = (r.x, r.y)
            start, push = plan_push(self.robot_id, ball, target, 0.105, 0.35, "pass")
            return (start.x, start.y), start.yaw, "pass", target
        if behavior in {"DRIBBLE", "SHOOT", "CLEAR", "COUNTER_ATTACK", "INTERCEPT_BALL"}:
            action = "shoot" if behavior == "SHOOT" else ("red1_clear" if behavior == "CLEAR" else ("red2_counter" if behavior == "COUNTER_ATTACK" else "dribble"))
            speed = {"SHOOT": 0.240, "CLEAR": 0.180, "COUNTER_ATTACK": 0.120}.get(behavior, 0.055)
            start, push = plan_push(self.robot_id, ball, enemy_goal, speed, 0.08, action)
            return (start.x, start.y), start.yaw, action, enemy_goal
        if behavior == "RECEIVE_PASS" and team_intent:
            target = tuple(team_intent.get("target", (ball[0] + 0.7, ball[1])))  # type: ignore[arg-type]
            yaw = math.atan2(ball[1] - target[1], ball[0] - target[0])
            return (float(target[0]), float(target[1])), yaw, "receive", enemy_goal
        if behavior == "OPEN_FOR_PASS":
            offset = (0.85, 0.55 if self.robot_id.endswith("_2") else -0.55)
            target = (max(-3.0, min(3.0, ball[0] + offset[0])), max(-2.1, min(2.1, ball[1] + offset[1])))
            return target, math.atan2(ball[1] - target[1], ball[0] - target[0]), None, None
        if behavior in {"PRESS_BALL", "CHASE_BALL"}:
            side = 0.22 if self.robot_id.endswith("_1") else -0.22
            target = (ball[0] - 0.20 if self.team == "red" else ball[0] + 0.20, max(-2.2, min(2.2, ball[1] + side)))
            return target, math.atan2(ball[1] - target[1], ball[0] - target[0]), None, None
        if behavior == "BLOCK_LINE":
            gx, gy = own_goal
            alpha = 0.45
            target = (ball[0] * alpha + gx * (1 - alpha), ball[1] * alpha + gy * (1 - alpha))
            return target, math.atan2(ball[1] - target[1], ball[0] - target[0]), None, None
        home_y = -1.35 if (self.team == "blue") == self.robot_id.endswith("_1") else 1.35
        home_x = -1.7 if self.team == "blue" else 1.7
        return (home_x, home_y), math.atan2(ball[1] - home_y, ball[0] - home_x), None, None

    def decision_dict(self) -> dict[str, object]:
        return {
            "robot": self.robot_id,
            "team": self.team,
            "role": self.current_role,
            "behavior": self.current_behavior,
            "target": self.target_position,
            "kick_target": self.kick_target,
            "confidence": self.decision_confidence,
        }
