from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from common.world_state import OpponentState, Point, RobotState, WorldState


@dataclass(frozen=True)
class PassConfig:
    field_margin: float = 0.15
    min_pass_distance: float = 0.7
    max_pass_distance: float = 5.5
    min_receive_clearance: float = 0.85
    hard_receive_clearance: float = 0.55
    line_static_clearance: float = 0.35
    opponent_max_speed: float = 1.6
    ball_base_speed: float = 2.5
    min_ball_speed: float = 1.2
    max_ball_speed: float = 4.0
    pass_time_margin: float = 0.25
    pass_distance_margin: float = 0.12
    dynamic_receive_horizon: float = 1.4
    receive_lead_scale: float = 0.8
    forward_receive_offset: float = 0.35
    allow_emergency_risky_pass: bool = False
    weights: dict[str, float] = field(
        default_factory=lambda: {
            "distance": 0.16,
            "safety": 0.25,
            "space": 0.18,
            "line": 0.24,
            "advance": 0.10,
            "attack": 0.07,
        }
    )

    @classmethod
    def from_yaml(cls, path: Path) -> "PassConfig":
        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        weights = data.pop("weights", None)
        cfg = cls(**data)
        if weights:
            merged = dict(cfg.weights)
            merged.update({str(k): float(v) for k, v in weights.items()})
            cfg = cls(**{**cfg.__dict__, "weights": merged})
        return cfg


@dataclass(frozen=True)
class PassCandidateScore:
    receiver_id: str
    target_point: Point
    distance_score: float
    safety_score: float
    space_score: float
    line_score: float
    advance_score: float
    attack_score: float
    total_score: float
    risk: float
    eliminated: bool
    elimination_reasons: list[str]
    pass_time: float
    nearest_receive_opponent: float
    min_line_clearance: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "receiver_id": self.receiver_id,
            "target_point": {"x": self.target_point.x, "y": self.target_point.y},
            "distance_score": self.distance_score,
            "safety_score": self.safety_score,
            "space_score": self.space_score,
            "line_score": self.line_score,
            "advance_score": self.advance_score,
            "attack_score": self.attack_score,
            "total_score": self.total_score,
            "risk": self.risk,
            "eliminated": self.eliminated,
            "elimination_reasons": list(self.elimination_reasons),
            "pass_time": self.pass_time,
            "nearest_receive_opponent": self.nearest_receive_opponent,
            "min_line_clearance": self.min_line_clearance,
        }


@dataclass(frozen=True)
class PassDecision:
    should_pass: bool
    receiver_id: str | None
    target_point: Point | None
    pass_speed: float
    total_score: float
    risk_level: str
    reason: str
    component_scores: list[PassCandidateScore]

    def to_dict(self) -> dict[str, Any]:
        return {
            "should_pass": self.should_pass,
            "receiver_id": self.receiver_id,
            "target_point": (
                {"x": self.target_point.x, "y": self.target_point.y}
                if self.target_point
                else None
            ),
            "pass_speed": self.pass_speed,
            "total_score": self.total_score,
            "risk_level": self.risk_level,
            "reason": self.reason,
            "component_scores": [score.to_dict() for score in self.component_scores],
        }


class PassStrategy:
    def __init__(self, config: PassConfig | None = None) -> None:
        self.config = config or PassConfig()

    def is_pass_line_clear(
        self,
        world: WorldState,
        passer_id: str,
        receiver_id: str,
        config: PassConfig | None = None,
    ) -> bool:
        cfg = config or self.config
        passer = next((robot for robot in world.robots if robot.robot_id == passer_id), None)
        receiver = next((robot for robot in world.robots if robot.robot_id == receiver_id), None)
        if passer is None or receiver is None:
            return False
        target = self._dynamic_receive_point(world, passer, receiver, cfg)
        pass_time = passer.point.distance_to(target) / max(self._pass_speed(passer.point, target, cfg), 0.01)
        return not bool(self._line_risk(passer.point, target, world.opponents, pass_time, cfg)["blocked"])

    def is_receiver_safe(
        self,
        world: WorldState,
        receiver_id: str,
        config: PassConfig | None = None,
    ) -> bool:
        cfg = config or self.config
        receiver = next((robot for robot in world.robots if robot.robot_id == receiver_id), None)
        if receiver is None:
            return False
        return self._nearest_opponent_distance(receiver.point, world.opponents) >= cfg.hard_receive_clearance

    def evaluate_receiver(
        self,
        world: WorldState,
        passer_id: str,
        receiver_id: str,
        config: PassConfig | None = None,
    ) -> PassCandidateScore | None:
        cfg = config or self.config
        passer = next((robot for robot in world.robots if robot.robot_id == passer_id), None)
        receiver = next((robot for robot in world.robots if robot.robot_id == receiver_id), None)
        if passer is None or receiver is None:
            return None
        return self._score_candidate(world, passer, receiver, cfg)

    def decide_pass(
        self,
        world: WorldState,
        passer_id: str,
        config: PassConfig | None = None,
    ) -> PassDecision:
        cfg = config or self.config
        passer = next((robot for robot in world.robots if robot.robot_id == passer_id), None)
        if passer is None:
            return self._no_pass([], f"未找到传球机器人 {passer_id}。")

        candidates = [
            self._score_candidate(world, passer, teammate, cfg)
            for teammate in world.teammates_except(passer.robot_id)
        ]
        safe_candidates = [item for item in candidates if not item.eliminated]
        if safe_candidates:
            best = max(safe_candidates, key=lambda item: item.total_score)
            return PassDecision(
                should_pass=True,
                receiver_id=best.receiver_id,
                target_point=best.target_point,
                pass_speed=self._pass_speed(passer.point, best.target_point, cfg),
                total_score=best.total_score,
                risk_level=self._risk_level(best.risk),
                reason=f"选择 {best.receiver_id}：硬安全和线路检测通过，总分最高。",
                component_scores=candidates,
            )

        if cfg.allow_emergency_risky_pass and candidates:
            best = max(candidates, key=lambda item: item.total_score)
            return PassDecision(
                should_pass=True,
                receiver_id=best.receiver_id,
                target_point=best.target_point,
                pass_speed=self._pass_speed(passer.point, best.target_point, cfg),
                total_score=best.total_score,
                risk_level="HIGH",
                reason=f"紧急降级：无安全候选，选择风险最低的 {best.receiver_id}。",
                component_scores=candidates,
            )

        return self._no_pass(candidates, "无安全传球候选，降级为带球或支援移动。")

    def fixed_point_pass(
        self,
        world: WorldState,
        passer_id: str,
        target_point: Point,
        config: PassConfig | None = None,
    ) -> PassDecision:
        cfg = config or self.config
        passer = next((robot for robot in world.robots if robot.robot_id == passer_id), None)
        if passer is None:
            return self._no_pass([], f"未找到传球机器人 {passer_id}。")

        synthetic = RobotState(
            robot_id="fixed_point",
            team=passer.team,
            x=target_point.x,
            y=target_point.y,
            theta=0.0,
            role="fixed",
        )
        score = self._score_candidate(
            world,
            passer,
            synthetic,
            cfg,
            fixed_target=target_point,
        )
        if score.eliminated and not cfg.allow_emergency_risky_pass:
            return self._no_pass([score], "固定点传球线路或接球安全不满足硬约束。")
        return PassDecision(
            should_pass=True,
            receiver_id=None,
            target_point=target_point,
            pass_speed=self._pass_speed(passer.point, target_point, cfg),
            total_score=score.total_score,
            risk_level=self._risk_level(score.risk),
            reason="固定点传球通过线路检测。" if not score.eliminated else "紧急固定点传球。",
            component_scores=[score],
        )

    def _score_candidate(
        self,
        world: WorldState,
        passer: RobotState,
        receiver: RobotState,
        cfg: PassConfig,
        fixed_target: Point | None = None,
    ) -> PassCandidateScore:
        target = fixed_target or self._dynamic_receive_point(world, passer, receiver, cfg)
        distance = passer.point.distance_to(target)
        pass_speed = self._pass_speed(passer.point, target, cfg)
        pass_time = distance / max(pass_speed, 0.01)
        nearest_receive = self._nearest_opponent_distance(target, world.opponents)
        line = self._line_risk(passer.point, target, world.opponents, pass_time, cfg)

        reasons: list[str] = []
        if not self._inside_field(world, target, cfg):
            reasons.append("接球点超出安全场地边界")
        if distance < cfg.min_pass_distance:
            reasons.append("传球距离过近")
        if distance > cfg.max_pass_distance:
            reasons.append("传球距离过远")
        if nearest_receive < cfg.hard_receive_clearance:
            reasons.append("接球点硬安全距离不足")
        if nearest_receive < cfg.min_receive_clearance:
            reasons.append("接球点附近有对手")
        if line["blocked"]:
            reasons.append("传球线路存在可拦截对手")

        distance_score = self._distance_score(distance, cfg)
        safety_score = self._clip(nearest_receive / max(cfg.min_receive_clearance, 0.01))
        space_score = self._clip(nearest_receive / 2.0)
        line_score = 1.0 - line["risk"]
        advance_score = self._advance_score(passer.point, target, world.enemy_goal)
        attack_score = 1.0 - self._clip(target.distance_to(world.enemy_goal) / world.field_width)
        total = (
            distance_score * cfg.weights.get("distance", 0.0)
            + safety_score * cfg.weights.get("safety", 0.0)
            + space_score * cfg.weights.get("space", 0.0)
            + line_score * cfg.weights.get("line", 0.0)
            + advance_score * cfg.weights.get("advance", 0.0)
            + attack_score * cfg.weights.get("attack", 0.0)
        )
        risk = self._clip(max(1.0 - safety_score, line["risk"]))

        return PassCandidateScore(
            receiver_id=receiver.robot_id,
            target_point=target,
            distance_score=distance_score,
            safety_score=safety_score,
            space_score=space_score,
            line_score=line_score,
            advance_score=advance_score,
            attack_score=attack_score,
            total_score=total,
            risk=risk,
            eliminated=bool(reasons),
            elimination_reasons=reasons,
            pass_time=pass_time,
            nearest_receive_opponent=nearest_receive,
            min_line_clearance=line["min_clearance"],
        )

    def _dynamic_receive_point(
        self,
        world: WorldState,
        passer: RobotState,
        receiver: RobotState,
        cfg: PassConfig,
    ) -> Point:
        estimate_target = receiver.point
        for _ in range(2):
            pass_time = passer.point.distance_to(estimate_target) / cfg.ball_base_speed
            lead_time = min(cfg.dynamic_receive_horizon, pass_time) * cfg.receive_lead_scale
            gx, gy = self._unit_vector(receiver.point, world.enemy_goal)
            estimate_target = Point(
                receiver.x + receiver.vx * lead_time + gx * cfg.forward_receive_offset,
                receiver.y + receiver.vy * lead_time + gy * cfg.forward_receive_offset,
            )
        return self._clamp_to_field(world, estimate_target, cfg)

    def _line_risk(
        self,
        start: Point,
        end: Point,
        opponents: list[OpponentState],
        pass_time: float,
        cfg: PassConfig,
    ) -> dict[str, float | bool]:
        length = start.distance_to(end)
        if length == 0:
            return {"blocked": True, "risk": 1.0, "min_clearance": 0.0}

        min_clearance = float("inf")
        max_risk = 0.0
        blocked = False
        for opponent in opponents:
            projection = self._segment_projection(start, end, opponent.point)
            if projection < 0.0 or projection > 1.0:
                continue
            closest = Point(
                start.x + (end.x - start.x) * projection,
                start.y + (end.y - start.y) * projection,
            )
            clearance = opponent.point.distance_to(closest)
            min_clearance = min(min_clearance, clearance)
            ball_arrival = pass_time * projection
            opponent_time = max(0.0, clearance - cfg.pass_distance_margin) / max(
                cfg.opponent_max_speed,
                0.01,
            )
            time_margin = ball_arrival - opponent_time
            distance_risk = self._clip((cfg.line_static_clearance - clearance) / cfg.line_static_clearance)
            time_risk = self._clip((time_margin + cfg.pass_time_margin) / (cfg.pass_time_margin * 2.0))
            risk = max(distance_risk, time_risk)
            max_risk = max(max_risk, risk)
            if clearance < cfg.line_static_clearance or opponent_time <= ball_arrival + cfg.pass_time_margin:
                blocked = True

        if min_clearance == float("inf"):
            min_clearance = 99.0
        return {
            "blocked": blocked,
            "risk": self._clip(max_risk),
            "min_clearance": min_clearance,
        }

    def _pass_speed(self, start: Point, end: Point, cfg: PassConfig) -> float:
        distance = start.distance_to(end)
        return self._clamp(cfg.ball_base_speed + distance * 0.18, cfg.min_ball_speed, cfg.max_ball_speed)

    def _nearest_opponent_distance(self, point: Point, opponents: list[OpponentState]) -> float:
        if not opponents:
            return 99.0
        return min(point.distance_to(opponent.point) for opponent in opponents)

    def _inside_field(self, world: WorldState, point: Point, cfg: PassConfig) -> bool:
        return (
            -world.field_width / 2 + cfg.field_margin <= point.x <= world.field_width / 2 - cfg.field_margin
            and -world.field_height / 2 + cfg.field_margin <= point.y <= world.field_height / 2 - cfg.field_margin
        )

    def _clamp_to_field(self, world: WorldState, point: Point, cfg: PassConfig) -> Point:
        return Point(
            self._clamp(point.x, -world.field_width / 2 + cfg.field_margin, world.field_width / 2 - cfg.field_margin),
            self._clamp(point.y, -world.field_height / 2 + cfg.field_margin, world.field_height / 2 - cfg.field_margin),
        )

    def _segment_projection(self, start: Point, end: Point, point: Point) -> float:
        dx = end.x - start.x
        dy = end.y - start.y
        denom = dx * dx + dy * dy
        if denom == 0:
            return 0.0
        return ((point.x - start.x) * dx + (point.y - start.y) * dy) / denom

    def _distance_score(self, distance: float, cfg: PassConfig) -> float:
        ideal = (cfg.min_pass_distance + cfg.max_pass_distance) / 2.0
        span = max(cfg.max_pass_distance - cfg.min_pass_distance, 0.01) / 2.0
        return self._clip(1.0 - abs(distance - ideal) / span)

    def _advance_score(self, start: Point, target: Point, enemy_goal: Point) -> float:
        before = start.distance_to(enemy_goal)
        after = target.distance_to(enemy_goal)
        return self._clip((before - after + 1.0) / 2.0)

    def _unit_vector(self, start: Point, end: Point) -> tuple[float, float]:
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        if length == 0:
            return 0.0, 0.0
        return dx / length, dy / length

    def _risk_level(self, risk: float) -> str:
        if risk >= 0.66:
            return "HIGH"
        if risk >= 0.33:
            return "MEDIUM"
        return "LOW"

    def _no_pass(self, scores: list[PassCandidateScore], reason: str) -> PassDecision:
        return PassDecision(
            should_pass=False,
            receiver_id=None,
            target_point=None,
            pass_speed=0.0,
            total_score=0.0,
            risk_level="HIGH" if scores else "LOW",
            reason=reason,
            component_scores=scores,
        )

    def _clip(self, value: float) -> float:
        return self._clamp(value, 0.0, 1.0)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))
