from __future__ import annotations

import math

from common.robot_action import ActionType, RobotAction
from common.world_state import OpponentState, Point, RobotState, WorldState
from strategy.pass_strategy import PassConfig, PassDecision, PassStrategy
from strategy.state_machine import StrategyStateMachine


class TeamStrategy:
    """基于统一 WorldState 的两机器人协同决策策略。"""

    def __init__(
        self,
        shoot_distance: float = 2.0,
        pass_safe_distance: float = 1.0,
        defend_distance: float = 1.8,
        action_speed: float = 0.8,
        pass_config: PassConfig | None = None,
    ) -> None:
        self.shoot_distance = shoot_distance
        self.pass_safe_distance = pass_safe_distance
        self.defend_distance = defend_distance
        self.action_speed = action_speed
        self.pass_strategy = PassStrategy(pass_config)
        self.last_pass_decision: PassDecision | None = None
        self.state_machine = StrategyStateMachine()
        self.last_decision_summary = ""

    def decide(self, world_state: WorldState) -> list[RobotAction]:
        carrier = world_state.ball_carrier()
        if carrier is None:
            actions = self._decide_without_ball(world_state)
            if actions and actions[0].action_type == ActionType.BLOCK:
                self._finish_decision(actions, actions[0].reason)
            elif actions and actions[0].action_type == ActionType.HOLD:
                self._finish_decision(actions, actions[0].reason)
            else:
                self._finish_decision(actions, "我方无人持球，最近机器人追球。")
            return actions

        support = self._choose_support_robot(world_state, carrier)
        distance_to_goal = carrier.point.distance_to(world_state.enemy_goal)
        if distance_to_goal < self.shoot_distance:
            actions = [self._shoot_action(carrier, world_state)]
            self._finish_decision(actions, "持球机器人进入射门距离，射门优先级最高。")
            return actions

        nearest_threat = world_state.nearest_opponent_to_our_goal()
        if nearest_threat and self._is_near_our_goal(world_state, nearest_threat):
            actions = self._mark_goal_threat(world_state, carrier, support, nearest_threat)
            self._finish_decision(actions, "对手靠近我方球门，优先卡位防守。")
            return actions

        if support is None:
            actions = [self._dribble_action(carrier, world_state)]
            self._finish_decision(actions, "没有可配合队友，持球机器人向敌方球门带球推进。")
            return actions

        pass_decision = self.pass_strategy.decide_pass(world_state, carrier.robot_id)
        self.last_pass_decision = pass_decision
        if not pass_decision.should_pass:
            actions = [
                self._dribble_action(carrier, world_state),
                self._support_when_marked(support, world_state),
            ]
            self._finish_decision(actions, pass_decision.reason)
            return actions

        receiver = self._robot_by_id(world_state, pass_decision.receiver_id)
        if receiver is None or pass_decision.target_point is None:
            actions = [self._dribble_action(carrier, world_state)]
            self._finish_decision(actions, "传球决策缺少接球者，降级为带球推进。")
            return actions

        actions = [
            self._pass_action(carrier, receiver, pass_decision),
            self._move_to_receive_action(receiver, carrier, world_state, pass_decision),
        ]
        self._finish_decision(actions, pass_decision.reason)
        return actions

    def _finish_decision(self, actions: list[RobotAction], summary: str) -> None:
        self.state_machine.update(actions)
        self.last_decision_summary = summary

    def _decide_without_ball(self, world_state: WorldState) -> list[RobotAction]:
        if world_state.opponents:
            blocker = next(
                (robot for robot in world_state.robots if robot.role.upper() in {"BLOCK", "DEFENDER"}),
                None,
            ) or world_state.nearest_robot_to_ball()
            if blocker is not None:
                threat = min(
                    world_state.opponents,
                    key=lambda opponent: opponent.point.distance_to(world_state.enemy_goal),
                )
                return [
                    RobotAction(
                        robot_id=blocker.robot_id,
                        action_type=ActionType.BLOCK,
                        target={"opponent_id": threat.opponent_id, "x": threat.x, "y": threat.y},
                        reason=(
                            f"我方无人持球且 {threat.opponent_id} 形成进攻威胁，"
                            f"{blocker.robot_id} 封堵推进线路。"
                        ),
                        confidence=0.81,
                    )
                ]
        nearest = world_state.nearest_robot_to_ball()
        if nearest is None:
            return [
                RobotAction(
                    robot_id="TEAM",
                    action_type=ActionType.HOLD,
                    target={},
                    reason="WorldState 中没有可行动机器人，保持阵型。",
                    confidence=0.5,
                )
            ]
        return [
            RobotAction(
                robot_id=nearest.robot_id,
                action_type=ActionType.CHASE_BALL,
                target={"x": world_state.ball.x, "y": world_state.ball.y},
                reason=(
                    f"我方无人持球，{nearest.robot_id} 距离球最近，"
                    "先抢占球权。"
                ),
                confidence=0.86,
            )
        ]

    def _choose_support_robot(
        self, world_state: WorldState, carrier: RobotState
    ) -> RobotState | None:
        teammates = world_state.teammates_except(carrier.robot_id)
        if not teammates:
            return None
        return min(teammates, key=lambda robot: robot.point.distance_to(carrier.point))

    def _is_near_our_goal(
        self, world_state: WorldState, opponent: OpponentState
    ) -> bool:
        return opponent.point.distance_to(world_state.our_goal) < self.defend_distance

    def _mark_goal_threat(
        self,
        world_state: WorldState,
        carrier: RobotState,
        support: RobotState | None,
        threat: OpponentState,
    ) -> list[RobotAction]:
        marker = support or carrier
        return [
            RobotAction(
                robot_id=marker.robot_id,
                action_type=ActionType.MARK_OPPONENT,
                target={"opponent_id": threat.opponent_id, "x": threat.x, "y": threat.y},
                reason=(
                    f"{threat.opponent_id} 距离我方球门过近，"
                    f"{marker.robot_id} 执行卡位限制射门线路。"
                ),
                confidence=0.9,
            )
        ]

    def _shoot_action(self, carrier: RobotState, world_state: WorldState) -> RobotAction:
        vx, vy = self._direction_vector(carrier.point, world_state.enemy_goal)
        return RobotAction(
            robot_id=carrier.robot_id,
            action_type=ActionType.SHOOT,
            target={"x": world_state.enemy_goal.x, "y": world_state.enemy_goal.y},
            vx=vx * self.action_speed,
            vy=vy * self.action_speed,
            reason=(
                f"{carrier.robot_id} 到敌方球门距离 "
                f"{carrier.point.distance_to(world_state.enemy_goal):.2f}m，"
                f"小于射门阈值 {self.shoot_distance:.2f}m。"
            ),
            confidence=0.92,
        )

    def _pass_action(
        self, carrier: RobotState, support: RobotState, decision: PassDecision
    ) -> RobotAction:
        target_point = decision.target_point or support.point
        vx, vy = self._direction_vector(carrier.point, target_point)
        return RobotAction(
            robot_id=carrier.robot_id,
            action_type=ActionType.PASS,
            target={
                "robot_id": support.robot_id,
                "x": target_point.x,
                "y": target_point.y,
                "pass_speed": decision.pass_speed,
                "risk_level": decision.risk_level,
                "component_scores": [
                    score.to_dict() for score in decision.component_scores
                ],
            },
            vx=vx * min(self.action_speed, decision.pass_speed),
            vy=vy * min(self.action_speed, decision.pass_speed),
            reason=(
                f"{support.robot_id} 通过接球安全和线路拦截检测，"
                f"{carrier.robot_id} 传向动态接球点；"
                f"总分 {decision.total_score:.2f}，风险 {decision.risk_level}。"
            ),
            confidence=max(0.1, min(0.98, decision.total_score)),
        )

    def _move_to_receive_action(
        self,
        support: RobotState,
        carrier: RobotState,
        world_state: WorldState,
        decision: PassDecision | None = None,
    ) -> RobotAction:
        receive_point = decision.target_point if decision and decision.target_point else self._receive_point(support, world_state)
        return RobotAction(
            robot_id=support.robot_id,
            action_type=ActionType.MOVE_TO_RECEIVE,
            target={"x": receive_point.x, "y": receive_point.y},
            reason=(
                f"{support.robot_id} 移动到动态接球点，"
                f"与 {carrier.robot_id} 形成二机器人传接配合。"
            ),
            confidence=0.84,
        )

    def _dribble_action(
        self, carrier: RobotState, world_state: WorldState
    ) -> RobotAction:
        vx, vy = self._direction_vector(carrier.point, world_state.enemy_goal)
        target = Point(
            x=carrier.x + vx * min(1.0, self.shoot_distance),
            y=carrier.y + vy * min(1.0, self.shoot_distance),
        )
        return RobotAction(
            robot_id=carrier.robot_id,
            action_type=ActionType.DRIBBLE,
            target={"x": target.x, "y": target.y},
            vx=vx * self.action_speed,
            vy=vy * self.action_speed,
            reason="传球不安全，持球机器人沿敌方球门方向带球推进。",
            confidence=0.82,
        )

    def _support_when_marked(
        self, support: RobotState, world_state: WorldState
    ) -> RobotAction:
        nearest = self._nearest_opponent(support, world_state.opponents)
        if nearest is not None:
            return RobotAction(
                robot_id=support.robot_id,
                action_type=ActionType.MARK_OPPONENT,
                target={"opponent_id": nearest.opponent_id, "x": nearest.x, "y": nearest.y},
                reason=(
                    f"{support.robot_id} 被 {nearest.opponent_id} 贴近，"
                    "转为卡位牵制对手。"
                ),
                confidence=0.76,
            )

        support_point = self._receive_point(support, world_state)
        return RobotAction(
            robot_id=support.robot_id,
            action_type=ActionType.MOVE_TO_SUPPORT,
            target={"x": support_point.x, "y": support_point.y},
            reason="支援机器人移动到侧翼支援点，等待下一次传球窗口。",
            confidence=0.72,
        )

    def _receive_point(self, support: RobotState, world_state: WorldState) -> Point:
        # 固定接球区域略向敌方球门前插，并限制在场地边界内。
        dx, dy = self._direction_vector(support.point, world_state.enemy_goal)
        x = self._clamp(
            support.x + dx * 0.5,
            -world_state.field_width / 2,
            world_state.field_width / 2,
        )
        y = self._clamp(
            support.y + dy * 0.5,
            -world_state.field_height / 2,
            world_state.field_height / 2,
        )
        return Point(x, y)

    def _nearest_opponent(
        self, robot: RobotState, opponents: list[OpponentState]
    ) -> OpponentState | None:
        if not opponents:
            return None
        return min(opponents, key=lambda opponent: opponent.point.distance_to(robot.point))

    def _robot_by_id(
        self, world_state: WorldState, robot_id: str | None
    ) -> RobotState | None:
        if robot_id is None:
            return None
        return next(
            (robot for robot in world_state.robots if robot.robot_id == robot_id),
            None,
        )

    def _direction_vector(self, start: Point, end: Point) -> tuple[float, float]:
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        if length == 0:
            return 0.0, 0.0
        return dx / length, dy / length

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))
