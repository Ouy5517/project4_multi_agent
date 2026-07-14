"""
决策状态机
===========
多机器人协同决策引擎：
- 状态定义: IDLE → CHASE → (DRIBBLE | PASS | SHOOT) → ... → IDLE
                           ↓
                         BLOCK (防守状态)
- 角色分配: ball_carrier, supporter, defender
- 决策日志: 记录每次状态转换和决策原因
"""

from typing import Dict, List, Optional, Tuple
from enum import Enum, auto
from dataclasses import dataclass, field
import math

from common.config import (
    DT, REEVALUATE_INTERVAL, STATE_MIN_DURATION,
    ROBOT_KICK_RANGE, SHOOT_RANGE, TEAM_BLUE
)
from common.world_state import (
    WorldState, Robot, Ball, Team, RobotRole
)
from common.robot_action import RobotActionInterface
from strategy.strategy_pass import PassStrategy
from strategy.strategy_dribble import DribbleStrategy
from strategy.strategy_shoot import ShootStrategy
from strategy.strategy_position import PositionStrategy
from strategy.strategy_block import BlockStrategy


# ================================================================
# 状态定义
# ================================================================

class DecisionState(Enum):
    """决策状态枚举"""
    IDLE = "IDLE"              # 空闲/初始
    CHASE = "CHASE"            # 追球
    DRIBBLE = "DRIBBLE"        # 带球推进
    PASS = "PASS"              # 传球
    SHOOT = "SHOOT"            # 射门
    BLOCK = "BLOCK"            # 防守卡位


@dataclass
class FSMTransition:
    """状态转换记录"""
    robot_id: int
    from_state: DecisionState
    to_state: DecisionState
    reason: str
    timestamp: float = 0.0


@dataclass
class DecisionLog:
    """单条决策日志"""
    tick: int = 0
    timestamp: float = 0.0
    robot_id: int = 0
    state: str = ""
    role: str = ""
    x: float = 0.0
    y: float = 0.0
    action: str = ""          # "move", "kick", "stop", "wait"
    action_params: dict = field(default_factory=dict)
    reason: str = ""


# ================================================================
# 单机器人的状态机
# ================================================================

class RobotFSM:
    """单个机器人的有限状态机"""

    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        self.state = DecisionState.IDLE
        self.previous_state = DecisionState.IDLE
        self.state_timer = 0.0          # 在当前状态的持续时间
        self.target: Optional[Tuple[float, float]] = None
        self.pass_target_id: Optional[int] = None

    def transition(self, new_state: DecisionState, reason: str = "") -> FSMTransition:
        """执行状态转换"""
        old = self.state
        self.previous_state = old
        self.state = new_state
        self.state_timer = 0.0
        return FSMTransition(
            robot_id=self.robot_id,
            from_state=old,
            to_state=new_state,
            reason=reason
        )

    def can_reevaluate(self) -> bool:
        """是否可以重新评估状态 (防止状态震荡)"""
        return self.state_timer >= REEVALUATE_INTERVAL


# ================================================================
# 顶层决策引擎
# ================================================================

class DecisionFSM:
    """
    多机器人协同决策引擎。
    每帧调用 update() 驱动本队机器人的决策。
    通过 WorldState.perspective_for(team) 支持蓝/黄双向攻防。
    """

    def __init__(
        self,
        world_state: WorldState,
        action: RobotActionInterface,
        num_robots: int = 3,
        robot_ids: Optional[List[int]] = None,
        team: Team = Team.BLUE,
    ):
        self._ws = world_state
        self._action = action
        self._team = team
        if robot_ids is not None:
            self._robot_ids = list(robot_ids)
        else:
            self._robot_ids = list(range(num_robots))
        self._num_robots = len(self._robot_ids)

        # 策略模块
        self.pass_strategy = PassStrategy(world_state)
        self.dribble_strategy = DribbleStrategy(world_state)
        self.shoot_strategy = ShootStrategy(world_state)
        self.position_strategy = PositionStrategy(world_state)
        self.block_strategy = BlockStrategy(world_state)

        # 每个机器人一个状态机
        self._fsms: Dict[int, RobotFSM] = {}
        for rid in self._robot_ids:
            self._fsms[rid] = RobotFSM(rid)

        # 角色
        self._ball_carrier_id: Optional[int] = None
        self._supporter_id: Optional[int] = None

        # 日志
        self.transitions: List[FSMTransition] = []
        self.decision_logs: List[DecisionLog] = []

        self.tick_count: int = 0

    # ================================================================
    # 主更新
    # ================================================================

    def update(self, world_state: WorldState, dt: float = DT):
        """
        每帧调用。
        1. 更新世界状态
        2. 分配角色
        3. 对每个机器人运行 FSM
        4. 记录决策
        """
        self._ws = world_state
        self.transitions.clear()
        self.tick_count += 1

        # 更新策略模块的世界状态
        self.pass_strategy.update_world_state(world_state)
        self.dribble_strategy.update_world_state(world_state)
        self.shoot_strategy.update_world_state(world_state)
        self.position_strategy.update_world_state(world_state)
        self.block_strategy.update_world_state(world_state)

        # 分配角色
        roles = self._assign_roles()

        # 对每个机器人执行决策
        for robot_id in self._robot_ids:
            fsm = self._fsms[robot_id]
            old_state = fsm.state
            role = roles.get(robot_id, RobotRole.IDLE)

            fsm.state_timer += dt
            self._update_robot_fsm(robot_id, role, fsm)

            if fsm.state != old_state:
                transition = FSMTransition(
                    robot_id=robot_id,
                    from_state=old_state,
                    to_state=fsm.state,
                    reason="reevaluation",
                    timestamp=world_state.timestamp
                )
                self.transitions.append(transition)

        self._log_decisions(world_state, roles)

    # ================================================================
    # 角色分配
    # ================================================================

    def _assign_roles(self) -> Dict[int, RobotRole]:
        """
        根据球的位置分配角色。
        - 距球最近 → BALL_CARRIER
        - 距球第二近 → SUPPORTER
        - 其余 → DEFENDER
        """
        if not self._ws.teammates:
            return {}

        # 计算每个队友到球的距离
        dists = []
        for r in self._ws.teammates:
            d = math.sqrt(
                (r.x - self._ws.ball.x)**2 + (r.y - self._ws.ball.y)**2)
            dists.append((r.id, d))

        dists.sort(key=lambda x: x[1])

        roles = {}
        if len(dists) >= 1:
            roles[dists[0][0]] = RobotRole.BALL_CARRIER
            self._ball_carrier_id = dists[0][0]

            # 更新机器人角色
            robot = self._ws.get_robot_by_id(dists[0][0])
            if robot:
                robot.role = RobotRole.BALL_CARRIER

        if len(dists) >= 2:
            roles[dists[1][0]] = RobotRole.SUPPORTER
            self._supporter_id = dists[1][0]

            robot = self._ws.get_robot_by_id(dists[1][0])
            if robot:
                robot.role = RobotRole.SUPPORTER

        for i in range(2, len(dists)):
            roles[dists[i][0]] = RobotRole.DEFENDER

            robot = self._ws.get_robot_by_id(dists[i][0])
            if robot:
                robot.role = RobotRole.DEFENDER

        return roles

    # ================================================================
    # 单机器人决策
    # ================================================================

    def _update_robot_fsm(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        """根据角色和状态决定每个机器人的行为"""

        if fsm.state == DecisionState.IDLE:
            self._handle_idle(robot_id, role, fsm)

        elif fsm.state == DecisionState.CHASE:
            self._handle_chase(robot_id, role, fsm)

        elif fsm.state == DecisionState.DRIBBLE:
            self._handle_dribble(robot_id, role, fsm)

        elif fsm.state == DecisionState.PASS:
            self._handle_pass(robot_id, role, fsm)

        elif fsm.state == DecisionState.SHOOT:
            self._handle_shoot(robot_id, role, fsm)

        elif fsm.state == DecisionState.BLOCK:
            self._handle_block(robot_id, role, fsm)

    # ---- IDLE ----

    def _handle_idle(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "assigned BALL_CARRIER")
        elif role == RobotRole.SUPPORTER:
            if not self._ws.team_has_possession():
                # 丢球时支援前压逼抢
                ball = self._ws.ball
                self._action.move_to(robot_id, ball.x, ball.y)
                fsm.transition(DecisionState.CHASE, "press when team lost ball")
            else:
                target = self.position_strategy.calculate_support_position(
                    self._ball_carrier_id or robot_id, robot_id)
                self._action.move_to(robot_id, *target)
        elif role == RobotRole.DEFENDER:
            fsm.transition(DecisionState.BLOCK, "hold defensive line")

    # ---- CHASE: 追球 ----

    def _handle_chase(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        # 角色变化
        if role == RobotRole.DEFENDER:
            fsm.transition(DecisionState.BLOCK, "role changed to DEFENDER")
            return
        if role == RobotRole.SUPPORTER:
            fsm.transition(DecisionState.IDLE, "role changed to SUPPORTER")
            return

        ball = self._ws.ball
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return

        # 检查是否已到达球
        if self.dribble_strategy.is_ball_controlled(robot_id):
            # 评估下一步
            if fsm.can_reevaluate():
                self._evaluate_next_action(robot_id, fsm)
        else:
            # 追球 (略预瞄球速, 争球更主动)
            _, tx, ty = self.dribble_strategy.approach_ball(robot_id)
            tx += ball.vx * 0.22
            ty += ball.vy * 0.22
            self._action.move_to(robot_id, tx, ty)

    # ---- DRIBBLE: 带球 ----

    def _handle_dribble(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if role != RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.IDLE, "lost ball carrier role")
            return

        # 检查是否丢失球
        if not self.dribble_strategy.is_ball_controlled(robot_id):
            fsm.transition(DecisionState.CHASE, "lost ball")
            return

        # 带球朝对方球门
        goal = self._ws.opponent_goal
        target_x = goal.x
        target_y = goal.center[1]

        should_dribble, direction, power, dist = \
            self.dribble_strategy.dribble_toward(robot_id, target_x, target_y)

        if should_dribble:
            # 在球的后面, 向目标踢球
            robot = self._ws.get_robot_by_id(robot_id)
            if robot:
                self._action.turn_to(robot_id, direction)
                self._action.kick(robot_id, power, direction)
            # 移动机器人在球后
            behind_x, behind_y = self.dribble_strategy._position_behind_ball(
                robot_id, direction)
            self._action.move_to(robot_id, behind_x, behind_y)

        # 重新评估
        if fsm.can_reevaluate():
            self._evaluate_next_action(robot_id, fsm)

    # ---- PASS: 传球 ----

    def _handle_pass(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if fsm.pass_target_id is None:
            fsm.transition(DecisionState.CHASE, "no pass target")
            return

        robot = self._ws.get_robot_by_id(robot_id)
        receiver = self._ws.get_robot_by_id(fsm.pass_target_id)

        if robot is None or receiver is None:
            fsm.transition(DecisionState.CHASE, "invalid target")
            return

        # 执行传球
        success, direction, power = self.pass_strategy.execute_pass(
            robot_id, fsm.pass_target_id)

        if success:
            self._action.turn_to(robot_id, direction)
            self._action.kick(robot_id, power, direction)

        # 传球完成后回到追球状态
        if fsm.state_timer > 2.0:
            fsm.transition(DecisionState.CHASE, "pass completed")
            fsm.pass_target_id = None

    # ---- SHOOT: 射门 ----

    def _handle_shoot(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        evaluation = self.shoot_strategy.evaluate_shoot_opportunity(robot_id)

        if evaluation.is_viable:
            self._action.turn_to(robot_id, evaluation.best_angle)
            self._action.kick(robot_id, evaluation.power, evaluation.best_angle)

        # 射门后等待, 重新评估
        if fsm.state_timer > 2.0:
            fsm.transition(DecisionState.CHASE, "shoot completed")

    # ---- BLOCK: 防守 ----

    def _handle_block(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        # 如果重新分配为持球者 → 上前争球
        if role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "now ball carrier")
            return
        if role == RobotRole.SUPPORTER and self._ws.team_has_possession():
            fsm.transition(DecisionState.IDLE, "support after regain")
            return

        # 防守者始终卡在 球—己方球门 连线上; 受威胁时更贴球
        if self.block_strategy.is_goal_threatened():
            target = self.block_strategy.calculate_defensive_position(robot_id)
            # 略前移压迫
            ball = self._ws.ball
            gx, gy = self._ws.our_goal.center
            tx = 0.55 * target[0] + 0.45 * ball.x
            ty = 0.55 * target[1] + 0.45 * ball.y
            self._action.move_to(robot_id, tx, ty)
        else:
            target = self.block_strategy.calculate_defensive_position(robot_id)
            self._action.move_to(robot_id, *target)

    # ---- 评估下一步动作 ----

    def _evaluate_next_action(self, robot_id: int, fsm: RobotFSM):
        """
        在控制球后评估最佳动作: SHOOT > PASS > DRIBBLE
        争球不利时优先带球摆脱或回传。
        """
        shoot_eval = self.shoot_strategy.evaluate_shoot_opportunity(robot_id)
        if shoot_eval.is_viable and shoot_eval.score > 0.45:
            fsm.transition(DecisionState.SHOOT, f"shoot opportunity (score={shoot_eval.score:.2f})")
            return

        pass_options = self.pass_strategy.evaluate_pass_options(robot_id)
        # 对方逼抢近时更倾向传球
        press_pass_boost = 0.0
        opp = self._ws.closest_opponent_to_ball()
        robot = self._ws.get_robot_by_id(robot_id)
        if opp is not None and robot is not None:
            if self._ws.distance(robot, opp) < 1.2:
                press_pass_boost = 0.15
        pass_threshold = 0.35 - press_pass_boost
        if pass_options and pass_options[0].score > pass_threshold:
            fsm.transition(
                DecisionState.PASS,
                f"pass to {pass_options[0].receiver_id} (score={pass_options[0].score:.2f})",
            )
            fsm.pass_target_id = pass_options[0].receiver_id
            return

        fsm.transition(DecisionState.DRIBBLE, "advance toward goal")

    # ================================================================
    # 日志
    # ================================================================

    def _log_decisions(self, ws: WorldState, roles: Dict[int, RobotRole]):
        """记录当前帧的决策"""
        for robot_id in self._robot_ids:
            fsm = self._fsms[robot_id]
            robot = ws.get_robot_by_id(robot_id)
            role = roles.get(robot_id, RobotRole.IDLE)

            if robot:
                log = DecisionLog(
                    tick=self.tick_count,
                    timestamp=ws.timestamp,
                    robot_id=robot_id,
                    state=fsm.state.value,
                    role=role.value,
                    x=robot.x,
                    y=robot.y,
                    action="move" if self._action.is_moving(robot_id) else "wait",
                    reason=""
                )
                self.decision_logs.append(log)

    # ================================================================
    # 查询接口
    # ================================================================

    def get_state(self, robot_id: int) -> DecisionState:
        return self._fsms[robot_id].state

    def get_pass_target_id(self, robot_id: int) -> Optional[int]:
        """获取传球目标机器人 ID (PASS 状态下有效)"""
        fsm = self._fsms.get(robot_id)
        return fsm.pass_target_id if fsm else None

    def get_decision_summary(self) -> dict:
        """决策统计摘要"""
        if not self.decision_logs:
            return {}

        states = {}
        roles_count = {}
        for log in self.decision_logs:
            states[log.state] = states.get(log.state, 0) + 1
            roles_count[log.role] = roles_count.get(log.role, 0) + 1

        return {
            "total_ticks": self.tick_count,
            "total_decisions": len(self.decision_logs),
            "state_distribution": states,
            "role_distribution": roles_count,
            "total_transitions": sum(
                1 for log in self.decision_logs if log.action != "wait"
            ),
        }

    def export_csv(self, filepath: str):
        """导出决策日志为 CSV"""
        import csv
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                "tick", "timestamp", "robot_id", "state", "role",
                "x", "y", "action", "reason"
            ])
            for log in self.decision_logs:
                writer.writerow([
                    log.tick, f"{log.timestamp:.2f}", log.robot_id,
                    log.state, log.role, f"{log.x:.2f}", f"{log.y:.2f}",
                    log.action, log.reason
                ])

    def reset_round(self) -> None:
        """开球回合: 清空状态机到 IDLE。"""
        for fsm in self._fsms.values():
            fsm.state = DecisionState.IDLE
            fsm.previous_state = DecisionState.IDLE
            fsm.state_timer = 0.0
            fsm.target = None
            fsm.pass_target_id = None
        self._ball_carrier_id = None
        self._supporter_id = None
        self.transitions.clear()
