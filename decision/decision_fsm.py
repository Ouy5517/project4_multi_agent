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
    ROBOT_KICK_RANGE, SHOOT_RANGE, TEAM_BLUE,
    CARRIER_STICKY_COST, CROWD_NEAR_RADIUS, CROWD_FORCE_KICK_COUNT,
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
from strategy.booster_skills import (
    adjust_behind_ball,
    calc_kick_dir,
    count_crowd_near_ball,
    is_angle_good,
    keep_clear_of_ball,
    press_flank_position,
    rank_by_ball_cost,
    should_enter_kick,
)


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
        goalkeeper_id: Optional[int] = None,
    ):
        self._ws = world_state
        self._action = action
        self._team = team
        if robot_ids is not None:
            self._robot_ids = list(robot_ids)
        else:
            self._robot_ids = list(range(num_robots))
        self._num_robots = len(self._robot_ids)

        # 固定门将: 默认名单最后一人
        if goalkeeper_id is not None:
            self.goalkeeper_id = goalkeeper_id
        else:
            self.goalkeeper_id = self._robot_ids[-1] if self._robot_ids else None

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
        """固定 GOALKEEPER + cost 竞选 lead/assist; 其余 DEFENDER。"""
        if not self._ws.teammates:
            return {}

        roles: Dict[int, RobotRole] = {}
        gk = self.goalkeeper_id
        if gk is not None and any(r.id == gk for r in self._ws.teammates):
            roles[gk] = RobotRole.GOALKEEPER
            robot = self._ws.get_robot_by_id(gk)
            if robot:
                robot.role = RobotRole.GOALKEEPER

        field = [r for r in self._ws.teammates if r.id != gk]
        field_ws = WorldState(
            ball=self._ws.ball,
            teammates=field,
            opponents=list(self._ws.opponents),
            our_goal=self._ws.our_goal,
            opponent_goal=self._ws.opponent_goal,
            field_width=self._ws.field_width,
            field_height=self._ws.field_height,
            timestamp=self._ws.timestamp,
        )
        ranked = rank_by_ball_cost(field_ws) if field else []
        # 持球粘滞: 避免两前锋轮流冲球挤成一团
        if self._ball_carrier_id is not None and ranked:
            costs = {rid: c for rid, c in ranked}
            if self._ball_carrier_id in costs:
                best_cost = ranked[0][1]
                if costs[self._ball_carrier_id] <= best_cost + CARRIER_STICKY_COST:
                    sticky = self._ball_carrier_id
                    ranked = [(sticky, costs[sticky])] + [
                        (rid, c) for rid, c in ranked if rid != sticky
                    ]

        self._ball_carrier_id = None
        self._supporter_id = None

        if len(ranked) >= 1:
            roles[ranked[0][0]] = RobotRole.BALL_CARRIER
            self._ball_carrier_id = ranked[0][0]
            robot = self._ws.get_robot_by_id(ranked[0][0])
            if robot:
                robot.role = RobotRole.BALL_CARRIER

        if len(ranked) >= 2:
            roles[ranked[1][0]] = RobotRole.SUPPORTER
            self._supporter_id = ranked[1][0]
            robot = self._ws.get_robot_by_id(ranked[1][0])
            if robot:
                robot.role = RobotRole.SUPPORTER

        for i in range(2, len(ranked)):
            roles[ranked[i][0]] = RobotRole.DEFENDER
            robot = self._ws.get_robot_by_id(ranked[i][0])
            if robot:
                robot.role = RobotRole.DEFENDER

        return roles

    def _move_cleared(self, robot_id: int, role: RobotRole, tx: float, ty: float):
        """非持球人强制远离球, 防挤堆。"""
        if role != RobotRole.BALL_CARRIER:
            ball = self._ws.ball
            tx, ty = keep_clear_of_ball(tx, ty, ball.x, ball.y)
        self._action.move_to(robot_id, tx, ty)

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
        if role == RobotRole.GOALKEEPER:
            fsm.transition(DecisionState.BLOCK, "goalkeeper on line")
        elif role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "assigned BALL_CARRIER")
        elif role == RobotRole.SUPPORTER:
            if not self._ws.team_has_possession():
                target = press_flank_position(self._ws, robot_id)
                self._move_cleared(robot_id, role, *target)
                fsm.transition(DecisionState.CHASE, "press flank when contested")
            else:
                target = self.position_strategy.calculate_support_position(
                    self._ball_carrier_id or robot_id, robot_id)
                self._move_cleared(robot_id, role, *target)
        elif role == RobotRole.DEFENDER:
            fsm.transition(DecisionState.BLOCK, "hold defensive line")

    # ---- CHASE: 追球 ----

    def _handle_chase(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if role == RobotRole.GOALKEEPER or role == RobotRole.DEFENDER:
            fsm.transition(DecisionState.BLOCK, "back to defensive line")
            return
        if role == RobotRole.SUPPORTER:
            # 支援永不踩球: 争球站侧翼, 有球权站接应
            if not self._ws.team_has_possession():
                target = press_flank_position(self._ws, robot_id)
            else:
                target = self.position_strategy.calculate_support_position(
                    self._ball_carrier_id or robot_id, robot_id)
            self._move_cleared(robot_id, role, *target)
            return

        ball = self._ws.ball
        robot = self._ws.get_robot_by_id(robot_id)
        if robot is None:
            return

        kick_dir, _mode = calc_kick_dir(self._ws)
        dist = self._ws.distance(robot, ball)
        crowd = count_crowd_near_ball(
            self._ws, CROWD_NEAR_RADIUS, exclude_id=robot_id
        )
        # 挤死时优先解围, 不等完美角度
        if dist <= ROBOT_KICK_RANGE * 1.35 and crowd >= CROWD_FORCE_KICK_COUNT:
            self._action.turn_to(robot_id, kick_dir)
            self._action.kick(robot_id, 80.0, kick_dir)
            fsm.transition(DecisionState.DRIBBLE, "crowd clear kick")
            return

        if self.dribble_strategy.is_ball_controlled(robot_id) or should_enter_kick(
            robot, ball.x, ball.y, kick_dir
        ):
            if fsm.can_reevaluate() or self.dribble_strategy.is_ball_controlled(robot_id):
                self._evaluate_next_action(robot_id, fsm)
            else:
                tx, ty = adjust_behind_ball(ball.x, ball.y, kick_dir)
                self._action.move_to(robot_id, tx, ty)
                self._action.turn_to(robot_id, kick_dir)
        else:
            _arrived, tx, ty = self.dribble_strategy.approach_ball(robot_id, kick_dir)
            tx += ball.vx * 0.18
            ty += ball.vy * 0.18
            self._action.move_to(robot_id, tx, ty)
            self._action.turn_to(robot_id, math.atan2(ty - robot.y, tx - robot.x))

    # ---- DRIBBLE: 带球 ----

    def _handle_dribble(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if role != RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.IDLE, "lost ball carrier role")
            return

        if not self.dribble_strategy.is_ball_controlled(robot_id):
            fsm.transition(DecisionState.CHASE, "lost ball")
            return

        kick_dir, kick_mode = calc_kick_dir(self._ws)
        goal = self._ws.opponent_goal
        if kick_mode == "cross":
            target_x = goal.x - math.copysign(1.5, goal.x)
            target_y = 0.0
        else:
            target_x = goal.x
            target_y = goal.center[1]

        should_dribble, direction, power, dist = \
            self.dribble_strategy.dribble_toward(robot_id, target_x, target_y)
        direction = kick_dir

        if should_dribble:
            robot = self._ws.get_robot_by_id(robot_id)
            if robot:
                self._action.turn_to(robot_id, direction)
                self._action.kick(robot_id, power, direction)
            behind_x, behind_y = self.dribble_strategy._position_behind_ball(
                robot_id, direction)
            self._action.move_to(robot_id, behind_x, behind_y)

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
        robot = self._ws.get_robot_by_id(robot_id)
        ball = self._ws.ball
        evaluation = self.shoot_strategy.evaluate_shoot_opportunity(robot_id)
        angle_ok = (
            robot is not None
            and is_angle_good(robot, ball.x, ball.y, self._ws.opponent_goal)
        )
        crowd = count_crowd_near_ball(
            self._ws, CROWD_NEAR_RADIUS, exclude_id=robot_id
        )
        if evaluation.is_viable and (angle_ok or crowd >= CROWD_FORCE_KICK_COUNT):
            self._action.turn_to(robot_id, evaluation.best_angle)
            self._action.kick(robot_id, evaluation.power, evaluation.best_angle)
        elif robot is not None and not angle_ok:
            kick_dir, _ = calc_kick_dir(self._ws)
            tx, ty = adjust_behind_ball(ball.x, ball.y, kick_dir)
            self._action.move_to(robot_id, tx, ty)
            self._action.turn_to(robot_id, kick_dir)

        if fsm.state_timer > 2.0:
            fsm.transition(DecisionState.CHASE, "shoot completed")

    # ---- BLOCK: 防守 ----

    def _handle_block(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        from common.config import GOALIE_CHASE_RANGE
        from strategy.booster_skills import goal_line_block_position

        if role == RobotRole.GOALKEEPER:
            ball = self._ws.ball
            robot = self._ws.get_robot_by_id(robot_id)
            our_gx = self._ws.our_goal.x
            in_own_half = abs(ball.x - our_gx) < abs(ball.x - self._ws.opponent_goal.x)
            if robot is not None and in_own_half:
                dist = self._ws.distance(robot, ball)
                if dist <= GOALIE_CHASE_RANGE and dist <= ROBOT_KICK_RANGE * 1.3:
                    kick_dir, _ = calc_kick_dir(self._ws, defending_clear=True)
                    self._action.turn_to(robot_id, kick_dir)
                    self._action.kick(robot_id, 75.0, kick_dir)
                elif dist <= GOALIE_CHASE_RANGE:
                    # 门将逼近也避免踩进人群中心: 球后小偏移
                    kick_dir, _ = calc_kick_dir(self._ws, defending_clear=True)
                    tx, ty = adjust_behind_ball(ball.x, ball.y, kick_dir)
                    self._action.move_to(robot_id, tx, ty)
                    return
            target = goal_line_block_position(self._ws, as_goalkeeper=True)
            self._move_cleared(robot_id, role, *target)
            return

        if role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "now ball carrier")
            return
        if role == RobotRole.SUPPORTER and self._ws.team_has_possession():
            fsm.transition(DecisionState.IDLE, "support after regain")
            return

        # 卡位站线, 不再 45% 混向球 (那是挤堆主因之一)
        target = self.block_strategy.calculate_defensive_position(robot_id)
        self._move_cleared(robot_id, role, *target)

    # ---- 评估下一步动作 ----

    def _evaluate_next_action(self, robot_id: int, fsm: RobotFSM):
        """SHOOT (需射击窗) > PASS > DRIBBLE/cross。"""
        kick_dir, kick_mode = calc_kick_dir(self._ws)
        robot = self._ws.get_robot_by_id(robot_id)
        ball = self._ws.ball

        shoot_eval = self.shoot_strategy.evaluate_shoot_opportunity(robot_id)
        angle_ok = (
            robot is not None
            and is_angle_good(robot, ball.x, ball.y, self._ws.opponent_goal)
        )
        if (
            kick_mode == "shoot"
            and shoot_eval.is_viable
            and shoot_eval.score > 0.40
            and angle_ok
        ):
            fsm.transition(
                DecisionState.SHOOT,
                f"shoot opportunity (score={shoot_eval.score:.2f})",
            )
            return

        pass_options = self.pass_strategy.evaluate_pass_options(robot_id)
        press_pass_boost = 0.0
        opp = self._ws.closest_opponent_to_ball()
        if opp is not None and robot is not None:
            if self._ws.distance(robot, opp) < 1.2:
                press_pass_boost = 0.15
        if kick_mode == "cross":
            press_pass_boost += 0.12
        pass_threshold = 0.35 - press_pass_boost
        if pass_options and pass_options[0].score > pass_threshold:
            fsm.transition(
                DecisionState.PASS,
                f"pass to {pass_options[0].receiver_id} (score={pass_options[0].score:.2f})",
            )
            fsm.pass_target_id = pass_options[0].receiver_id
            return

        fsm.transition(DecisionState.DRIBBLE, f"advance ({kick_mode})")

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
