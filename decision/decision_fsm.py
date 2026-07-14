#!/usr/bin/env python3
"""
决策状态机 (整合固定点传球)
===========================
多机器人协同决策引擎：
- 通用状态: IDLE → CHASE → (DRIBBLE | PASS | SHOOT) → ... → IDLE
                           ↓
                         BLOCK (防守状态)
- 固定点传球场景: 内置 MockTeamBus + FixedPointPassFSM
  INIT → WAIT_RECEIVER → PASSING → DONE / FAILED
- 角色分配: ball_carrier, supporter, defender
- 决策日志: 记录每次状态转换和决策原因
"""

from typing import Dict, List, Optional, Tuple
from enum import Enum, auto
from dataclasses import dataclass, field
import math

from common.config import (
    DT, REEVALUATE_INTERVAL, STATE_MIN_DURATION,
    ROBOT_KICK_RANGE, SHOOT_RANGE, OUR_GOAL_X, GOAL_X,
    POSSESSION_CONTESTED_RANGE,
    TEAM_BLUE
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

# 固定点传球模块 (可选依赖)
try:
    from common.models import Vec2 as FixedVec2
    from communication.mock_team_bus import MockTeamBus, MessageType, TeamMessage
    from decision.pass_fsm import FixedPointPassFSM, PassConfig, PassState
    _FIXED_PASS_AVAILABLE = True
except ImportError:
    _FIXED_PASS_AVAILABLE = False


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
    # 固定点传球专用状态
    FIXED_PASS = "FIXED_PASS"  # 固定坐标点模拟通信传球


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
    agent: str = ""            # Fixed-point: "R1" / "R2" / "FSM"
    state: str = ""
    role: str = ""
    x: float = 0.0
    y: float = 0.0
    action: str = ""           # "move", "kick", "stop", "wait"
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
# 固定点传球场景结果
# ================================================================

@dataclass
class FixedPassResult:
    """固定点传球完成结果"""
    success: bool
    final_state: str
    elapsed_s: float
    ball_owner_id: str
    message_count: int
    events: list = field(default_factory=list)


# ================================================================
# 顶层决策引擎
# ================================================================

class DecisionFSM:
    """
    多机器人协同决策引擎。
    每帧调用 update() 驱动所有机器人的决策。

    支持两种模式:
    1. 通用模式 (默认): 基于策略模块的动态决策
    2. 固定点传球模式 (--scenario fixed-pass): 基于 MockTeamBus 的模拟通信传球
    """

    # ---- 固定点传球默认参数 (可通过 init_fixed_pass 覆盖) ----
    FIXED_PASS_DEFAULTS = {
        "passer_id": "R1",
        "receiver_id": "R2",
        "target_x": 7.0,
        "target_y": 4.0,
        "receiver_speed_mps": 1.5,
        "ball_speed_mps": 4.0,
        "arrival_tolerance_m": 0.12,
        "timeout_s": 15.0,
    }

    def __init__(self, world_state: WorldState,
                 action: RobotActionInterface,
                 num_robots: int = 3):
        self._ws = world_state
        self._action = action
        self._num_robots = num_robots
        self._opp_tick_counter = 0
        self._opp_reaction_interval = 3

        # 策略模块
        self.pass_strategy = PassStrategy(world_state)
        self.dribble_strategy = DribbleStrategy(world_state)
        self.shoot_strategy = ShootStrategy(world_state)
        self.position_strategy = PositionStrategy(world_state)
        self.block_strategy = BlockStrategy(world_state)

        # 每个机器人一个状态机
        self._fsms: Dict[int, RobotFSM] = {}
        for i in range(num_robots):
            self._fsms[i] = RobotFSM(i)

        # 角色
        self._ball_carrier_id: Optional[int] = None
        self._supporter_id: Optional[int] = None
        self._supporter_id: Optional[int] = None

        # 日志
        self.transitions: List[FSMTransition] = []  # 当前帧的转换
        self.decision_logs: List[DecisionLog] = []  # 累积日志

        # 回合统计
        self.tick_count: int = 0

        # ---- 固定点传球模式 ----
        self._fixed_pass_mode: bool = False
        self._fixed_pass_fsm: Optional["FixedPointPassFSM"] = None
        self._fixed_pass_bus: Optional["MockTeamBus"] = None
        self._fixed_pass_result: Optional["FixedPassResult"] = None

    # ================================================================
    # 固定点传球场景接口
    # ================================================================

    def init_fixed_pass_scenario(self, **kwargs) -> bool:
        """
        初始化固定点传球场景。
        必须在 update() 之前调用。

        可用参数 (均有默认值):
            passer_id, receiver_id, target_x, target_y,
            receiver_speed_mps, ball_speed_mps,
            arrival_tolerance_m, timeout_s

        Returns:
            True 如果初始化成功, False 如果模块不可用
        """
        if not _FIXED_PASS_AVAILABLE:
            print("[DecisionFSM] 错误: 固定点传球模块不可用")
            return False

        params = dict(self.FIXED_PASS_DEFAULTS)
        params.update(kwargs)

        # 创建配置和消息总线
        config = PassConfig(
            passer_id=params["passer_id"],
            receiver_id=params["receiver_id"],
            fixed_target=FixedVec2(params["target_x"], params["target_y"]),
            receiver_speed_mps=params["receiver_speed_mps"],
            ball_speed_mps=params["ball_speed_mps"],
            arrival_tolerance_m=params["arrival_tolerance_m"],
            timeout_s=params["timeout_s"],
        )
        self._fixed_pass_bus = MockTeamBus()
        self._fixed_pass_fsm = FixedPointPassFSM(
            config, self._fixed_pass_bus, self._fixed_pass_record_event
        )
        self._fixed_pass_mode = True
        self._fixed_pass_result = None

        # 将所有机器人设为 FIXED_PASS 状态
        for fsm in self._fsms.values():
            fsm.transition(DecisionState.FIXED_PASS, "entering fixed-point pass scenario")

        return True

    def _fixed_pass_record_event(
        self, time_s: float, actor: str, action: str, result: str, detail: str
    ) -> None:
        """固定点传球事件回调 — 桥接到 DecisionFSM 日志系统"""
        self.decision_logs.append(DecisionLog(
            tick=self.tick_count,
            timestamp=time_s,
            robot_id=0,
            agent=actor,
            state=self._fixed_pass_fsm.state.value if self._fixed_pass_fsm else "?",
            role="",
            x=0.0, y=0.0,
            action=action,
            action_params={"result": result, "detail": detail},
            reason=detail,
        ))

    @property
    def is_fixed_pass_mode(self) -> bool:
        return self._fixed_pass_mode

    @property
    def fixed_pass_result(self) -> Optional["FixedPassResult"]:
        return self._fixed_pass_result

    # ================================================================
    # 主更新
    # ================================================================

    def update(self, world_state: WorldState, dt: float = DT):
        """
        每帧调用。
        如果是固定点传球模式, 走简化仿真路径;
        否则走通用决策路径。
        """
        self._ws = world_state
        self.transitions.clear()
        self.tick_count += 1

        # ---- 固定点传球模式 ----
        if self._fixed_pass_mode:
            self._update_fixed_pass(dt)
            return

        # ---- 通用模式 ----
        # 更新策略模块的世界状态
        self.pass_strategy.update_world_state(world_state)
        self.dribble_strategy.update_world_state(world_state)
        self.shoot_strategy.update_world_state(world_state)
        self.position_strategy.update_world_state(world_state)
        self.block_strategy.update_world_state(world_state)

        # 分配角色
        roles = self._assign_roles()

        # 对手 AI
        self._update_opponents()

        # 对每个机器人执行决策
        for robot_id in range(self._num_robots):
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
    # 固定点传球更新
    # ================================================================

    def _update_fixed_pass(self, dt: float):
        """固定点传球模式的每帧更新"""
        if self._fixed_pass_fsm is None:
            return

        # 固定点传球使用自己的简化世界状态
        from common.models import BallState, RobotState, WorldState as FixedWorldState, Vec2 as FixedVec2

        # 从 FSM 配置中推断当前状态
        config = self._fixed_pass_fsm.config
        fsm = self._fixed_pass_fsm

        # 从消息历史推断 R2 当前位置
        receiver_pos = FixedVec2(5.0, 1.5)  # 初始位置
        for msg in (self._fixed_pass_bus.history if self._fixed_pass_bus else []):
            # R2 移动追踪 — 简化: 用事件日志中的最后位置
            pass

        # 构建当前时刻的简化世界状态
        # 注: 此时 world_state 由 FixedPointSimulator 内部维护
        # 这里仅填充 FSM.update 所需的最小字段
        fixed_ws = FixedWorldState(
            time_s=self.tick_count * dt,
            field_width=12.0,
            field_height=8.0,
            robots={
                config.passer_id: RobotState(
                    robot_id=config.passer_id,
                    position=FixedVec2(2.0, 4.0),
                    role="passer",
                    has_ball=(fsm.state not in (PassState.PASSING, PassState.DONE)),
                ),
                config.receiver_id: RobotState(
                    robot_id=config.receiver_id,
                    position=FixedVec2(5.0, 1.5),
                    role="receiver",
                    has_ball=(fsm.state == PassState.DONE),
                ),
            },
            ball=BallState(
                position=FixedVec2(2.2, 4.0),
                owner_id=(
                    config.passer_id if fsm.state in (PassState.INIT, PassState.WAIT_RECEIVER)
                    else (config.receiver_id if fsm.state == PassState.DONE else None)
                ),
            ),
        )

        self._fixed_pass_fsm.update(fixed_ws, dt)

        # 检查是否完成
        if fsm.state in (PassState.DONE, PassState.FAILED):
            self._fixed_pass_result = FixedPassResult(
                success=(fsm.state == PassState.DONE),
                final_state=fsm.state.value,
                elapsed_s=round(self.tick_count * dt, 2),
                ball_owner_id=config.receiver_id if fsm.state == PassState.DONE else config.passer_id,
                message_count=len(self._fixed_pass_bus.history) if self._fixed_pass_bus else 0,
            )

    # ================================================================
    # 角色分配
    # ================================================================

    # ---- Opponent AI (Active Interference) ----

    def _update_opponents(self):
        """Multi-role opponent AI with active interference.

        Roles:
          - pressure: chase ball, contest possession, shoot
          - interceptor: block passing lanes between blue teammates
          - blocker: defend goal, block shot angles

        Detects blue team intentions by reading their FSM states.
        """
        if not self._ws.opponents:
            return
        self._opp_tick_counter += 1
        if self._opp_tick_counter % self._opp_reaction_interval != 0:
            return

        ball = self._ws.ball

        # Gather intel on blue team
        blue_carrier_id = self._ball_carrier_id
        blue_carrier = self._ws.get_robot_by_id(blue_carrier_id) if blue_carrier_id is not None else None
        blue_passer = None
        blue_receiver_id = None

        # Detect if blue is attempting a pass
        for rid in range(self._num_robots):
            s = self._fsms[rid].state
            if s == DecisionState.PASS:
                blue_passer = self._ws.get_robot_by_id(rid)
                blue_receiver_id = self._fsms[rid].pass_target_id
                break

        blue_receiver = self._ws.get_robot_by_id(blue_receiver_id) if blue_receiver_id is not None else None

        # Count opponents and assign roles
        num_opp = len(self._ws.opponents)
        if num_opp == 0:
            return

        if num_opp == 1:
            opp = self._ws.opponents[0]
            if blue_passer is not None and blue_receiver is not None:
                self._opp_intercept_pass(opp, blue_passer, blue_receiver, ball)
            else:
                self._opp_pressure(opp, ball, blue_carrier)

        elif num_opp == 2:
            opp1, opp2 = self._ws.opponents[0], self._ws.opponents[1]
            if blue_passer is not None and blue_receiver is not None:
                self._opp_intercept_pass(opp1, blue_passer, blue_receiver, ball)
                self._opp_defend_receiver(opp2, blue_receiver, ball)
            else:
                self._opp_pressure(opp1, ball, blue_carrier)
                self._opp_block_goal(opp2, ball)

        else:
            opp1, opp2, opp3 = self._ws.opponents[0], self._ws.opponents[1], self._ws.opponents[2]
            if blue_passer is not None and blue_receiver is not None:
                self._opp_intercept_pass(opp1, blue_passer, blue_receiver, ball)
                self._opp_defend_receiver(opp2, blue_receiver, ball)
                self._opp_block_goal(opp3, ball)
            elif blue_carrier is not None:
                self._opp_pressure(opp1, ball, blue_carrier)
                supporter = self._ws.get_robot_by_id(self._supporter_id) if self._supporter_id is not None else None
                self._opp_intercept_pass(opp2, blue_carrier, supporter, ball)
                self._opp_block_goal(opp3, ball)
            else:
                self._opp_pressure(opp1, ball, blue_carrier)
                self._opp_block_goal(opp2, ball)
                self._opp_block_goal(opp3, ball)

    # ---- Individual opponent behaviors ----

    def _opp_pressure(self, opp, ball, carrier):
        """Pressure the ball: chase, contest, tackle, shoot toward blue goal."""
        our_goal_center = self._ws.our_goal.center
        dist_to_ball = ((opp.x - ball.x)**2 + (opp.y - ball.y)**2) ** 0.5

        if dist_to_ball < ROBOT_KICK_RANGE * 1.2:
            # Next to ball! Kick toward blue goal
            angle_to_blue_goal = math.atan2(our_goal_center[1] - opp.y,
                                           our_goal_center[0] - opp.x)
            self._action.kick(opp.id, 85, angle_to_blue_goal)
        elif dist_to_ball < 2.0:
            self._action.move_to(opp.id, ball.x, ball.y)
        else:
            # Position between ball and our goal
            mid_x = (ball.x + our_goal_center[0]) / 2
            mid_y = (ball.y + our_goal_center[1]) / 2
            self._action.move_to(opp.id, mid_x, mid_y)

    def _opp_intercept_pass(self, opp, passer, receiver, ball):
        """Intercept a pass: stand on the passing lane."""
        if receiver is None:
            self._opp_pressure(opp, ball, passer)
            return
        # Position on passing line, closer to passer
        mid_x = passer.x * 0.4 + receiver.x * 0.6
        mid_y = passer.y * 0.4 + receiver.y * 0.6
        # Add small offset perpendicular to the line
        dx = receiver.x - passer.x
        dy = receiver.y - passer.y
        d = ((dx)**2 + (dy)**2) ** 0.5
        if d > 0.01:
            perp_x = -dy / d * 0.3
            perp_y = dx / d * 0.3
            mid_x += perp_x
            mid_y += perp_y
        self._action.move_to(opp.id, mid_x, mid_y)

    def _opp_defend_receiver(self, opp, receiver, ball):
        """Mark a receiving blue robot tightly."""
        if receiver is None:
            return
        goal = self._ws.opponent_goal
        dx_away = receiver.x - goal.x
        dy_away = receiver.y - goal.center[1]
        d = ((dx_away)**2 + (dy_away)**2) ** 0.5
        if d < 0.1:
            target_x, target_y = receiver.x, receiver.y
        else:
            target_x = receiver.x - dx_away * 0.5 / d
            target_y = receiver.y - dy_away * 0.5 / d
        self._action.move_to(opp.id, target_x, target_y)

    def _opp_block_goal(self, opp, ball):
        """Block between ball and own goal (last line of defense)."""
        goal = self._ws.opponent_goal
        goal_cx = goal.x
        goal_cy = goal.center[1]
        dx = ball.x - goal_cx
        dy = ball.y - goal_cy
        d = ((dx)**2 + (dy)**2) ** 0.5
        ratio = 1.0 / max(d, 1.0)
        target_x = goal_cx + dx * ratio
        target_y = goal_cy + dy * ratio
        self._action.move_to(opp.id, target_x, target_y)

    # ---- End Opponent AI ----

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

        elif fsm.state == DecisionState.FIXED_PASS:
            pass  # 固定点传球由 _update_fixed_pass 统一处理

    # ---- IDLE ----

    def _handle_idle(self, robot_id: int, role: RobotRole, fsm: RobotFSM):
        if role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "assigned BALL_CARRIER")
        elif role == RobotRole.SUPPORTER:
            target = self.position_strategy.calculate_support_position(
                self._ball_carrier_id or 0, robot_id)
            self._action.move_to(robot_id, *target)
        elif role == RobotRole.DEFENDER:
            target = self.block_strategy.calculate_defensive_position(robot_id)
            self._action.move_to(robot_id, *target)

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
            # 追球
            _, tx, ty = self.dribble_strategy.approach_ball(robot_id)
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
        target_x = GOAL_X
        target_y = 0.0  # 球门中心

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
        # 如果重新分配为持球者
        if role == RobotRole.BALL_CARRIER:
            fsm.transition(DecisionState.CHASE, "now ball carrier")
            return

        if not self.block_strategy.is_goal_threatened():
            fsm.transition(DecisionState.CHASE, "threat cleared")
            return

        target = self.block_strategy.calculate_defensive_position(robot_id)
        self._action.move_to(robot_id, *target)

    # ---- 评估下一步动作 ----

    def _evaluate_next_action(self, robot_id: int, fsm: RobotFSM):
        """
        在控制球后评估最佳动作: SHOOT > PASS > DRIBBLE
        """
        # 先评估射门
        shoot_eval = self.shoot_strategy.evaluate_shoot_opportunity(robot_id)
        if shoot_eval.is_viable and shoot_eval.score > 0.5:
            fsm.transition(DecisionState.SHOOT, f"shoot opportunity (score={shoot_eval.score:.2f})")
            return

        # 再评估传球
        pass_options = self.pass_strategy.evaluate_pass_options(robot_id)
        if pass_options and pass_options[0].score > 0.4:
            fsm.transition(DecisionState.PASS,
                          f"pass to {pass_options[0].receiver_id} (score={pass_options[0].score:.2f})")
            fsm.pass_target_id = pass_options[0].receiver_id
            return

        # 默认带球
        fsm.transition(DecisionState.DRIBBLE, "no better option")

    # ================================================================
    # 日志
    # ================================================================

    def _log_decisions(self, ws: WorldState, roles: Dict[int, RobotRole]):
        """记录当前帧的决策"""
        for robot_id in range(self._num_robots):
            fsm = self._fsms[robot_id]
            robot = ws.get_robot_by_id(robot_id)
            role = roles.get(robot_id, RobotRole.IDLE)

            if robot:
                log = DecisionLog(
                    tick=self.tick_count,
                    timestamp=ws.timestamp,
                    robot_id=robot_id,
                    agent=f"robot_{robot_id}",
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
                "tick", "timestamp", "robot_id", "agent", "state", "role",
                "x", "y", "action", "reason"
            ])
            for log in self.decision_logs:
                writer.writerow([
                    log.tick, f"{log.timestamp:.2f}", log.robot_id,
                    log.agent, log.state, log.role,
                    f"{log.x:.2f}", f"{log.y:.2f}",
                    log.action, log.reason
                ])
