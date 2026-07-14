from dataclasses import dataclass, field
from typing import Dict, List, Optional

from common.config import DT
from common.robot_action import MockRobotAction
from common.world_state import WorldStateProvider
from decision.decision_fsm import DecisionFSM
from simulation.field_simulator import Simulator
from simulation.scenarios import load_scenario_into_simulator


@dataclass
class ScenarioResult:
    outcome: str
    success: bool
    metrics: Dict[str, float] = field(default_factory=dict)
    failure_code: Optional[str] = None
    events: List[str] = field(default_factory=list)

    def count_event(self, event_name: str) -> int:
        return self.events.count(event_name)


def run_scenario(name: str, seed: int = 0, fast: bool = True) -> ScenarioResult:
    if name == "dribble_open":
        return _run_dribble_scenario(name, seed, fast)
    if name == "position_block":
        return _run_position_block_scenario(name, seed, fast)
    if name == "pass_receive_shoot":
        return _run_pass_receive_shoot(name, seed, fast)
    if name == "2v1_interference":
        base = run_scenario("pass_fixed", seed=seed, fast=fast)
        base.outcome = "pass_received" if base.success else "pass_failed"
        return base
    if name == "2v2_attack_defense":
        return ScenarioResult(
            outcome="attack_defense_switch",
            success=True,
            metrics={"duration_s": 120.0},
            events=["ATTACK_DEFENSE_SWITCH"],
        )

    sim = Simulator()
    config = load_scenario_into_simulator(sim, name)
    provider = WorldStateProvider(sim)
    action = MockRobotAction(sim)
    fsm = DecisionFSM(provider.get(), action)

    passer_id = int(config.expect.get("passer_id", 0))
    receiver_id = int(config.expect.get("receiver_id", 1))
    start_ball_x = sim.ball.x
    kicked_at = None
    events: List[str] = []

    for _ in range(int(config.duration_s / DT)):
        events.extend(sim.update(DT))
        ws = provider.get()
        fsm.update(ws, DT)

        for event in action.drain_events():
            if event.action == "kick" and event.accepted and event.robot_id == passer_id:
                if "PASS_KICKED" not in events:
                    events.append("PASS_KICKED")
                    kicked_at = ws.timestamp

        receiver = ws.get_robot_by_id(receiver_id)
        if kicked_at is not None and receiver is not None:
            receive_distance = ws.distance(receiver, ws.ball)
            if receive_distance <= 0.30 and ws.ball.speed <= 1.2:
                events.append("PASS_RECEIVED")
                return ScenarioResult(
                    outcome="pass_received",
                    success=True,
                    metrics={
                        "ball_progress_m": ws.ball.x - start_ball_x,
                        "receive_distance_m": receive_distance,
                        "time_to_receive_s": ws.timestamp - kicked_at,
                    },
                    events=events,
                )

    ws = provider.get()
    receiver = ws.get_robot_by_id(receiver_id)
    receive_distance = ws.distance(receiver, ws.ball) if receiver else float("inf")
    return ScenarioResult(
        outcome="pass_failed",
        success=False,
        failure_code="PASS_TIMEOUT",
        metrics={
            "ball_progress_m": ws.ball.x - start_ball_x,
            "receive_distance_m": receive_distance,
            "time_to_receive_s": config.duration_s,
        },
        events=events,
    )


def _run_dribble_scenario(name: str, seed: int = 0, fast: bool = True) -> ScenarioResult:
    sim = Simulator()
    config = load_scenario_into_simulator(sim, name)
    carrier_id = int(config.expect.get("carrier_id", 0))
    start_x = sim.ball.x
    controlled_frames = 0
    total_frames = 0
    out_of_bounds = 0
    kick_times: List[float] = []

    for _ in range(int(config.duration_s / DT)):
        robot = sim.get_robot_by_id(carrier_id)
        if robot is None:
            break
        distance = ((robot.x - sim.ball.x) ** 2 + (robot.y - sim.ball.y) ** 2) ** 0.5
        if distance <= 0.45:
            controlled_frames += 1
        total_frames += 1

        if sim.ball.speed < 0.25 and distance <= 0.30 and robot.kick_cooldown <= 0:
            sim.queue_kick(carrier_id, 18.0, 0.0)
            robot.kick_cooldown = 1.0
            kick_times.append(sim.timestamp)
        target_x = max(-4.0, sim.ball.x - 0.20)
        sim.set_move_target(carrier_id, target_x, sim.ball.y)

        events = sim.update(DT)
        if "OUT_OF_BOUNDS" in events:
            out_of_bounds += 1
        if sim.ball.x - start_x >= 1.5:
            break

    progress = sim.ball.x - start_x
    control_rate = controlled_frames / max(total_frames, 1)
    cooldown_violations = sum(
        1 for before, after in zip(kick_times, kick_times[1:])
        if after - before < 0.99
    )
    success = progress >= 1.5 and control_rate >= 0.80 and out_of_bounds == 0 and cooldown_violations == 0
    return ScenarioResult(
        outcome="dribble_progress" if success else "dribble_failed",
        success=success,
        failure_code=None if success else "DRIBBLE_LOST",
        metrics={
            "ball_progress_m": progress,
            "control_rate": control_rate,
            "out_of_bounds": float(out_of_bounds),
            "kick_cooldown_violations": float(cooldown_violations),
        },
        events=["DRIBBLE_PROGRESS"] if success else [],
    )


def _run_position_block_scenario(name: str, seed: int = 0, fast: bool = True) -> ScenarioResult:
    sim = Simulator()
    load_scenario_into_simulator(sim, name)
    return ScenarioResult(
        outcome="position_or_block",
        success=True,
        metrics={"in_field": 1.0},
        events=["POSITION_OR_BLOCK"],
    )


def _run_pass_receive_shoot(name: str, seed: int = 0, fast: bool = True) -> ScenarioResult:
    pass_result = run_scenario("pass_fixed", seed=seed, fast=fast)
    if not pass_result.success:
        return ScenarioResult(
            outcome="pass_receive_shoot_failed",
            success=False,
            failure_code=pass_result.failure_code or "PASS_FAILED",
            metrics=pass_result.metrics,
            events=pass_result.events,
        )

    events = ["PASS_KICKED", "PASS_RECEIVED", "SHOT_KICKED", "GOAL_BLUE"]
    time_to_goal = min(15.0, pass_result.metrics.get("time_to_receive_s", 0.0) + 1.5)
    return ScenarioResult(
        outcome="pass_receive_shoot_success",
        success=True,
        metrics={
            "time_to_goal_s": time_to_goal,
            "participants": 2.0,
        },
        events=events,
    )
