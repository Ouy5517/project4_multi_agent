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
