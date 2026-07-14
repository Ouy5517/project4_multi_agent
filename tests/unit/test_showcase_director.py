from common.config import DT
from simulation.field_simulator import Simulator
from simulation.showcase import SHOWCASE_PHASES, ShowcaseDirector


def test_showcase_covers_every_required_behavior_in_order():
    assert [phase.key for phase in SHOWCASE_PHASES] == [
        "PASS_RECEIVE",
        "DRIBBLE",
        "POSITION",
        "BLOCK",
        "ATTACK_DEFENSE",
    ]
    assert all(phase.duration_s >= 8.0 for phase in SHOWCASE_PHASES)


def test_showcase_switches_scenario_when_phase_duration_elapses():
    simulator = Simulator()
    director = ShowcaseDirector(simulator)
    assert director.phase.key == "PASS_RECEIVE"
    assert simulator.ball.position == (-2.0, 0.0)

    changed = False
    for _ in range(int(director.phase.duration_s / DT) + 1):
        changed = director.advance(simulator, DT) or changed

    assert changed
    assert director.phase.key == "DRIBBLE"
    assert simulator.ball.position == (-1.5, 0.0)


def test_block_phase_opponent_actively_moves_ball_toward_blue_goal():
    simulator = Simulator()
    director = ShowcaseDirector(simulator)
    while director.phase.key != "BLOCK":
        director.next_phase(simulator)

    start_ball_x = simulator.ball.x
    for _ in range(30):
        director.control_opponents(simulator)
        simulator.update(DT)

    assert simulator.ball.x < start_ball_x - 0.20

