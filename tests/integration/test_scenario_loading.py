from common.world_state import WorldStateProvider
from simulation.field_simulator import Simulator
from simulation.scenarios import load_scenario_into_simulator


def test_pass_scenario_initializes_the_live_simulator():
    sim = Simulator()
    load_scenario_into_simulator(sim, "pass_fixed")
    ws = WorldStateProvider(sim).get()
    assert ws.ball.position == (-2.0, 0.0)
    assert ws.get_robot_by_id(0).position == (-2.0, 0.2)
    assert ws.get_robot_by_id(1).position == (0.0, 1.5)


def test_scenario_state_changes_after_action_and_step():
    sim = Simulator()
    load_scenario_into_simulator(sim, "pass_fixed")
    before = WorldStateProvider(sim).get()
    sim.queue_kick(0, 60.0, 0.0)
    sim.update(1 / 30)
    after = WorldStateProvider(sim).get()
    assert after.timestamp > before.timestamp
    assert after.ball.x > before.ball.x
