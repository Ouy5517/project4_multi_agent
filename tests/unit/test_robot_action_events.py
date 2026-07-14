from common.robot_action import MockRobotAction
from simulation.field_simulator import Simulator


def test_kick_rejects_missing_robot_with_event():
    action = MockRobotAction(Simulator())

    assert action.kick(999, 50, 0) is False

    event = action.drain_events()[0]
    assert event.accepted is False
    assert event.reject_code == "ROBOT_NOT_FOUND"


def test_kick_rejects_out_of_range_and_cooldown():
    sim = Simulator()
    action = MockRobotAction(sim)
    robot = sim.get_robot_by_id(0)
    sim.ball.x = robot.x + 2.0

    assert action.kick(0, 50, 0) is False
    assert action.drain_events()[0].reject_code == "KICK_OUT_OF_RANGE"

    sim.ball.x = robot.x
    robot.kick_cooldown = 0.5
    assert action.kick(0, 50, 0) is False
    assert action.drain_events()[0].reject_code == "KICK_COOLDOWN"


def test_stop_and_reset_return_action_events():
    sim = Simulator()
    action = MockRobotAction(sim)
    action.move_to(0, 1.0, 0.0)
    action.drain_events()

    stop_event = action.stop(0)
    reset_event = action.reset()

    assert stop_event.action == "stop"
    assert reset_event.action == "reset"
