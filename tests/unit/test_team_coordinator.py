from common.config import ROLE_HOLD_SECONDS, ROLE_SWITCH_MARGIN
from common.world_state import Ball, Goal, Robot, Team, RobotRole, WorldState
from decision.team_coordinator import TeamCoordinator


def make_world(timestamp, r0_x, r1_x):
    return WorldState(
        ball=Ball(x=0.0, y=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=r0_x, y=0.0),
            Robot(id=1, team=Team.BLUE, x=r1_x, y=0.0),
            Robot(id=2, team=Team.BLUE, x=2.0, y=0.0),
        ],
        opponents=[],
        our_goal=Goal(x=-4.5, y_min=-1.0, y_max=1.0),
        opponent_goal=Goal(x=4.5, y_min=-1.0, y_max=1.0),
        timestamp=timestamp,
    )


def test_ball_carrier_role_does_not_flip_within_hold_window():
    coordinator = TeamCoordinator()

    plan = coordinator.plan(make_world(0.0, r0_x=0.20, r1_x=0.22))
    assert plan.roles[0] == RobotRole.BALL_CARRIER

    plan = coordinator.plan(make_world(ROLE_HOLD_SECONDS / 2, r0_x=0.23, r1_x=0.21))

    assert plan.roles[0] == RobotRole.BALL_CARRIER
    assert plan.roles[1] == RobotRole.SUPPORTER


def test_ball_carrier_switches_after_margin_and_hold_window():
    coordinator = TeamCoordinator()
    coordinator.plan(make_world(0.0, r0_x=0.20, r1_x=0.22))

    plan = coordinator.plan(
        make_world(ROLE_HOLD_SECONDS + 0.1, r0_x=0.20 + ROLE_SWITCH_MARGIN + 0.05, r1_x=0.02)
    )

    assert plan.roles[1] == RobotRole.BALL_CARRIER
    assert plan.roles[0] == RobotRole.SUPPORTER
