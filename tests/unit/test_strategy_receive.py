from common.world_state import Ball, Goal, Robot, Team, WorldState
from strategy.strategy_receive import ReceiveStrategy


def make_world():
    return WorldState(
        ball=Ball(x=-1.0, y=0.0, vx=2.0, vy=0.0),
        teammates=[
            Robot(id=0, team=Team.BLUE, x=-2.0, y=0.0),
            Robot(id=1, team=Team.BLUE, x=0.0, y=0.0),
        ],
        opponents=[],
        our_goal=Goal(x=-4.5, y_min=-1.0, y_max=1.0),
        opponent_goal=Goal(x=4.5, y_min=-1.0, y_max=1.0),
    )


def test_receive_strategy_predicts_reachable_intercept():
    strategy = ReceiveStrategy(make_world())

    point = strategy.predict_receive_point(receiver_id=1)

    assert -0.25 <= point[0] <= 0.5
    assert abs(point[1]) < 0.01


def test_receive_strategy_detects_controlled_slow_ball():
    ws = make_world()
    ws.ball.x = 0.05
    ws.ball.vx = 0.5
    strategy = ReceiveStrategy(ws)

    assert strategy.has_received(receiver_id=1) is True
