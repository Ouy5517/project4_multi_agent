import math

from src.soccer_framework import (
    BallState,
    GameControlState,
    GameState,
    PlayContext,
    Pose2D,
    RobotState,
    SoccerConfig,
)
from src.tactics.geometry import TeamFieldFrame
from src.tactics.targeting.support import support_target


def _allowed(_game: GameControlState, _player_id: int) -> bool:
    return True


def test_supporter_moves_ahead_of_advanced_ball_as_forward_pass_option():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    ball = BallState(x=2.4, y=0.2, last_seen_at=1.0)
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(2.1, 0.1, 0.0)),
            2: RobotState(2, pose=Pose2D(-1.5, -1.0, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.0, 0.0, 0.0)),
        },
        ball=ball,
    )

    target = support_target(config, field, 2, context, _allowed)

    assert target.x > ball.x
    assert target.x < config.field_length / 2.0
    assert target.theta == math.atan2(ball.y - target.y, ball.x - target.x)


def test_supporter_stays_in_own_half_when_ball_is_not_advanced():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    ball = BallState(x=0.3, y=-0.1, last_seen_at=1.0)
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(0.1, 0.0, 0.0)),
            2: RobotState(2, pose=Pose2D(-2.0, 1.0, 0.0)),
        },
        ball=ball,
    )

    target = support_target(config, field, 2, context, _allowed)

    assert target.x <= -0.35


def test_supporter_screens_the_shot_lane_in_own_half():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    ball = BallState(x=-3.0, y=2.0, last_seen_at=1.0)
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(-2.8, 2.1, 0.0)),
            2: RobotState(2, pose=Pose2D(-1.0, -1.0, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.2, 0.0, 0.0)),
        },
        ball=ball,
    )

    target = support_target(config, field, 2, context, _allowed)

    own_goal_x = field.own_goal_x()
    lane_ratio = (target.x - own_goal_x) / (ball.x - own_goal_x)
    expected_lane_y = ball.y * lane_ratio
    assert own_goal_x < target.x < ball.x
    assert abs(target.y - expected_lane_y) <= 0.5


def test_attacking_support_target_stays_inside_field():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    ball = BallState(x=6.5, y=4.2, last_seen_at=1.0)
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={2: RobotState(2, pose=Pose2D(4.0, 2.0, 0.0))},
        ball=ball,
    )

    target = support_target(config, field, 2, context, _allowed)

    assert target.x <= config.field_length / 2.0 - 0.25
    assert target.y <= config.field_width / 2.0 - 0.25


def test_two_defensive_supporters_straddle_shot_lane():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    ball = BallState(x=-4.0, y=0.5, last_seen_at=1.0)
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(-2.5, -1.0, 0.0)),
            2: RobotState(2, pose=Pose2D(-2.5, 1.0, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.0, 0.0, 0.0)),
        },
        ball=ball,
    )

    lower = support_target(config, field, 1, context, _allowed)
    upper = support_target(config, field, 2, context, _allowed)

    assert lower.y < upper.y
    assert upper.y - lower.y >= 0.7
