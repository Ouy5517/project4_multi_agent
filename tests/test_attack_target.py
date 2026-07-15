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
from src.tactics.navigation import ObstacleCollector
from src.tactics.targeting.attack import best_pass_target, select_kick_target


def _allowed(_game: GameControlState, _player_id: int) -> bool:
    return True


def _context(ball: BallState, teammate_pose: Pose2D | None = None) -> PlayContext:
    teammates = {1: RobotState(1, pose=Pose2D(ball.x - 0.4, ball.y, 0.0))}
    if teammate_pose is not None:
        teammates[2] = RobotState(2, pose=teammate_pose)
    return PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates=teammates,
        ball=ball,
    )


def test_pass_aims_into_space_ahead_of_receiver():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    obstacles = ObstacleCollector(config, field)
    receiver = Pose2D(2.0, 0.8, 0.0)

    target = best_pass_target(
        config,
        obstacles,
        1,
        _context(BallState(0.0, 0.0, last_seen_at=1.0), receiver),
        _allowed,
    )

    assert target is not None
    assert target.x > receiver.x
    assert math.isclose(target.y, receiver.y)


def test_pass_is_rejected_when_opponent_blocks_lane():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    obstacles = ObstacleCollector(config, field)
    context = _context(
        BallState(0.0, 0.0, last_seen_at=1.0),
        Pose2D(2.0, 0.0, 0.0),
    )
    context.opponents[4] = RobotState(4, pose=Pose2D(1.0, 0.0, 0.0))

    target = best_pass_target(config, obstacles, 1, context, _allowed)

    assert target is None


def test_open_shot_in_final_third_takes_priority_over_pass():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    obstacles = ObstacleCollector(config, field)
    ball = BallState(4.2, 0.1, last_seen_at=1.0)

    target = select_kick_target(
        config,
        field,
        obstacles,
        1,
        _context(ball, Pose2D(5.0, 0.8, 0.0)),
        _allowed,
    )

    assert target.x == field.opponent_goal_x()
    assert target.y == 0.0


def test_clear_lane_from_midfield_dribbles_instead_of_long_shot():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    obstacles = ObstacleCollector(config, field)
    ball = BallState(0.0, 0.4, last_seen_at=1.0)

    target = select_kick_target(
        config,
        field,
        obstacles,
        1,
        _context(ball),
        _allowed,
    )

    assert target.x == ball.x + config.strategy.dribble_advance_m
    assert target.x < field.opponent_goal_x()
