from types import SimpleNamespace

from src.play.default_roles import SupporterRole
from src.play.playbook import DefaultPlaybook, ROLE_CHASER, ROLE_GOALKEEPER
from src.soccer_framework import (
    BallState,
    GameControlState,
    GameState,
    PlayContext,
    Penalty,
    Pose2D,
    RobotState,
    SoccerConfig,
)
from src.tactics.geometry import TeamFieldFrame
from src.tactics.navigation import ObstacleCollector
from src.tactics.targeting import Targeting


def _kit():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    return SimpleNamespace(
        config=config,
        targeting=Targeting(config, field, ObstacleCollector(config, field)),
        is_player_allowed=lambda game, player_id: game.is_active_player(
            config.team_id,
            player_id,
        ),
    )


def _context(p1_x: float, p2_x: float) -> PlayContext:
    return PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(p1_x, 0.0, 0.0)),
            2: RobotState(2, pose=Pose2D(p2_x, 0.0, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.0, 0.0, 0.0)),
        },
        ball=BallState(0.0, 0.0, last_seen_at=1.0),
    )


def test_chaser_hysteresis_ignores_small_score_advantage():
    playbook = DefaultPlaybook(_kit())

    assert playbook.select_chaser(_context(-2.0, -0.3)) == 2
    assert playbook.select_chaser(_context(-0.25, -0.2)) == 2


def test_chaser_hysteresis_switches_for_clear_advantage():
    playbook = DefaultPlaybook(_kit())

    assert playbook.select_chaser(_context(-2.0, -0.3)) == 2
    assert playbook.select_chaser(_context(0.0, -0.6)) == 1


def test_disallowed_player_is_never_selected_as_chaser():
    playbook = DefaultPlaybook(_kit())
    context = _context(-1.0, -0.1)
    context.known_game.teams[0].players[1].penalty = Penalty.PUSHING

    assert playbook.select_chaser(context) == 1


def test_goalkeeper_owns_dangerous_ball_without_outfield_chaser():
    playbook = DefaultPlaybook(_kit())
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(-5.0, 0.0, 0.0)),
            2: RobotState(2, pose=Pose2D(-3.0, 1.5, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.0, 0.0, 0.0)),
        },
        ball=BallState(-5.0, 0.0, last_seen_at=1.0),
    )

    assignment = playbook.assign_roles(context)

    assert assignment.role_of(3) == ROLE_GOALKEEPER
    assert assignment.players_of(ROLE_CHASER) == ()


def test_outfield_chaser_takes_over_when_goalkeeper_is_disallowed():
    playbook = DefaultPlaybook(_kit())
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(-5.0, 0.0, 0.0)),
            2: RobotState(2, pose=Pose2D(-3.0, 1.5, 0.0)),
            3: RobotState(3, pose=Pose2D(-6.0, 0.0, 0.0)),
        },
        ball=BallState(-5.0, 0.0, last_seen_at=1.0),
    )
    context.known_game.teams[0].players[2].penalty = Penalty.PUSHING

    assignment = playbook.assign_roles(context)

    assert assignment.players_of(ROLE_CHASER) == (1,)


def test_outfield_chaser_takes_over_when_goalkeeper_pose_is_missing():
    playbook = DefaultPlaybook(_kit())
    context = PlayContext(
        game_state=GameControlState(state=GameState.PLAYING),
        teammates={
            1: RobotState(1, pose=Pose2D(-5.0, 0.0, 0.0)),
            2: RobotState(2, pose=Pose2D(-3.0, 1.5, 0.0)),
            3: RobotState(3, pose=None),
        },
        ball=BallState(-5.0, 0.0, last_seen_at=1.0),
    )

    assignment = playbook.assign_roles(context)

    assert assignment.players_of(ROLE_CHASER) == (1,)


def test_supporter_does_not_spin_after_reaching_target():
    kit = _kit()

    move_leaf = SupporterRole().build_subtree(kit, 2)

    assert move_leaf._hold_vyaw == 0.0
