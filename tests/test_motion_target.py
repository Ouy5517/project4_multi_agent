import math

from src.soccer_framework import BallState, SoccerConfig
from src.tactics.geometry import TeamFieldFrame
from src.tactics.kick_hysteresis import KickHysteresis
from src.tactics.motion import MotionController
from src.tactics.navigation import ObstacleCollector


def test_approach_target_is_behind_ball_along_kick_direction():
    config = SoccerConfig()
    field = TeamFieldFrame(config)
    motion = MotionController(
        config,
        field,
        KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5),
        ObstacleCollector(config, field),
    )
    ball = BallState(x=1.0, y=2.0, last_seen_at=1.0)
    kick_theta = math.pi / 2.0

    target = motion.approach_target(ball, kick_theta, approach_offset=0.4)

    assert math.isclose(target.x, 1.0, abs_tol=1e-9)
    assert math.isclose(target.y, 1.6, abs_tol=1e-9)
    assert target.theta == kick_theta
