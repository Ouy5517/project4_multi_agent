"""Pure tactic-model layer: geometry and control tools independent of BT and ROS.

Pose2D geometry helpers and team-view field frame tools
obstacle collection for opponents, teammates, and goal structure
tactic targets for support, passing, shooting, sideline recovery, and more
motion controller with avoidance, walking control, and kick commands
kick enter/exit hysteresis model

The PLAY-stage "which player chases" decision is not in this layer; it belongs
to the playbook layer in :mod:`src.play.playbook`.
"""

from .kick_hysteresis import KickHysteresis
from .geometry import TeamFieldFrame
from .motion import MotionController
from .navigation import Obstacle, ObstacleCollector
from .ready_stance import ReadyStance
from .targeting import Targeting

__all__ = [
    "KickHysteresis",
    "MotionController",
    "Obstacle",
    "ObstacleCollector",
    "ReadyStance",
    "Targeting",
    "TeamFieldFrame",
]
