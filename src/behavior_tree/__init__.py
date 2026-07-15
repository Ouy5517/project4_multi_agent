"""Behavior-tree infrastructure: blackboard, nodes, subtree factories, and top-level assembly.

This package owns BT mechanics and is independent of specific playbooks:

blackboard key table and client
data leaves writing clock/context/ball/rule/hardware state
common condition leaves for ball/rules/hardware/kicking
common action leaves such as StopAll, READY, and CommitTeamCommands
READY subtree factory
SafetyGuards and SafetyOverrides subtree factory
TeamStrategyTree top-level assembly

Dependency direction is ``behavior_tree -> runtime(types only) + tactics +
soccer_framework``. ``SoccerKit`` appears only under ``TYPE_CHECKING``, so runtime
can import this package for facade assembly without a cycle.
"""

from .blackboard import BlackboardClient, BlackboardKeys, cmd_key, robot_status_key
from .tree import TeamCommandExecutor, TeamStrategyTree, create_team_tree

__all__ = [
    "BlackboardClient",
    "BlackboardKeys",
    "TeamCommandExecutor",
    "TeamStrategyTree",
    "cmd_key",
    "create_team_tree",
    "robot_status_key",
]
