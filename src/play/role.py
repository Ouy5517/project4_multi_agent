"""Hot-swappable dynamic-role abstraction that keeps role definition and action execution together.

Design layering, kept to two inheritance layers:

:class:`RoleStrategy` is the only contract base. Subclasses return their role subtree from :meth:`RoleStrategy.build_subtree`; everything else is implementation detail.
:class:`KickAction` / :class:`IsKickWanted`
Shared utility leaves and :func:`build_attack_subtree` live in :mod:`play.nodes`
as default building blocks. Most roles compose them directly; advanced roles
can override :meth:`RoleStrategy.build_subtree` and reuse pieces selectively.

Starter role, pure positioning::

from src.play import RoleStrategy, MoveToTarget

class SupporterRole(RoleStrategy):
name = "supporter"
def target(self, kit, player_id, context) -> Pose2D:
return kit.targeting.support_target(...)
def build_subtree(self, kit, player_id):
return MoveToTarget(kit, player_id, lambda context: self.target(kit, player_id, context), reason_fn=lambda: "supporter hold")

Composite role, conditional kicking::

from src.play import AttackSubtreeConfig, RoleStrategy, build_attack_subtree

class GoalkeeperRole(RoleStrategy):
name = "goalkeeper"
def target(self, kit, player_id, context) -> Pose2D:
return kit.ready_stance.goalkeeper_guard_target(context.known_ball)
def wants_to_kick(self, kit, player_id, context) -> bool:
return kit.targeting.ball_in_own_defensive_area(context.known_ball)
def kick_target(self, kit, player_id, context) -> Pose2D:
return Pose2D(kit.field.opponent_goal_x(), 0.0, 0.0)
def build_subtree(self, kit, player_id):
return build_attack_subtree(
kit,
player_id,
AttackSubtreeConfig(
target_fn=lambda context: self.target(kit, player_id, context),
kick_target_fn=lambda context: self.kick_target(kit, player_id, context),
wants_kick_fn=lambda context: self.wants_to_kick(kit, player_id, context),
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import TYPE_CHECKING, ClassVar

import py_trees

if TYPE_CHECKING:
    from ..runtime import SoccerKit


# ----------------------------------------------------------------------
# Role strategy base class
# ----------------------------------------------------------------------


class RoleStrategy:
    """Base class for role strategies; the only contract is implementing :meth:`build_subtree`.

    ``target``
    ``KickAction`` /
    ``target``, ``wants_to_kick``, and ``kick_target`` are not base-class
    contracts; they are helper methods roles may define for :mod:`play.nodes`
    utilities. Advanced roles that do not reuse those utilities need not define them.
    """

    # Dynamic role label written into ``RoleAssignment.by_player``.
    name: ClassVar[str] = ""

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        """Return this role's behavior subtree; subclasses must implement it."""
        raise NotImplementedError


# ----------------------------------------------------------------------
# Role registry
# ----------------------------------------------------------------------


class RoleRegistry:
    def __init__(self) -> None:
        self._roles: list[RoleStrategy] = []

    def register(self, role: RoleStrategy) -> "RoleRegistry":
        if not role.name:
            raise ValueError(
                f"RoleStrategy {type(role).__name__} 必须声明非空 ``name``",
            )
        if self.get(role.name) is not None:
            raise ValueError(
                f"角色 {role.name!r} 已注册；用 replace() 显式覆盖或先 unregister()",
            )
        self._roles.append(role)
        return self

    def replace(self, role: RoleStrategy) -> "RoleRegistry":
        if not role.name:
            raise ValueError("RoleStrategy 必须声明非空 ``name``")
        for idx, existing in enumerate(self._roles):
            if existing.name == role.name:
                self._roles[idx] = role
                return self
        self._roles.append(role)
        return self

    def unregister(self, name: str) -> None:
        self._roles = [r for r in self._roles if r.name != name]

    def get(self, name: str) -> RoleStrategy | None:
        for role in self._roles:
            if role.name == name:
                return role
        return None

    def __iter__(self) -> Iterator[RoleStrategy]:
        return iter(self._roles)

    def __len__(self) -> int:
        return len(self._roles)
