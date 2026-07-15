"""Playbook registry, the visible registration center for play styles.

Design intent:
The ``team/`` framework depends on no concrete Playbook, including :class:`DefaultPlaybook`.
All Playbooks, default or custom, are explicitly registered through :data:`PLAYBOOKS`.
Framework code, tests, and custom entries create instances with
``PLAYBOOKS.create_default(kit)`` or ``PLAYBOOKS.create(name, kit)`` and pass them to :class:`TeamStrategyTree`.

Registration is intentionally visible: :class:`DefaultPlaybook` is registered at
the end of :mod:`play.__init__`, and custom Playbooks use the same one-line call.

Typical usage:

.. code-block:: python

src/play/__init__.py
from .playbook import DefaultPlaybook
from .registry import PLAYBOOKS
PLAYBOOKS.register("default", DefaultPlaybook, default=True)

from src.behavior_tree import TeamStrategyTree
from src.play import PLAYBOOKS, DefaultPlaybook
from src.runtime import SoccerKit

class AggressivePlaybook(DefaultPlaybook): ...
PLAYBOOKS.register("aggressive", AggressivePlaybook)

kit = SoccerKit(config)
tree = TeamStrategyTree(kit, PLAYBOOKS.create("aggressive", kit), context_provider)
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..runtime import SoccerKit
    from .playbook import Playbook


PlaybookFactory = Callable[["SoccerKit"], "Playbook"]


class PlaybookRegistry:
    """Register Playbook factories by name, with optional default selection."""

    def __init__(self) -> None:
        self._factories: dict[str, PlaybookFactory] = {}
        self._default_name: str | None = None

    def register(
        self,
        name: str,
        factory: PlaybookFactory,
        *,
        default: bool = False,
    ) -> None:
        """Register one Playbook factory.

        ``name`` must be unique, ``factory`` accepts :class:`SoccerKit` and returns
        :class:`Playbook`, and ``default=True`` selects the single default.
        """

        if not name:
            raise ValueError("Playbook 名字不能为空")
        if name in self._factories:
            raise ValueError(f"Playbook {name!r} 已注册")
        self._factories[name] = factory
        if default:
            if self._default_name is not None and self._default_name != name:
                raise ValueError(
                    f"已有默认 Playbook {self._default_name!r}；"
                    f"想改默认请先 unregister 或调 set_default"
                )
            self._default_name = name

    def unregister(self, name: str) -> None:
        self._factories.pop(name, None)
        if self._default_name == name:
            self._default_name = None

    def set_default(self, name: str) -> None:
        if name not in self._factories:
            raise KeyError(f"Playbook {name!r} 未注册")
        self._default_name = name

    def names(self) -> tuple[str, ...]:
        return tuple(self._factories)

    def get_factory(self, name: str) -> PlaybookFactory:
        if name not in self._factories:
            raise KeyError(
                f"Playbook {name!r} 未注册；"
                f"已注册：{sorted(self._factories)}"
            )
        return self._factories[name]

    def create(self, name: str, kit: "SoccerKit") -> "Playbook":
        """Create a Playbook instance by name."""

        return self.get_factory(name)(kit)

    def create_default(self, kit: "SoccerKit") -> "Playbook":
        """Create the default Playbook instance, raising ``RuntimeError`` when none is set."""

        if self._default_name is None:
            raise RuntimeError(
                "PlaybookRegistry 没有默认 Playbook；"
                "在 play/__init__.py 末尾用 register(..., default=True) 指定"
            )
        return self.create(self._default_name, kit)


# Module-level singleton shared inside play; default registration happens at the end of :mod:`play.__init__`.
PLAYBOOKS = PlaybookRegistry()
