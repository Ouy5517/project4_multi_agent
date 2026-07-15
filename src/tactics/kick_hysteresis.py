"""Extract the implicit kick mini-state-machine from ``soccer_strategy.py`` into a pure model.

The old implementation used two dicts, ``_soccer_kick_active`` and
``_soccer_kick_far_since``, to track whether each player was kicking. This replaces
that with an explicit state machine:

Each ``player_id`` owns one :class:`PlayerKickState`.
The ``IsInKickRange`` condition queries :meth:`KickHysteresis.in_kick_range`.
The ``KickBall`` action marks state active via :meth:`KickHysteresis.mark_kicking`.
``ApproachBall`` clears state through :meth:`KickHysteresis.clear_player`.

Enter/exit thresholds and delay come from ``SoccerConfig.strategy``. Competitors
can read this file to understand why stepping slightly away from the ball does not
immediately leave the kicking state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class PlayerKickState:
    """Kick-state snapshot for one player."""

    active: bool = False
    far_since: float | None = None


@dataclass
class KickHysteresis:
    """Hysteresis model for entering and exiting kicking.

    Usage:

    hyst = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
    if hyst.in_kick_range(player_id, distance, now):
    In kicking range, issue a kick command.
    else:
    Move behind the ball.
    hyst.clear_player(player_id)

    Design goal: turn implicit "active dict" state into readable method calls so
    nodes no longer manipulate structures like ``self._soccer_kick_active`` directly.
    """

    enter: float
    exit: float
    exit_delay: float
    _state: dict[int, PlayerKickState] = field(default_factory=dict)

    def in_kick_range(self, player_id: int, distance: float, now: float) -> bool:
        """Decide whether ``player_id`` is currently in the kicking state.

        Logic matches the old ``soccer_strategy._chase_and_kick`` mini-state-machine:
        stay active inside exit, delay exit outside exit, enter active inside enter, otherwise stay inactive.
        """

        state = self._state.setdefault(player_id, PlayerKickState())

        if state.active:
            if distance <= self.exit:
                state.far_since = None
                return True
            if state.far_since is None:
                state.far_since = now
            elapsed = now - state.far_since
            if elapsed < self.exit_delay:
                return True
            state.active = False
            state.far_since = None
            return False

        if distance <= self.enter:
            state.active = True
            state.far_since = None
            return True
        return False

    def mark_kicking(self, player_id: int) -> None:
        """Explicitly mark ``player_id`` as kicking, used by the KickBall action."""

        state = self._state.setdefault(player_id, PlayerKickState())
        state.active = True
        state.far_since = None

    def clear_player(self, player_id: int) -> None:
        """Clear one player's kick state when leaving the ball or becoming penalized."""

        self._state.pop(player_id, None)

    def clear_all(self) -> None:
        """Clear all players, used on GameState changes or stops."""

        self._state.clear()

    def is_active(self, player_id: int) -> bool:
        """Read the current active state without mutating it."""

        state = self._state.get(player_id)
        return state.active if state is not None else False


def distance_to_ball(robot_x: float, robot_y: float, ball_x: float, ball_y: float) -> float:
    """Kept here so ``in_kick_range`` callers do not need to import math separately."""

    return math.hypot(ball_x - robot_x, ball_y - robot_y)
