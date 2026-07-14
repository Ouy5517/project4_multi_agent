import math
from typing import Tuple

from common.config import FIELD_HEIGHT, FIELD_WIDTH, ROBOT_MAX_SPEED
from common.world_state import WorldState


class ReceiveStrategy:
    """Predict reachable receive points and detect completed receives."""

    def __init__(self, world_state: WorldState):
        self._ws = world_state

    def update_world_state(self, ws: WorldState):
        self._ws = ws

    def predict_receive_point(self, receiver_id: int) -> Tuple[float, float]:
        receiver = self._ws.get_robot_by_id(receiver_id)
        ball = self._ws.ball
        if receiver is None:
            return self._clamp(ball.x, ball.y)

        for i in range(1, 31):
            t = i * 0.1
            bx = ball.x + ball.vx * t
            by = ball.y + ball.vy * t
            bx, by = self._clamp(bx, by)
            travel_time = math.hypot(receiver.x - bx, receiver.y - by) / max(ROBOT_MAX_SPEED, 1e-6)
            if travel_time <= t:
                return (bx, by)
        return self._clamp(ball.x + ball.vx, ball.y + ball.vy)

    def has_received(self, receiver_id: int) -> bool:
        receiver = self._ws.get_robot_by_id(receiver_id)
        if receiver is None:
            return False
        return self._ws.distance(receiver, self._ws.ball) <= 0.30 and self._ws.ball.speed <= 1.2

    @staticmethod
    def _clamp(x: float, y: float) -> Tuple[float, float]:
        margin = 0.05
        return (
            max(-FIELD_WIDTH / 2 + margin, min(FIELD_WIDTH / 2 - margin, x)),
            max(-FIELD_HEIGHT / 2 + margin, min(FIELD_HEIGHT / 2 - margin, y)),
        )
