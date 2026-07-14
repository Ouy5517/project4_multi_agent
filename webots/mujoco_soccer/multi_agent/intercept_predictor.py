from __future__ import annotations


def predict_intercept(ball_xy: tuple[float, float], ball_v: tuple[float, float], horizon: float = 0.5) -> tuple[float, float]:
    return (ball_xy[0] + ball_v[0] * horizon, ball_xy[1] + ball_v[1] * horizon)

