import math
import time

import pytest

from common.config import DT
from simulation.field_simulator import Simulator


@pytest.mark.performance
def test_120_second_headless_simulation_is_fast_and_finite():
    sim = Simulator()
    start = time.time()
    for _ in range(int(120 / DT)):
        sim.update(DT)
        assert math.isfinite(sim.ball.x)
        assert math.isfinite(sim.ball.y)
    elapsed = time.time() - start
    assert 120 / max(elapsed, 1e-6) >= 20
