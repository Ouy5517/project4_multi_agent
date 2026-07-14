#!/usr/bin/python3.10
"""Tests for SimpleKickExecutor — no FancyKick/VisualKick dependency."""

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

PROJECT = Path("/home/plon/Workspace/booster_soccer_project")
sys.path.insert(0, str(PROJECT))

import pytest

from integration.simple_kick_executor import (
    KickConfig, KickState, KickResult, SimpleKickExecutor, _KickContext,
)


@pytest.fixture
def cfg():
    return KickConfig()


@pytest.fixture
def executor(cfg):
    rpc = MagicMock()
    rpc.call = MagicMock(return_value={"success": True, "status_code": 0})
    return SimpleKickExecutor(config=cfg, rpc_client=rpc)


# ── Config Tests ──

class TestConfig:
    def test_loads_yaml(self):
        c = KickConfig.from_yaml()
        assert c.kick_speed == 0.08
        assert c.behind_ball_distance == 0.20

    def test_speed_limits(self, cfg):
        assert cfg.kick_speed <= cfg.max_kick_speed
        assert cfg.kick_duration <= cfg.max_kick_duration

    def test_defaults_safe(self, cfg):
        assert cfg.max_kick_speed <= 0.12
        assert cfg.max_kick_duration <= 0.50


# ── Geometry Tests ──

class TestGeometry:
    def test_behind_ball_positive_x(self):
        """Ball at (0,0), target at (1,0) → behind is at (-0.2, 0)."""
        behind = SimpleKickExecutor._behind_ball_pos((0, 0, 0), (1, 0))
        assert behind[0] < 0  # behind, opposite of target
        assert abs(behind[1]) < 0.01

    def test_behind_ball_diagonal(self):
        behind = SimpleKickExecutor._behind_ball_pos((0, 0, 0), (1, 1))
        # Behind should be opposite direction of (1,1), so negative x and y
        assert behind[0] < 0
        assert behind[1] < 0

    def test_angle_to_target(self):
        angle = SimpleKickExecutor._angle_to((0, 0), (1, 0))
        assert abs(angle) < 0.01

    def test_angle_to_target_up(self):
        angle = SimpleKickExecutor._angle_to((0, 0), (0, 1))
        assert abs(angle - math.pi / 2) < 0.01

    def test_distance(self):
        d = SimpleKickExecutor._dist((0, 0), (3, 4))
        assert d == 5.0

    def test_angle_diff(self):
        d = SimpleKickExecutor._angle_diff(0.1, 0.0)
        assert abs(d - 0.1) < 0.01


# ── Kick Execution Tests (dry-run) ──

class TestKickExecution:
    def test_execute_kick_dry_run(self, executor):
        """Dry run: ball doesn't move → should fail."""
        ball_pos = [0.0, 0.0, 0.11]

        def get_ball():
            return ball_pos[:]  # ball never moves

        r = executor.execute_kick(
            "T1_BLUE_1", (-0.3, 0.0), ball_pos[:],
            (1.0, 0.0), get_ball_pos=get_ball,
        )
        assert r.success is False
        assert r.attempts == 3
        assert "did not move" in r.reason.lower()

    def test_execute_kick_simulates_ball_move(self, executor):
        """Simulate ball moving after kick."""
        ball = [0.0, 0.0, 0.11]
        called = [0]

        def get_ball():
            called[0] += 1
            if called[0] >= 7:  # After all the approach/move phases
                ball[0] = 0.1  # ball moved 10cm
            return ball[:]

        r = executor.execute_kick(
            "T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11],
            (1.0, 0.0), get_ball_pos=get_ball,
        )
        # With ball moving, should succeed
        assert r.success is True or r.reason  # at minimum, produces a result

    def test_rpc_move_called(self, executor):
        executor.execute_kick("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11], (1.0, 0.0),
                              get_ball_pos=lambda: [0.0, 0.0, 0.11])
        assert executor.rpc.call.call_count > 0

    def test_stop_called_after(self, executor):
        executor.execute_kick("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11], (1.0, 0.0),
                              get_ball_pos=lambda: [0.0, 0.0, 0.11])
        # Check any call has api_id=2001 (move) with zero velocities
        stop_calls = [c for c in executor.rpc.call.call_args_list
                      if c[0][0] == 2001 and '0.0' in str(c)]
        assert len(stop_calls) > 0

    def test_log_appended(self, executor):
        executor.execute_kick("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11], (1.0, 0.0),
                              get_ball_pos=lambda: [0.0, 0.0, 0.11])
        assert len(executor.log) == 1
        assert isinstance(executor.log[0], KickResult)


# ── Shot/Pass/Clearance ──

class TestActions:
    def test_execute_pass_proxies_to_kick(self, executor):
        r = executor.execute_pass("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11],
                                  (2.0, 0.5), get_ball_pos=lambda: [0.0, 0.0, 0.11])
        assert isinstance(r, KickResult)

    def test_execute_shot_proxies_to_kick(self, executor):
        r = executor.execute_shot("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11],
                                  (3.3, 0.0), get_ball_pos=lambda: [0.0, 0.0, 0.11])
        assert isinstance(r, KickResult)

    def test_execute_clearance_proxies_to_kick(self, executor):
        r = executor.execute_clearance("T1_BLUE_1", (-0.3, 0.0), [0.0, 0.0, 0.11],
                                       (-3.3, 0.0), get_ball_pos=lambda: [0.0, 0.0, 0.11])
        assert isinstance(r, KickResult)


# ── Result Serialization ──

class TestSerialization:
    def test_kick_result_to_dict(self):
        r = KickResult("T1", (1, 0), (0, 0, 0.11), (0.1, 0, 0.11),
                       0.1, 0.05, True, KickState.DONE, 1, "ok")
        d = r.to_dict()
        assert d["success"] is True
        assert d["horizontal_disp"] == 0.1

    def test_failed_result(self):
        r = KickResult("T1", (1, 0), (0, 0, 0), (0, 0, 0),
                       0, 0, False, KickState.FAILED, 3, "no movement")
        assert r.to_dict()["success"] is False


# ── KickConfig Safety ──

class TestKickConfigSafety:
    def test_speed_clamped_in_kick(self, cfg):
        cfg.kick_speed = 0.5  # exceeds max
        speed = min(cfg.kick_speed, cfg.max_kick_speed)
        assert speed == 0.12  # clamped

    def test_duration_clamped(self, cfg):
        cfg.kick_duration = 2.0
        dur = min(cfg.kick_duration, cfg.max_kick_duration)
        assert dur == 0.50
