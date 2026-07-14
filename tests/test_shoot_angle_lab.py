"""单人多角度射门实验 — 基础单元测试"""
from __future__ import annotations

import math

from common.config import GOAL_WIDTH, GOAL_X
from common.world_state import create_shoot_angle_scenario
from decision.match_controller import MatchController
from experiments.shoot_angle_lab import (
    classify_outcome,
    place_shot,
    run_trial,
    sample_goal_targets,
)
from simulation.field_simulator import Simulator


def test_shoot_angle_scenario_is_solo():
    ws = create_shoot_angle_scenario()
    assert len(ws.teammates) == 1
    assert ws.opponents == []
    assert ws.teammates[0].id == 0


def test_sample_goal_targets_cover_mouth():
    ys = sample_goal_targets(5)
    assert len(ys) == 5
    assert ys[0] < 0 < ys[-1]
    assert abs(ys[0]) <= GOAL_WIDTH / 2
    assert abs(ys[-1]) <= GOAL_WIDTH / 2


def test_classify_goal_and_wide():
    match = MatchController()
    assert classify_outcome(GOAL_X + 0.05, 0.0, match) == "GOAL"
    assert classify_outcome(GOAL_X + 0.05, GOAL_WIDTH, match) == "MISS_WIDE"
    assert classify_outcome(3.0, 0.0, match) == "IN_FLIGHT"
    assert classify_outcome(3.0, 0.0, match, stopped=True) == "MISS_SHORT"


def test_center_shot_scores_headless():
    sim = Simulator(num_blue=1, num_yellow=0)
    match = MatchController()
    trial, _kick_at = run_trial(
        sim,
        match,
        index=0,
        target_y=0.0,
        ball_x=3.0,
        ball_y=0.0,
        power=90.0,
        settle_s=3.0,
        kick_interval=0.0,
        viz=None,
    )
    assert trial.result == "GOAL"
    assert abs(trial.angle_deg) < 5.0


def test_place_shot_puts_robot_behind_ball():
    sim = Simulator(num_blue=1, num_yellow=0)
    place_shot(sim, ball_x=2.0, ball_y=0.0, kick_dir=0.0, behind=0.25)
    assert sim.ball.x == 2.0
    assert sim.blue_robots[0].x < sim.ball.x
    assert abs(sim.blue_robots[0].theta) < 1e-6


def test_set_visible_robots_hides_extras():
    import pytest

    pytest.importorskip("mujoco")
    from simulation.mujoco_simulator import MuJoCoSimulator

    sim = MuJoCoSimulator(num_blue=1, num_yellow=0)
    assert 0 in sim._scene_robot_mocap
    extras = [rid for rid in sim._scene_robot_mocap if rid != 0]
    if not extras:
        pytest.skip("scene has only robot_0")
    sim.set_visible_robots([0])
    for rid in extras:
        mocap_id = sim._scene_robot_mocap[rid]
        assert sim.data.mocap_pos[mocap_id][2] < -1.0
        for gid, _ in sim._scene_robot_geom_alphas.get(rid, []):
            assert float(sim.model.geom_rgba[gid][3]) == 0.0
