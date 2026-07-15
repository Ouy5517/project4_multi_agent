"""双队 DecisionFSM 与队伍视角测试"""
from __future__ import annotations

import math

from common.config import GOAL_X, OUR_GOAL_X, NUM_ROBOTS_PER_TEAM
from common.robot_action import MockRobotAction
from common.world_state import (
    Team,
    WorldStateProvider,
    create_pass_scenario,
)
from decision.decision_fsm import DecisionFSM, DecisionState
from simulation.field_simulator import Simulator


def test_perspective_swaps_goals_and_rosters():
    ws = create_pass_scenario()
    yv = ws.perspective_for(Team.YELLOW)
    assert yv.our_goal.x == ws.opponent_goal.x == GOAL_X
    assert yv.opponent_goal.x == ws.our_goal.x == OUR_GOAL_X
    assert {r.id for r in yv.teammates} == {10, 11, 12}
    assert {r.id for r in yv.opponents} == {0, 1, 2}


def test_yellow_fsm_activates_and_moves():
    sim = Simulator()
    action = MockRobotAction(sim)
    provider = WorldStateProvider(sim)
    ws = provider.get()

    yellow_ids = [10, 11, 12]
    yellow = DecisionFSM(
        ws.perspective_for(Team.YELLOW),
        action,
        robot_ids=yellow_ids,
        team=Team.YELLOW,
    )
    blue = DecisionFSM(ws, action, num_robots=NUM_ROBOTS_PER_TEAM, team=Team.BLUE)

    start_xy = {rid: (sim.yellow_robots[rid].x, sim.yellow_robots[rid].y) for rid in yellow_ids}

    for _ in range(40):
        sim.update()
        ws = provider.get()
        blue.update(ws)
        yellow.update(ws.perspective_for(Team.YELLOW))

    # 黄队最近者应进入 chase/dribble/block 等活跃态
    states = {rid: yellow.get_state(rid) for rid in yellow_ids}
    assert any(s != DecisionState.IDLE for s in states.values())

    # 至少有一台黄车移动过
    moved = False
    for rid in yellow_ids:
        x, y = sim.yellow_robots[rid].x, sim.yellow_robots[rid].y
        if math.hypot(x - start_xy[rid][0], y - start_xy[rid][1]) > 0.05:
            moved = True
    assert moved


def test_yellow_dribbles_toward_blue_goal():
    """黄队控球时带球方向应朝左门 (OUR_GOAL_X)。"""
    sim = Simulator(num_blue=3, num_yellow=1)
    # 球靠近黄队 10
    sim.yellow_robots[10].x, sim.yellow_robots[10].y = 0.5, 0.0
    sim.ball.x, sim.ball.y = 0.6, 0.0
    action = MockRobotAction(sim)
    provider = WorldStateProvider(sim)
    ws = provider.get()
    yellow = DecisionFSM(
        ws.perspective_for(Team.YELLOW),
        action,
        robot_ids=[10],
        team=Team.YELLOW,
    )
    for _ in range(30):
        sim.update()
        ws = provider.get()
        yellow.update(ws.perspective_for(Team.YELLOW))
        if yellow.get_state(10) == DecisionState.DRIBBLE:
            break
    # 若进入带球, kick 方向应大致朝 -X
    if sim._kick_queue:
        _, _, direction = sim._kick_queue[0]
        # 朝左半球门: cos(direction) < 0
        assert math.cos(direction) < 0.2
