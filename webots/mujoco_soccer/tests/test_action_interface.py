from __future__ import annotations

import pytest

from mujoco_soccer.control.action_interface import BEHAVIOR_TO_PRIMITIVE, SoccerActionInterface
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.multi_agent.robot_agent import AgentCommand


def test_behavior_to_action_primitives_are_explicit() -> None:
    assert BEHAVIOR_TO_PRIMITIVE["PASS"] == "pass"
    assert BEHAVIOR_TO_PRIMITIVE["SHOOT"] == "shoot"
    assert BEHAVIOR_TO_PRIMITIVE["BLOCK_LINE"] == "block"
    assert BEHAVIOR_TO_PRIMITIVE["OPEN_FOR_PASS"] == "move_to"


def test_kick_command_first_moves_to_staging_pose() -> None:
    interface = SoccerActionInterface(controllers={})
    cmd = AgentCommand(
        "T1_BLUE_1",
        "PASS",
        "BALL_HANDLER",
        BaseTarget(0.1, 0.0, 0.0, max_speed=0.18),
        kick_action="pass",
        kick_target=(1.0, 0.0),
    )
    primitive = interface.primitive_for(cmd, robot_xy=(-2.0, 0.0), ball_xy=(0.0, 0.0))
    assert primitive.kind == "move_to"
    assert primitive.kick_action == "pass"
    assert primitive.push_pose == 0.2
    assert primitive.backend == "assisted_planar"


def test_kick_command_becomes_action_near_staging_pose() -> None:
    interface = SoccerActionInterface(controllers={})
    cmd = AgentCommand(
        "T1_BLUE_1",
        "SHOOT",
        "BALL_HANDLER",
        BaseTarget(0.1, 0.0, 0.0, max_speed=0.18),
        kick_action="shoot",
        kick_target=(1.0, 0.0),
    )
    staged = interface.primitive_for(cmd, robot_xy=(-0.105, 0.0), ball_xy=(0.0, 0.0))
    primitive = interface.primitive_for(cmd, robot_xy=(staged.target.x, staged.target.y), ball_xy=(0.0, 0.0))
    assert primitive.kind == "shoot"
    assert primitive.push_pose == 0.35


def test_trajectory_joint_backend_is_selectable() -> None:
    interface = SoccerActionInterface(controllers={}, backend="trajectory_joint")
    cmd = AgentCommand("T1_BLUE_1", "OPEN_FOR_PASS", "SUPPORT", BaseTarget(0.2, 0.3, 0.4))
    primitive = interface.primitive_for(cmd, robot_xy=(0.0, 0.0), ball_xy=(0.1, 0.1))
    assert interface.backend_name == "trajectory_joint"
    assert primitive.kind == "move_to"
    assert primitive.backend == "trajectory_joint"


def test_rl_policy_backend_fails_until_policy_adapter_exists() -> None:
    interface = SoccerActionInterface(controllers={}, backend="rl_policy")
    cmd = AgentCommand("T1_BLUE_1", "OPEN_FOR_PASS", "SUPPORT", BaseTarget(0.2, 0.3, 0.4))
    with pytest.raises(NotImplementedError, match="trained policy adapter"):
        interface.primitive_for(cmd, robot_xy=(0.0, 0.0), ball_xy=(0.1, 0.1))


def test_unknown_backend_is_rejected() -> None:
    with pytest.raises(ValueError, match="Unknown action backend"):
        SoccerActionInterface(controllers={}, backend="unknown")


class _FakeTrajectory:
    active = False


class _FakeController:
    def __init__(self) -> None:
        self.push_pose = 0.0
        self.joint_trajectory = _FakeTrajectory()
        self.started: list[str] = []
        self.target: BaseTarget | None = None

    def start_joint_trajectory(self, action: str) -> None:
        self.started.append(action)

    def set_target(self, target: BaseTarget) -> None:
        self.target = target


def test_trajectory_backend_triggers_block_joint_trajectory() -> None:
    controller = _FakeController()
    interface = SoccerActionInterface(controllers={"T1_BLUE_1": controller}, backend="trajectory_joint")
    cmd = AgentCommand("T1_BLUE_1", "BLOCK_LINE", "COVER", BaseTarget(0.2, 0.3, 0.4))
    primitive = interface.apply(cmd, robot_xy=(0.0, 0.0), ball_xy=(0.1, 0.1))
    assert primitive.kind == "block"
    assert controller.started == ["block"]
    assert controller.target is not None
