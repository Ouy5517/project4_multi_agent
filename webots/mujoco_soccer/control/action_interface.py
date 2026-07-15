from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal

from mujoco_soccer.control.foot_push_controller import plan_push
from mujoco_soccer.control.planar_base_controller import BaseTarget
from mujoco_soccer.control.robot_controller import RobotController
from mujoco_soccer.multi_agent.robot_agent import AgentCommand


ActionBackendName = Literal["assisted_planar", "trajectory_joint", "rl_policy"]


@dataclass(frozen=True)
class ActionPrimitive:
    robot_id: str
    kind: str
    target: BaseTarget
    kick_action: str | None = None
    kick_target: tuple[float, float] | None = None
    push_pose: float = 0.0
    backend: str = "assisted_planar"


BEHAVIOR_TO_PRIMITIVE = {
    "PASS": "pass",
    "SHOOT": "shoot",
    "BLOCK_LINE": "block",
    "CLEAR": "clear",
    "INTERCEPT_BALL": "intercept",
    "PRESS_BALL": "intercept",
    "DRIBBLE": "dribble",
    "COUNTER_ATTACK": "shoot",
    "RECEIVE_PASS": "move_to",
    "OPEN_FOR_PASS": "move_to",
    "COVER": "move_to",
    "PROTECT_BALL": "move_to",
    "RECOVER_POSITION": "move_to",
    "HOLD_POSITION": "hold",
}


class ActionBackend(ABC):
    name: ActionBackendName

    def __init__(self, controllers: dict[str, RobotController]) -> None:
        self.controllers = controllers

    @abstractmethod
    def primitive_for(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        raise NotImplementedError

    @abstractmethod
    def apply(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        raise NotImplementedError


class AssistedPlanarBackend(ActionBackend):
    name: ActionBackendName = "assisted_planar"

    def primitive_for(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        kind = BEHAVIOR_TO_PRIMITIVE.get(command.behavior, "move_to")
        if command.kick_action is None or command.kick_target is None:
            return ActionPrimitive(command.robot_id, kind, command.target, backend=self.name)

        start, push = plan_push(
            command.robot_id,
            ball_xy,
            command.kick_target,
            command.target.max_speed,
            0.05,
            command.kick_action,
        )
        if math.hypot(robot_xy[0] - start.x, robot_xy[1] - start.y) >= 0.10:
            return ActionPrimitive(
                command.robot_id,
                "move_to",
                start,
                command.kick_action,
                command.kick_target,
                push_pose=0.2,
                backend=self.name,
            )
        target = BaseTarget(
            push.target_x,
            push.target_y,
            math.atan2(push.direction_y, push.direction_x),
            max_speed=max(0.045, min(0.24, command.target.max_speed)),
        )
        return ActionPrimitive(
            command.robot_id,
            kind,
            target,
            command.kick_action,
            command.kick_target,
            push_pose=0.35,
            backend=self.name,
        )

    def apply(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        primitive = self.primitive_for(command, robot_xy, ball_xy)
        controller = self.controllers[command.robot_id]
        controller.push_pose = primitive.push_pose
        if primitive.kick_action is not None and primitive.push_pose >= 0.35 and not controller.kick_swing.active:
            controller.start_kick_swing(primitive.kick_action)
        controller.set_target(primitive.target)
        return primitive


class TrajectoryJointBackend(AssistedPlanarBackend):
    """Keeps planar transport, but routes actions through an explicit joint-trajectory backend."""

    name: ActionBackendName = "trajectory_joint"

    def apply(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        primitive = self.primitive_for(command, robot_xy, ball_xy)
        controller = self.controllers[command.robot_id]
        controller.push_pose = primitive.push_pose
        if primitive.kind in {"pass", "shoot", "clear", "intercept", "dribble"} and primitive.push_pose >= 0.35:
            if not controller.joint_trajectory.active:
                controller.start_joint_trajectory(primitive.kind)
        elif primitive.kind == "block" and not controller.joint_trajectory.active:
            controller.start_joint_trajectory("block")
        controller.set_target(primitive.target)
        return primitive


class RlPolicyBackend(ActionBackend):
    name: ActionBackendName = "rl_policy"

    def primitive_for(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        raise NotImplementedError("rl_policy backend requires a trained policy adapter")

    def apply(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        raise NotImplementedError("rl_policy backend requires a trained policy adapter")


def create_action_backend(name: str, controllers: dict[str, RobotController]) -> ActionBackend:
    if name == "assisted_planar":
        return AssistedPlanarBackend(controllers)
    if name == "trajectory_joint":
        return TrajectoryJointBackend(controllers)
    if name == "rl_policy":
        return RlPolicyBackend(controllers)
    raise ValueError(f"Unknown action backend: {name}")


class SoccerActionInterface:
    """Maps high-level soccer commands onto a selectable low-level action backend."""

    def __init__(self, controllers: dict[str, RobotController], backend: str = "assisted_planar") -> None:
        self.backend = create_action_backend(backend, controllers)

    @property
    def backend_name(self) -> str:
        return self.backend.name

    def primitive_for(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        return self.backend.primitive_for(command, robot_xy, ball_xy)

    def apply(
        self,
        command: AgentCommand,
        robot_xy: tuple[float, float],
        ball_xy: tuple[float, float],
    ) -> ActionPrimitive:
        return self.backend.apply(command, robot_xy, ball_xy)
