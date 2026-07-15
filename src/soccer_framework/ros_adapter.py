"""ROS integration adapter for SoccerSim framework components.

This class owns the ROS node lifecycle for the normal runtime path and wires the
truth and GameController providers together. Lower-level callers may still pass
their own node and keep spinning externally.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from .config import SoccerConfig
from .game_controller import GameControllerRosProvider
from .ros_truth import RosTruthProvider
from .types import PlayContextProvider

if TYPE_CHECKING:
    from rclpy.context import Context


__all__ = ["SoccerRosAdapter"]


class SoccerRosAdapter:
    """Owns ROS node, subscriptions, and executor for SoccerSim framework providers.

    If ``node`` is provided, ownership stays with the caller and this adapter only
    creates subscriptions. If ``node`` is omitted, an internal ``sim_bridge`` node
    and executor are created and cleaned up here.
    """

    def __init__(
        self,
        config: SoccerConfig,
        logger: Any,
        node: Node | None = None,
    ):
        self.config = config
        self._logger = logger
        self._owns_ros_node = node is None
        self._owns_rclpy_context = False
        self._ros_context: Context | None = None
        self._ros_executor: SingleThreadedExecutor | None = None
        self._ros_spin_thread: threading.Thread | None = None
        self._ros_node_destroyed = False
        self._started = False

        self.node: Node = self._create_ros_node(node)
        self.context_provider: PlayContextProvider = RosTruthProvider(
            self.node,
            self.config,
            self._logger,
        )
        self.game_controller = GameControllerRosProvider(
            node=self.node,
            config=self.config,
            logger=self._logger,
            on_state=self.context_provider.set_game_state,
        )

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        try:
            self.context_provider.start()
            self.game_controller.start()
            self._start_ros_spin()
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        if self._ros_node_destroyed:
            return
        if self._started:
            self._started = False
        self.game_controller.stop()
        self.context_provider.stop()
        self._stop_ros_spin()
        self._destroy_ros_node()

    def _create_ros_node(self, node: Node | None) -> Node:
        if node is not None:
            return node
        context = rclpy.get_default_context()
        self._ros_context = context
        if not rclpy.ok(context=context):
            context.init(args=None, initialize_logging=False)
            self._owns_rclpy_context = True
        return rclpy.create_node("sim_bridge", context=context)

    def _start_ros_spin(self) -> None:
        if not self._owns_ros_node:
            return
        if self._ros_spin_thread and self._ros_spin_thread.is_alive():
            return
        context = self._ros_context
        if context is None:
            return
        self._ros_executor = SingleThreadedExecutor(context=context)
        self._ros_executor.add_node(self.node)
        self._ros_spin_thread = threading.Thread(
            target=self._spin_ros,
            name="soccer_ros_adapter_spin",
            daemon=True,
        )
        self._ros_spin_thread.start()

    def _spin_ros(self) -> None:
        if self._ros_executor is None:
            return
        try:
            self._ros_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            self._logger.warn(
                f"SoccerRosAdapter spin failed: {exc.__class__.__name__}: {exc}",
                event="ros_adapter_spin_failed",
                team_id=self.config.team_id,
                error_type=exc.__class__.__name__,
                error=str(exc),
            )

    def _stop_ros_spin(self) -> None:
        if not self._owns_ros_node:
            return
        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown()
            except Exception as exc:
                self._logger.warn(
                    "SoccerRosAdapter executor shutdown failed: "
                    f"{exc.__class__.__name__}: {exc}",
                    event="ros_adapter_executor_shutdown_failed",
                    team_id=self.config.team_id,
                    error_type=exc.__class__.__name__,
                    error=str(exc),
                )
        if self._ros_spin_thread and self._ros_spin_thread.is_alive():
            self._ros_spin_thread.join(timeout=2.0)
        self._ros_spin_thread = None
        self._ros_executor = None

    def _destroy_ros_node(self) -> None:
        if not self._owns_ros_node:
            return
        if self._ros_node_destroyed:
            return
        try:
            self.node.destroy_node()
        except Exception as exc:
            self._logger.warn(
                f"SoccerRosAdapter node destroy failed: "
                f"{exc.__class__.__name__}: {exc}",
                event="ros_adapter_node_destroy_failed",
                team_id=self.config.team_id,
                error_type=exc.__class__.__name__,
                error=str(exc),
            )
        context = self._ros_context
        if (
            self._owns_rclpy_context
            and context is not None
            and rclpy.ok(context=context)
        ):
            context.shutdown()
        self._ros_node_destroyed = True
