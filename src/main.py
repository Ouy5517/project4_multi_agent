# coding: utf-8  #  File encoding.
"""AgentBase entry for SoccerSim.

This module is intentionally kept as the process entry point. The soccer
environment data types and runtime adapters live in ``soccer_framework``; the
Agent lifecycle stays visible here while runtime-owned adapters manage ROS.
"""

from __future__ import annotations

import threading

from booster_agent_framework import AgentBase, AgentFeatures

from .runtime import SoccerTeamRuntime
from .soccer_framework import SoccerConfig
from .soccer_framework.telemetry import SoccerLogger, create_soccer_logger


class SoccerSimAgent(AgentBase):
    """A single Agent controlling a configured soccer team."""

    def __init__(self):
        super().__init__(AgentFeatures())
        self.config = SoccerConfig.from_env()
        self.soccer_logger: SoccerLogger = create_soccer_logger(
            self.logger,
            source=f"soccersim.team{self.config.team_id}",
        )
        self._structured_path_logged = False
        self._runtime_start_thread: threading.Thread | None = None
        self._closing = False
        self.runtime = SoccerTeamRuntime(
            logger=self.soccer_logger,
            config=self.config,
        )
        self.soccer_logger.info(
            "SoccerSimAgent initialized",
            event="agent_initialized",
            console=False,
            team_id=self.config.team_id,
            robot_names=list(self.config.robot_names),
            control_hz=self.config.control_hz,
        )

    def on_agent_activated(self):
        self.soccer_logger.info("SoccerSimAgent is activated")
        self._log_structured_log_path()
        self.soccer_logger.info(
            "SoccerSimAgent activated",
            event="agent_activated",
            console=False,
            team_id=self.config.team_id,
        )
        self._closing = False
        self._start_runtime_async()

    def on_agent_close(self):
        self.soccer_logger.info("SoccerSimAgent is closing")
        self.soccer_logger.info(
            "SoccerSimAgent closing",
            event="agent_closing",
            console=False,
            team_id=self.config.team_id,
        )
        self._closing = True
        try:
            if self._runtime_start_thread and self._runtime_start_thread.is_alive():
                self._runtime_start_thread.join(timeout=5.0)
                if self._runtime_start_thread.is_alive():
                    self._warn("SoccerSimAgent runtime start is still in progress")
            self.runtime.stop()
        finally:
            self.soccer_logger.info(
                "SoccerSimAgent closed",
                event="agent_closed",
                console=False,
                team_id=self.config.team_id,
            )
            self.soccer_logger.close()

    def _start_runtime_async(self) -> None:
        if self._runtime_start_thread and self._runtime_start_thread.is_alive():
            return
        self._runtime_start_thread = threading.Thread(
            target=self._start_runtime,
            name="soccer_sim_agent_runtime_start",
            daemon=True,
        )
        self._runtime_start_thread.start()

    def _start_runtime(self) -> None:
        try:
            self.runtime.start()
        except Exception as exc:
            self._error(
                f"SoccerSimAgent runtime start failed: {exc.__class__.__name__}: {exc}"
            )
            self.runtime.stop()
            return
        if self._closing:
            self.runtime.stop()

    def _log_structured_log_path(self) -> None:
        if self._structured_path_logged:
            return
        self._structured_path_logged = True
        path = self.soccer_logger.path
        if path is not None:
            self.soccer_logger.info(f"Soccer structured log: {path}")
        pretty_path = self.soccer_logger.pretty_path
        if pretty_path is not None:
            self.soccer_logger.info(f"Soccer structured pretty log: {pretty_path}")

    def _warn(self, message: str) -> None:
        self.soccer_logger.warn(message, event="agent_warning")

    def _error(self, message: str) -> None:
        self.soccer_logger.error(message, event="agent_error")
