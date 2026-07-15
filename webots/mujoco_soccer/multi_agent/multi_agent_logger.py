from __future__ import annotations

from pathlib import Path
from typing import Any

from mujoco_soccer.logging.async_run_logger import AsyncRunLogger
from mujoco_soccer.logging.run_logger import RunLogger


class MultiAgentLogger(RunLogger):
    def __init__(self, run_dir: Path, async_mode: bool = False) -> None:
        super().__init__(run_dir)
        self.async_mode = async_mode
        self._async_logger = AsyncRunLogger(run_dir) if async_mode else None

    def write_json(self, name: str, data: dict[str, Any]) -> None:
        if self._async_logger is not None:
            self._async_logger.write_json(name, data)
            return
        super().write_json(name, data)

    def append_jsonl(self, name: str, data: dict[str, Any], priority: str = "NORMAL") -> None:
        if self._async_logger is not None:
            self._async_logger.append_jsonl(name, data, priority)
            return
        super().append_jsonl(name, data)

    def log_decision_bundle(self, data: dict[str, Any]) -> None:
        self.append_jsonl("agent_decisions.jsonl", data)

    def close(self) -> None:
        if self._async_logger is not None:
            self._async_logger.close()

    def metrics(self) -> dict[str, float | int | bool]:
        if self._async_logger is None:
            return {
                "async_logging": False,
                "log_queue_high_watermark": 0,
                "log_queue_dropped_low_priority": 0,
                "log_enqueue_average_ms": 0.0,
                "synchronous_flush_on_main_thread": True,
            }
        return self._async_logger.metrics()
