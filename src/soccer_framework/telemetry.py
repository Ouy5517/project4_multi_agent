"""Structured logger plugins for SoccerSim.

Runtime code should log through :class:`SoccerLogger` with normal
``info/warn/error`` calls.  The shell logger receives concise human messages,
while structured plugins receive event records for match analysis.
Runtime code should log through :class:`SoccerLogger` using normal
``info/warn/error`` calls. The shell logger receives concise human messages, while
structured plugins receive event records for match analysis.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from enum import Enum
import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Protocol, TextIO, TypeAlias


JsonValue: TypeAlias = (
    None | bool | int | float | str | list["JsonValue"] | dict[str, "JsonValue"]
)
DEFAULT_LOG_DIR = Path("/tmp/booster_agent/soccer_logs")
_OFF_VALUES = {"0", "false", "off", "none", "disabled", "no"}
_ON_VALUES = {"1", "true", "on", "jsonl", "yes"}


class StructuredLogPlugin(Protocol):
    """Best-effort structured log plugin protocol."""

    @property
    def path(self) -> Path | None:
        """Return the backing file path when the plugin writes one."""
        ...

    @property
    def pretty_path(self) -> Path | None:
        """Return the human-readable companion file path when enabled."""
        ...

    @property
    def run_id(self) -> str:
        """Return the run id attached to every event."""
        ...

    def record(
        self,
        event: str,
        level: str = "INFO",
        message: str | None = None,
        **fields: object,
    ) -> None:
        """Record one structured event."""
        ...

    def close(self) -> None:
        """Release plugin resources."""
        ...


class NullLogPlugin:
    """No-op structured log plugin."""

    @property
    def path(self) -> Path | None:
        return None

    @property
    def pretty_path(self) -> Path | None:
        return None

    @property
    def run_id(self) -> str:
        return ""

    def record(
        self,
        event: str,
        level: str = "INFO",
        message: str | None = None,
        **fields: object,
    ) -> None:
        return

    def event(self, name: str, level: str = "INFO", **fields: object) -> None:
        return

    def close(self) -> None:
        return


class JsonlLogPlugin:
    """Append-only JSON Lines structured log plugin."""

    def __init__(
        self,
        path: Path,
        run_id: str,
        source: str,
        pretty_path: Path | None = None,
    ):
        self._path = path
        self._pretty_path = pretty_path
        self._run_id = run_id
        self._source = source
        self._lock = threading.Lock()
        self._closed = False
        self._disabled = False
        self._pretty_disabled = False
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fp: TextIO = self._path.open("a", encoding="utf-8", buffering=1)
        self._pretty_fp: TextIO | None = None
        if self._pretty_path is not None:
            self._pretty_path.parent.mkdir(parents=True, exist_ok=True)
            self._pretty_fp = self._pretty_path.open(
                "a", encoding="utf-8", buffering=1
            )

    @property
    def path(self) -> Path | None:
        return self._path

    @property
    def pretty_path(self) -> Path | None:
        return self._pretty_path

    @property
    def run_id(self) -> str:
        return self._run_id

    def record(
        self,
        event: str,
        level: str = "INFO",
        message: str | None = None,
        **fields: object,
    ) -> None:
        if self._closed or self._disabled:
            return

        record: dict[str, JsonValue] = {
            "ts_unix_ns": time.time_ns(),
            "monotonic_sec": round(time.monotonic(), 6),
            "run_id": self._run_id,
            "source": self._source,
            "level": level.upper(),
            "event": event,
        }
        if message is not None:
            record["message"] = message
        for key, value in fields.items():
            if key in record:
                key = f"field_{key}"
            record[key] = _to_json_value(value)

        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        pretty_block = json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True)
        with self._lock:
            if self._closed or self._disabled:
                return
            try:
                self._fp.write(line + "\n")
            except OSError:
                self._disabled = True
                return
            if self._pretty_fp is not None and not self._pretty_disabled:
                try:
                    self._pretty_fp.write(pretty_block + "\n\n")
                except OSError:
                    self._pretty_disabled = True

    def event(self, name: str, level: str = "INFO", **fields: object) -> None:
        """Compatibility shim for legacy telemetry callers."""

        message: str | None = None
        if "message" in fields:
            raw_message = fields.pop("message")
            message = (
                raw_message
                if isinstance(raw_message, str)
                else str(raw_message)
            )
        self.record(name, level=level, message=message, **fields)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._fp.close()
            except OSError:
                pass
            if self._pretty_fp is not None:
                try:
                    self._pretty_fp.close()
                except OSError:
                    pass


class SoccerLogger:
    """Logger wrapper that fans concise messages to shell and records events.

    ``console=False`` keeps high-volume structured events out of the shell while
    still writing them to configured plugins.
    ``console=False`` keeps high-volume structured events out of the shell while
    still writing them to configured plugins.
    """

    def __init__(
        self,
        console_logger: Any,
        plugin: StructuredLogPlugin | None = None,
    ):
        self._console_logger = console_logger
        self._plugin = NullLogPlugin() if plugin is None else plugin

    @property
    def path(self) -> Path | None:
        return self._plugin.path

    @property
    def pretty_path(self) -> Path | None:
        return self._plugin.pretty_path

    @property
    def run_id(self) -> str:
        return self._plugin.run_id

    def info(
        self,
        message: str,
        *,
        event: str | None = None,
        console: bool = True,
        **fields: object,
    ) -> None:
        self._log("INFO", message, event=event, console=console, **fields)

    def warn(
        self,
        message: str,
        *,
        event: str | None = None,
        console: bool = True,
        **fields: object,
    ) -> None:
        self._log("WARN", message, event=event, console=console, **fields)

    def warning(
        self,
        message: str,
        *,
        event: str | None = None,
        console: bool = True,
        **fields: object,
    ) -> None:
        self.warn(message, event=event, console=console, **fields)

    def error(
        self,
        message: str,
        *,
        event: str | None = None,
        console: bool = True,
        **fields: object,
    ) -> None:
        self._log("ERROR", message, event=event, console=console, **fields)

    def event(
        self,
        name: str,
        level: str = "INFO",
        message: str | None = None,
        **fields: object,
    ) -> None:
        """Compatibility shim; prefer info/warn/error with ``event=``."""

        self._record(name, level, message, **fields)

    def close(self) -> None:
        self._plugin.close()

    def _log(
        self,
        level: str,
        message: str,
        *,
        event: str | None,
        console: bool,
        **fields: object,
    ) -> None:
        if console:
            self._console(level, message)
        if event is not None:
            self._record(event, level, message, **fields)

    def _record(
        self,
        event: str,
        level: str,
        message: str | None,
        **fields: object,
    ) -> None:
        try:
            self._plugin.record(event, level=level, message=message, **fields)
        except Exception:
            return

    def _console(self, level: str, message: str) -> None:
        if level == "ERROR":
            error = getattr(self._console_logger, "error", None)
            if callable(error):
                error(message)
                return
        if level == "WARN":
            warn = getattr(self._console_logger, "warn", None)
            if callable(warn):
                warn(message)
                return
            warning = getattr(self._console_logger, "warning", None)
            if callable(warning):
                warning(message)
                return
            message = f"WARNING: {message}"

        info = getattr(self._console_logger, "info", None)
        if callable(info):
            info(message)


def create_soccer_logger(
    console_logger: Any,
    source: str = "soccersim",
) -> SoccerLogger:
    """Create a SoccerSim logger with the configured structured plugin.

    Environment variables:
    SOCCER_LOG: jsonl/on/true/1 by default, or off/none/false/0.
    SOCCER_TELEMETRY: legacy alias used when SOCCER_LOG is unset.
    SOCCER_LOG_DIR: directory for generated JSONL files.
    Defaults to /tmp/booster_agent/soccer_logs.
    SOCCER_LOG_FILE: exact JSONL file path, overrides SOCCER_LOG_DIR.
    SOCCER_PRETTY_LOG: pretty companion log is on by default; set off/false/0.
    SOCCER_RUN_ID: stable run id for correlating multiple channels.
    Environment variables:
    SOCCER_LOG: jsonl/on/true/1 by default, or off/none/false/0.
    SOCCER_TELEMETRY: legacy alias used when SOCCER_LOG is unset.
    SOCCER_LOG_DIR: directory for generated JSONL files, defaulting to /tmp/booster_agent/soccer_logs.
    SOCCER_LOG_FILE: exact JSONL file path, overriding SOCCER_LOG_DIR.
    SOCCER_PRETTY_LOG: pretty companion log is on by default; set off/false/0.
    SOCCER_RUN_ID: stable run id for correlating multiple channels.
    """

    return SoccerLogger(
        console_logger=console_logger,
        plugin=create_structured_log_plugin(source=source),
    )


def create_structured_log_plugin(source: str = "soccersim") -> StructuredLogPlugin:
    mode = _log_mode()
    if mode in _OFF_VALUES:
        return NullLogPlugin()
    if mode not in _ON_VALUES:
        return NullLogPlugin()

    run_id = os.environ.get("SOCCER_RUN_ID")
    if run_id is None or not run_id.strip():
        run_id = _default_run_id()
    else:
        run_id = _safe_path_part(run_id)

    log_file = os.environ.get("SOCCER_LOG_FILE")
    if log_file is not None and log_file.strip():
        path = Path(log_file).expanduser()
    else:
        log_dir = Path(os.environ.get("SOCCER_LOG_DIR", str(DEFAULT_LOG_DIR)))
        path = log_dir.expanduser() / run_id / f"{_safe_path_part(source)}.jsonl"

    pretty_path = _pretty_log_path(path)
    pretty_mode = os.environ.get("SOCCER_PRETTY_LOG", "on").strip().lower()
    if pretty_mode in _OFF_VALUES:
        pretty_path = None

    try:
        return JsonlLogPlugin(
            path=path,
            run_id=run_id,
            source=source,
            pretty_path=pretty_path,
        )
    except OSError:
        return NullLogPlugin()


def _log_mode() -> str:
    mode = os.environ.get("SOCCER_LOG")
    if mode is None:
        mode = os.environ.get("SOCCER_TELEMETRY", "jsonl")
    return mode.strip().lower()


def _default_run_id() -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return f"{stamp}_{uuid.uuid4().hex[:8]}"


def _safe_path_part(value: str) -> str:
    clean = []
    for char in value.strip():
        if char.isalnum() or char in {"-", "_", "."}:
            clean.append(char)
        else:
            clean.append("_")
    return "".join(clean) or "run"


def _pretty_log_path(path: Path) -> Path:
    if path.suffix:
        return path.with_suffix(".pretty.log")
    return path.with_name(path.name + ".pretty.log")


def _to_json_value(value: object) -> JsonValue:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Enum):
        enum_value = value.value
        if isinstance(enum_value, (bool, int, float, str)):
            return enum_value
        return str(enum_value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        result: dict[str, JsonValue] = {}
        for key, item in value.items():
            result[str(key)] = _to_json_value(item)
        return result
    if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
        return [_to_json_value(item) for item in value]
    return str(value)


# Legacy telemetry names kept for external scripts/tests during migration.
TelemetrySink = StructuredLogPlugin
NullTelemetry = NullLogPlugin
JsonlTelemetry = JsonlLogPlugin


def create_soccer_telemetry(source: str = "soccersim") -> TelemetrySink:
    return create_structured_log_plugin(source=source)
