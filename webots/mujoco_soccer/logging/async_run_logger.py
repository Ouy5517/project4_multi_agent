from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class LogItem:
    name: str
    data: dict[str, Any]
    priority: str = "NORMAL"


class AsyncRunLogger:
    def __init__(self, run_dir: Path, max_queue: int = 8192, flush_interval: float = 0.5) -> None:
        self.run_dir = run_dir
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.queue: queue.Queue[LogItem | None] = queue.Queue(maxsize=max_queue)
        self.flush_interval = flush_interval
        self.high_watermark = 0
        self.dropped_low_priority = 0
        self.enqueue_seconds = 0.0
        self.enqueued = 0
        self._closed = False
        self._thread = threading.Thread(target=self._worker, name="async-run-logger", daemon=True)
        self._thread.start()

    def write_json(self, name: str, data: dict[str, Any]) -> None:
        (self.run_dir / name).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    def append_jsonl(self, name: str, data: dict[str, Any], priority: str = "NORMAL") -> None:
        if self._closed:
            return
        started = time.perf_counter()
        item = LogItem(name, data, priority)
        try:
            if priority == "HIGH":
                self.queue.put(item, timeout=0.05)
            else:
                self.queue.put_nowait(item)
            self.enqueued += 1
            self.high_watermark = max(self.high_watermark, self.queue.qsize())
        except queue.Full:
            if priority == "LOW":
                self.dropped_low_priority += 1
            else:
                self.queue.put(item, timeout=0.1)
                self.enqueued += 1
        finally:
            self.enqueue_seconds += time.perf_counter() - started

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.queue.put(None)
        self._thread.join(timeout=5.0)

    def metrics(self) -> dict[str, float | int | bool]:
        return {
            "async_logging": True,
            "log_queue_high_watermark": self.high_watermark,
            "log_queue_dropped_low_priority": self.dropped_low_priority,
            "log_enqueue_average_ms": (self.enqueue_seconds / max(1, self.enqueued)) * 1000.0,
            "synchronous_flush_on_main_thread": False,
        }

    def _worker(self) -> None:
        handles: dict[str, Any] = {}
        last_flush = time.perf_counter()
        try:
            while True:
                try:
                    item = self.queue.get(timeout=self.flush_interval)
                except queue.Empty:
                    item = None
                    should_stop = False
                else:
                    should_stop = item is None
                if item is not None:
                    handle = handles.get(item.name)
                    if handle is None:
                        handle = (self.run_dir / item.name).open("a", encoding="utf-8")
                        handles[item.name] = handle
                    handle.write(json.dumps(item.data, ensure_ascii=False, sort_keys=True) + "\n")
                now = time.perf_counter()
                if now - last_flush >= self.flush_interval or should_stop:
                    for handle in handles.values():
                        handle.flush()
                    last_flush = now
                if should_stop:
                    while True:
                        try:
                            pending = self.queue.get_nowait()
                        except queue.Empty:
                            break
                        if pending is None:
                            continue
                        handle = handles.get(pending.name)
                        if handle is None:
                            handle = (self.run_dir / pending.name).open("a", encoding="utf-8")
                            handles[pending.name] = handle
                        handle.write(json.dumps(pending.data, ensure_ascii=False, sort_keys=True) + "\n")
                    break
        finally:
            for handle in handles.values():
                handle.flush()
                handle.close()
