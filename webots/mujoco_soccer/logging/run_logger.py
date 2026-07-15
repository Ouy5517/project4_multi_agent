from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class RunLogger:
    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        self.run_dir.mkdir(parents=True, exist_ok=True)

    def write_json(self, name: str, data: dict[str, Any]) -> None:
        (self.run_dir / name).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    def append_jsonl(self, name: str, data: dict[str, Any]) -> None:
        with (self.run_dir / name).open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")

