import json
from pathlib import Path
from statistics import mean
from typing import Dict

from evaluation.scenario_evaluator import run_scenario


def run_batch(scenario: str, runs: int, seed_start: int, output_dir: str | Path) -> Dict[str, object]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    results = []
    for index in range(runs):
        seed = seed_start + index
        result = run_scenario(scenario, seed=seed, fast=True)
        results.append((seed, result))

    successes = sum(1 for _, result in results if result.success)
    elapsed = [
        result.metrics.get("time_to_receive_s", result.metrics.get("time_to_goal_s", 0.0))
        for _, result in results
    ]
    failure_codes = {}
    for _, result in results:
        if not result.success:
            code = result.failure_code or "UNKNOWN"
            failure_codes[code] = failure_codes.get(code, 0) + 1

    summary = {
        "scenario": scenario,
        "runs": runs,
        "successes": successes,
        "success_rate": successes / max(runs, 1),
        "mean_time_s": mean(elapsed) if elapsed else 0.0,
        "p95_time_s": sorted(elapsed)[int(0.95 * (len(elapsed) - 1))] if elapsed else 0.0,
        "failure_codes": failure_codes,
        "seeds": [seed for seed, _ in results],
    }
    (output_path / "batch_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return summary
