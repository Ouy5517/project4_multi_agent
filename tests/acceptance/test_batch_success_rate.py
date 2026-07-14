import json

from evaluation.batch_runner import run_batch


def test_pass_fixed_batch_success_rate(tmp_path):
    summary = run_batch(
        scenario="pass_fixed",
        runs=3,
        seed_start=2000,
        output_dir=tmp_path,
    )

    assert summary["runs"] == 3
    assert summary["successes"] == 3
    assert summary["success_rate"] == 1.0
    assert len(summary["seeds"]) == 3
    written = json.loads((tmp_path / "batch_summary.json").read_text(encoding="utf-8"))
    assert written["scenario"] == "pass_fixed"
