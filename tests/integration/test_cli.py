import subprocess
import sys


def run_cli(*args):
    return subprocess.run(
        [sys.executable, "main.py", *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )


def test_help_exits_zero():
    result = run_cli("--help")
    assert result.returncode == 0
    assert "--scenario" in result.stdout


def test_help_exposes_mujoco_live_viewer():
    result = run_cli("--help")
    assert result.returncode == 0
    assert "--viewer {ascii,mujoco,none}" in result.stdout


def test_help_exposes_showcase_scenario():
    result = run_cli("--help")
    assert result.returncode == 0
    assert "showcase" in result.stdout


def test_unknown_scenario_fails():
    result = run_cli("--scenario", "missing")
    assert result.returncode != 0


def test_strict_pass_fixed_exits_zero():
    result = run_cli("--scenario", "pass_fixed", "--headless", "--fast", "--strict")
    assert result.returncode == 0


def test_real_mode_without_endpoints_fails_fast():
    result = run_cli("--mode", "real", "--headless", "--duration", "1")
    assert result.returncode == 3
    assert "world-source" in result.stdout
