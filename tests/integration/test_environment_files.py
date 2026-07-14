from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_required_environment_files_exist():
    for relative in (
        "pyproject.toml",
        "requirements-dev.txt",
        "scripts/bootstrap_wsl.sh",
        "scripts/check_env.sh",
    ):
        assert (ROOT / relative).is_file(), relative
