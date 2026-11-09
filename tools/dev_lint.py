#!/usr/bin/env python3
"""Run the project-local Ruff binary (or a Ruff executable on PATH)."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def find_ruff() -> str:
    candidates = [
        ROOT / ".venv" / "bin" / "ruff",
        ROOT / ".venv" / "local" / "bin" / "ruff",  # Debian pip --prefix layout
        ROOT / ".venv" / "Scripts" / "ruff.exe",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return shutil.which("ruff") or ""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=["ruff-check", "ruff-format-check", "ruff-fix", "ruff-format"],
    )
    args = parser.parse_args()

    ruff = find_ruff()
    if not ruff:
        raise SystemExit(
            "Ruff CLI not found. The VS Code extension can work using its bundled "
            "Ruff. For command-line tasks, create a .venv and run: "
            ".venv/bin/python -m pip install ruff"
        )

    commands = {
        "ruff-check": [ruff, "check", "tools", "tests"],
        "ruff-format-check": [ruff, "format", "--check", "tools", "tests"],
        "ruff-fix": [ruff, "check", "--fix", "tools", "tests"],
        "ruff-format": [ruff, "format", "tools", "tests"],
    }
    command = commands[args.action]
    result = subprocess.run(command, cwd=ROOT, check=False)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
