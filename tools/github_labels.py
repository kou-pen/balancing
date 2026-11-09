"""Apply the repository-owned GitHub issue label definitions through gh CLI.

Read-only by default. Never deletes labels and never logs credentials.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path

REPOSITORY = "kou-pen/balancing"
LABELS_PATH = Path(__file__).resolve().parents[1] / ".github" / "labels.json"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="show commands without writes")
    mode.add_argument("--apply", action="store_true", help="create/update GitHub labels via gh")
    args = parser.parse_args()

    labels = json.loads(LABELS_PATH.read_text(encoding="utf-8"))
    names = [entry["name"] for entry in labels]
    if len(names) != len(set(names)):
        raise SystemExit("Duplicate labels in labels.json")

    if args.apply and shutil.which("gh") is None:
        raise SystemExit("GitHub CLI (gh) is not available. Install it and run gh auth login.")

    if args.apply:
        subprocess.run(["gh", "auth", "status"], check=True)

    for entry in labels:
        argv = [
            "gh",
            "label",
            "create",
            entry["name"],
            "--repo",
            REPOSITORY,
            "--color",
            entry["color"],
            "--description",
            entry["description"],
            "--force",
        ]
        if args.dry_run:
            print(entry["name"], entry["color"], entry["description"])
        else:
            subprocess.run(argv, check=True)

    print(f"{len(labels)} labels " + ("listed (dry run)" if args.dry_run else "applied"))


if __name__ == "__main__":
    main()
