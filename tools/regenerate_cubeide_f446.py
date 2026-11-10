#!/usr/bin/env python3
"""Regenerate F446 Debug/Release makefiles with STM32CubeIDE's managed builder.

Use on the authorized development VM. No flashing, Git operations, or firmware
runtime changes. An old manually patched makefile is never treated as proof of
CubeIDE-generated metadata.

The --dry-run mode prints the two exact commands without launching CubeIDE.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path

from verify_f446_build import app_sources, generated_makefile, verify_configuration

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "firmware" / "f446" / "balancing_f446"
DEFAULT_HEADLESS = Path("/opt/st/stm32cubeide_1.19.0/headless-build.sh")
CONFIGURATIONS = ("Debug", "Release")
MANUAL_PATCH_MARKER = "Temporary additions to verify moved C sources"


def managed_build_commands(
    headless: Path, workspace_root: Path, project: Path = PROJECT
) -> list[tuple[str, list[str]]]:
    """Each configuration gets an independent Eclipse workspace."""
    return [
        (
            configuration,
            [
                str(headless),
                "-data",
                str(workspace_root / configuration.lower()),
                "-import",
                str(project),
                "-cleanBuild",
                f"balancing_f446/{configuration}",
            ],
        )
        for configuration in CONFIGURATIONS
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--headless", type=Path, default=DEFAULT_HEADLESS)
    parser.add_argument(
        "--workspace-root", type=Path, default=ROOT / "build" / "cubeide-managed-pr16"
    )
    args = parser.parse_args()
    headless = args.headless.expanduser().resolve()
    workspace_root = args.workspace_root.expanduser().resolve()

    if not headless.is_file():
        print(f"ERROR: CubeIDE launcher not found: {headless}", file=sys.stderr)
        return 2
    if not (PROJECT / ".cproject").is_file():
        print(f"ERROR: missing .cproject in {PROJECT}", file=sys.stderr)
        return 2

    commands = managed_build_commands(headless, workspace_root)
    if args.dry_run:
        for config, command in commands:
            print(f"{config}: {shlex.join(command)}")
        print("DRY RUN: CubeIDE not started; no build verified.")
        return 0

    print("Checking tracked CubeIDE project metadata...", flush=True)
    metadata_check = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "sync_cubeide_cpp.py")],
        cwd=ROOT,
        check=False,
    )
    if metadata_check.returncode:
        print("ERROR: .cproject/.project audit failed; no build attempted.", file=sys.stderr)
        return 1

    workspace_root.mkdir(parents=True, exist_ok=True)
    for config, command in commands:
        log = workspace_root / f"{config.lower()}.log"
        print(f"CubeIDE managed clean-build: {config} (log: {log})", flush=True)
        with log.open("w", encoding="utf-8") as output:
            result = subprocess.run(
                command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, check=False
            )
        if result.returncode:
            tail = log.read_text(encoding="utf-8", errors="replace").splitlines()[-50:]
            print("\n".join(tail), file=sys.stderr)
            print(
                f"NOT VERIFIED: CubeIDE {config} returned {result.returncode}.",
                file=sys.stderr,
            )
            return 1

        app_src = PROJECT / "App" / "Src"
        for source in app_sources(PROJECT):
            generated = generated_makefile(PROJECT / config, source, app_src)
            if not generated.is_file():
                print(f"NOT VERIFIED: {generated} missing", file=sys.stderr)
                return 1
            if MANUAL_PATCH_MARKER in generated.read_text(encoding="utf-8"):
                print(
                    f"NOT VERIFIED: CubeIDE {config} left manually patched subdir.mk "
                    f"in {generated}; cannot claim automatic regeneration.",
                    file=sys.stderr,
                )
                return 1

        errors = verify_configuration(PROJECT, config)
        if errors:
            for error in errors:
                print(f"NOT VERIFIED: {error}", file=sys.stderr)
            return 1
        print(f"PASS: CubeIDE-generated {config} build has all App sources and fresh ARM ELF.")

    print("PASS: CubeIDE managed regeneration and both cross-builds completed.")
    print("This does not certify motor control behavior or physical safety.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
