#!/usr/bin/env python3
"""Verify that fresh STM32CubeIDE Debug/Release builds contain every F446 App source.

The generated makefiles and ELF files are ignored build artifacts. This is a
post-build check: run CubeIDE headless/IDE builds before invoking it. It never
launches a build or flashes a device.
"""

from __future__ import annotations

import argparse
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1] / "firmware" / "f446" / "balancing_f446"
SOURCES = (
    "application.cpp",
    "balancing_core.c",
    "balancing_motor_can.c",
    "balancing_motor_bench.c",
    "balancing_f446_motor_bench_port.c",
)


def verify_configuration(project: Path, configuration: str) -> list[str]:
    """Return reasons why the generated build cannot be accepted as verified."""
    build = project / configuration
    app_src = project / "App" / "Src"
    subdir = build / "App" / "Src" / "subdir.mk"
    objects_list = build / "objects.list"
    elf = build / "balancing_f446.elf"
    metadata = project / ".cproject"

    errors: list[str] = []
    inputs = [metadata, subdir, objects_list]
    sources = [app_src / name for name in SOURCES]
    for path in [*inputs, *sources, elf]:
        if not path.is_file():
            errors.append(f"missing: {path.relative_to(project)}")
    if errors:
        return errors

    generated = subdir.read_text(encoding="utf-8")
    object_names = objects_list.read_text(encoding="utf-8")
    for source in sources:
        source_entry = f"../App/Src/{source.name}"
        obj_entry = f"./App/Src/{source.stem}.o"
        if source_entry not in generated:
            errors.append(f"{configuration}: generated makefile omits {source.name}")
        if obj_entry not in object_names:
            errors.append(f"{configuration}: objects.list omits {source.stem}.o")
        obj = build / "App" / "Src" / f"{source.stem}.o"
        if not obj.is_file() or obj.stat().st_size == 0:
            errors.append(f"{configuration}: missing/empty object {source.stem}.o")
        elif obj.stat().st_mtime_ns < source.stat().st_mtime_ns:
            errors.append(f"{configuration}: stale object {source.stem}.o")

    with elf.open("rb") as stream:
        header = stream.read(20)
    # STM32F446 ELF should be 32-bit little-endian and e_machine=40 (ARM).
    if len(header) < 20 or header[:6] != b"\x7fELF\x01\x01" or header[18:20] != b"\x28\x00":
        errors.append(f"{configuration}: output is not an ARM ELF32 executable")

    newest_input = max(path.stat().st_mtime_ns for path in [*inputs, *sources])
    if elf.stat().st_mtime_ns < newest_input:
        errors.append(f"{configuration}: ELF is older than source/build metadata; rebuild required")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--configuration",
        choices=("Debug", "Release"),
        action="append",
        help="Restrict checks to one or both configurations; default checks both",
    )
    args = parser.parse_args()
    configurations = args.configuration or ["Debug", "Release"]
    failures = []
    for name in configurations:
        errors = verify_configuration(PROJECT, name)
        if errors:
            print(f"NOT VERIFIED: {name}")
            for error in errors:
                print(f"  - {error}")
            failures.extend(errors)
        else:
            print(f"PASS: {name} has all {len(SOURCES)} sources, objects and a fresh ARM ELF32")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
