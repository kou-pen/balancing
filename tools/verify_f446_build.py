#!/usr/bin/env python3
"""Verify fresh STM32CubeIDE Debug/Release outputs for nested F446 App modules.

Only artifacts are inspected. A PASS does not establish whether CubeIDE itself
generated the makefiles, that the firmware has been flashed, or hardware safety.
Run a native CubeIDE clean-build and inspect the corresponding log first.
"""

from __future__ import annotations

import argparse
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1] / "firmware" / "f446" / "balancing_f446"

# Mandatory translations units in the agreed layout. Other App C/C++ files
# are discovered recursively so later additions cannot silently be skipped.
SOURCES = (
    "balancing/app/application.cpp",
    "balancing/app/balancing_motor_bench.c",
    "balancing/communication/balancing_motor_can.c",
    "balancing/communication/balancing_tx_scheduler.c",
    "balancing/control/controller/balancing_core.c",
    "balancing/device/balancing_f446_motor_bench_port.c",
)


def app_sources(project: Path) -> list[Path]:
    """Return mandatory and discovered nested C/C++ files, using full paths."""
    app_src = project / "App" / "Src"
    names = {Path(name) for name in SOURCES}
    if app_src.is_dir():
        names.update(
            path.relative_to(app_src)
            for path in app_src.rglob("*")
            if path.is_file() and path.suffix in (".c", ".cpp")
        )
    return [app_src / name for name in sorted(names)]


def generated_makefile(build: Path, source: Path, app_src: Path) -> Path:
    """CubeIDE writes one subdir.mk in each generated source directory."""
    return build / "App" / "Src" / source.relative_to(app_src).parent / "subdir.mk"


def verify_configuration(project: Path, configuration: str) -> list[str]:
    """Explain any missing, incomplete, non-ARM or stale output."""
    build = project / configuration
    app_src = project / "App" / "Src"
    objects_list = build / "objects.list"
    elf = build / "balancing_f446.elf"
    metadata = project / ".cproject"
    sources = app_sources(project)
    makefiles = sorted({generated_makefile(build, source, app_src) for source in sources})
    inputs = [metadata, objects_list, *makefiles]

    errors: list[str] = []
    for path in [*inputs, *sources, elf]:
        if not path.is_file():
            errors.append(f"missing: {path.relative_to(project)}")
    if errors:
        return errors

    objects = objects_list.read_text(encoding="utf-8")
    for source in sources:
        relative = source.relative_to(app_src)
        makefile = generated_makefile(build, source, app_src)
        generated = makefile.read_text(encoding="utf-8")
        source_entry = f"../App/Src/{relative.as_posix()}"
        obj_relative = relative.with_suffix(".o")
        obj_entry = f"./App/Src/{obj_relative.as_posix()}"
        if source_entry not in generated:
            errors.append(f"{configuration}: generated makefile omits {relative.as_posix()}")
        if obj_entry not in objects:
            errors.append(f"{configuration}: objects.list omits {obj_relative.as_posix()}")
        obj = build / "App" / "Src" / obj_relative
        if not obj.is_file() or obj.stat().st_size == 0:
            errors.append(f"{configuration}: missing/empty object {obj_relative.as_posix()}")
        elif obj.stat().st_mtime_ns < source.stat().st_mtime_ns:
            errors.append(f"{configuration}: stale object {obj_relative.as_posix()}")

    with elf.open("rb") as stream:
        header = stream.read(20)
    # e_ident: ELF32/LE; e_machine = ARM (40).
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
        help="Restrict to one or both configurations; default checks both",
    )
    args = parser.parse_args()
    configurations = args.configuration or ["Debug", "Release"]
    failures: list[str] = []
    for name in configurations:
        errors = verify_configuration(PROJECT, name)
        if errors:
            print(f"NOT VERIFIED: {name}")
            for error in errors:
                print(f"  - {error}")
            failures.extend(errors)
        else:
            print(
                f"PASS: {name} has all {len(app_sources(PROJECT))} sources, "
                "objects and a fresh ARM ELF32 (artifact checks only)"
            )
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
