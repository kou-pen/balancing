#!/usr/bin/env python3
"""Verify fresh STM32CubeIDE Debug/Release/BenchDebug outputs for nested F446 App modules.

Only artifacts are inspected. A PASS does not establish whether CubeIDE itself
generated the makefiles, that the firmware has been flashed, or hardware safety.
Run a native CubeIDE clean-build and inspect the corresponding log first.
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1] / "firmware" / "f446" / "balancing_f446"

# Mandatory translations units in the agreed layout. Other App C/C++ files
# are discovered recursively so later additions cannot silently be skipped.
SOURCES = (
    "balancing/app/application.cpp",
    "balancing/bench/MotorBench.cpp",
    "balancing/bench/f446_motor_bench_runtime.cpp",
    "balancing/communication/Rs06Codec.cpp",
    "balancing/communication/C620Codec.cpp",
    "balancing/communication/can/CanTxScheduler.cpp",
    "balancing/control/controller/BalanceControl.cpp",
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
    elf = build / ("balancing_f446_bench.elf" if configuration=="BenchDebug" else "balancing_f446.elf")
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


# C ABI entrypoints are matched as complete demangled symbols. Other checks use
# distinctive qualified names. A normal object can still be compiled into
# BenchDebug, but the normal execution paths must not survive the linker.
NORMAL_SYMBOLS = ("balancing_app_init", "balancing_app_begin_control_tick",
                  "balancing_app_control_tick", "balancing_app_poll",
                  "balancing_f446_can_start", "balancing_f446_can1_capture_irq",
                  "balancing_f446_can2_capture_irq", "balancing::RobotSequence::update(",
                  "balancing::startCanObservation(")
BENCH_SYMBOLS = ("balancing_f446_bench_start", "balancing_f446_bench_service",
                 "balancing_f446_bench_capture_irq", "balancing::startCanBench(",
                 "balancing::MotorBench::poll(")
FORBIDDEN_SYMBOLS = ("balancing::MotorBench::requestPulse(", "DriveCycle::",
                     "driveSilObservation", "initializeDriveSil")


def verify_mode_symbols(symbols: str, configuration: str) -> list[str]:
    required = BENCH_SYMBOLS if configuration == "BenchDebug" else NORMAL_SYMBOLS
    forbidden = (NORMAL_SYMBOLS if configuration == "BenchDebug" else BENCH_SYMBOLS) + FORBIDDEN_SYMBOLS
    # Only definitions (addresses and symbol types) count, not an incidental
    # mention in a global constructor name or an undefined reference.
    definitions = []
    for line in symbols.splitlines():
        fields = line.split(maxsplit=2)
        if len(fields) == 3 and all(ch in "0123456789abcdefABCDEF" for ch in fields[0]):
            definitions.append(fields[2])
    def present(entry: str) -> bool:
        if entry.endswith("("):
            return any(name.startswith(entry) for name in definitions)
        if entry.endswith("::") or entry in ("driveSilObservation", "initializeDriveSil"):
            return any(entry in name for name in definitions)
        return entry in definitions
    return [f"{configuration}: missing linked entry {x}" for x in required if not present(x)] + [
        f"{configuration}: forbidden linked entry {x}" for x in forbidden if present(x)]


def verify_linked_mode(project: Path, configuration: str, nm: Path) -> list[str]:
    artifact = "balancing_f446_bench.elf" if configuration == "BenchDebug" else "balancing_f446.elf"
    result = subprocess.run([str(nm), "-C", str(project / configuration / artifact)],
                            capture_output=True, text=True, check=False)
    if result.returncode:
        return [f"{configuration}: nm failed: {result.stderr.strip()}"]
    return verify_mode_symbols(result.stdout, configuration)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--configuration",
        choices=("Debug", "Release", "BenchDebug"),
        action="append",
        help="Restrict to one or both configurations; default checks both",
    )
    parser.add_argument("--nm", type=Path, help="Also verify exclusive mode symbols using arm-none-eabi-nm")
    args = parser.parse_args()
    configurations = args.configuration or ["Debug", "Release", "BenchDebug"]
    failures: list[str] = []
    for name in configurations:
        errors = verify_configuration(PROJECT, name)
        if not errors and args.nm:
            errors = verify_linked_mode(PROJECT, name, args.nm)
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
