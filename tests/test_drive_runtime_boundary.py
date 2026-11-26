#!/usr/bin/env python3
"""Prevent host-only drive implementations from leaking into the firmware source tree."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "firmware/f446/balancing_f446/App"

# 実機側のコンパイル対象に試験条件・試験の型や入口を置かない。
for path in APP.rglob("*"):
    if path.suffix not in (".cpp", ".hpp", ".c", ".h"):
        continue
    content = path.read_text(encoding="utf-8")
    for token in ("BALANCING_HOST_SIL", "DriveSilProfile", "DriveSilResult",
                  "initializeDriveSil", "driveSilObservation", "DriveCycle.hpp"):
        assert token not in content, f"{path.relative_to(ROOT)} contains {token}"

# Host driverの誤組込みはコンパイル時に明示エラーにする。
guard = (ROOT / "tests/sil/DriveCycle.hpp").read_text(encoding="utf-8")
assert "#if defined(__arm__) || defined(__thumb__)" in guard
assert "#error" in guard

cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
assert "BALANCING_HOST_SIL" not in cmake
assert "tests/sil/DriveRuntime.cpp" in cmake
assert "${F446_APP}/Src/balancing/app/DriveRuntime.cpp" in cmake
assert "tests/sil/DriveCycle.cpp" in cmake

print("PASS: firmware and host drive runtime source boundary")
