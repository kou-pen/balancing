#!/usr/bin/env python3
"""Add STM32F446 Core source flags to the host CMake clangd compilation database.

Run after: cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
The result stays in ignored build/compile_commands.json; no CubeMX files are modified.
"""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DB_PATH = ROOT / "build" / "compile_commands.json"
MCU = ROOT / "firmware" / "f446" / "balancing_f446"

INCLUDES = [
    MCU / "Core" / "Inc",
    MCU / "Drivers" / "STM32F4xx_HAL_Driver" / "Inc",
    MCU / "Drivers" / "STM32F4xx_HAL_Driver" / "Inc" / "Legacy",
    MCU / "Drivers" / "CMSIS" / "Device" / "ST" / "STM32F4xx" / "Include",
    MCU / "Drivers" / "CMSIS" / "Include",
    MCU / "App" / "Inc",
]


def main() -> None:
    if not DB_PATH.exists():
        raise SystemExit(
            "CMake database missing. Run: cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        )

    commands = json.loads(DB_PATH.read_text(encoding="utf-8"))
    if not isinstance(commands, list):
        raise SystemExit("Invalid CMake compilation database format")
    # Host CMake retains real build flags for App C++ sources.
    sources = sorted((MCU / "Core" / "Src").glob("*.c"))
    # The HAL port has its own host-stub entry. Replace it with real STM32 HAL flags.
    sources += sorted((MCU / "App" / "Src").glob("*.c"))
    if not sources:
        raise SystemExit("No generated MCU Core/Src found")
    source_keys = {str(source.resolve()) for source in sources}
    commands = [item for item in commands if str(Path(item["file"]).resolve()) not in source_keys]
    flags = [
        "clang",
        "--target=armv7em-none-eabi",
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-ffreestanding",
        "-std=c11",
        "-DSTM32F446xx",
        "-DUSE_HAL_DRIVER",
        *[f"-I{folder.resolve()}" for folder in INCLUDES],
    ]
    for source in sources:
        commands.append(
            {
                "directory": str(ROOT),
                "file": str(source.resolve()),
                "arguments": [*flags, "-c", str(source.resolve())],
            }
        )
    DB_PATH.write_text(json.dumps(commands, indent=2) + "\n", encoding="utf-8")
    print(
        f"clangd database ready: {DB_PATH} ({len(commands)} translation units, "
        f"{len(sources)} STM32 C files)"
    )


if __name__ == "__main__":
    main()
