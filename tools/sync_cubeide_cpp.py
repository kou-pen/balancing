#!/usr/bin/env python3
"""Audit/repair STM32CubeIDE C++ metadata without modifying generated HAL code.

The .ioc remains user-owned by CubeIDE. Use --check after each CubeMX regeneration;
use --fix if project metadata dropped the C++ nature/source directory/link script or App include path.
"""

from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "firmware" / "f446" / "balancing_f446"


def cxx_metadata(project: str, cproject: str) -> tuple[str, str]:
    c_nature = "<nature>org.eclipse.cdt.core.cnature</nature>"
    cpp_nature = "<nature>org.eclipse.cdt.core.ccnature</nature>"
    if cpp_nature not in project:
        if c_nature not in project:
            raise ValueError("Missing Eclipse C nature; manual review required")
        project = project.replace(c_nature, c_nature + "\n\t\t" + cpp_nature, 1)

    driver_entry = '<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Drivers"/>'
    app_entry = '<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="App"/>'
    if cproject.count(driver_entry) != 2:
        raise ValueError("Expected two project configurations (Debug and Release)")
    if cproject.count(app_entry) == 0:
        cproject = cproject.replace(driver_entry, driver_entry + "\n\t\t\t\t\t\t" + app_entry)
    elif cproject.count(app_entry) != 2:
        raise ValueError("App source configuration is inconsistent")

    # C files in nested App/Src modules include hand-written App/Inc headers.
    # CubeMX may remove this setting during regeneration; --fix restores it.
    core_include = '<listOptionValue builtIn="false" value="../Core/Inc"/>'
    app_include = '<listOptionValue builtIn="false" value="../App/Inc"/>'
    if cproject.count(core_include) != 2:
        raise ValueError("Expected two C compiler Core/Inc include paths")
    if cproject.count(app_include) == 0:
        cproject = cproject.replace(
            core_include,
            core_include + "\n\t\t\t\t\t\t\t\t\t" + app_include,
        )
    elif cproject.count(app_include) != 2:
        raise ValueError("App/Inc C include path configuration is inconsistent")

    linker = re.compile(
        r'<tool id="(com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.cpp\.linker\.\d+)"'
        r' name="MCU/MPU G\+\+ Linker"'
        r' superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.cpp\.linker"/>'
    )

    def add_link_script(match: re.Match[str]) -> str:
        identifier = match.group(1)
        option_id = identifier + ".script"
        return (
            match.group(0)[:-2]
            + ">"
            + f'<option id="{option_id}" name="Linker Script (-T)" '
            + 'superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.linker.option.script" '
            + 'value="${workspace_loc:/${ProjName}/STM32F446RETX_FLASH.ld}" valueType="string"/>'
            + "</tool>"
        )

    cproject = linker.sub(add_link_script, cproject)
    if (
        cproject.count(
            'superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.linker.option.script"'
        )
        != 2
    ):
        raise ValueError("Both C++ linkers must explicitly use the FLASH linker script")

    # Without an explicit setting CubeIDE currently invokes GNU++14, even though
    # the application contract requires GNU++17. Keep both build configurations
    # stable across CubeMX metadata regeneration.
    standard_key = (
        "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.languagestandard"
    )
    standard_value = standard_key + ".value.gnupp17"
    compiler = re.compile(
        r'(?P<open><tool id="(?P<id>com\.st\.stm32cube\.ide\.mcu\.gnu\.'
        r'managedbuild\.tool\.cpp\.compiler\.\d+)"'
        r' name="MCU/MPU G\+\+ Compiler"'
        r' superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.'
        r'managedbuild\.tool\.cpp\.compiler">)'
        r"(?P<body>.*?)</tool>",
        re.DOTALL,
    )
    existing_standard = re.compile(
        rf'<option\b(?=[^>]*superClass="{re.escape(standard_key)}")[^>]*/>'
    )

    def require_cpp17(match: re.Match[str]) -> str:
        option = (
            f'<option id="{match.group("id")}.languagestandard" '
            'name="Language standard" '
            f'superClass="{standard_key}" value="{standard_value}" '
            'valueType="enumerated"/>'
        )
        body = match.group("body")
        found = existing_standard.search(body)
        if found:
            existing = found.group(0)
            # CubeIDE managed builds add scanner/input metadata and may reorder
            # option attributes. Keep a semantically correct GNU++17 option
            # intact instead of rewriting it on every audit.
            if f'value="{standard_value}"' in existing:
                return match.group(0)
            # Repair GNU++14 (or other explicit standard) without discarding
            # CubeIDE's additional option attributes.
            value_attr = ' value="'
            if value_attr in existing:
                start = existing.index(value_attr) + len(value_attr)
                end = existing.index('"', start)
                repaired = existing[:start] + standard_value + existing[end:]
            else:
                repaired = option
            body = body[: found.start()] + repaired + body[found.end() :]
        else:
            body = "\n\t\t\t\t\t\t\t\t" + option + body
        return match.group("open") + body + "</tool>"

    cproject, compiler_count = compiler.subn(require_cpp17, cproject)
    if compiler_count != 2 or cproject.count(f'value="{standard_value}"') != 2:
        raise ValueError("Expected two C++ compiler configurations using GNU++17")
    # The F446 C11 core now lives inside the CubeIDE App source path and uses
    # lround/lroundf/fminf/fmaxf from libm. Retain the library through regeneration.
    math_key = "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.linker.option.libraries"
    cpp_linker = re.compile(
        r'(?P<open><tool id="(?P<id>com\.st\.stm32cube\.ide\.mcu\.gnu\.'
        r'managedbuild\.tool\.cpp\.linker\.\d+)"'
        r' name="MCU/MPU G\+\+ Linker"'
        r' superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.'
        r'managedbuild\.tool\.cpp\.linker">)'
        r"(?P<body>.*?)</tool>",
        re.DOTALL,
    )
    existing_math_lib = '<listOptionValue builtIn="false" value="m"/>'
    existing_lib_option = re.compile(
        rf'<option\b(?=[^>]*superClass="{re.escape(math_key)}")[^>]*>.*?</option>',
        re.DOTALL,
    )

    def require_cpp_math_link(match: re.Match[str]) -> str:
        body = match.group("body")
        found = existing_lib_option.search(body)
        if found:
            option = found.group(0)
            if existing_math_lib not in option:
                updated_option = option.replace("</option>", existing_math_lib + "</option>")
                body = body[: found.start()] + updated_option + body[found.end() :]
        else:
            option = (
                f'<option id="{match.group("id")}.libraries" name="Libraries (-l)" '
                f'superClass="{math_key}" valueType="libs">'
                f"{existing_math_lib}</option>"
            )
            body += "\n\t\t\t\t\t\t\t\t" + option
        return match.group("open") + body + "</tool>"

    cproject, linker_count = cpp_linker.subn(require_cpp_math_link, cproject)
    if linker_count != 2 or cproject.count(f'superClass="{math_key}"') != 2:
        raise ValueError("Expected two C++ linkers explicitly linking libm")
    if cproject.count(existing_math_lib) != 2:
        raise ValueError("Missing C++ libm on Debug or Release")

    return project, cproject


def audit() -> None:
    ioc = (PROJECT / "balancing_f446.ioc").read_text(encoding="utf-8")
    main = (PROJECT / "Core/Src/main.c").read_text(encoding="utf-8")
    bridge = PROJECT / "App/Inc/balancing_app_bridge.h"
    app = PROJECT / "App/Src/balancing/app/application.cpp"
    expected_ioc = {
        "HSE 16 MHz": "RCC.HSE_VALUE=16000000",
        "SYSCLK 168 MHz": "RCC.SYSCLKFreq_VALUE=168000000",
        "USB clock 48 MHz": "RCC.USBFreq_Value=48000000",
        "CAN1 1 Mbps": "CAN1.CalculateBaudRate=1000000",
        "CAN2 1 Mbps": "CAN2.CalculateBaudRate=1000000",
        "TIM7 prescaler": "TIM7.Prescaler=8399",
        "TIM7 period": "TIM7.Period=19",
        "TIM7 IRQ": "NVIC.TIM7_IRQn=true",
        "I2C1 SCL": "PB6.Signal=I2C1_SCL",
        "I2C1 SDA": "PB7.Signal=I2C1_SDA",
        "CAN2 TX": "PB13.Signal=CAN2_TX",
        "USB Host D-": "PB14.Signal=USB_OTG_HS_DM",
        "USB Host D+": "PB15.Signal=USB_OTG_HS_DP",
        "KeepUserCode": "ProjectManager.KeepUserCode=true",
    }
    missing = [name for name, value in expected_ioc.items() if value not in ioc]
    expected_main = [
        '#include "../../App/Inc/balancing_app_bridge.h"',
        "HAL_TIM_Base_Start_IT(&htim7)",
        "balancing_app_control_tick();",
        "balancing_app_poll();",
        "HAL_TIM_PeriodElapsedCallback(",
    ]
    missing += ["main.c: " + text for text in expected_main if text not in main]
    if not bridge.is_file() or not app.is_file():
        missing.append("C++ App bridge files")
    if missing:
        raise ValueError("Regenerated project lost required configuration: " + ", ".join(missing))

    project, cproject = (
        (PROJECT / ".project").read_text(encoding="utf-8"),
        (PROJECT / ".cproject").read_text(encoding="utf-8"),
    )
    ET.fromstring(project)
    ET.fromstring(cproject)
    fixed_project, fixed_cproject = cxx_metadata(project, cproject)
    if project != fixed_project or cproject != fixed_cproject:
        raise ValueError("CubeIDE C++ build metadata differs; run this script with --fix")
    print("PASS: .ioc clock/CAN/TIM7/I2C/USB pins, main.c USER CODE, C++ Eclipse project metadata")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fix", action="store_true", help="Repair Eclipse C++ metadata only")
    args = parser.parse_args()
    if args.fix:
        project_path = PROJECT / ".project"
        cproject_path = PROJECT / ".cproject"
        current_project = project_path.read_text(encoding="utf-8")
        current_cproject = cproject_path.read_text(encoding="utf-8")
        new_project, new_cproject = cxx_metadata(current_project, current_cproject)
        ET.fromstring(new_project)
        ET.fromstring(new_cproject)
        if current_project != new_project:
            project_path.write_text(new_project, encoding="utf-8")
            print("Repaired .project C++ nature")
        if current_cproject != new_cproject:
            cproject_path.write_text(new_cproject, encoding="utf-8")
            print("Repaired .cproject C++ paths/link scripts")
    audit()


if __name__ == "__main__":
    main()
