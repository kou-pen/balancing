"""Regression tests for STM32CubeIDE-generated C++ build metadata."""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tools.sync_cubeide_cpp import PROJECT, cxx_metadata
from tools.verify_f446_build import SOURCES, verify_configuration


class CubeIdeMetadataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.project = (PROJECT / ".project").read_text(encoding="utf-8")
        cls.cproject = (PROJECT / ".cproject").read_text(encoding="utf-8")

    def test_both_configurations_explicitly_select_gnupp17(self) -> None:
        self.assertEqual(
            self.cproject.count(
                "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler."
                "option.languagestandard.value.gnupp17"
            ),
            2,
        )
        self.assertEqual(cxx_metadata(self.project, self.cproject)[1], self.cproject)

    def test_c_compilers_have_app_headers(self) -> None:
        self.assertEqual(self.cproject.count('value="../App/Inc"'), 2)
        self.assertEqual(cxx_metadata(self.project, self.cproject)[1], self.cproject)

    def test_missing_app_include_is_repaired(self) -> None:
        stale = self.cproject.replace('<listOptionValue builtIn="false" value="../App/Inc"/>', "")
        repaired = cxx_metadata(self.project, stale)[1]
        self.assertEqual(repaired.count('value="../App/Inc"'), 2)
        self.assertEqual(cxx_metadata(self.project, repaired)[1], repaired)

    def test_firmware_sources_live_inside_cubeide_project(self) -> None:
        root = PROJECT.parents[2]
        for old_path in (
            "firmware/core",
            "firmware/f446/include",
            "firmware/f446/src",
            "firmware/examples",
        ):
            with self.subTest(old_path=old_path):
                self.assertFalse(any(item.is_file() for item in (root / old_path).rglob("*")))

        for filename in (
            "balancing_core.c",
            "balancing_motor_can.c",
            "balancing_motor_bench.c",
            "balancing_f446_motor_bench_port.c",
        ):
            with self.subTest(filename=filename):
                self.assertTrue((PROJECT / "App" / "Src" / filename).is_file())

    def test_both_cpp_linkers_explicitly_link_math_library(self) -> None:
        self.assertEqual(self.cproject.count('<listOptionValue builtIn="false" value="m"/>'), 2)
        self.assertEqual(cxx_metadata(self.project, self.cproject)[1], self.cproject)

    def test_regeneration_loss_of_math_library_is_repaired(self) -> None:
        stale = self.cproject.replace('<listOptionValue builtIn="false" value="m"/>', "")
        repaired = cxx_metadata(self.project, stale)[1]
        self.assertEqual(repaired.count('value="m"'), 2)
        self.assertEqual(cxx_metadata(self.project, repaired)[1], repaired)

    def test_regeneration_loss_is_repaired(self) -> None:
        option_pattern = (
            r"\s*<option\b(?=[^>]*superClass=\""
            r"com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.cpp\."
            r"compiler\.option\.languagestandard\")[^>]*/>"
        )
        regenerated, count = re.subn(option_pattern, "", self.cproject)
        self.assertEqual(count, 2)
        repaired = cxx_metadata(self.project, regenerated)[1]
        self.assertEqual(repaired.count("value.gnupp17"), 2)
        self.assertEqual(cxx_metadata(self.project, repaired)[1], repaired)

    def test_previous_gnupp14_setting_is_repaired(self) -> None:
        stale = self.cproject.replace("value.gnupp17", "value.gnupp14")
        self.assertNotEqual(stale, self.cproject)
        repaired = cxx_metadata(self.project, stale)[1]
        self.assertEqual(repaired, self.cproject)


class CubeIdeBuildArtifactTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.project = Path(self.temp.name)
        app_src = self.project / "App" / "Src"
        generated = self.project / "Debug" / "App" / "Src"
        app_src.mkdir(parents=True)
        generated.mkdir(parents=True)
        (self.project / ".cproject").write_text("metadata", encoding="utf-8")
        for name in SOURCES:
            (app_src / name).write_text("/* simulated source */", encoding="utf-8")
            (generated / f"{Path(name).stem}.o").write_bytes(b"test object")
        (generated / "subdir.mk").write_text(
            "\n".join(f"../App/Src/{name}" for name in SOURCES),
            encoding="utf-8",
        )
        (self.project / "Debug" / "objects.list").write_text(
            "\n".join(f"./App/Src/{Path(name).stem}.o" for name in SOURCES),
            encoding="utf-8",
        )
        self.elf = self.project / "Debug" / "balancing_f446.elf"
        self.elf.write_bytes(b"\x7fELF\x01\x01" + bytes(12) + b"\x28\x00")

    def test_complete_fresh_arm_build_is_accepted(self) -> None:
        self.assertEqual(verify_configuration(self.project, "Debug"), [])

    def test_future_app_c_source_cannot_be_silently_omitted(self) -> None:
        new_source = self.project / "App" / "Src" / "future_scheduler.c"
        new_source.write_text("/* new C implementation */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(
            any("generated makefile omits future_scheduler.c" in error for error in errors)
        )
        self.assertTrue(any("objects.list omits future_scheduler.o" in error for error in errors))

    def test_future_cpp_source_cannot_be_silently_omitted(self) -> None:
        new_source = self.project / "App" / "Src" / "future_adapter.cpp"
        new_source.write_text("/* new C++ implementation */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(
            any("generated makefile omits future_adapter.cpp" in error for error in errors)
        )

    def test_stale_makefile_is_detected_even_with_elf(self) -> None:
        mk = self.project / "Debug" / "App" / "Src" / "subdir.mk"
        mk.write_text("../App/Src/application.cpp\n", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(any("omits balancing_core.c" in error for error in errors))

    def test_stale_object_is_rejected(self) -> None:
        obj = self.project / "Debug" / "App" / "Src" / "balancing_core.o"
        os.utime(obj, ns=(1, 1))
        self.assertTrue(
            any(
                "stale object balancing_core.o" in error
                for error in verify_configuration(self.project, "Debug")
            )
        )

    def test_non_arm_elf_is_rejected(self) -> None:
        self.elf.write_bytes(b"not an ARM ELF")
        self.assertTrue(
            any(
                "not an ARM ELF32" in error for error in verify_configuration(self.project, "Debug")
            )
        )

    def test_stale_elf_is_rejected(self) -> None:
        os.utime(self.elf, ns=(1, 1))
        self.assertTrue(
            any("ELF is older" in error for error in verify_configuration(self.project, "Debug"))
        )


class CubeIdeManagedRegenerationTests(unittest.TestCase):
    def test_dry_run_generates_two_clean_build_commands_without_launching(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            launcher = root / "headless-build.sh"
            launcher.write_text("#!/bin/sh\\nexit 99\\n", encoding="utf-8")
            workspace = root / "workspaces"
            script = Path(__file__).resolve().parents[1] / "tools" / "regenerate_cubeide_f446.py"
            result = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--dry-run",
                    "--headless",
                    str(launcher),
                    "--workspace-root",
                    str(workspace),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertIn("-cleanBuild balancing_f446/Debug", result.stdout)
            self.assertIn("-cleanBuild balancing_f446/Release", result.stdout)
            self.assertIn("DRY RUN: CubeIDE not started", result.stdout)
            self.assertFalse(workspace.exists())


if __name__ == "__main__":
    unittest.main()
