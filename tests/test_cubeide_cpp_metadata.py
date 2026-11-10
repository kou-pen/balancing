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

        for filename in SOURCES:
            with self.subTest(filename=filename):
                self.assertTrue((PROJECT / "App" / "Src" / filename).is_file())
        # All hand-written translation units belong under the agreed module root.
        self.assertFalse(list((PROJECT / "App" / "Src").glob("*.c")))
        self.assertFalse(list((PROJECT / "App" / "Src").glob("*.cpp")))

    def test_public_headers_mirror_source_modules(self) -> None:
        expected = (
            "balancing/app/balancing_motor_bench.h",
            "balancing/communication/balancing_motor_can.h",
            "balancing/communication/balancing_tx_scheduler.h",
            "balancing/control/controller/balancing_core.h",
            "balancing/device/balancing_f446_motor_bench_port.h",
        )
        inc = PROJECT / "App" / "Inc"
        self.assertTrue((inc / "balancing_app_bridge.h").is_file())
        for relative in expected:
            with self.subTest(header=relative):
                self.assertTrue((inc / relative).is_file())
                self.assertFalse((inc / Path(relative).name).exists())

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
        build = self.project / "Debug"
        app_src.mkdir(parents=True)
        build.mkdir(parents=True)
        (self.project / ".cproject").write_text("metadata", encoding="utf-8")
        generated_by_folder: dict[Path, list[str]] = {}
        object_list: list[str] = []
        for name in SOURCES:
            relative = Path(name)
            source = app_src / relative
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_text("/* simulated source */", encoding="utf-8")
            generated = build / "App" / "Src" / relative.parent
            generated.mkdir(parents=True, exist_ok=True)
            (generated / f"{relative.stem}.o").write_bytes(b"test object")
            generated_by_folder.setdefault(generated, []).append(f"../App/Src/{name}")
            object_list.append(f"./App/Src/{relative.with_suffix('.o').as_posix()}")
        for folder, entries in generated_by_folder.items():
            (folder / "subdir.mk").write_text(chr(10).join(entries), encoding="utf-8")
        (build / "objects.list").write_text(chr(10).join(object_list), encoding="utf-8")
        self.elf = build / "balancing_f446.elf"
        self.elf.write_bytes(
            bytes.fromhex("7f 45 4c 46 01 01") + bytes(12) + bytes.fromhex("28 00")
        )

    def test_complete_fresh_arm_build_is_accepted(self) -> None:
        self.assertEqual(verify_configuration(self.project, "Debug"), [])

    def test_future_app_c_source_cannot_be_silently_omitted(self) -> None:
        new_source = self.project / "App/Src/balancing/communication/future_scheduler.c"
        new_source.write_text("/* new C implementation */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(
            any(
                "generated makefile omits" in error and "future_scheduler.c" in error
                for error in errors
            )
        )
        self.assertTrue(
            any("objects.list omits" in error and "future_scheduler.o" in error for error in errors)
        )

    def test_future_cpp_source_cannot_be_silently_omitted(self) -> None:
        new_source = self.project / "App/Src/balancing/app/future_adapter.cpp"
        new_source.write_text("/* new C++ implementation */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(
            any(
                "generated makefile omits" in error and "future_adapter.cpp" in error
                for error in errors
            )
        )

    def test_missing_generated_nested_makefile_is_rejected(self) -> None:
        future = self.project / "App/Src/balancing/control/filter/future_filter.c"
        future.parent.mkdir(parents=True)
        future.write_text("/* future filter */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(any("balancing/control/filter/subdir.mk" in error for error in errors))

    def test_orphan_source_at_old_app_root_is_rejected(self) -> None:
        (self.project / "App/Src/orphan.c").write_text("/* source */", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(any("Debug/App/Src/subdir.mk" in error for error in errors))

    def test_missing_mandatory_source_is_rejected(self) -> None:
        (self.project / "App/Src/balancing/control/controller/balancing_core.c").unlink()
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(
            any("missing:" in error and "balancing_core.c" in error for error in errors)
        )

    def test_stale_makefile_is_detected_even_with_elf(self) -> None:
        mk = self.project / "Debug/App/Src/balancing/control/controller/subdir.mk"
        mk.write_text("../App/Src/balancing/app/application.cpp\\n", encoding="utf-8")
        errors = verify_configuration(self.project, "Debug")
        self.assertTrue(any("omits" in error and "balancing_core.c" in error for error in errors))

    def test_stale_object_is_rejected(self) -> None:
        obj = self.project / "Debug/App/Src/balancing/control/controller/balancing_core.o"
        os.utime(obj, ns=(1, 1))
        self.assertTrue(
            any(
                "stale object" in error and "balancing_core.o" in error
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
