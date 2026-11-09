"""Regression tests for STM32CubeIDE-generated C++ build metadata."""

from __future__ import annotations

import re
import unittest

from tools.sync_cubeide_cpp import PROJECT, cxx_metadata


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
        stale = self.cproject.replace(
            '<listOptionValue builtIn="false" value="../App/Inc"/>', ""
        )
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
                self.assertFalse((root / old_path).exists())

        for filename in (
            "balancing_core.c",
            "balancing_motor_can.c",
            "balancing_motor_bench.c",
            "balancing_f446_motor_bench_port.c",
        ):
            with self.subTest(filename=filename):
                self.assertTrue((PROJECT / "App" / "Src" / filename).is_file())

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


if __name__ == "__main__":
    unittest.main()
