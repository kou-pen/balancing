"""Mutation tests for the Issue #6 CAN specification consistency checker."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from check_can_contract import (  # noqa: E402
    C620_SPEC,
    CAN_SUMMARY,
    CODEC,
    CODEC_TEST,
    POWER_SPEC,
    REMOTE_SPEC,
    RS_SPEC,
    audit,
    read_sources,
)


class CanContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.baseline = read_sources()

    def changed(self, file: str, before: str, after: str) -> list[str]:
        sources = dict(self.baseline)
        self.assertIn(before, sources[file])
        sources[file] = sources[file].replace(before, after, 1)
        return audit(sources)

    def test_current_contract(self) -> None:
        self.assertEqual(audit(dict(self.baseline)), [])

    def test_duplicate_numeric_id(self) -> None:
        errors = self.changed(C620_SPEC, "0x202", "0x201")
        self.assertTrue(any("duplicate" in err for err in errors))

    def test_wrong_numeric_id(self) -> None:
        errors = self.changed(C620_SPEC, "0x201", "0x205")
        self.assertTrue(any("not declared" in err for err in errors))

    def test_standard_id_overflow(self) -> None:
        errors = self.changed(C620_SPEC, "0x202", "0x800")
        self.assertTrue(any("out of range" in err for err in errors))

    def test_c620_row_dlc(self) -> None:
        errors = self.changed(C620_SPEC, "| 8バイト |", "| 7バイト |")
        self.assertTrue(any("DLC" in err for err in errors))

    def test_rs06_row_dlc(self) -> None:
        errors = self.changed(RS_SPEC, "| 8 |", "| 7 |")
        self.assertTrue(any("invalid 5-bit Type or DLC" in err for err in errors))

    def test_rs06_type24_response_absent(self) -> None:
        errors = self.changed(RS_SPEC, "| RS06→F446 | 24 =", "| invalid→F446 | 24 =")
        self.assertTrue(any("Type/direction" in err for err in errors))

    def test_can_summary_drift(self) -> None:
        errors = self.changed(CAN_SUMMARY, "0x202", "0x205")
        self.assertTrue(any("summary missing" in err for err in errors))

    def test_power_guessed_id(self) -> None:
        errors = self.changed(
            POWER_SPEC,
            "| **未確定** | **未確定** | **未確定** |",
            "| **未確定** | **0x305** | **8** |",
        )
        self.assertTrue(any("F303" in err for err in errors))

    def test_remote_guessed_id(self) -> None:
        errors = self.changed(REMOTE_SPEC, "| **未確定** | **未確定** |", "| **0x305** | **8** |")
        self.assertTrue(any("remote CAN" in err for err in errors))

    def test_codec_command_id_drift(self) -> None:
        errors = self.changed(CODEC, "parsed.id = 0x200u;", "parsed.id = 0x205u;")
        self.assertTrue(any("C620 cmd standard ID" in err for err in errors))

    def test_codec_frame_format_drift(self) -> None:
        errors = self.changed(CODEC, "parsed.extended = false;", "parsed.extended = true;")
        self.assertTrue(any("C620 cmd standard flag" in err for err in errors))

    def test_codec_quantization_drift(self) -> None:
        errors = self.changed(
            CODEC,
            "#define C620_PROTOCOL_CURRENT_RAW 16384.0f",
            "#define C620_PROTOCOL_CURRENT_RAW 8192.0f",
        )
        self.assertTrue(any("current raw scale" in err for err in errors))

    def test_golden_type24_test_drift(self) -> None:
        errors = self.changed(CODEC_TEST, "0x188001fdu", "0x188101fdu")
        self.assertTrue(any("golden test absent" in err for err in errors))


if __name__ == "__main__":
    unittest.main()
