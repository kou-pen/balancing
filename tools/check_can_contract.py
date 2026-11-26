#!/usr/bin/env python3
"""Host-only CAN contract checks against canonical protocol specs and C++ codecs."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAN_SUMMARY = "docs/can-id-table.md"
C620_SPEC = "docs/specs/protocols/c620_can2.md"
RS_SPEC = "docs/specs/protocols/rs06_can1.md"
POWER_SPEC = "docs/specs/protocols/power_can2.md"
REMOTE_SPEC = "docs/specs/protocols/remote_can2.md"
CODEC = "firmware/f446/balancing_f446/App/Src/balancing/communication/C620Codec.cpp"
RS_CODEC = "firmware/f446/balancing_f446/App/Src/balancing/communication/Rs06Codec.cpp"
CODEC_TEST = "tests/test_motor_can.cpp"
V1_TEST = "tests/test_f446_protocol_codec.cpp"

# In this robot: 0x1FF is unused, 0x200 is a command and 0x201/202
# are feedback. This is a regression boundary, NOT a firmware config.
CAN2 = {
    0x1FF: ("F446→C620", 8),
    0x200: ("F446→C620", 8),
    0x201: ("C620→F446", 8),
    0x202: ("C620→F446", 8),
}
RS_TYPES = {(1, "TX"), (2, "RX"), (3, "TX"), (4, "TX"), (24, "TX"), (24, "RX")}
def read_sources(root: Path = ROOT) -> dict[str, str]:
    """Return the small set of authoritative sources and code being audited."""
    return {
        name: (root / name).read_text(encoding="utf-8")
        for name in (CAN_SUMMARY, C620_SPEC, RS_SPEC, POWER_SPEC, REMOTE_SPEC, CODEC, RS_CODEC, CODEC_TEST, V1_TEST)
    }


def c620_rows(spec: str) -> list[tuple[int, str, int | None]]:
    """Extract the four CAN2 entries, preserving duplicates for detection."""
    rows = []
    for line in spec.splitlines():
        if not re.match(r"^\|\s*(?:F446→C620|C620→F446)\s*\|", line):
            continue
        parts = [field.strip() for field in line.split("|")]
        id_match = re.search(r"0x[0-9A-Fa-f]+", parts[2])
        dlc_match = re.search(r"(\d+)\s*バイト", parts[4])
        if id_match is None:
            rows.append((-1, parts[1], None))
        else:
            rows.append(
                (
                    int(id_match.group(), 16),
                    parts[1],
                    int(dlc_match.group(1)) if dlc_match else None,
                )
            )
    return rows


def rs_type_rows(spec: str) -> list[tuple[int, str, int | None]]:
    """RS06 Type is an ID bit-field, not a complete numeric CAN identifier."""
    result = []
    for line in spec.splitlines():
        match = re.match(r"^\|\s*(F446→RS06|RS06→F446)\s*\|\s*(\d+)\s*=\s*\x60", line)
        if match:
            parts = [field.strip() for field in line.split("|")]
            dlc = parts[5]
            result.append(
                (
                    int(match.group(2)),
                    "TX" if match.group(1) == "F446→RS06" else "RX",
                    int(dlc) if dlc.isdecimal() else None,
                )
            )
    return result


def v1_rows(spec: str) -> tuple[list[tuple[int, int]], list[str]]:
    """承認表からID/DLCを抽出し、分類・優先度・方向の整合を確認する。"""
    rows, errors = [], []
    service = None
    nodes = {"F446": 1, "F303": 2, "Pi": 3, "全体": 15}
    for line in spec.splitlines():
        heading = re.search(r"Data type = (0x[0-9a-fA-F]+)", line)
        if heading:
            service = int(heading[1], 16)
        if not re.match(r"^\|\s*0x[0-9a-fA-F]{3}\s*\|", line):
            continue
        cells = [x.strip() for x in line.strip("|").split("|")]
        try:
            reg = int(cells[0], 16)
            priority = int(cells[2])
            ids = [int(x.strip(), 16) for x in cells[3].split("/")]
            dlc = int(cells[4].rsplit("/", 1)[1].strip())
            src, dst = cells[6].replace("（順）", "").split("→")
            routes = [(nodes[a], nodes[b]) for a in src.split("/") for b in dst.split("/")]
            if len(routes) != len(ids) or not 0 <= dlc <= 8:
                raise ValueError("invalid direction/DLC")
            for identifier, (a, b) in zip(ids, routes):
                rows.append((identifier, dlc))
                prefix = identifier >> 18
                if any(0x1ff <= n <= 0x208 or n == 0x700 for n in (identifier, prefix)):
                    errors.append(f"v1 DJI reserved ID: {identifier:#x}")
                expected = (priority << 26) | (service << 18) | (a << 14) | (b << 10) | reg
                if not 0 <= identifier <= 0x1fffffff or not 0 <= priority <= 3 or identifier != expected:
                    errors.append(f"v1 ID fields/direction mismatch: {identifier:#x}")
            if cells[1] == "POWER_STATUS" and cells[7] != "100 ms":
                errors.append("F303: POWER_STATUS period changed")
        except (ValueError, TypeError, KeyError, IndexError):
            errors.append(f"v1 malformed row: {line}")
    return rows, errors


def audit(sources: dict[str, str]) -> list[str]:
    """Return violations. A clean result is not real-hardware verification."""
    errors = []

    c620 = sources[C620_SPEC]
    actual_can2 = c620_rows(c620)
    if len(actual_can2) != len(CAN2) or len({row[0] for row in actual_can2}) != len(CAN2):
        errors.append("CAN2: duplicate, missing or extra numeric ID")
    for identifier, direction, dlc in actual_can2:
        if identifier > 0x7FF or identifier < 0:
            errors.append(f"CAN2: standard 11-bit ID out of range: {identifier:#x}")
        if CAN2.get(identifier) != (direction, dlc):
            errors.append(f"CAN2: 0x{identifier:x} has wrong direction/DLC or is not declared")
    if "標準11bit" not in c620:
        errors.append("C620 spec lost standard 11-bit format")
    if "500 Hz" not in c620:
        errors.append("C620 command rate is not documented")

    overview = sources[CAN_SUMMARY]
    for identifier in CAN2:
        token = f"0x{identifier:X}"
        line = next(
            (
                line
                for line in overview.splitlines()
                if line.startswith("|") and re.search(rf"^\|\s*\**\x60{token}\x60", line)
            ),
            "",
        )
        if not line or "標準11bit、DLC8" not in line:
            errors.append(f"CAN summary missing {token} or standard/DLC8")

    rs = sources[RS_SPEC]
    entries = rs_type_rows(rs)
    observed = {(typ, direction) for typ, direction, _ in entries}
    if observed != RS_TYPES or len(entries) != len(RS_TYPES):
        errors.append("RS06 Type/direction set differs from contract")
    for typ, direction, dlc in entries:
        if not 0 <= typ <= 31 or dlc != 8:
            errors.append(f"RS06 Type {typ}/{direction}: invalid 5-bit Type or DLC")
    if "29bit拡張ID" not in rs or "データ長8バイト" not in rs:
        errors.append("RS06 extended/8-byte format is missing")
    for required in ("250 Hz", "100 Hz", "±15 rad/s", "±50 rad/s"):
        if required not in rs:
            errors.append(f"RS06 source lacks rate/version difference: {required}")

    # v1表と独立したC++ golden fixtureを照合。既存モータ契約の監査は維持する。
    power_rows, power_errors = v1_rows(sources[POWER_SPEC])
    remote_rows, remote_errors = v1_rows(sources[REMOTE_SPEC])
    errors.extend(power_errors + remote_errors)
    v1 = power_rows + remote_rows
    golden = [(int(i, 16), int(n)) for i, n in re.findall(
        r"\{(0x[0-9a-fA-F]{8})u,\s*(\d+)\}", sources[V1_TEST])]
    if len(v1) != 35 or len({i for i, _ in v1}) != 35:
        errors.append("v1 duplicate, missing or extra ID")
    if len(golden) != 35 or len({i for i, _ in golden}) != 35 or sorted(v1) != sorted(golden):
        errors.append("v1 spec/golden ID/DLC mismatch")
    if "0x700" not in overview or "0x1FF–0x208" not in overview:
        errors.append("v1 DJI reservation missing in summary")

    # Contract-oriented checks for the existing C++ codecs, not a general
    # C++ parser. Runtime golden-frame tests are run separately, including UBSan.
    code = sources[CODEC] + "\n" + sources[RS_CODEC]
    patterns = {
        "C620 cmd standard ID": r"parsed\.id\s*=\s*0x200u\s*;",
        "C620 cmd DLC": r"parsed\.dlc\s*=\s*8u\s*;",
        "C620 cmd standard flag": r"parsed\.extended\s*=\s*false\s*;",
        "C620 feedback IDs": r"frame\.id\s*<\s*0x201u\s*\|\|\s*frame\.id\s*>\s*0x204u",
        "C620 current raw scale": r"constexpr float kProtocolCurrentRaw = 16384\.0f",
        "C620 current A scale": r"constexpr float kProtocolCurrentA = 20\.0f",
        "RS06 type1": r"\(1u\s*<<\s*24\)",
        "RS06 type3/4": r"enable\s*\?\s*\(3u\s*<<\s*24\)\s*:\s*\(4u\s*<<\s*24\)",
        "RS06 type2/24 RX": r"type\s*!=\s*2u\s*&&\s*type\s*!=\s*24u",
        "RS06 ext flag": r"frame\.extended\s*=\s*true\s*;",
        "RS06 DLC": r"frame\.dlc\s*=\s*8u\s*;",
        "RS06 torque range": r"constexpr float kTorqueRangeNm = 36\.0f",
        "RS06 speed range": r"constexpr float kSpeedRangeRadS = 50\.0f",
        "RX frame DLC": r"frame\.dlc\s*!=\s*8u",
    }
    for name, pattern in patterns.items():
        if re.search(pattern, code) is None:
            errors.append(f"C++ codecs: {name} differs from known contract")

    tests = sources[CODEC_TEST]
    for name, token in {
        "RS06 enable": "0x0300fd01u",
        "RS06 disable": "0x0400fd01u",
        "RS06 torque": "0x01800001u",
        "RS06 status": "0x028001fdu",
        "RS06 Type24": "0x188001fdu",
        "C620 command": "0x200u",
        "C620 feedback": "0x202u",
        "C620 packed currents": "0x03u,0x33u,0xfcu,0xcdu",
    }.items():
        if token not in tests:
            errors.append(f"C++ golden test absent: {name}")
    return errors


def main() -> int:
    errors = audit(read_sources())
    for message in errors:
        print("FAIL:", message)
    if errors:
        print(f"CAN static contract audit failed: {len(errors)} issue(s)")
        return 1
    print(
        "PASS: CAN contract audit (4 fixed CAN2 IDs; 6 RS06 direction/Type rows; 35 CAN2 v1 IDs)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
