#!/usr/bin/env python3
"""Host-only CAN contract checks against canonical protocol specs and C11 codec."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAN_SUMMARY = "docs/can-id-table.md"
C620_SPEC = "docs/specs/protocols/c620_can2.md"
RS_SPEC = "docs/specs/protocols/rs06_can1.md"
POWER_SPEC = "docs/specs/protocols/power_can2.md"
REMOTE_SPEC = "docs/specs/protocols/remote_can2.md"
CODEC = "firmware/f446/balancing_f446/App/Src/balancing/communication/balancing_motor_can.c"
CODEC_TEST = "tests/test_motor_can.c"

# In this robot: 0x1FF is unused, 0x200 is a command and 0x201/202
# are feedback. This is a regression boundary, NOT a firmware config.
CAN2 = {
    0x1FF: ("F446→C620", 8),
    0x200: ("F446→C620", 8),
    0x201: ("C620→F446", 8),
    0x202: ("C620→F446", 8),
}
RS_TYPES = {(1, "TX"), (2, "RX"), (3, "TX"), (4, "TX"), (24, "TX"), (24, "RX")}
POWER_MESSAGES = {"POWER_STATUS", "POWER_ON", "POWER_OFF", "EMERGENCY_STOP"}
REMOTE_MESSAGES = {
    "運動指令",
    "ARM要求",
    "DISARM要求",
    "CLEAR_FAULT",
    "EMERGENCY_STOP",
    "設定変更",
    "状態通知",
}


def read_sources(root: Path = ROOT) -> dict[str, str]:
    """Return the small set of authoritative sources and code being audited."""
    return {
        name: (root / name).read_text(encoding="utf-8")
        for name in (CAN_SUMMARY, C620_SPEC, RS_SPEC, POWER_SPEC, REMOTE_SPEC, CODEC, CODEC_TEST)
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


def unknown_rows(spec: str, names: set[str], fields: tuple[int, ...]) -> bool:
    """Reject guessed IDs, formats and DLC in contracts that remain unresolved."""
    found = set()
    for line in spec.splitlines():
        if not line.startswith("|"):
            continue
        parts = [cell.strip() for cell in line.split("|")]
        if len(parts) <= max(fields):
            continue
        name = parts[1].strip("*\x60 ")
        if name in names:
            if any("未確定" not in parts[i] for i in fields):
                return False
            found.add(name)
    return found == names


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

    power = sources[POWER_SPEC].split("## データ内容")[0]
    if not unknown_rows(power, POWER_MESSAGES, (3, 4, 5)) or "100 ms" not in power:
        errors.append("F303: unresolved format/ID/DLC or status period changed")
    if not unknown_rows(sources[REMOTE_SPEC], REMOTE_MESSAGES, (3, 4)):
        errors.append("Pi remote CAN: unresolved ID/DLC changed")

    # Contract-oriented checks for the existing C11 codec, not a general
    # C parser. Runtime golden-frame tests are run separately, including UBSan.
    code = sources[CODEC]
    patterns = {
        "C620 cmd standard ID": r"parsed\.id\s*=\s*0x200u\s*;",
        "C620 cmd DLC": r"parsed\.dlc\s*=\s*8u\s*;",
        "C620 cmd standard flag": r"parsed\.extended\s*=\s*false\s*;",
        "C620 feedback IDs": r"frame->id\s*<\s*0x201u\s*\|\|\s*frame->id\s*>\s*0x204u",
        "C620 current raw scale": r"#define C620_PROTOCOL_CURRENT_RAW 16384\.0f",
        "C620 current A scale": r"#define C620_PROTOCOL_CURRENT_A 20\.0f",
        "RS06 type1": r"\(1u\s*<<\s*24\)",
        "RS06 type3/4": r"enable\s*\?\s*\(3u\s*<<\s*24\)\s*:\s*\(4u\s*<<\s*24\)",
        "RS06 type2/24 RX": r"type\s*!=\s*2u\s*&&\s*type\s*!=\s*24u",
        "RS06 ext flag": r"out->extended\s*=\s*true\s*;",
        "RS06 DLC": r"out->dlc\s*=\s*8u\s*;",
        "RS06 torque range": r"#define RS_TORQUE_RANGE_NM 36\.0f",
        "RS06 speed range": r"#define RS_SPEED_RANGE_RAD_S 50\.0f",
        "RX frame DLC": r"frame->dlc\s*!=\s*8u",
    }
    for name, pattern in patterns.items():
        if re.search(pattern, code) is None:
            errors.append(f"C11 codec: {name} differs from known contract")

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
            errors.append(f"C11 golden test absent: {name}")
    return errors


def main() -> int:
    errors = audit(read_sources())
    for message in errors:
        print("FAIL:", message)
    if errors:
        print(f"CAN static contract audit failed: {len(errors)} issue(s)")
        return 1
    print(
        "PASS: CAN contract audit (4 fixed CAN2 IDs; 6 RS06 direction/Type rows; F303/Pi unresolved)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
