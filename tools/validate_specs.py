#!/usr/bin/env python3
"""仕様ID、文書リンク、Doxygen参照およびCAN通信表の整合性を検証する。"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPECS = ROOT / "docs" / "specs"
TODO = ROOT / "docs" / "todo"
INDEX = ROOT / "docs" / "spec-index.md"
CAN_TABLE = ROOT / "docs" / "can-id-table.md"

# 文書単位で固定した仕様ID。名前変更や重複を検出する。
EXPECTED = {
    "BOARD-F446-001",
    "BOARD-F303-001",
    "PROTO-RS06-001",
    "PROTO-C620-001",
    "PROTO-PWR-001",
    "PROTO-REMOTE-001",
    "PROTO-PS5-001",
    "PROTO-IMU-001",
    "CTRL-EST-001",
    "CTRL-LQR-001",
    "BEHAV-001",
    "BEHAV-JUMP-001",
}

ID_RE = re.compile(r"(?m)^\*\*仕様ID:\*\*\s+([A-Z][A-Z0-9-]+)\b")
LINK_RE = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")
REF_RE = re.compile(r"@spec\{([A-Z0-9-]+)\}")


def record(message: str, errors: list[str]) -> None:
    """検証エラーを蓄積する。"""
    errors.append(message)


def main() -> int:
    """仕様・リンク・CAN表を検査し、正常なら0を返す。"""
    errors: list[str] = []
    seen: dict[str, Path] = {}
    docs = sorted(SPECS.rglob("*.md"))

    for document in docs:
        matches = ID_RE.findall(document.read_text(encoding="utf-8"))
        if len(matches) != 1:
            record(f"仕様IDは1つ必要: {document}、検出={matches}", errors)
            continue
        identifier = matches[0]
        if identifier in seen:
            record(f"仕様IDの重複: {identifier}、{seen[identifier]}、{document}", errors)
        seen[identifier] = document

    if set(seen) != EXPECTED:
        record(
            "仕様IDが期待値と一致しない: "
            f"不足={sorted(EXPECTED - set(seen))} "
            f"余剰={sorted(set(seen) - EXPECTED)}",
            errors,
        )

    markdowns = sorted(ROOT.rglob("*.md"))
    for document in markdowns:
        body = document.read_text(encoding="utf-8")
        for raw in LINK_RE.findall(body):
            target = raw.split("#", 1)[0].strip()
            if not target or target.startswith(("http:", "https:", "mailto:")):
                continue
            if not (document.parent / target).exists():
                record(f"文書リンク切れ: {document.relative_to(ROOT)} → {target}", errors)

    if not INDEX.exists():
        record("仕様ID索引が存在しない", errors)
    else:
        contents = INDEX.read_text(encoding="utf-8")
        for identifier in EXPECTED:
            if identifier not in contents:
                record(f"索引に仕様IDがない: {identifier}", errors)

    references = 0
    for source in (ROOT / "firmware").rglob("*"):
        if not source.is_file() or source.suffix not in {".c", ".h", ".hpp", ".cpp"}:
            continue
        for identifier in REF_RE.findall(source.read_text(encoding="utf-8")):
            references += 1
            if identifier not in seen:
                record(f"Doxygen参照先がない: {source.relative_to(ROOT)} の {identifier}", errors)

    todo_count = len(list(TODO.glob("*.md")))
    if todo_count != 7:
        record(f"TODO文書は7件必要だが{todo_count}件", errors)

    if not CAN_TABLE.is_file():
        record("CAN ID総覧が存在しない", errors)
    else:
        body = CAN_TABLE.read_text(encoding="utf-8")
        for expected in ("0x200", "0x201", "0x202", "Type 1", "Type 24", "未確定"):
            if expected not in body:
                record(f"CAN ID総覧に必要な項目がない: {expected}", errors)

    for path in [
        ROOT / "README.md",
        ROOT / "AGENTS.md",
        ROOT / "docs" / "spec-index.md",
        ROOT / "docs" / "architecture.md",
    ]:
        if not path.is_file():
            record(f"基本文書がない: {path}", errors)

    print(
        f"仕様書={len(seen)}件、TODO={todo_count}件、"
        f"Markdown={len(markdowns)}件、API仕様参照={references}件"
    )
    for error in errors:
        print("エラー:", error)
    print("不合格" if errors else "合格：仕様ID、文書リンク、参照、CAN表")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
