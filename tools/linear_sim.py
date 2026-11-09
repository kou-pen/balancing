#!/usr/bin/env python3
"""実機パラメータを持たない、6状態・2入力の離散線形モデル検査器。

状態更新 x[k+1] = A_d x[k] + B_d u[k] のみを担当する。
LQRゲインの生成、ARM、CAN指令、実機の安定性判定は行わない。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

STATES = (
    "theta_rad",
    "theta_rate_rad_s",
    "position_m",
    "velocity_m_s",
    "pitch_rad",
    "pitch_rate_rad_s",
)
N_STATES = 6
N_INPUTS = 2


@dataclass(frozen=True)
class LinearCase:
    """2 ms離散化済み行列を用いる数学テストケース。"""

    name: str
    dt_s: float
    a: tuple[tuple[float, ...], ...]
    b: tuple[tuple[float, ...], ...]
    x0: tuple[float, ...]
    u: tuple[float, ...]
    ticks: int


def _numbers(value: object, expected: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != expected:
        raise ValueError(f"{label}: 要素数は{expected}が必要")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in value):
        raise ValueError(f"{label}: 数値以外を含む")
    numbers = tuple(float(x) for x in value)
    if not all(math.isfinite(x) for x in numbers):
        raise ValueError(f"{label}: 有限値ではない")
    return numbers


def _matrix(value: object, rows: int, cols: int, label: str) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or len(value) != rows:
        raise ValueError(f"{label}: 行数は{rows}が必要")
    return tuple(_numbers(row, cols, f"{label}の行{i}") for i, row in enumerate(value))


def parse_case(data: object) -> LinearCase:
    """JSONモデルの寸法・範囲を検証する。"""
    if not isinstance(data, dict):
        raise ValueError("モデルのルートはJSONオブジェクト")
    name = data.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("モデル名が必要")
    dt = data.get("dt_s")
    if isinstance(dt, bool) or not isinstance(dt, (float, int)) or not math.isfinite(dt):
        raise ValueError("dt_sは有限の正数")
    if abs(float(dt) - 0.002) > 1e-12:
        raise ValueError("現在の仕様は2 ms離散モデルのみ")
    ticks = data.get("ticks")
    if isinstance(ticks, bool) or not isinstance(ticks, int) or not 1 <= ticks <= 100000:
        raise ValueError("ticksは1〜100000の整数")
    return LinearCase(
        name,
        float(dt),
        _matrix(data.get("A_d"), N_STATES, N_STATES, "A_d"),
        _matrix(data.get("B_d"), N_STATES, N_INPUTS, "B_d"),
        _numbers(data.get("initial_state"), N_STATES, "初期状態"),
        _numbers(data.get("constant_input"), N_INPUTS, "一定入力"),
        ticks,
    )


def step(
    case: LinearCase, state: tuple[float, ...], input_torque: tuple[float, ...]
) -> tuple[float, ...]:
    """次の1制御周期を数学的に計算する。物理的な安定性は判定しない。"""
    try:
        result = tuple(
            math.fsum(case.a[i][j] * state[j] for j in range(N_STATES))
            + math.fsum(case.b[i][k] * input_torque[k] for k in range(N_INPUTS))
            for i in range(N_STATES)
        )
    except OverflowError as exc:
        raise ArithmeticError("状態演算でオーバーフローが発生") from exc
    if not all(math.isfinite(x) for x in result):
        raise ArithmeticError("状態が非有限値になった")
    return result


def trajectory(case: LinearCase) -> Iterator[tuple[int, tuple[float, ...]]]:
    """t=0を含む決定論的な状態列を返す。"""
    state = case.x0
    yield 0, state
    for tick in range(1, case.ticks + 1):
        state = step(case, state, case.u)
        yield tick, state


def write_csv(case: LinearCase, output: Path) -> tuple[float, ...]:
    """CSVへ出力して、最終状態を返す。"""
    output.parent.mkdir(parents=True, exist_ok=True)
    last = case.x0
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *STATES))
        for tick, state in trajectory(case):
            writer.writerow((f"{tick * case.dt_s:.6f}", *state))
            last = state
    return last


def main() -> int:
    """外部JSONモデルで実行し、固定入力の線形応答を出力する。"""
    parser = argparse.ArgumentParser(description="6状態離散モデルの数学的シミュレーション")
    parser.add_argument("model", type=Path, help="A_d、B_d等を含むJSON")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/host/linear_sim.csv"),
        help="出力CSV（既定：build/host/linear_sim.csv）",
    )
    args = parser.parse_args()
    case = parse_case(json.loads(args.model.read_text(encoding="utf-8")))
    final = write_csv(case, args.output)
    print(f"数学シミュレーション完了：{case.name}、{case.ticks}周期、最終状態={final}")
    print(f"CSV：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
