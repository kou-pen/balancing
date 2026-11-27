#!/usr/bin/env python3
"""6状態・2入力の数学シミュレータ。

一定入力と人工モデルの離散LQRを検証する。実機のゲイン認定・ARM・CAN指令は行わない。
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
INPUTS = ("wheel_total_nm", "hip_total_nm")
SCENARIOS = ("open-loop", "ideal", "limited", "held", "delayed")


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


@dataclass(frozen=True)
class LqrCase:
    """平衡点周りの連続数学モデル。実機用の設定ファイルではない。"""

    name: str
    provenance: str
    dt_s: float
    a: tuple[tuple[float, ...], ...]
    b: tuple[tuple[float, ...], ...]
    q: tuple[tuple[float, ...], ...]
    r: tuple[tuple[float, ...], ...]
    x0: tuple[float, ...]
    reference: tuple[float, ...]
    equilibrium: tuple[float, ...]
    limits: tuple[float, ...]
    delay_ticks: tuple[int, int]
    ticks: int


@dataclass(frozen=True)
class LqrDesign:
    """離散化とDAREの結果。行列は既存stepと同じ順序を持つ。"""

    a: tuple[tuple[float, ...], ...]
    b: tuple[tuple[float, ...], ...]
    gain: tuple[tuple[float, ...], ...]
    poles: tuple[complex, ...]
    residual: float


def parse_lqr_case(data: object) -> LqrCase:
    """単位・出典・次元の明示された人工モデルだけを受け入れる。"""
    if not isinstance(data, dict) or data.get("mode") != "lqr":
        raise ValueError("mode=lqr が必要")
    if data.get("model_kind") != "math_fixture":
        raise ValueError("実機モデルは認定対象外。model_kind=math_fixture が必要")
    name, provenance = data.get("name"), data.get("provenance")
    if not all(isinstance(v, str) and v.strip() for v in (name, provenance)):
        raise ValueError("モデル名と出典が必要")
    if data.get("state_order") != list(STATES) or data.get("input_order") != list(INPUTS):
        raise ValueError("状態・入力の順序と単位が一致しない")
    dt, ticks = data.get("dt_s"), data.get("ticks")
    if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt):
        raise ValueError("dt_s は有限数")
    if abs(dt - 0.002) > 1e-12:
        raise ValueError("制御周期は2 ms")
    if isinstance(ticks, bool) or not isinstance(ticks, int) or not 1 <= ticks <= 100000:
        raise ValueError("ticks は1〜100000の整数")
    limits = _numbers(data.get("torque_limits"), 2, "人工トルク上限")
    equilibrium = _numbers(data.get("u_eq"), 2, "平衡入力")
    if any(limit <= 0 or abs(eq) > limit for limit, eq in zip(limits, equilibrium)):
        raise ValueError("トルク上限は正、平衡入力は上限内")
    delay = data.get("delay_ticks")
    if (
        not isinstance(delay, list)
        or len(delay) != 2
        or any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 1000 for v in delay)
    ):
        raise ValueError("delay_ticks は車輪・脚それぞれ0〜1000の整数")
    return LqrCase(
        name,
        provenance,
        float(dt),
        _matrix(data.get("A"), 6, 6, "A"),
        _matrix(data.get("B"), 6, 2, "B"),
        _matrix(data.get("Q"), 6, 6, "Q"),
        _matrix(data.get("R"), 2, 2, "R"),
        _numbers(data.get("initial_state"), 6, "初期状態"),
        _numbers(data.get("x_ref"), 6, "基準状態"),
        equilibrium,
        limits,
        tuple(delay),
        ticks,
    )


def design_lqr(case: LqrCase) -> LqrDesign:
    """ZOH離散化、DARE、ゲインの数学検査。依存はLQR利用時のみ読み込む。"""
    try:
        import numpy as np
        from scipy.linalg import expm, solve_discrete_are
    except ImportError as exc:
        raise RuntimeError("LQR用仮想環境に requirements-lqr.txt を導入してください") from exc

    a, b, q, r = (np.asarray(v, dtype=float) for v in (case.a, case.b, case.q, case.r))
    # 重みの対称性・半正定値性を明示確認。小さな負値も勝手に補正しない。
    if not np.array_equal(q, q.T) or not np.array_equal(r, r.T):
        raise ValueError("Q/R は対称行列")
    if np.min(np.linalg.eigvalsh(q)) < 0 or np.min(np.linalg.eigvalsh(r)) <= 0:
        raise ValueError("Q は半正定値、R は正定値")
    # 拡大行列の指数関数でZOH離散化。Aの逆行列が不要で積分系にも使用可能。
    augmented = np.zeros((8, 8))
    augmented[:6, :6], augmented[:6, 6:] = a, b
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        discrete = expm(augmented * case.dt_s)
        ad, bd = discrete[:6, :6], discrete[:6, 6:]
        if not np.isfinite(discrete).all():
            raise ValueError("離散化が非有限値")
        try:
            p = solve_discrete_are(ad, bd, q, r)
            gain = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
            closed = ad - bd @ gain
            poles = np.linalg.eigvals(closed)
            residual = float(
                np.linalg.norm(ad.T @ p @ closed - p + q) / max(1.0, np.linalg.norm(p))
            )
        except np.linalg.LinAlgError as exc:
            raise ValueError("安定化する離散LQRを計算できない") from exc
    if (
        not np.isfinite(gain).all()
        or not np.isfinite(poles).all()
        or not math.isfinite(residual)
        or residual > 1e-7
        or max(abs(poles)) >= 1
    ):
        raise ValueError("DARE残差または理想閉ループ安定条件が不成立")
    return LqrDesign(
        tuple(map(tuple, ad.tolist())),
        tuple(map(tuple, bd.tolist())),
        tuple(map(tuple, gain.tolist())),
        tuple(map(complex, poles)),
        residual,
    )


def lqr_trajectory(case: LqrCase, design: LqrDesign, scenario: str):
    """状態と各区間の要求/適用入力を返す。最終時刻の入力は未適用のためNone。

    A/Bは平衡点からの偏差モデル。保持→飽和ではなく、
    発行周期で制限した値を保持し、整数制御tickの遅延を経て適用する。
    """
    if scenario not in SCENARIOS:
        raise ValueError("未定義の比較条件")
    from collections import deque

    state = case.x0
    held = list(case.equilibrium)
    # 初期状態の指令履歴は平衡入力。遅延0なら同一tickに適用する。
    delay = case.delay_ticks if scenario == "delayed" else (0, 0)
    queues = [deque([eq] * n) for eq, n in zip(case.equilibrium, delay)]
    model = LinearCase(
        case.name, case.dt_s, design.a, design.b, case.x0, case.equilibrium, case.ticks
    )
    for tick in range(case.ticks):
        error = tuple(x - ref for x, ref in zip(state, case.reference))
        requested = (
            case.equilibrium
            if scenario == "open-loop"
            else tuple(
                eq - math.fsum(k * x for k, x in zip(row, error))
                for eq, row in zip(case.equilibrium, design.gain)
            )
        )
        if not all(math.isfinite(v) for v in requested):
            raise ArithmeticError("LQR要求が非有限値")
        saturated = []
        for axis, value in enumerate(requested):
            # 250 Hzの脚は偶数tickのみ発行。車輪は毎tick発行。
            if axis == 0 or scenario not in ("held", "delayed") or tick % 2 == 0:
                limited = value
                if scenario in ("limited", "held", "delayed"):
                    limited = max(-case.limits[axis], min(case.limits[axis], value))
                held[axis] = limited
            # フラグは現在の要求に対する制限要否。保持中の差分は飽和と混同しない。
            saturated.append(
                scenario in ("limited", "held", "delayed") and abs(value) > case.limits[axis]
            )
        applied = []
        for axis, queue in enumerate(queues):
            queue.append(held[axis])
            applied.append(queue.popleft())
        yield tick, state, requested, tuple(held), tuple(applied), tuple(saturated)
        delta = step(model, error, tuple(u - eq for u, eq in zip(applied, case.equilibrium)))
        state = tuple(x + ref for x, ref in zip(delta, case.reference))
        if not all(math.isfinite(v) for v in state):
            raise ArithmeticError("状態が非有限値")
    yield case.ticks, state, None, None, None, None


def write_lqr_csv(case: LqrCase, design: LqrDesign, scenario: str, output: Path):
    """入力は行の時刻から次の時刻までの区間に対応。終端の入力欄は空白。"""
    output.parent.mkdir(parents=True, exist_ok=True)
    last = case.x0
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "time_s",
                *STATES,
                *(f"requested_{x}" for x in INPUTS),
                *(f"held_{x}" for x in INPUTS),
                *(f"applied_{x}" for x in INPUTS),
                "wheel_limit_required",
                "hip_limit_required",
            )
        )
        for tick, state, requested, held, applied, saturated in lqr_trajectory(
            case, design, scenario
        ):
            values = (
                [""] * 8
                if requested is None
                else [*requested, *held, *applied, *(int(v) for v in saturated)]
            )
            writer.writerow((f"{tick * case.dt_s:.6f}", *state, *values))
            last = state
    return last


def write_lqr_report(case: LqrCase, design: LqrDesign, scenario: str, output: Path):
    """ゲインの出典と数学的な成立条件を記録。実機K_300として出力しない。"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "model_kind": "math_fixture",
                "physical_verified": False,
                "name": case.name,
                "provenance": case.provenance,
                "scenario": scenario,
                "dt_s": case.dt_s,
                "ticks": case.ticks,
                "state_order": STATES,
                "input_order": INPUTS,
                "A": case.a,
                "B": case.b,
                "Q": case.q,
                "R": case.r,
                "initial_state": case.x0,
                "x_ref": case.reference,
                "u_eq": case.equilibrium,
                "torque_limits": case.limits,
                "delay_ticks": case.delay_ticks,
                "A_d": design.a,
                "B_d": design.b,
                "K_math_fixture": design.gain,
                "ideal_closed_loop_poles": [[p.real, p.imag] for p in design.poles],
                "normalized_dare_residual": design.residual,
            },
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> int:
    """既存の一定入力モードと人工LQR比較モードを実行する。"""
    parser = argparse.ArgumentParser(description="6状態離散モデルの数学的シミュレーション")
    parser.add_argument("model", type=Path, help="A_d、B_d等を含むJSON")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/host/linear_sim.csv"),
        help="出力CSV（既定：build/host/linear_sim.csv）",
    )
    parser.add_argument("--scenario", choices=SCENARIOS, default=None)
    parser.add_argument("--report", type=Path, help="LQR数学結果JSON")
    args = parser.parse_args()
    data = json.loads(args.model.read_text(encoding="utf-8"))
    if isinstance(data, dict) and data.get("mode") == "lqr":
        case = parse_lqr_case(data)
        design = design_lqr(case)
        scenario = args.scenario or "delayed"
        report = args.report or args.output.with_suffix(".lqr.json")
        if len({args.model.resolve(), args.output.resolve(), report.resolve()}) != 3:
            parser.error("入力・CSV・JSONは別ファイルにする")
        final = write_lqr_csv(case, design, scenario, args.output)
        write_lqr_report(case, design, scenario, report)
        print(f"数学fixture LQR: {scenario}、最終状態={final}、CSV={args.output}、JSON={report}")
        return 0
    if args.scenario or args.report:
        parser.error("LQRモード専用の引数です")
    case = parse_case(data)
    final = write_csv(case, args.output)
    print(f"数学シミュレーション完了：{case.name}、{case.ticks}周期、最終状態={final}")
    print(f"CSV：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
