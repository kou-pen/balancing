"""6状態線形シミュレータの数学テスト。実機モデルではない。"""

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from linear_sim import (  # noqa: E402
    SCENARIOS,
    design_lqr,
    lqr_trajectory,
    parse_case,
    parse_lqr_case,
    step,
    trajectory,
    write_csv,
    write_lqr_csv,
    write_lqr_report,
)


def valid_case():
    matrix = [[float(i == j) for j in range(6)] for i in range(6)]
    matrix[2][3] = 0.002  # 速度一定の位置更新だけを追加した数学的例
    return {
        "name": "数学試験用：速度一定の位置積分",
        "dt_s": 0.002,
        "A_d": matrix,
        "B_d": [[0.0, 0.0] for _ in range(6)],
        "initial_state": [0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
        "constant_input": [0.0, 0.0],
        "ticks": 1000,
    }


class LinearSimTests(unittest.TestCase):
    def test_1000_ticks_const_velocity(self):
        case = parse_case(valid_case())
        states = list(trajectory(case))
        self.assertEqual(len(states), 1001)
        self.assertAlmostEqual(states[-1][1][2], 1.0, places=9)
        self.assertAlmostEqual(states[-1][1][3], 0.5, places=9)

    def test_reproducibility(self):
        case = parse_case(valid_case())
        self.assertEqual(list(trajectory(case)), list(trajectory(case)))

    def test_reject_wrong_dimensions(self):
        data = valid_case()
        data["B_d"] = [[0.0, 0.0]]
        with self.assertRaises(ValueError):
            parse_case(data)

    def test_reject_nan(self):
        data = valid_case()
        data["A_d"][0][0] = math.nan
        with self.assertRaises(ValueError):
            parse_case(data)

    def test_reject_wrong_dt(self):
        data = valid_case()
        data["dt_s"] = 0.01
        with self.assertRaises(ValueError):
            parse_case(data)

    def test_overflow_rejected(self):
        data = valid_case()
        data["A_d"][0][1] = 1.0
        case = parse_case(data)
        state = (1e308,) * 6
        with self.assertRaises(ArithmeticError):
            step(case, state, (0.0, 0.0))

    def test_csv(self):
        case = parse_case(valid_case())
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "results.csv"
            write_csv(case, out)
            rows = out.read_text().splitlines()
            self.assertEqual(len(rows), 1002)
            self.assertEqual(rows[0].split(",")[0], "time_s")
            self.assertEqual(rows[-1].split(",")[0], "2.000000")


def lqr_fixture():
    return json.loads((Path(__file__).parent / "fixtures/math_lqr_model.json").read_text())


class LqrInputTests(unittest.TestCase):
    def test_fixture_schema(self):
        case = parse_lqr_case(lqr_fixture())
        self.assertEqual(case.delay_ticks, (2, 3))
        self.assertEqual(case.dt_s, 0.002)
        self.assertEqual(len(case.a), 6)

    def test_bad_inputs(self):
        for key, value in (
            ("model_kind", "physical"),
            ("provenance", ""),
            ("state_order", ["x"] * 6),
            ("input_order", ["hip_total_nm", "wheel_total_nm"]),
            ("dt_s", 0.004),
            ("ticks", True),
            ("A", [[0.0] * 6]),
            ("B", [[0.0] * 2]),
            ("Q", [[0.0] * 6]),
            ("R", [[1.0, math.nan], [0.0, 1.0]]),
            ("initial_state", [math.inf] * 6),
            ("torque_limits", [0.0, 1.0]),
            ("u_eq", [1.0, 0.0]),
            ("delay_ticks", [True, 0]),
            ("delay_ticks", [-1, 0]),
            ("delay_ticks", [1001, 0]),
        ):
            with self.subTest(key=key, value=value):
                data = lqr_fixture()
                data[key] = value
                with self.assertRaises(ValueError):
                    parse_lqr_case(data)


@unittest.skipUnless(
    importlib.util.find_spec("numpy") and importlib.util.find_spec("scipy"),
    "LQR専用仮想環境で実行する数値試験",
)
class LqrNumericalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.case = parse_lqr_case(lqr_fixture())
        cls.design = design_lqr(cls.case)

    def test_zoh_analytic(self):
        # 車輪の二重積分系の厳密解と、角度の不安定連続系の厳密解。
        dt = self.case.dt_s
        ad, bd = self.design.a, self.design.b
        self.assertAlmostEqual(ad[2][3], dt, places=13)
        self.assertAlmostEqual(bd[2][0], dt * dt / 2, places=13)
        self.assertAlmostEqual(bd[3][0], dt, places=13)
        self.assertAlmostEqual(ad[0][0], math.cosh(2 * dt), places=13)
        self.assertAlmostEqual(ad[0][1], math.sinh(2 * dt) / 2, places=13)
        self.assertAlmostEqual(bd[1][1], math.sinh(2 * dt) / 2, places=13)

    def test_scalar_dare_analytic(self):
        # 離散A=B=1, Q=R=1のスカラー解を6次元中の一軸へ埋め込む。
        # 連続A=0, B=1/dtのZOHで離散B=1。残りは安定自律系。
        data = lqr_fixture()
        data["A"] = [[-1.0 if i == j and i != 0 else 0.0 for j in range(6)] for i in range(6)]
        data["B"] = [[500.0, 0.0]] + [[0.0, 0.0] for _ in range(5)]
        data["Q"] = [[float(i == j) for j in range(6)] for i in range(6)]
        data["R"] = [[1, 0], [0, 1]]
        d = design_lqr(parse_lqr_case(data))
        p = (1 + math.sqrt(5)) / 2
        self.assertAlmostEqual(d.gain[0][0], p / (1 + p), places=11)
        self.assertAlmostEqual(d.gain[1][0], 0.0, places=12)
        self.assertLess(d.residual, 1e-10)
        self.assertLess(max(map(abs, d.poles)), 1.0)

    def test_bad_weights_or_unstabilizable(self):
        for key, i, j, value in (
            ("Q", 0, 1, 1.0),
            ("Q", 0, 0, -1.0),
            ("R", 0, 1, 1.0),
            ("R", 1, 1, 0.0),
            ("B", 1, 1, 0.0),
        ):
            with self.subTest(key=key, i=i, j=j):
                data = lqr_fixture()
                data[key][i][j] = value
                with self.assertRaises(ValueError):
                    design_lqr(parse_lqr_case(data))

    def test_open_and_closed(self):
        trajectories = {s: list(lqr_trajectory(self.case, self.design, s)) for s in SCENARIOS}
        self.assertGreater(abs(trajectories["open-loop"][-1][1][0]), 1.0)
        for s in SCENARIOS[1:]:
            with self.subTest(scenario=s):
                rows = trajectories[s]
                self.assertEqual(len(rows), self.case.ticks + 1)
                self.assertLess(math.hypot(*rows[-1][1]), 0.001)
        self.assertEqual(
            trajectories["delayed"], list(lqr_trajectory(self.case, self.design, "delayed"))
        )
        self.assertNotEqual(trajectories["held"][-1], trajectories["delayed"][-1])

    def test_saturation_hold_and_delay(self):
        ideal = list(lqr_trajectory(self.case, self.design, "ideal"))
        limited = list(lqr_trajectory(self.case, self.design, "limited"))
        self.assertTrue(any(abs(v) > lim for v, lim in zip(ideal[0][2], self.case.limits)))
        self.assertTrue(any(limited[0][5]))
        rows = list(lqr_trajectory(self.case, self.design, "delayed"))
        for i, row in enumerate(rows[:-1]):
            _, _, request, held, applied, flags = row
            self.assertEqual(
                flags, tuple(abs(v) > lim for v, lim in zip(request, self.case.limits))
            )
            for axis, lag in enumerate(self.case.delay_ticks):
                self.assertLessEqual(abs(applied[axis]), self.case.limits[axis])
                self.assertEqual(
                    applied[axis],
                    rows[i - lag][3][axis] if i >= lag else self.case.equilibrium[axis],
                )
                if axis == 1 and i % 2:
                    self.assertEqual(held[axis], rows[i - 1][3][axis])
                else:
                    lim = self.case.limits[axis]
                    self.assertEqual(held[axis], max(-lim, min(lim, request[axis])))
        self.assertEqual(rows[-1][2:], (None, None, None, None))

    def test_nonzero_equilibrium(self):
        # 偏差モデルで基準状態なら入力平衡値を保ち、状態は変わらない。
        data = lqr_fixture()
        data["x_ref"] = data["initial_state"]
        data["u_eq"] = [0.1, -0.2]
        data["ticks"] = 5
        case = parse_lqr_case(data)
        for tick, state, request, held, applied, _ in lqr_trajectory(case, self.design, "delayed"):
            self.assertEqual(state, case.reference)
            if tick < case.ticks:
                self.assertEqual(request, case.equilibrium)
                self.assertEqual(held, case.equilibrium)
                self.assertEqual(applied, case.equilibrium)

    def test_csv_and_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "trajectory.csv"
            report = Path(tmp) / "design.json"
            final = write_lqr_csv(self.case, self.design, "held", out)
            write_lqr_report(self.case, self.design, "held", report)
            import csv

            with out.open(newline="") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), self.case.ticks + 1)
            self.assertEqual(rows[-1]["time_s"], "6.000000")
            self.assertEqual(float(rows[-1]["theta_rad"]), final[0])
            self.assertEqual(rows[-1]["applied_hip_total_nm"], "")
            metadata = json.loads(report.read_text())
            self.assertFalse(metadata["physical_verified"])
            self.assertEqual(metadata["model_kind"], "math_fixture")
            self.assertEqual(metadata["scenario"], "held")
            self.assertEqual(len(metadata["K_math_fixture"]), 2)
            self.assertNotIn("K_300", metadata)


if __name__ == "__main__":
    unittest.main()
