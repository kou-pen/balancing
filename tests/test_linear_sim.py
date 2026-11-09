"""6状態線形シミュレータの数学テスト。実機モデルではない。"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from linear_sim import parse_case, step, trajectory, write_csv  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
