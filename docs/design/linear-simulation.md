# 実機非依存の離散線形シミュレーション

**状態：** 行列演算とCSV出力のホスト試験が成功。**ロボットの物理モデルや倒立安定性を再現したものではない。**（方針更新：2026-11-09）

**関連仕様：** [状態推定](../specs/control/state_estimation.md)、[LQR/VMC](../specs/control/balance_vmc.md)、[共通C11コア](../software-core.md)。

## 数学モデル

6状態・2入力の離散モデルを外部JSONで指定する。

\[
x_{k+1} = A_d x_k + B_d u_k
\]

- 状態順序：`[仮想脚傾斜 rad, 角速度 rad/s, 車輪位置 m, 速度 m/s, Pitch rad, Pitch速度 rad/s]`。
- 入力：`[左右車輪合計トルク N·m, 左右仮想股関節合計トルク N·m]`。
- 計算周期：`dt_s = 0.002`（2 ms）。
- `A_d`は6×6、`B_d`は6×2。各要素は有限数のみ有効。
- `constant_input`は全周期で一定。制御コアのLQRとの閉ループ接続は**まだ実装していない**。

### モデルファイルの扱い

サンプル`tests/fixtures/math_linear_model.json`は数学検証専用。位置について`x[k+1] = x[k] + 0.002 v[k]`を確認する**簡易テスト**で、車体質量・慣性・倒立運動・ガスシリンダ特性を再現しない。

実機用`A_d/B_d`は、CAD・質量・慣性・タイヤ・駆動・リンク・重力のモデルから別途導出する。実機値がない状態でその行列を捏造しない。

## 再現実行

```sh
cd ~/chatgpt-workspace/balancing
python3 -m unittest discover -s tests -p test_linear_sim.py -v
python3 tools/linear_sim.py tests/fixtures/math_linear_model.json --output build/host/math_linear.csv
```

出力列：`time_s,theta_rad,theta_rate_rad_s,position_m,velocity_m_s,pitch_rad,pitch_rate_rad_s`。

現行サンプルは`v=0.5 m/s`、`1000周期=2秒`なので`position=1.0 m`になる計算。これは**数学的な基準値**であり、実機の走行精度や安定性を表す測定値ではない。

## MOTOR-214〜224に基づく物理モデル作成方針

- まず固定脚長・2次元・等価剛体脚の6状態・2入力モデルを構築し、CADの質量・重心・慣性から平衡点を計算する。
- 連続時間モデルを2 msで離散化し、Pythonで離散LQRのゲインを計算する。STM32では計算済みゲイン行列のみ使用する。
- 500 Hz演算、500 Hz M3508指令、250 Hz RS06指令のサンプルホールドと個別トルク制限を模擬し、**姿勢安定を優先して**Q/Rを調整する。
- **現在ある数学テスト用シミュレータは、この物理モデル・閉ループ検証をまだ実装していない。** 新規実装は[制御TODO](../todo/control.md)T-CTRL-06/07で管理する。

## 後続の実機非依存開発

1. `A_d/B_d`の物理単位・状態の符号・基準姿勢の整合性を検証するモデルローダーを拡張。
2. 共通C11コアの`b_lqr_compute`をホストモデルへ接続し、ゲイン注入による閉ループ挙動を数値比較。
3. 飽和、遅延、観測ノイズ、操作断、周期欠落を再現可能な入力シナリオとして追加。
4. ヤコビアンとVMCの数学的検証を、CADに依存しないテスト関数から開始。

このツールは実機の安全許可や駆動指令生成には使用しない。