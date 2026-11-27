# 実機非依存の線形モデルと離散LQR

関連：CTRL-LQR-001、T-CTRL-06/07、Issue #8。[状態推定](../specs/control/state_estimation.md)、[LQR/VMC](../specs/control/balance_vmc.md)。

**数学fixtureのみ。実機CADから導出した力学モデル、K_300、実機安定性は未検証。** 既存の一定入力モードとCSVの形式は維持する。

## 数学モデルと入力形式

共通状態順：仮想脚傾斜[rad]、その速度[rad/s]、車輪位置[m]、速度[m/s]、Pitch[rad]、Pitch速度[rad/s]。入力順：左右車輪合計トルク、左右仮想股関節合計トルク[N m]。制御周期2 ms。

従来モードは離散A_d/B_dとinitial_state、constant_inputをJSONで渡す。LQRモードは `tests/fixtures/math_lqr_model.json` を例とする。

| JSONキー | 意味 |
|---|---|
| mode / model_kind | lqr / math_fixture固定 |
| name / provenance | モデル名と出典、人工モデルである根拠 |
| state_order / input_order | 単位付きの固定順序 |
| A / B | 平衡点周りの連続偏差系、6×6 / 6×2 |
| Q / R | 離散LQRの対称半正定値6×6 / 正定値2×2 |
| x_ref / u_eq / initial_state | 固定平衡状態、平衡入力、初期状態 |
| torque_limits | 2入力それぞれの人工絶対値上限 |
| delay_ticks | 2入力それぞれの整数tick遅延、1 tick=2 ms |
| dt_s / ticks | 0.002 s / 1〜100000 |

平衡点周りの偏差をZOH（零次保持）の拡大行列指数で離散化し、SciPyの離散代数リカッチ方程式からKを計算する。x_refは固定平衡状態であり、移動目標ではない。u_eq - K(x - x_ref)を要求し、状態偏差の更新では適用入力からu_eqを差し引く。対称性、重みの定値性、有限値、正規化DARE残差、理想閉ループ固有値の絶対値1未満を検査する。

## 比較条件

| scenario | 制御 | トルク制限 | 発行周期 | 遅延 |
|---|---|---|---|---|
| open-loop | 平衡入力固定 | なし | — | なし |
| ideal | LQR | なし | 両方500 Hz | なし |
| limited | LQR | あり | 両方500 Hz | なし |
| held | LQR | あり | 車輪500 Hz / 脚250 Hz | なし |
| delayed | LQR | あり | 車輪500 Hz / 脚250 Hz | 指定tick |

250 Hz脚入力は偶数tickで更新し、次の発行まで保持する。発行時に制限した値を遅延キューへ渡し、初期履歴は平衡入力とする。CSVのrequested/held/appliedは各時刻から次時刻までの区間について要求・保持・適用を区別する。limit_requiredはそのtickの要求が上限を超えるかを示し、保持中に新たな指令を発行したことを意味しない。終端行の入力欄は空白。

## 再現

Python 3.12.3、NumPy 2.2.6、SciPy 1.15.3で実施。専用venvをbuild下に置く（製品FWや標準ライブラリの一定入力モードには不要）。

```sh
python3 -m venv build/lqr-venv
build/lqr-venv/bin/python -m pip install -r requirements-lqr.txt
build/lqr-venv/bin/python -m unittest discover -s tests -p test_linear_sim.py -v
for scenario in open-loop ideal limited held delayed; do
  build/lqr-venv/bin/python tools/linear_sim.py tests/fixtures/math_lqr_model.json \
    --scenario "$scenario" --output "build/lqr-results/$scenario.csv"
done
python3 tools/linear_sim.py tests/fixtures/math_linear_model.json --output build/lqr-results/legacy.csv
```

CSVと同名の.lqr.jsonに入力行列・出典・数学用K・理想閉ループ極・残差を保存する。physical_verified=false固定。依存なし環境では旧7試験と入力2試験を実行、数値7試験は明示skip。LQR検証時は専用venvで16/16（skipなし）を確認する。

## 実測した数学結果（2026-11-27）

人工モデルは不安定脚角成分、車輪の二重積分、安定な自律Pitchを組み合わせたもの。実ロボットの相互作用を再現しない。初期脚角0.05 rad、位置0.3 m、Pitch 0.08 rad、人工トルク上限各0.4 N m、遅延車輪2 tick/脚3 tick、3000周期（6 s）。

| 比較 | 最終脚角[rad] | 最終位置[m] | 制限要否tick数 |
|---|---:|---:|---:|
| open-loop | 4068.8698 | 0.3 | 0 |
| ideal | -1.00434e-7 | -8.81708e-6 | 0 |
| limited | -1.06518e-7 | -3.58957e-5 | 129 |
| held | -1.02396e-7 | -3.58957e-5 | 129 |
| delayed | -8.35506e-8 | -2.95119e-5 | 131 |

理想閉ループ最大固有値絶対値0.9976634748188532、正規化DARE残差9.60597261125959e-16。16/16数値/入力/旧モード回帰PASS、Ruffチェック・整形PASS、CLI全5比較＋旧モード実行PASS。旧モードの位置積分は2秒後1 m。出力はVM build/lqr-results/。人工モデルの発散値は線形数学モデルの範囲内の計算値にすぎない。制限・保持・遅延下の安定性は固有値では保証せず、この初期値の軌道比較のみ。

## 実機物理モデルの残件

CADの節構成・寸法・質量・重心・慣性、車輪半径/減速比、平衡姿勢と入力、ガスシリンダ荷重、摩擦・効率、各モータ実トルク/電流限界、測定した遅延・制御周期を根拠付きで確定する。これらから連続モデルと線形化の妥当性を検証してA/Bを導出する。現行入力形式の状態順と単位は維持するが、人工モデルの行列・ゲイン・制限値を実機認定しない。

この比較は2つの合計入力の制限のみで、6軸の実トルク配分・個別クリップ、移動位置基準・FWの位置誤差クリップ、観測ノイズ、物理モデルやF446のC++制御との閉ループ接続は未実装。STM32へのゲイン転記・実機駆動を行わない。実機モデルと性能の検証は[制御TODO](../todo/control.md)で継続する。
