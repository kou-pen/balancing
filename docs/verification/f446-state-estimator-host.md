> **現在の実装:** 車輪の折返し積算は受信側C620Motorに移行済み。[後続の検証](f446-c620-continuity-host.md) を参照。以下は当時の記録。

# F446 6軸・IMU同一周期StateEstimatorのホスト検証

**対象:** CTRL-EST-001、T-CTRL-02/04/05/11、Issue #10/#11/#27。  
**ベース:** master PR #45統合 `6ba8f66`。実機CAN・機構CAD・実トルク出力は含まない。

## 何を実装したか

`StateEstimator` が1回の更新で、C11の姿勢フィルタから得た `ImuAttitude`、`MotorService::feedback(nowUs)` のRS06脚4軸・C620車輪2軸を結合し、C++17 `ControlSnapshot` または `StateEstimateError` を返す。

- `StateEstimator::update(input, elapsed)`：入力の単調時刻は `MotorFeedbackSnapshot::sampledAtUs` に固定。実測周期間隔は `std::chrono::microseconds` で記録し、常に2msだったと捏造しない。
- `LegKinematics::solve()`：旧C11 `b_leg_motor_to_joint` と `b_leg_fixture_forward` の数式を再利用。左右の原点・ギヤ比・向き・組立分岐を別々に保持し、左右の脚長・機体基準角・角速度・解析ヤコビアンを算出する。新たな機構形状は推測しない。
- `WheelOdometry::update()`：#45で実装した左右の独立時刻・ロータ折返し・RPM速度換算をそのまま使用。IMU一時欠損中も新規車輪報告を積算しておき、IMU復旧時に偽の位置ジャンプを生じさせない。
- `ControlSnapshot`：Body、Chassis、左右Leg、左右Wheel、元の観測時刻、実測周期間隔を返す。**fixtureOnly=true固定**であり、実機での駆動許可を表さない。
- 初期化・`reset()`：既存観測原点を明示的に変更するときに車輪積算を再初期化する。通常ARM/DISARMではリセットしない。姿勢フィルタのバイアス学習・センサの新規性証明は上流所有。

## ホスト試験

| 観測点 | 実施する判定 |
|---|---|
| C11姿勢フィルタからC++入力 | 静止サンプル2件で人工バイアスを学習し、実際のC11 `b_attitude_update()` の時刻・姿勢を渡す |
| CAN2と左右車輪 | C11のC620実デコーダ → MotorService → WheelOdometry。ロータ8190→2/2→8190と左右の速度・位置、二重積算しないこと |
| RS06 4軸と脚の幾何 | 4軸個別の受信時刻を保存。C11の数学fixtureを左右独立のLegKinematicsで解き、脚長/脚角とヤコビアンを返す |
| 同一周期 | IMU時刻、各RS06時刻、左右車輪の時刻、制御評価時刻、実測経過時間を混同しない |
| 不成立 | IMU未受信・NaN、RS06 1軸期限切れ、片脚の未校正/組立分岐不正、車輪20ms期限ちょうど、周期間隔0を拒否。部分的なControlSnapshotを返さない |
| 欠損からの復旧 | IMU欠損中も受信済みの車輪変位を追跡し、復旧直後に二重積算しない。明示resetで車輪原点0に戻る |

**数学fixtureのみ:** 車輪半径0.1m・ロータ/車輪比10:1、片脚の固定軸間隔0.2m、クランク0.15m、ロッド0.25m、水平座標の下向き0、IMUフィルタ係数0.9などは実機パラメータではない。入力角に用いた -π/2 rad や各軸方向+1も実機の原点・ギヤ比ではない。

## 検証結果と再現

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

- CMake/CTest：**19/19 PASS**（新規 `balancing_f446_state_estimator_tests` と既存18件）。
- 通常ホスト＋UBSan：**PASS**。`python3 tools/run_host_tests.py` で新C++17結合試験を含め全件正常終了。
- CubeIDE Debug/Release：結果が取得できなければ**NOT RUN**と記録し、設定監査だけでクロスビルドしたとは主張しない。
- Doxygen：HTML生成とフォルダ一覧に加え、Mermaidは生成HTMLの描画対象として検査。ブラウザ上のSVG化は別途確認する。

## 実機へ接続する前の境界

現時点の脚数値は **実機六節リンクCADそのものではない**。外部16:24伝達、モータ原点・正方向、組立分岐、機械可動域、取付軸とPitch加算方向、各軸の実校正値は未検証である。この数学的な成功を駆動許可にしない。実CAN接続、物理I2C新規性証明、HAL実時間周期、監視・安全停止と非ゼロ指令は別途実機検証する。

旧C11 `b_check_freshness` は受信期限ちょうどを有効、新C++17 `C620Motor::feedback` と `Rs06Motor` は期限ちょうどを失効とする既知差分は未修正。上流MotorServiceが判定したfreshを本モジュールが再計算しない。
