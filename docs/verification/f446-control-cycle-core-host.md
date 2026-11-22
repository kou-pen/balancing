# F446 ControlCycleのC++17本体化・ホスト検証

**仕様:** BOARD-F446-001、CTRL-EST-001、CTRL-LQR-001、BEHAV-001。  
**対象:** Issue #11・#27。PR #49統合後の`master`（`aa83b00`）。  
**変更目的:** 従来`tests/support/SilControlCycle.hpp`にのみ存在した周期の実行順序を`App/{Inc,Src}/balancing/app/ControlCycle.{hpp,cpp}`へ移し、テストコードと本体で一つの処理を共有する。

## 呼出しと責務

`ControlCycle::step(const ControlTickInput&)` は、実行側が同じ単調時計で固定した`StateEstimatorInput`、`SequenceInterlocks`、`OperatorEvents`、セッション付き`Rs06ArmFeedback`、`BalanceReference`を入力として受ける。

1. 初回は呼出側が実測した`firstElapsed`を用いる。ゼロや未供給なら`StateEstimator`は`InvalidPeriod`を返し、ARM条件は成立しない。TIM7回数を暗黙に2 ms換算しない。
2. 2回目以降は`MotorFeedbackSnapshot.sampledAtUs`の単調時刻差を使う。時計の逆行/重複を故障として通知し、逆行時に基準を巻き戻さない。稼働中（BALANCING）だけ6 ms以上の更新停止を`ControlTiming`として報告する。DISARMED中の保守・接続待ちで無意味なFAULTを作らない。
3. `StateEstimator::update()`の結果を`RobotSequenceInput`の`imuReady/motorsReady`へ反映し、`MotorCommand`構築時の6軸係数検証を`driveLimitsVerified`へ適用する。外部のready=trueが欠損観測を上書きできない。
4. DISARMEDでARM要求があり、推定が成立した場合にのみ`BalanceControl::initialize()`が現脚長を固定目標として取得する（MOTOR-159）。
5. すでにBALANCING中で、明示停止・既知故障がなく、今周期の推定が成立したときだけ`BalanceControl`→`MotorCommand`を数学的に実行する。失敗なら同周期`ExternalFault`。**`RobotSequence::update()`は1回だけ実行**し、確定した`StopAll`やFAULTでは古い候補を返さず制御器をresetする。
6. 返す`ControlTickResult`は観測・故障・数学プレビューであり、`MotorService`、`CanTxScheduler`、`CanPort`、HALへ指令を送る入口を持たない。

## 受入試験と結果

`tests/test_f446_sil_control_cycle.cpp`は、新しい**本体実装**`ControlCycle.cpp`を呼び、以前と同じfixture・100回の2ms周期・ARMING→BALANCING→FAULT・6軸飽和・換算失敗を検証する。追加の境界は以下。

| ケース | 判定 |
|---|---|
| 初回経過時間が未供給 | 2msを捏造せず`InvalidPeriod`、ARMせずDISARMED |
| 2回目以降の実測時刻差 | 時刻差を`ControlSnapshot.elapsed`へ渡し、初回無効から次周期にARM可 |
| DISARMEDで100ms停止 | 6ms以上でも`ControlTiming`でFAULTにしない |
| BALANCINGで6ms停止 | 同周期に`ControlTiming`をラッチ、数学値と6軸プレビューを返さない |
| 既知EmergencyStop＋数学上の変換異常 | 計算せずE-STOPだけをFAULT原因に保持、不要な`ExternalFault`を付けない |
| RS06 1軸欠落、モータ校正未検証 | ARMを拒否またはBALANCINGをFAULTに移し、古いトルクを返さない |

### 検証実行

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

- CTestは**22/22 PASS**（新規ターゲットは増やさず既存のSIL実行コードを本体へ移行）。
- 通常ホスト/UBSan：**PASS**（`python3 tools/run_host_tests.py` の終了コード0）。CubeIDE Debug/ReleaseのARMクロスビルド：**NOT RUN**。CubeIDE設定監査PASSとは区別する。
- `python3 tools/validate_specs.py`：**PASS**（仕様12件、TODO7件、Markdown70件、API参照218件）。`python3 tools/sync_cubeide_cpp.py`：**PASS**。`doxygen Doxyfile`：**PASS**（既知の日本語翻訳警告のみ）。新しいソース2件・フォルダ一覧2件・Mermaid HTML16ブロックを確認。ChromiumでのSVG描画は未検証。

## 実機で未接続・未検証の事項

`application.cpp`の`Application::controlTick()`は依然として処理済み回数を増やすだけであり、`ControlCycle`を呼び出さない。実機のTIM7 tickと単調時計の接続、IMU実I2C、CAN1/2受信/送信、実機CAD/軸符号/原点、ゲイン、動力の安全確認は**すべて未完了**。この段階の本体化は、検証済みの実機入力やDriveAllowedを意味しない。

旧C11と新C++17のCAN受信期限ちょうどの解釈差は、このPRでは変更しない。実機用の`verified=true`を数学fixtureからコピーすることは禁止する。
