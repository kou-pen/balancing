> **現在の配置:** この文書は当時のSIL試験記録です。周期実行器は後続の `ControlCycle` 本体へ移りました。[移行後の検証](f446-control-cycle-core-host.md) を参照してください。

# F446 C++17 BalanceControl・500 Hz SIL周期結合の検証

**対象:** CTRL-LQR-001、CTRL-EST-001、BEHAV-001、Issue #9/#10/#11/#27。  
**ベース:** PR #46 統合後のmaster（`660514e`）。  
**境界:** C++17の数値演算と `tests/support/SilControlCycle.hpp` だけを接続し、実機 `Application::controlTick()` / `MotorService::applySequence()` / `MotorCommand` / `CanPort` / HALへは接続しない。

## 機能と所有権

`BalanceControl::initialize(ControlSnapshot)` はDISARMED中に得た左右脚長の平均・差を旧C11 `b_leg_pd_capture` で取得し、Yaw PIの内部積分をリセットする（MOTOR-159）。

`BalanceControl::compute(ControlSnapshot,BalanceReference)` は旧C11の数値演算を順に適用する。まず左右脚の仮想鉛直角・角速度と車体位置・速度・PitchでLQR 6状態を作る。LQRの `u=[T_w,T_p]` はそれぞれ左右車輪合計・左右仮想股関節合計トルク。脚長/ Roll PDの支持力と、脚側の各ヤコビアンの仮想仕事 `J^T [F,T_p/2]` から4関節軸の数学トルクを作る。車輪側はLQR倒立トルクを先に確保し、左右車輪に残る容量だけをYaw PIと倒立優先配分に渡す。Yaw PIの飽和と積分凍結の診断も返す。

`BalanceReference` では前後位置基準、前後速度基準、Yaw角速度基準を**SIL呼出側が明示する**。PS5の加減速制限、移動位置基準の生成則は未合意/未検証なので、ここで新しい制御方針を追加しない。`BalanceControlOutput` に4関節・2車輪の機構側トルクを示すが、`fixtureOnly=true` 固定。車輪電流・モータ軸トルク・ガスシリンダ補償・駆動許可は行わない。

`tests/support/SilControlCycle.hpp` は**ホスト試験のみに存在する実行器**。入力を2ms周期で固定し、`StateEstimator` → `RobotSequence` → 数学的な `BalanceControl` をつなぐ。ARM前の脚長取得、同周期の観測欠損・失敗理由、ARMセッション、FAULTラッチと非再ARMを模擬する。制御計算に失敗した場合は、送信機能のないSILでも、`RobotSequence` へ同周期の `ExternalFault` を通知する。出力の数値はCANに変換できず、実機に送らない。

## 数値fixtureと受入試験

| ケース | 検証対象 |
|---|---|
| 6状態LQR | Pitch 0.1radと人工ゲインに対する車輪合計・仮想股関節合計トルクの符号・計算値 |
| ARM時脚長参照 | 現在の左右脚長を固定目標として取得。初期化前・reset後に計算を拒否する |
| 脚側PD / VMC | 旧C11脚長/Roll PDを使い、各脚のヤコビアンから4関節軸の数学トルクを求める |
| 車輪倒立優先 | LQR共通トルクを先に確保。Yaw大入力でも各車輪の人工上限5Nmを超えず、飽和・積分凍結を明示 |
| 不正な設定 | 未検証LQRゲインなら数学トルクを返さない。前周期結果を再利用しない |
| ARM / BALANCING / FAULT | 模擬した4軸のセッション一致と正常応答でARMINGからBALANCINGへ進み、故障で停止・FAULTに移行 |
| 500 Hzの仮想周期 | 2msごとの100周期で順序と数学結果を確認。1周期内の観測と実測差分時刻を保持 |
| 3周期欠落 | 6ms以上の更新空白でControlTimingを記録し、同じ周期の計算候補を返さない |
| 鮮度・時計 | RS06の1軸欠落を受信済みと誤認しない。時計逆行でClockRegression、FAULT後は自動再ARMしない |

**fixtureの例:** 数学上の人工LQRゲイン、Pitch 0.1 rad、人工車輪限界5 Nm、半径0.1 m・比10:1、各脚0.2m/0.15m/0.25mの幾何、ARMの仮想期限50 ms～2 s。いずれも本番設定ではなく、`verified=true` はホスト入力中の数学的検証用の値であり、実機ゲイン・CAD・センサ・通信を検証した意味ではない。人工のARM確認はハードウェアのTx完了や実モータ応答ではない。

## 実行結果

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

- CMake/CTest: **20/20 PASS**（SIL数学演算と500 Hzの擬似周期結合が新規1件）。
- 通常ホスト・UBSan: **PASS**（追加ヘルパーを含め最終回帰の再実行で確認）。
- Doxygen・仕様リンク: **確認結果はPR本文に記録**。
- 実STM32CubeIDE Debug/Releaseクロスビルド: **NOT RUN**（接続先実行ゲートでCubeIDE本体が直接起動不可。設定監査はクロスビルドではない）。
- 実機CAN、電流、モータ出力、制御周期の実測、CAD、実LQR `K_300`、原点・方向・ギヤ比: **NOT VERIFIED**。

## 既知の残件

`Application::controlTick()` は依然として診断カウンタだけを増やす。実タイマISRからの通知、実CAN1/2・IMU入力、`RobotSequence`、`StateEstimator`、`BalanceControl` を同一主ループへ結ぶ作業は未着手であり、物理安全ゲートと実測パラメータのレビューが先に必要。本PRで `BALANCING` の実機駆動を可能にしたと解釈してはいけない。

旧C11とC++17の20/30ms期限境界の不一致は本変更で直さない。SILで利用する最新の `MotorFeedbackSnapshot.fresh` は上流の判定結果を再利用する。正常な飽和だけでFAULTへ移さず、重大な観測欠落・更新周期違反や未成立の制御計算は個別の事象として区別する。
