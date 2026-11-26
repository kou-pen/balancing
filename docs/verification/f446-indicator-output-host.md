# F446 RGB・ブザー出力境界

対象: BOARD-F446-001 / BEHAV-001、T-F446-05。基準 master 49ea4ef（#71統合後）。

| ファイル | 役割 |
|---|---|
| device/Led.hpp | 従来のGPIO通知に失敗報告可能な書込みを追加、失敗時にキャッシュを無効化 |
| device/StatusIndicatorPort.hpp/.cpp | 4チャネルを全て検証済みの場合だけ出力。変更時だけ書き込み、失敗を診断コピーへ保持 |
| device/f446_indicator_output.cpp | CubeMX生成main.hのPC3/PC2/PC1/PA15を参照。ARM用出力ラッチ照合 |
| app/application.cpp | 明示プロファイルで既存の論理表示と物理出力を接続、観測欠損・時計逆行時OFF |

## 検証対象

- tests/test_f446_status_indicator.cpp: 未検証・部分設定で書込み0、混在極性、変更なしの書込み抑制、FAULT優先、点滅・単発鳴動境界、時計逆行、GPIO書込み失敗時の全OFF試行、失敗ラッチ、再初期化。
- tests/test_cpp_app.cpp: 既定非駆動初期化に明示模擬GPIOを接続し、状態・FAULT・点滅、失敗時の表示診断、既存FAULT主因と診断状態の保持を確認。
- 最大4チャネルを一度処理し、失敗時のみ4チャネルのOFFを一度試行する。外部コールバックは非ブロッキングが前提。遅延測定は実機段階。

## 非変更・残件

- 起動時の既定設定は無効。実GPIO操作・実機書込みなし。実機のLED極性、ブザー方式・PWM周波数、電圧、表示パターンは決定していない。
- ARMのGPIOアダプタは出力ラッチを照合するだけで、出力電圧・LED発光・鳴動は保証しない。失敗チャネルへのOFFも成功するとは限らない。
- LED1〜5の用途、追加診断CANのID・ビット割当は未変更。既存CAN2診断は引き続き使用し、表示失敗数はApplicationObservationのみで参照する。

## 検証結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| CMake Debug / CTest | 44/44 PASS（既存2試験を拡張、job 5d400d3f33194a8e） |
| 通常ホスト / UBSan | 全件PASS（job 16c7703b26654d0c、終了0） |
| CAN監査 / CubeIDE設定 | 19/19、22/22 PASS |
| CubeIDE 1.19.0 Debug / Release cleanBuild | 全49 Appソース、ARM ELF32、両方0エラー/0警告（job a216c0f2431f4142、終了0） |
| 仕様 / Doxygen / 差分 | PASS（既知の日本語翻訳警告のみ） |

Release text=61788 B、data=92 B、bss=11352 B。ログ: build/cubeide-headless-indicator/{Debug,Release}.log。再現: cmake -S . -B build/indicator -DCMAKE_BUILD_TYPE=Debug、cmake --build build/indicator -j 8、ctest --test-dir build/indicator --output-on-failure、python3 tools/run_host_tests.py、python3 tools/verify_f446_build.py。実機GPIO出力は未試験。
