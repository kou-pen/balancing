# F303 BoardIO論理出力・LED表示の検証（2026-11-27）

対象：BOARD-F303-001（F303-03/06/08/09）、Issue #102。実GPIO・IRQ未接続。

| ファイル | 役割 |
|---|---|
| [BoardIO.hpp](../../firmware/f303/balancing_f303/App/Inc/balancing/power/BoardIO.hpp) | 論理出力前の安全条件・LED・公開API |
| [BoardIO.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/BoardIO.cpp) | 出力整合確認、未検証時の投入拒否、点滅位相 |
| [test_f303_board_io.cpp](../../tests/test_f303_board_io.cpp) | 全状態・両出力の組合せ、遮断条件、点滅境界試験 |

## 実装境界

Controllerの出力状態と要求の組が承認済み状態表に一致することを検査する。能動出力はピン・極性・重複導通の検証済み、ESTOP解除、ハード異常なし、未処理遮断イベントなしの全成立でのみ通す。既定値では投入不可、OFF要求は遮断条件中でも通す。時刻逆行・不整合・能動出力拒否では両出力OFF。これは論理結果であり実GPIOを書かない。LEDはOFF消灯、PRECHARGE 2 Hz、ON点灯、FAULT 5 Hz、50%デューティ。状態変更時に点灯から開始し、PRECHARGE内部の重複段階では位相を維持。拒否時はFAULT表示を選択するがController状態を変更しない。待機・専用タイマ・LED専用クラスなし。

実機接続時には出力直前のラッチ確認と両GPIO適用を不可分にし、ISR直接遮断、拒否時のControllerへの通知および古いCAN候補破棄を別途検証する。論理判定だけでIRQ競合を解消したとは扱わない。

## 開発環境と回路参照

Balancing VMのCubeIDE 1.19.0にCubeMX 6.15.0 jar、GNU Tools for STM32 13.3.1を確認。STM32Cube RepositoryにはF4 V1.28.0/V1.28.3があるが、確認範囲でF3 HAL/CMSISパッケージは未検出。CubeMXによるF303プロジェクト生成は未実施。 CubeMX同梱MCUデータベースのSTM32F303C(B-C)Tx.xml（LQFP48）でPA0/1/2=ADC1_IN1/2/3、PA11/12=CAN_RX/TX、PA13/14=SWD、PA8・PB3/4/5/6/7/9のGPIO選択を確認した。PB3/PB4をTPS異常入力に使用するためJTAG/SWOではなくSWDのみが候補。ADCのTIMトリガ・DMA・CAN設定は未生成。

OOEDO VMの参照ネットリスト `/home/dev-agent/workspace/balancing-power/pcba-orders/power-top-20261121/full.net` を再照合。PA8→R34→D4アノード、D4カソード→GNDなので参照回路上はHigh点灯。PB5/PB6はU7入力・R46/R45、PB7はU7両ゲート入力、PB3/PB4はTPS異常通知の直列抵抗先。発注Gerberとの完全一致は未確認であり、物理GPIO極性をコードで確定・有効化しない。

## 検証

BoardIO単体通常/UBSan PASS。CMake Debug全CTest 85/85 PASS（job fbe8eb03b9194208）。仕様整合・Doxygen生成とソースページ・差分検査PASS（既知の日本語翻訳警告のみ）。各能動段階の4遮断条件、状態と出力の全組合せ、LED半周期・周期境界、状態遷移、時刻逆行を確認。GNU Tools for STM32 13.3.1でF303の4実装（BoardIO/PowerCan/PowerController/PowerMonitor）と共用Codec 3実装のARMオブジェクト生成7/7 PASS。オプションは `-std=c++17 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -Wall -Wextra -Werror -pedantic -c`。初回の追加 `-ffreestanding` は既存Controllerのcmathが当該libstdc++で利用できず失敗し、通常のC++コンパイル設定で再試験した。ELF32 ARM relocatableを確認。リンク済みファームウェア・フラッシュ・実機タイミングの証明ではない。

## 未完了

CubeF3 HAL/CMSIS、CubeMX設定と起動・リンカ、実GPIO/ADC/DMA/IRQ/CAN/IWDG、実機閾値・主電源成立・遮断ラッチ競合と通電試験は残件。T-F303-01〜06は未完了維持。
