# F303・F446独立周期の電源連携SIL（2026-11-27）

対象：BOARD-F303-001、BOARD-F446-001、BEHAV-001、PROTO-PWR-001。Issue #102。CubeMX生成・実機は使用しない。

## 試験の構成

[試験コード](../../tests/test_f303_f446_power_sil.cpp) はF303の実Application（全5モジュール）を1 ms、F446の実PowerBoardServiceとRobotSequenceを2 msの仮想時計で実行する。POWER_STATUSは実Codecで符号化されたフレームを受信サービスに渡す。ON受理応答と実ON状態を区別する。F303の電源出力適用、ADC、主電源成立、F446のモータ・IMU等の保護条件およびRS06応答は人工値。実CAN送信器、F446の全Application・制御演算・モータ送信はこの試験の対象外。

<pre class="mermaid">
flowchart LR
    REQ["F446/Pi要求（試験入力）"] --> F303["F303 Application 1 kHz"]
    MOCK["模擬ADC・出力適用"] <--> F303
    F303 -->|"POWER_STATUS"| P["F446 PowerBoardService"]
    P -->|"powerReady・FAULT"| R["F446 RobotSequence 500 Hz"]
    R --> O["StopAll / EnableRs06 / DriveAllowed"]
</pre>

## 7シナリオ

| シナリオ | 確認条件 |
|---|---|
| 持続過電流 | F303の両出力OFF、F446 ExternalFault、異常継続中の解除拒否、復旧順序 |
| Pi POWER_OFF | F303 OFF保持、F446 InterlockLost、明示ONから復旧 |
| 物理非常停止 | F303 OFF・ESTOP通知、F446 EmergencyStop、電源が自動復帰しても運転はFAULT保持 |
| POWER_STATUS途絶 | 100 ms周期報告の確認後に欠落を注入、500 ms等号で鮮度失効、F303独立ON維持、F446停止要求 |
| 出力適用拒否 | 拒否周期のON候補なし、次周期のFAULT報告でF446停止、明示復旧 |
| F446起点CAN ESTOP | F303 FAULT保持、反射送信なし、明示ONから復旧 |
| ARMING中のPOWER_OFF | 4軸確認が揃う周期でも停止優先、F446 FAULT |

各通常復旧ではPOWER_ONのAccepted応答時点をPRECHARGEとして確認し、ON報告まで待つ。ON報告だけではF446のFAULTは解除されない。CLEAR_FAULTとARMを同時入力した場合もDISARMED止まりであり、別の明示ARMが必要。物理停止解除と通信途絶回復は電源の既存挙動を維持し、F446は明示CLEAR_FAULT・ARMまで停止する。

## 検証

新規SILの通常・UBSanはPASS。CMake Debug全CTest 87/87 PASS（job 2959b2b3432946e2）。仕様整合・Doxygen生成・差分検査PASS（既知の日本語翻訳警告のみ）。#107のARMリンク試験も統合後に再実行し28ソースのコンパイル・既存試験ELFのリンク・ベクタ検査PASS。application.cppはコンパイル対象だが試験mainから未呼出しであり、applicationのリンク保持・実行を証明しない。製品ソース、CAN契約、実機用設定の変更なし。

## 限界と残件

物理RC充電モデル、負荷モデル、CAN帯域・伝搬・IRQ競合、実F446全Applicationとモータ送信、Piの実装、実機通電はこの試験では検証しない。モータが物理的に止まったという証拠ではなく、実RobotSequenceがStopAllを要求することの確認。F303の主電源導通確認と閾値は引き続き未確定。CubeMX生成・HAL接続は保留。次はPi側の通信・操作実装と3機器結合試験。既存実機TODOは閉じない。
