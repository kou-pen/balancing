# 独立単軸試運転のC++統合

旧C11の単軸試運転と専用HALポートを `balancing/bench/MotorBench.hpp/.cpp` に統合する。通常Applicationとは独立し、CanPort・CanRouter・CanTxScheduler・Rs06Motor・C620Motorと既存コーデックを再利用する。旧C APIは廃止する。

## 実行契約
- 専用のCAN所有者を呼出側が用意する。通常運転との同時使用は禁止。通常起動には接続しない。実機用の独立起動経路・IRQ所有者の切替は未実装。
- 構築で送信しない。検証済み設定と停止確認、対象軸の新規受信が必要。RS06ではType4完了後のType2 Reset、Type3完了後のType2 Runningを要求する。旧Type24のみでの有効化より厳しい。
- 旧コードの上限0.5 N·m/A、200 ms、RS06有効化150 ms、4/2 msの送信周期を保持。既存デバイスの受信期限30/20 msを使用。送信期限は明示設定、既存の制御欠落6 msを適用。実機安全値の確定ではない。
- 停止要求は旧送信の終端を待ち、Type4または全チャネルゼロを優先。FAULTは自動解除しない。CAN2の他3チャネルはゼロ。停止確認は物理停止を保証しない。

## 試験
`tests/test_motor_bench.cpp` は現行CANレジスタスタブ・送信スケジューラを使用し、設定拒否、停止確認、有効化完了と新規Type2、短時間パルス、中止、受信欠落、送信期限、周期欠落、bus-off、停止再試行を検証する。旧C試運転専用試験を置換し、Cコーデック試験は維持する。

2026-11-26: CTest 49/49、通常・UBSan全件合格、CubeIDE Debug/Release各56 Appソース、0 errors / 0 warnings。Release text=69868, data=2828, bss=12216。模擬TME更新の不足を補正して中止試験も合格。実機未実施。
