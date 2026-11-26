# 旧C11送信モデルのC++実装への統合

旧 `balancing_tx_scheduler.c/.h` はフレーム・HAL・本番の状態機械を所有しないIssue #5の初期モデルだった。製品ソースに参照元はなく、同じ責務をCanTxScheduler、CanRouter、MotorService、ControlCycleとホストSILが実装済み。二つ目の送信スケジューラを維持せず、旧Cの動作確認を実際のC++経路の試験へ移す。旧コードはGit履歴（基準0cc9e7b）で参照可能。

| 旧C試験の観点 | 現行の検証 |
|---|---|
| 250/500Hz、3メールボックス、旧待ち置換、4番目の軸、期限 | test_f446_can_event_poll: testFullCan1ReplacesTheFourthPeriodicFrame / testFourthPeriodicFrameCanUseTheFirstReleasedMailbox / testPeriodicDeadlineAndAtomicValidation / testIntegratedObservationAndFrameScheduling |
| 受付と完了の区別、旧/不明トークン、バス別終端 | testSchedulerIgnoresUnacceptedAndStaleEvents / testTerminalOnlyReleasesTheOriginatingBus / testNonSendTerminalEventsReleaseTheEntry |
| CAN2の背圧、世代・バス独立、停止優先 | testPeriodicGenerationIsIndependentOnCan2 / testFourStopsHavePriorityWhileTheNormalFifoIsFull / testAbortPeriodicDoesNotDelayOtherAxisStop |
| RX/RS06 TX/C620 TXの独立した異常 | --drive-sil-rx-loss / --drive-sil-rs-timeout / --drive-sil-timeout / --drive-sil-tx-error |
| 旧世代の遅延完了は鮮度を回復しない | --drive-sil-stale |
| 6ms欠落・時計異常・重複 | --drive-sil-gap、ControlCycle/RobotSequenceの時刻試験、--watchdog、周期世代の重複拒否 |
| 不正設定・FAULT保持・DISARM・停止 | --drive-sil-invalid / --drive-sil-invalid-model / --drive-sil-estop / --drive-sil、testStopWaitSurvivesNotReadyAndBusOff |
| TX期限を注入可能 | --drive-sil-short-deadline（4ms） / --drive-sil-long-deadline（10ms）を追加 |

旧モデルの全APIを互換実装するのではなく、現行の本来の責務へ統合する。旧モデルの同時刻DISARM等の内部状態遷移は互換を保証しない。C++の状態機械・安全判定を変更しない。新規試験は人工設定のみ。通常運転の実機出力は引き続き未接続。

## 検証結果

2026-11-26: CTest 50/50、通常・UBSan 全件合格、CubeIDE Debug/Release 各56 Appソース、0 errors / 0 warnings。Release text=69868, data=2828, bss=12216。実機未実施。
