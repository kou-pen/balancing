# F446 CAN1ポートとRS06の接続検証

対象は bxCAN のCAN1ポートとRS06四軸サービスの接続です。基準は [PR #31](https://github.com/kou-pen/balancing/pull/31)。要求は [BOARD-F446-001](../specs/boards/f446_control.md)、[PROTO-RS06-001](../specs/protocols/rs06_can1.md)、[BEHAV-001](../specs/behavior/robot_operation.md) を参照します。

## 呼び出し順

<pre class="mermaid">
sequenceDiagram
    participant H as bxCAN1
    participant P as F446Can1Port
    participant A as 呼出側
    participant M as MotorService
    participant R as RobotSequence

    H-->>P: captureInterrupt でFIFO0・完了・エラーを採取
    A->>P: drainRs06Can1 から popEvent を呼ぶ
    A->>M: onRs06Frame / onRs06TxResult / setCan1BusOff
    A->>M: 軸の状態と異常を取得
    A->>R: update に安全入力とポート異常を渡す
    R-->>A: 周期のモータ要求とARMセッション
    A->>M: applySequence
    A->>M: submitRs06Can1 から pendingRs06Requests を取得
    A->>P: trySend
    P->>H: メールボックスを選びトークン保存後にTXRQ
    A->>M: onRs06TxResult で受付を記録
</pre>

HALによるクロック、フィルタ、CAN開始に続いて、CAN1のIRQ入口が `captureInterrupt(atUs)` を呼ぶ構成です。現在の検証ではホスト試験がその入口と制御周期を直接呼びます。F446の生成コード側に割込みをつなぐ工程が残っています。CAN1ポートと `HAL_CAN_IRQHandler` が同じフラグを消費するとイベントが失われるので、接続時は割込み入口を一つにします。

## 不変条件と試験

| 場面 | 確認した契約 |
|---|---|
| メールボックスが満杯 | 4軸目をBusyとして残し、完了イベント取出し後に投入する。ハードウェア上の空きだけで同じ枠を再利用しない |
| 完了・中止・送信エラー | RQCPが立った枠のトークンを返す。TXOKは完了、中止や調停負けは再試行可、TERRは送信失敗として上位へ通知 |
| バスオフ | 新規送信を保留し、受付済み枠には中止要求。解放は終端を取得してから。主ループが読む前に復旧してもバスオフ発生を保持 |
| 受信あふれ | FIFO0または固定キューの欠落を外部異常としてFAULTへ。送信終端3件用の枠を確保し、受信欠落時も各軸に終端を渡す |
| 受信と完了が同じ採取回 | 同一の時刻を付与し、送信完了より後の応答としては採用しない。RTRと生DLCが8を超えるフレームを軸の判定に使わない |
| ARM正常系・送信確定失敗 | 4軸それぞれのTx完了より後の正常応答でARM成立。受付済みType 3のTERRは期限を待たずFAULTと停止要求 |

レジスタの取得時刻は、実測したバス上の到着時刻とは異なります。同一採取回の順序は保守的に扱っています。モータ状態報告のセッション識別、実機でのRxとTxの割込み順、CANフィルタ、バス復旧と停止送達は実機での照合が必要です。

## 実行結果

- ooedo-vmのCMake/CTest Debug全15件、ReleaseのCAN1ポート試験、通常とUBSanの全ホスト試験が合格。
- STM32CubeIDE 1.19.0のDebug／Releaseの管理クロスビルドでARM ELF生成を確認。両構成ともエラー・警告0件。
- 実機書き込み、実CAN通信、モータ駆動は未実施。
