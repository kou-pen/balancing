# F446 共通CANポートとRS06のホスト検証

同じ `CanPort` をCAN1/CAN2の独立したHALハンドルに接続し、各ポートが自分のメールボックス、FIFO0、イベントキューを扱うことを確認します。RS06四軸との結合はCAN1を模したホスト試験で検証します。基準は [PR #31](https://github.com/kou-pen/balancing/pull/31)。要求は [BOARD-F446-001](../specs/boards/f446_control.md)、[PROTO-RS06-001](../specs/protocols/rs06_can1.md)、[BEHAV-001](../specs/behavior/robot_operation.md) を参照します。

## RS06回帰試験の呼び出し順

<pre class="mermaid">
sequenceDiagram
    participant H as 模擬bxCAN1
    participant P as CanPort
    participant A as ホスト試験
    participant M as MotorService
    participant R as RobotSequence

    H-->>P: captureInterrupt でFIFO0・完了・エラーを採取
    A->>P: popEvent で受信・TX終端を回収
    A->>M: onRs06Frame / onRs06TxResult / setCan1BusOff
    A->>M: 軸の状態と異常を取得
    A->>R: update に安全入力とポート異常を渡す
    R-->>A: 周期のモータ要求とARMセッション
    A->>M: applySequence
    A->>M: pendingRs06Requests で候補を取得
    A->>P: フレームと照合キーを変換して trySend
    P->>H: メールボックスを選びトークン保存後にTXRQ
    A->>M: onRs06TxResult で受付を記録
</pre>

実行時は、基板側でHAL・受信フィルタ・対象バスを起動した後、対象バスの割込み入口から `captureInterrupt(atUs)` を呼びます。割込みフラグを `HAL_CAN_IRQHandler` と二重に消費するとイベントが失われます。フィルタバンクの分割はCAN1/CAN2で共有するため、基板初期化で管理します。

`trySend` は投入結果、`popEvent` はハードウェアの送信終端を返します。ポートは `TxToken` の発行元キーとシリアルを、担当バス内で照合します。図中のRS06型への変換と `MotorService` への直接接続は試験内の処理です。実行時のフレーム配送と送信調停は、今後 `Application`、`CanRouter`、バス別 `CanTxScheduler` から行います。

F446基板側のCAN1起動と割込み入口の接続は [CAN1の起動・割込みの検証](f446-can1-runtime-host.md) を参照します。

## 不変条件と試験

| 場面 | 確認した契約 |
|---|---|
| 2バスの独立性 | 同じトークンを両バスで同時に受け付ける。標準／拡張、受信、中止、送信終端、bus-off、復旧は対応するバスでのみ観測する |
| 入力・起動状態 | IDとDLCの境界を検査する。不正入力と未起動は区別し、どちらもTXRQを立てない |
| メールボックスが満杯 | 4軸目をBusyとして残し、完了イベント取出し後に投入する。ハードウェア上の空きだけで同じ枠を再利用しない |
| 完了・中止・送信エラー | RQCPが立った枠のトークンを返す。TXOK、TERR、ALST、その他未送信を区別する。RS06の再試行可否は試験側で対応付ける |
| バスオフ | 新規送信を保留し、受付済み枠には中止要求。解放は終端を取得してから。主ループが読む前に復旧してもバスオフ発生を保持 |
| 受信あふれ | FIFO0または固定キューの欠落を通知する。キュー16枠のうち3枠を送信終端用に守り、片側の欠落後も各バスの終端を全件回収する。RS06結合試験では欠落を外部異常としてFAULTへ渡す |
| 受信と完了が同じ採取回 | 同一の時刻を付与し、送信完了より後の応答としては採用しない。RTRと生DLCが8を超えるフレームを軸の判定に使わない |
| ARM正常系・送信確定失敗 | 4軸それぞれのTx完了より後の正常応答でARM成立。受付済みType 3のTERRは期限を待たずFAULTと停止要求 |

レジスタの取得時刻は、実測したバス上の到着時刻とは異なります。同一採取回の順序は保守的に扱っています。モータ状態報告のセッション識別、実機でのRxとTxの割込み順、CANフィルタ、バス復旧と停止送達は実機での照合が必要です。

## 実行結果

- ooedo-vmのCMake/CTest Debug全15件、Releaseの共通CANポート試験、通常とUBSanの全ホスト試験が合格。
- STM32CubeIDE 1.19.0のDebug／Releaseの管理クロスビルドでARM ELF生成を確認。両構成ともエラー・警告0件。
- 実機書き込み、実CAN通信、モータ駆動は未実施。
