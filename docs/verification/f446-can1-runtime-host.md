# F446 CAN1の起動・割込みの検証

基準は [PR #33](https://github.com/kou-pen/balancing/pull/33) のCAN1ポートです。CubeMXが設定するCAN1のクロック、ビットレート、NVICを使用します。要求は [BOARD-F446-001](../specs/boards/f446_control.md) と [PROTO-RS06-001](../specs/protocols/rs06_can1.md) を参照します。

## 呼び出し順

<pre class="mermaid">
sequenceDiagram
    participant A as main
    participant C as CAN1とNVIC
    participant R as CAN1 C ABI
    participant P as F446Can1Port

    A->>C: MX_CAN1_Init
    A->>R: balancing_f446_can1_start
    R->>C: HAL_CAN_ConfigFilter でType 2とType 24をFIFO0へ
    R->>C: HAL_CAN_Start
    R->>P: ポートと時計を構築
    R->>C: HAL_CAN_ActivateNotification
    C-->>R: TX／RX0／SCEの各IRQから capture_irq
    R->>P: captureInterrupt にDWT時刻を渡す
    loop TIM7通知をmainが受け取った周期
        A->>R: balancing_f446_can1_service
        R->>P: popEvent で取得済みイベントを集計
        A->>A: balancing_app_control_tick
    end
</pre>

CAN1の3割込みでは、専用ポートがイベントを取り込んだときに生成コードの `HAL_CAN_IRQHandler` を通過せず復帰します。接続の差分はCubeMXのUSER CODE区画にあります。ポートはFIFO0の状態報告、受信欠落、バスオフを取り込みます。主ループは固定数まで取り出し、受信数と直近時刻、受信欠落数、バスオフ状態を集計します。

フィルタはRS06 Type 2／24の拡張データフレームを受けます。軸ID、主機ID、校正値を使う `MotorService` への接続は後続工程です。今回の起動経路ではモータ指令の送信を呼びません。CAN1は既存の通常CANモードで開始します。

## 時計と失敗時の動作

- DWTの32 bitサイクルカウンタをIRQと500 Hz主処理から読み、前回からの差を64 bitへ累積してµsに換算します。168 MHzで約25.6秒以内に1回以上サンプリングすると周回を追跡できます。取得時刻は実際にCAN上へ到着した時刻の代用なので、同じ取得回の順序は保守的に扱います。
- フィルタ設定やCAN開始に失敗すると起動を拒否します。通知有効化が失敗した場合は通知をマスクしてCAN停止を要求し、割込みの所有権を解放します。起動失敗は `main` の `Error_Handler` へ渡ります。

## 検証結果

| 試験 | 結果 |
|---|---|
| ホストの起動順、失敗後の通知解除、起動直後のIRQ、FIFO0集計、DWTの周回 | 通常とUBSanで合格 |
| ホストの回帰 | CTest Debug全16件、Releaseの新規CAN1起動試験が合格 |
| CubeIDE管理クロスビルド | Debug／Releaseともにエラー・警告0件。ELFでCAN1の3割込みと起動・集計シンボルを確認 |
| 基板への書き込み、実CAN通信、実機の周期・遅延 | 未実施 |

関連：[CAN1ポートとRS06の接続](f446-can1-rs06-port-host.md)。
