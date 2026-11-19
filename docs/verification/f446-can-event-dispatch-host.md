# F446 CANイベント回収・配送の検証

[共通CANポート](f446-can1-rs06-port-host.md) のイベントをメインループで回収し、CAN1/CAN2を識別したまま配送します。契約は [BOARD-F446-001](../specs/boards/f446_control.md) と [公開API設計](../design/public-api.md) を参照してください。

<pre class="mermaid">
sequenceDiagram
    participant I as CAN IRQ
    participant P as CanPort
    participant A as 主ループ
    participant R as CanRouter
    participant S as 受け取り側
    I->>P: captureInterrupt
    loop 交互に1件、各バス最大19件
        A->>P: pollCanEvents から popEvent
        P-->>A: 採取済みイベント
        A->>R: 発生元のバスとイベントを渡す
        R->>S: 受信・TX終端・ポート状態を通知
    end
</pre>

F446の現在の起動経路では `balancing_f446_can1_service()` がCAN1だけを渡し、500 Hzのメイン処理で [既存のCAN1監視結果](f446-can1-runtime-host.md) を更新します。CAN2はホスト試験で同じ回収処理に接続します。

## 回収順と境界

- 一度の呼び出しでCAN1、CAN2の順に各1件を試みます。各ポート最大19件で、両ポートが満杯のときは最大38件です。空を観測したポートにその後届いたイベントは次回回収します。
- `CanPort` が先に返すbus-off・復旧・受信欠落をその順で配送します。異なるバス間やイベント種別間の時刻順ソートは行わず、採取時刻を保持します。
- `CanRouter` は受信、ハードウェアで確定した送信終端、ポート状態を別々のコールバックへ渡します。同じ値の送信トークンも発生元のバスが異なれば別の終端です。コールバックはポートのIRQマスク区間を抜けた後に実行します。

## 確認した範囲

| ケース | 確認項目 |
| --- | --- |
| 両バスにイベントが滞留 | CAN1の受信キューが空く前にCAN2のbus-offと送信終端を配送する。同値トークンがバス・時刻と組のまま届く |
| 配送のたびにIRQで補充 | ポートごとの上限で戻る。上限で残ったイベントは次回配送する |
| CAN1のみ・CAN2のみ・両方未接続 | 利用できるポートだけを回収する。CAN1監視は既存の受信件数とバス状態を維持する |

イベント回収件数の上限は実行時間の実測値とは別です。機器別のCAN ID振り分け、バス別の `CanTxScheduler`、F446上のCAN2起動とモーター送信は後続工程です。実機の割込み負荷と500 Hz下での最悪実行時間は未測定です。
