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

## 機器別の受信振り分け

| 発生元 | CAN ID | DLC | 配送先 |
| --- | --- | --- | --- |
| CAN1 | 拡張29 bit、上位5 bitがType 2または24 | 8 | RS06状態報告 |
| CAN2 | 標準 `0x201` | 8 | C620左車輪フィードバック |
| CAN2 | 標準 `0x202` | 8 | C620右車輪フィードバック |
| その他 | バス・ID形式・ID・DLCのいずれかが不一致 | 任意 | 未分類 |

詳細は [RS06](../specs/protocols/rs06_can1.md) と [C620](../specs/protocols/c620_can2.md) の仕様を参照してください。RS06の主機ID・軸IDとペイロードの検証はCodec／機器サービスで行います。未確定のF303・PiフレームとCAN2の電流指令 `0x200` は未分類で、診断側は元フレームと採取時刻を引き続き受け取れます。既存のCAN1監視カウンタは全受信イベントの集計です。

## バスごとの送信終端の照合

`CanTxScheduler::trySubmit(frame, token, nowUs)` は1バスの `CanPort` に投入し、`Accepted` のときにトークンと受付時刻を記録します。最大3件まで記録し、`Busy` の要求は呼出側で扱います。終端していない番号の再投入は `Busy` です。終端した番号の再利用を避ける責任は送信要求元にあります。

<pre class="mermaid">
sequenceDiagram
    participant S as 送信要求元
    participant T as CanTxScheduler
    participant P as CanPort
    participant R as CanRouter
    S->>T: trySubmit
    T->>P: trySend
    P-->>T: Accepted
    T-->>S: 受付結果
    Note over P,R: IRQで終端を記録し、主ループで回収
    P-->>R: 発生元バスと終端イベント
    R->>T: 発生元バスだけ照合
    T-->>R: 受付済みの枠を解放
    R-->>S: onCanTransmit
</pre>

完了の時刻にはIRQ採取時刻を使います。受付時刻と同時刻は有効で、受付より前の時刻、未受付のトークン、配送済みの重複を棄却します。送信成功、仲裁負け、送信エラー、中止を含む未送信のいずれも終端として配送し、上位側が結果に応じて処理します。受信イベントから送信完了を補いません。

`CanRouter` の照合先はポートごとに設定し、従来のポート単体試験のように照合先が未設定の場合はポートの終端を直接配送します。F446の起動経路ではCAN1のスケジューラを接続しました。CAN1監視には送信要求元がなく、CAN2との組み合わせはホストで確認しました。

## 確認した範囲

| ケース | 確認項目 |
| --- | --- |
| 両バスにイベントが滞留 | CAN1の受信キューが空く前にCAN2のbus-offと送信終端を配送する。同値トークンがバス・時刻と組のまま届く |
| 配送のたびにIRQで補充 | ポートごとの上限で戻る。上限で残ったイベントは次回配送する |
| 受信先の判定 | 両ポートの受信キューからRS06とC620左右を配送する。バス違い、標準・拡張違い、短いDLC、範囲外ID、電流指令と未確定IDは未分類になる |
| CAN1/CAN2で同じトークンを各3件受付 | CAN2のエラー終端はCAN2だけを解放。CAN1が4件目を投入できるのはCAN1の終端を回収した後 |
| 受付状態と終端の照合 | `Busy`・`Invalid`・`NotReady` 後、発行元不一致、時刻逆行、重複を棄却。同時刻の正当な終端と、全4種の終端結果を配送 |
| CAN1のみ・CAN2のみ・両方未接続 | 利用できるポートだけを回収する。CAN1監視は既存の受信件数とバス状態を維持する |

イベント回収件数の上限は実行時間の実測値とは別です。送信候補の優先順位・最新値への置換・期限管理、機器サービスへの終端配送、F446上のCAN2起動とモーター送信は後続工程です。実機の割込み負荷と500 Hz下での最悪実行時間は未測定です。
