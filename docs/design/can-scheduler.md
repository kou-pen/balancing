# CAN送信調停（現行C++17）

CAN1とCAN2はそれぞれCanPortとCanTxSchedulerを所有します。CAN IDの決定は [通信仕様](../can-id-table.md)、送信調停は下記のファイルです。

| 役割 | ファイル |
|---|---|
| HAL/bxCAN、3メールボックス・IRQイベント | communication/can/CanPort.hpp/.cpp |
| 停止・周期・通常イベント・背景枠・トークン照合 | communication/can/CanTxScheduler.hpp/.cpp |
| 受信分類・送信終端・バス異常の配送 | communication/can/CanRouter.hpp/.cpp |
| 4軸の停止要求・受付・終端 | app/MotorCanLink.hpp/.cpp、app/MotorService.hpp/.cpp |
| #88停止専用のC620全ゼロ | app/DriveRuntime.hpp/.cpp |
| 模擬通常駆動の4ms/2ms世代 | tests/sil/DriveCycle.cpp |

Schedulerは停止を優先し、周期要求を最新世代で置換します。期限切れの旧周期を後から積まないこと、受付済み要求は終端まで照合することが重要です。通常イベントはFIFO、診断の背景送信は枠を制限します。

<pre class="mermaid">
flowchart LR
    REQ["停止 / 周期 / 通常イベント / 背景"] --> S["CanTxScheduler"]
    S --> PORT["CanPort"]
    PORT --> HW["3メールボックス"]
    HW --> IRQ["TX IRQ"]
    IRQ --> ROUTER["CanRouter"]
    ROUTER --> MATCH["Scheduler.consumeTerminal"]
    MATCH --> OWNER["要求元"]
</pre>

CANでの受付、実送信の終端、モータの応答、物理停止は同一ではありません。バスオフ・失敗・欠落を区別します。現在の通常停止専用経路は明示設定が必要で、通常駆動指令はSILのみです。

[送信調停のC++統合](../verification/f446-c-scheduler-migration.md) ／ [2バス配送検証](../verification/f446-can-event-dispatch-host.md) ／ [通常停止検証](../verification/f446-stop-output.md)。旧C11モデルの詳細は [整理前の全文（Git履歴）](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/design/can-scheduler.md)。
