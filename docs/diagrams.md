# 起動・周期・通信の処理順序

対象は停止専用送信・ARM準備診断と周期送信共通化の実装。実際の呼び出し順と将来仕様を分けます。クラスの所在は [フォルダ別コード案内](design/public-api.md)。

## 1. 通常起動と試運転起動

<pre class="mermaid">
flowchart TB
    RESET["main: RCCリセット原因取得"] --> HAL["HAL_Init / クロック / 周辺初期化"]
    HAL --> MODE{"ビルド構成"}
    MODE -->|Debug / Release| APP["balancing_app_init"]
    APP --> CAN["balancing_f446_can_start"]
    CAN --> IMU["balancing_f446_imu_start"]
    MODE -->|BenchDebug| BENCH["balancing_f446_bench_start"]
    BENCH -->|既定無効| ERROR["Error_Handler"]
    BENCH -->|設定成立時| TIMER["HAL_TIM_Base_Start_IT"]
    IMU --> TIMER
    TIMER --> WDG["balancing_f446_watchdog_start"]
    WDG -->|無効または開始成功| LOOP["while 1 主ループ"]
    WDG -->|失敗| ERROR
</pre>

`Core/Src/main.c` のUSER CODEが入口です。USB Hostは明示起動APIを持ちますが既定mainからは呼びません。IWDGは既定無効。周辺初期化と実際の通信開始は別です。

## 2. 割り込みと通常制御周期

<pre class="mermaid">
sequenceDiagram
    participant T as TIM7 ISR
    participant M as main
    participant A as Application
    participant C as CanRuntime
    participant P as ControlCycle
    participant W as WatchdogPort
    T->>M: pending と sequence を更新
    M->>A: begin_control_tick(sequence)
    A->>A: CycleHealthMonitor.begin
    M->>C: balancing_f446_can_service
    C->>C: CAN1/2イベント回収と観測コピー
    M->>A: balancing_app_control_tick
    A->>A: 観測と操作・電源を固定
    A->>P: drive_runtime::step → ControlCycle.step
    P-->>A: ControlTickResult
    Note over A: #88の明示設定時は停止予約・送信処理も行う
    A->>A: 診断・表示・観測ack
    A->>W: 健全な完了証跡がある場合のみrefresh
    A->>A: ARM準備の観測診断を更新
    M->>M: imu_poll と app_poll
</pre>

タイマISRは演算しません。主ループが未処理番号を回収します。実測時刻を使用し、同じデータに新しい測定時刻を付けません。欠落・重複・未完了は周期監視で区別されます。

## 3. CAN受信から観測まで

<pre class="mermaid">
flowchart LR
    IRQ["CAN1/2 IRQ"] --> PORT["CanPort.captureInterrupt"]
    PORT --> QUEUE["時刻付きRX/TX/異常イベント"]
    QUEUE --> ROUTER["CanRouter: 主ループ配送"]
    ROUTER --> ML["MotorCanLink"]
    ML --> MS["MotorService"]
    MS --> M["RS06×4 / C620×2"]
    MS --> SNAP["MotorObservationSnapshot"]
    ROUTER --> C2["Can2ServiceLink"]
    C2 --> P["PowerBoardService"]
    C2 --> R["RemoteControlService"]
    C2 --> D["DiagnosticsService"]
</pre>

CAN2自作通信は設定が有効な場合だけ配送します。受信の分類と数値の復号、機器状態の保持は別の責務です。TX終端はSchedulerで照合した後で要求元へ配送します。

## 4. ControlCycleの数値処理

<pre class="mermaid">
flowchart TB
    IN["ControlTickInput"] --> EST["StateEstimator.update"]
    EST --> SNAP{"ControlSnapshotが成立?"}
    SNAP -->|成功| MATH["必要時 BalanceControl.initialize / compute"]
    SNAP -->|失敗| SEQ["RobotSequence.update"]
    MATH --> CMD["MotorCommand.evaluate"]
    CMD --> SEQ
    IN --> SEQ
    SEQ --> RES["ControlTickResult"]
    RES --> PREVIEW["条件成立時のみ数学プレビュー"]
    PREVIEW -. PC上のSILだけ .-> FRAME["MotorFramePlan → 模擬CAN"]
</pre>

演算はARM前の脚長初期化や、既にBALANCINGの場合など必要なときだけ行います。状態機械は一周期に一度更新します。現行通常実機設定ではARM条件は成立せず、数学プレビューを実機へ送りません。

## 5. 停止専用送信（#88）

<pre class="mermaid">
flowchart LR
    APP["DriveRuntime"] --> CHECK{"明示検証済み設定"}
    CHECK -->|未設定| OFF["送信しない"]
    CHECK -->|成立| STOP["MotorService: StopAll"]
    STOP --> LINK["MotorCanLink"]
    LINK --> S1["CAN1 Scheduler"]
    S1 --> P1["CanPort: Type4×4"]
    CHECK -->|新規正常周期| ZERO["C620Codec: 0x200 全ゼロ"]
    ZERO --> TX["MotorPeriodicTx"]
    SIL["PCのみ: DriveCycle"] -->|4ms/2msの模擬指令| TX
    TX --> S1
    TX --> S2["CAN2 Scheduler 最新世代"]
    S2 --> P2["CanPort"]
    P1 --> ACK["IRQ → Router → 終端照合"]
    P2 --> ACK
</pre>

通常とSILは同じMotorPeriodicTxを使用し、RS06の停止・有効化はMotorCanLinkが管理する。RS06は3メールボックスを使って4軸を停止します。C620ゼロは2ms周期予約枠を使い、期限切れと置換を記録します。非ゼロ指令・Type3への経路は追加していません。

## 6. 操作・電源・表示

<pre class="mermaid">
flowchart LR
    USB["USB HCD / Host"] --> PROTO["DualSenseProtocol"]
    PROTO --> HID["DualSenseService"]
    HID --> R["RemoteControlService"]
    CAN["Can2ServiceLink"] --> R
    CAN --> PWR["PowerBoardService"]
    R --> A["Application"]
    PWR --> A
    A --> S["RobotSequence"]
    S --> D["DiagnosticsService"]
    S --> LED["StatusIndicator"]
    LED --> GPIO["StatusIndicatorPort / GPIO"]
</pre>

操作量とARM/DISARMイベントは別です。電源報告はON操作の代わりではありません。USB・CAN2自作通信・GPIO出力には明示設定が必要です。

## 7. 運転状態の仕様と実装境界

<pre class="mermaid">
stateDiagram-v2
    [*] --> DISARMED
    DISARMED --> ARMING: ARMと全前提成立
    ARMING --> BALANCING: 4軸の当該セッション確認
    ARMING --> DISARMED: DISARM
    BALANCING --> DISARMED: DISARM
    DISARMED --> FAULT: 故障
    ARMING --> FAULT: 故障/期限/有効化失敗
    BALANCING --> FAULT: 故障
    FAULT --> DISARMED: 原因解消と明示解除許可
</pre>

これは [動作仕様](specs/behavior/robot_operation.md) の簡略図で、詳細な例外や条件は仕様と `RobotSequence.cpp` に従います。SILでは一連の遷移を検証済みですが、通常起動のARMING・BALANCING駆動接続は未完了です。物理非常停止は別系統です。

## 8. IWDG

<pre class="mermaid">
flowchart LR
    START["番号付き周期開始"] --> PROCESS["CAN回収・制御/停止・診断"]
    PROCESS --> DONE{"CycleHealthMonitor.complete"}
    DONE -->|有効な新規完了| PORT["WatchdogPort"]
    PORT --> WDG["F446Watchdog / HAL_IWDG_Refresh"]
    DONE -->|未完了・重複・欠落等| NO["更新しない"]
</pre>

IWDGを有効にした場合のみ更新されます。FAULT中でも停止処理を完了した健全周期は更新可。ISR・背景pollだけでは更新しません。物理リセット期限は未測定。

## 9. ARM準備診断

<pre class="mermaid">
flowchart LR
    CFG["起動固定の数学設定"] --> FORM["inspectArmConfiguration"]
    BUS["CAN起動成功時の軸割当"] --> FORM2["inspectArmMotorConfiguration"]
    FORM --> STATIC["configuration 不足ビット"]
    FORM2 --> STATIC
    OBS["最新周期の観測・推定・電源"] --> DYNAMIC["inspectArmObservations"]
    DYNAMIC --> LIVE["observation 不足ビット"]
    STATIC --> REPORT["ApplicationObservation.armReadiness"]
    LIVE --> REPORT
</pre>

ARM準備診断は状態機械へ許可を与えない。実機駆動経路が未接続であることと、実機機構安全の未検証を常に報告する。起動時の数値形式検査と毎周期の観測不成立を区別する。

## 10. 詳細資料

- [CAN送信調停](design/can-scheduler.md)
- [観測データの流れ](design/observation-snapshot.md)
- [制御数学](software-core.md)
- [単軸試運転](design/motor-layer.md)
- [検証記録](verification/index.md)

整理前の詳細図（旧C11、SILの各段階、ジャンプ等）は [整理前の全文（Git履歴）](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/diagrams.md) に保存されています。ジャンプは [後続仕様](specs/behavior/jump.md) です。
