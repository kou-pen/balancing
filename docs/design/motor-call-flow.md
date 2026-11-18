# モータを回す処理の呼び出し順

F446の起動から、単軸試運転、通常運転の部品、C++17統合まで、入口からCAN送信へ至る処理を示します。

| 調べたいこと | 最初に見る関数 | 確認した範囲 |
|---|---|---|
| F446を起動するとき | §1の `main` | masterの起動経路。CANは初期化まで |
| 関節RS06・車輪C620を1台だけ試すとき | §2・§3の `b_f446_motor_port_pulse_rs06` / `b_f446_motor_port_pulse_c620` | 単軸ベンチとHALスタブ |
| 入力・推定・トルク計算・送信周期を追うとき | §4の `b_observation_freeze` / `b_mode_step` / `b_sched_tick` | C11ホスト試験 |
| ARM要求からRS06 4軸を有効化・停止するとき | §5の `Fixture::cycle` | PR #31の模擬CAN |
| 通常運転で6軸に出力するとき | §6の `Application::controlTick` | C++17の接続設計 |

確認したコードは master の **818efcc** と [PR #31](https://github.com/kou-pen/balancing/pull/31) の **bbcf6f9** です。CAN送信の受付、Tx完了、モータからの状態報告は別々のイベントです。

## 1. F446実行経路 · 起動と500 Hzの入口

<pre class="mermaid">
sequenceDiagram
    participant C as main.c
    participant H as HAL
    participant T as TIM7割込み
    participant A as Application

    C->>H: HAL_Init
    C->>C: SystemClock_Config → MX_*_Init
    C->>A: balancing_app_init → Application::init
    C->>H: HAL_TIM_Base_Start_IT
    loop メインループ
        T-->>C: HAL_TIM_PeriodElapsedCallback で実行フラグを立てる
        opt フラグあり
            C->>A: balancing_app_control_tick → Application::controlTick
            A->>A: 診断カウンタを進める
        end
        C->>A: balancing_app_poll → Application::poll
    end
</pre>

割込みから戻った後にメインループがフラグを取り出します。現在の `controlTick` は呼び出し回数を更新し、`poll` は統合用の入口です。`MX_CAN1_Init` と `MX_CAN2_Init` は設定まで行い、通常駆動のCAN開始や `RobotSequence` の呼び出しは未接続です。
[main.c](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/Core/Src/main.c#L88) · [application.cpp](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/app/application.cpp#L18)

## 2. 単軸RS06 · HALスタブで通した関節トルク経路

試験側の `b_f446_motor_port_init` から `b_bench_init` を呼び、設定と状態を初期化します。図の「試験側」はHALスタブ試験でこれらの関数を直接呼びます。

<pre class="mermaid">
sequenceDiagram
    participant C as 試験側
    participant P as F446試運転ポート
    participant B as BMotorBench
    participant E as codec
    participant H as CAN1 HAL

    C->>P: b_f446_motor_port_pulse_rs06
    P->>B: b_bench_request_rs06
    B->>B: B_BENCH_WAIT_RS06 に入る
    C->>P: b_f446_motor_port_tick_2ms
    Note over P: CAN1は2回に1回処理
    P->>B: b_bench_rs06_frame
    B->>E: b_rs06_control_frame でType 3
    P->>H: tx_frame → HAL_CAN_AddTxMessage
    C->>P: b_f446_motor_port_on_tx_done
    P->>B: b_bench_rs06_tx_done でTx完了を記録
    C->>P: b_f446_motor_port_on_rx0
    P->>H: HAL_CAN_GetRxMessage
    P->>E: b_rs06_parse_feedback
    P->>B: b_bench_on_rs06_feedback で時刻と状態を保存
    C->>P: 次のCAN1処理の b_f446_motor_port_tick_2ms
    P->>B: b_bench_rs06_frame で有効化を評価
    alt Tx完了より後の正常応答を確認
        B->>E: b_rs06_torque_frame でType 1
        P->>H: HAL_CAN_AddTxMessage
    else 有効化の期限到達
        B->>E: b_rs06_control_frame でType 4
        P->>H: HAL_CAN_AddTxMessage
    end
</pre>

| タイミング | 呼び出し順 |
|---|---|
| トルク継続 | ポートが受信鮮度を評価し、CAN1処理回の `b_bench_rs06_frame` がパルス期限内のType 1を返す |
| 終了・中止・受信異常 | 明示中止は `b_f446_motor_port_cancel` → `b_bench_cancel`。期限や鮮度異常では `b_bench_rs06_frame` が停止要求を保持。次に送れるCAN1処理回でType 4を出す |
| 停止送信の完了 | `b_f446_motor_port_on_tx_done` → `b_bench_rs06_tx_done` が、該当Type 4の完了で停止送信待ちを解除 |

Type 4のTx完了で分かるのはフレームの送信までです。
[HALポート](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/device/balancing_f446_motor_bench_port.c#L98) · [単軸状態機械](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/app/balancing_motor_bench.c#L143) · [HALスタブ試験](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/tests/test_motor_hal_port.c)

## 3. 単軸C620 · HALスタブで通した車輪電流経路

<pre class="mermaid">
sequenceDiagram
    participant C as 試験側
    participant P as F446試運転ポート
    participant B as BMotorBench
    participant E as codec
    participant H as CAN2 HAL

    C->>P: b_f446_motor_port_on_rx0
    P->>H: HAL_CAN_GetRxMessage
    P->>E: b_c620_parse_feedback
    P->>P: 対象軸の受信時刻を保存
    C->>P: b_f446_motor_port_pulse_c620
    P->>P: rx_alive で受信鮮度を確認
    P->>B: b_bench_request_c620
    loop 各2 msの送信機会
        C->>P: b_f446_motor_port_tick_2ms
        opt 空きがあり受付済み要求がない
            P->>B: b_bench_c620_frame
            B->>E: b_c620_current_frame で共通0x200
            P->>H: tx_frame → HAL_CAN_AddTxMessage
            C->>P: b_f446_motor_port_on_tx_done で送信枠を解放
        end
    end
</pre>

電流を入れるのは対象1軸です。期限・受信断・中止の後、次に送れる共通0x200の全スロットが0 Aになります。
[電流要求の入口](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/device/balancing_f446_motor_bench_port.c#L225) · [0x200の生成](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/app/balancing_motor_bench.c#L229)

## 4. C11通常運転の部品 · ホスト試験の呼び出し順

| 処理 | 現時点で確認できる呼び出し・出力 |
|---|---|
| センサ・推定 | `test_issue10_flow.c` で `b_observation_imu` / `b_observation_wheel` → `b_observation_freeze`。新規IMUを `b_attitude_update` に渡し、`b_check_freshness` で鮮度を評価 |
| 操作・状態判定 | 同試験で `b_operator_accept` → `b_operator_tick` → `b_mode_step`。操作元が有効でも明示ARMがなければ `DISARMED` のまま |
| 倒立・車輪配分 | `b_lqr_compute` は状態と目標から制御量を計算。`b_wheel_allocate` は左右の上限と倒立・Yawの配分を処理。個別のホスト試験で検証 |
| 脚長・関節トルク | `b_leg_pd_capture` → `b_leg_pd_compute` の順で実測脚長を保持。`b_leg_fixture_forward` と `b_leg_virtual_to_joint_torque` は運動学fixtureの個別試験で検証 |
| 送信タイミング | `test_tx_scheduler.c` で `b_sched_tick` → `b_sched_next` → `b_sched_accepted` → `b_sched_completed`。CANフレーム生成前の送信機会を検証 |

<pre class="mermaid">
sequenceDiagram
    participant C as スケジューラ試験
    participant S as BTxScheduler
    participant M as 模擬メールボックス

    C->>S: b_sched_tick に駆動可否と時刻を渡す
    Note over S: CAN1は4 msごとに4軸、CAN2は2 msごとに共通要求
    C->>S: b_sched_next で送信候補を取得
    C->>M: 空きメールボックスを確認
    alt 受付できる
        C->>S: b_sched_accepted で占有を記録
        C->>S: b_sched_completed で完了時刻と空きを更新
    else 空きなし
        Note over C,S: 枠が空くか次の要求で更新されるまで待つ
    end
    opt フィードバック・バスオフの通知
        C->>S: b_sched_feedback または b_sched_bus_off
    end
</pre>

この段階のC620の0 Aは送信要求フラグです。C11の状態判定、トルク計算、送信モデルは、いまのF446上では一本の駆動経路になっていません。
[入力・状態試験](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/tests/test_issue10_flow.c#L67) · [制御コア](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/firmware/f446/balancing_f446/App/Src/balancing/control/controller/balancing_core.c#L277) · [送信モデルの試験](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/tests/test_tx_scheduler.c#L23)

## 5. C++17 RS06 4軸 · PR #31の模擬CAN結合

<pre class="mermaid">
sequenceDiagram
    participant C as Fixtureと模擬CAN
    participant S as MotorService
    participant R as RobotSequence
    participant M as Rs06Motor

    opt 送信結果・受信のイベント
        C->>S: onRs06TxResult または onRs06Frame
        S->>M: onTxResult または onFeedback
    end
    C->>S: rs06ArmFeedback
    S->>M: status を4軸分取得
    C->>S: takeObservedFaults
    S->>M: status を読み、継続中の異常も集約
    C->>R: update に操作・状態・異常を渡す
    R-->>C: 周期の要求とARMセッション番号
    C->>S: applySequence
    S->>M: 必要時に requestEnable または requestStop
    C->>S: pendingRs06Requests
    S->>M: nextRequest → Rs06Codec::encodeControl
    S-->>C: 軸別フレームとトークン
</pre>

| 処理 | `RobotSequence::update` の結果と次の呼び出し |
|---|---|
| 起動直後 | `onRs06TxResult` で4軸の停止Tx完了を記録するまで新規ARMを受理しない |
| ARMを受理 | `EnableRs06` を返す。各軸のType 3を送信候補に出し、受付と完了をトークンで照合する |
| 4軸の正常応答 | 同じセッションで、Type 3完了後の正常なType 2が期限前に全軸でそろい、機構安全も成立すれば `DriveAllowed` を返す |
| DISARM／FAULT | `StopAll` を返し、未完了軸のType 4を送信候補へ。同じ軸に受付済みType 3があれば、その完了または中止の後に送る |
| Type 4完了後 | Tx完了と、新しい正常Type 2による無効化確認を個別に追跡する |

`onRs06Frame` は `Rs06Codec::decodeFeedback` で形式を確認してから軸へ配送します。Type 24は鮮度・故障監視に使います。実CAN上の状態報告にはセッション番号がなく、報告種別と時刻の関係は実機で照合します。`DriveAllowed` は全体状態の結果で、模擬試験で送る指令はType 3とType 4です。
[Fixture::cycle](https://github.com/kou-pen/balancing/blob/bbcf6f9457685cc95b47548d8b7f7d771c9b3660/tests/test_motor_service_rs06.cpp#L34) · [MotorService](https://github.com/kou-pen/balancing/blob/bbcf6f9457685cc95b47548d8b7f7d771c9b3660/firmware/f446/balancing_f446/App/Src/balancing/app/MotorService.cpp#L25) · [Rs06Motor](https://github.com/kou-pen/balancing/blob/bbcf6f9457685cc95b47548d8b7f7d771c9b3660/firmware/f446/balancing_f446/App/Src/balancing/device/Rs06Motor.cpp#L112)

## 6. C++17通常駆動 · 今後の接続設計

クラス名だけを示した工程の公開関数名は接続時に確定します。

<pre class="mermaid">
sequenceDiagram
    participant C as CAN・タイマー
    participant A as Application
    participant E as 状態推定
    participant R as RobotSequence
    participant M as MotorService

    C-->>A: Rx・Tx完了・バス異常・2 ms実行要求
    A->>M: CANイベントを反映して6軸の観測を固定
    A->>E: StateEstimator::update
    E-->>A: ControlSnapshot または推定失敗
    A->>R: update に操作・観測・安全状態を渡す
    alt 駆動許可と有効な制御入力
        opt 制御に入る周期
            A->>A: BalanceControl::initialize
        end
        A->>A: BalanceControl::compute → MotorCommandで6軸へ変換
        opt 停止が必要な計算・変換異常
            A->>R: 異常を送信前に反映
        end
    else 駆動許可なし
        A->>A: 必要時に BalanceControl::reset
    end
    A->>M: 最終要求・ARMセッション番号・6軸指令
    M->>C: CAN別の送信調停からCanPortへ
</pre>

CAN1は4 msごとの最新RS06 Type 1指令、CAN2は2 msごとのC620共通電流指令を対象にします。停止を決めた周期は旧非ゼロ指令を無効化し、RS06では軸別Type 4、C620では0 Aを優先します。
[F446公開API設計](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/docs/design/public-api.md) · [RS06通信仕様](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/docs/specs/protocols/rs06_can1.md) · [C620通信仕様](https://github.com/kou-pen/balancing/blob/818efcc77f6b05e1b71fd2515944391f47670f27/docs/specs/protocols/c620_can2.md)
