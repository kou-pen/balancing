# 図で見る制御システム

**設計図更新：** 2026-11-09（MOTOR-146〜224。実装済みを意味しない）

このページでは、データの流れ、起動・駆動許可、定周期処理、状態遷移、非常停止の関係を示す。**図は設計仕様の視覚化であり、コード実装済み・実機動作検証済みを意味しない。**

- [通信IDとデータ単位](can-id-table.md)
- [ソフトウェア構成](architecture.md)
- [ロボット操作・状態遷移の詳細](specs/behavior/robot_operation.md)
- [F446基板](specs/boards/f446_control.md)／[F303電源基板](specs/boards/f303_power.md)

## 1. 全体データフロー

<pre class="mermaid">
flowchart TB
    USB["有線PS5コントローラ"] --> OP["操作入力の受付"]
    PI["上位機器：Piなど（CAN2）"] --> OP
    IMU["BNO055（I2C）"] --> SNAP["時刻付きセンサ取得と更新監視"]
    JOINT["RS06 4台（CAN1受信）"] --> SNAP
    WHEEL["C620・車輪2台（CAN2受信）"] --> SNAP
    PWR["F303電源状態（CAN2受信）"] --> SAFE{"状態と保護条件の監視"}
    SNAP --> EST["状態推定・6節リンク計算（500 Hz）"]
    SNAP --> SAFE
    OP --> TARGET["目標速度・脚長の生成、変化率制限"]
    EST --> CTRL["LQR・脚長PD・Roll PD・Yaw PI・VMC（500 Hz）"]
    TARGET --> CTRL
    CTRL --> LIMIT["ガスシリンダ補償・車輪Yaw配分・モータ個別の上限"]
    LIMIT --> GATE{"駆動許可の判定"}
    SAFE --> GATE
    GATE --> RSOUT["RS06へのトルク指令（250 Hz）"]
    GATE --> WOUT["C620への電流指令（500 Hz）"]
</pre>

**読み方：** センサの新規更新頻度と500 Hzの演算周期は同一ではない。安全判定は制御器の出力に対して独立して行い、トルク飽和のみをFAULT扱いしない。実装時には「駆動許可なし」がトルク出力を有効化しないことを検証する。

## 2. ARM要求から倒立開始まで

<pre class="mermaid">
sequenceDiagram
    autonumber
    participant USB as 有線PS5
    participant F446 as F446主制御
    participant F303 as F303電源
    participant RS as RS06 4台
    participant C620 as C620 2台

    F303-->>F446: POWER_STATUS（100 msごと、CAN ID未確定）
    RS-->>F446: 関節状態・故障・温度（CAN1）
    C620-->>F446: 車輪状態（CAN2）
    USB->>F446: OPTIONSを約1秒長押し（ARM要求）
    F446->>F446: センサ・通信・原点・モデル・制限値の健全性を検査（初期姿勢／脚長範囲のARMゲートなし）
    alt ARM前提が正常
        F446->>RS: Type 3 有効化（4台へ送信要求）
        RS-->>F446: 各軸から有効化後の新しい正常応答
        alt 4台すべて有効化確認
            F446->>F446: 現在脚長を目標へ設定・制御器初期化
            F446->>F446: BALANCINGへ移行（トルク急変制限）
        else 1台でも有効化失敗
            F446->>RS: 全4台へ駆動無効化を要求
            F446->>F446: FAULTへ移行・原因記録
        end
    else ARM前提が不成立
        F446->>F446: DISARMEDを維持し拒否理由を記録
    end
</pre>

**注意：** ARM時の目標脚長は現在の左右脚長から初期化する（MOTOR-159）。自動起立・起き上がりモーションはMOTOR-132として保留。CAN2の電源状態通知と上位操作の具体的ID・DLCは実装照合待ち。

## 3. 倒立中の周期処理

<pre class="mermaid">
sequenceDiagram
    autonumber
    participant S as センサ受信割り込み
    participant F446 as F446：2 ms制御周期
    participant RS as RS06：CAN1
    participant C620 as C620：CAN2
    participant F303 as F303：電源CAN2

    loop 2 msごと（500 Hz）
        S-->>F446: 最新測定値と受信時刻
        F446->>F446: 更新時刻・状態の有効性を確認
        F446->>F446: 6節リンク・状態推定・LQR・VMC・保護
        F446->>C620: 0x200：左右電流指令
        opt 4 msごと（250 Hz）
            F446->>RS: Type 1：4台へ最新トルク送信要求（3メールボックスに順次投入）
        end
    end
    loop 非同期の受信
        RS-->>F446: 状態報告（初期100 Hz想定）
        C620-->>F446: 車輪状態（メーカー既定1 kHz、実測待ち）
        F303-->>F446: POWER_STATUS（100 msごと）
    end
</pre>

**注意：** 図の時間関係は各周期の論理的な相対関係であり、同じ制御tick内の具体的な命令順序・割り込み優先度を固定するものではない。CAN2の電流指令は2台分を**1フレーム（0x200）**に格納する。RS06/M3508とも送信完了とフィードバック受信を別々に監視する。DISARMED中のM3508は0電流を継続送信、RS06は通常のトルク送信を停止。

## 4. ロボットの状態遷移

<pre class="mermaid">
stateDiagram-v2
    [*] --> DISARMED: 起動後
    DISARMED --> BALANCING: ARM要求・安全条件成立・RS06正常応答確認
    BALANCING --> DISARMED: DISARM要求
    BALANCING --> FAULT: 重大保護条件／CAN非常停止
    DISARMED --> FAULT: CAN非常停止
    FAULT --> DISARMED: 原因解消後にCLEAR_FAULT
    note right of DISARMED
      駆動無効
      センサと通信を監視
      設定変更はRAMのみ
    end note
    note right of BALANCING
      姿勢保持と移動・旋回
      操作入力断では目標を減速
    end note
    note right of FAULT
      トルク出力停止
      原因を記録
      自動ARM・自動復帰はしない
    end note
</pre>

**補足：** FAULTから直接BALANCINGへ遷移しない。原因解消→明示的CLEAR_FAULT→DISARMED→別ARMが必要。物理非常停止はこの状態機械の外側で最優先に駆動を遮断し、解除しても自動ARMしない。電源断後のFAULT状態保持はMOTOR-095として保留。

## 5. 非常停止と電源遮断

<pre class="mermaid">
sequenceDiagram
    autonumber
    participant EXT as 外部CANの非常停止
    participant F446 as F446主制御
    participant F303 as F303電源
    participant MOT as RS06・C620

    Note over EXT,F303: 物理非常停止は別経路で最優先
    par 主制御側
        EXT->>F446: EMERGENCY_STOP（ID未確定）
        F446->>F446: FAULTへ遷移しトルク出力を停止
    and 電源基板側
        EXT->>F303: EMERGENCY_STOP（ID未確定）
        F303->>F303: 主電源OFFをラッチ
    end
    Note over F446,MOT: 電源が先に切れる可能性があり停止指令の到達を保証しない
    Note over F446,F303: 復旧の基本順序：POWER_ON → CLEAR_FAULT → ARM
</pre>

**注意：** CAN非常停止に加え、物理非常停止は制御ソフトと独立した遮断手段とする。フレーム番号と送信元認証方式は[電源CAN仕様](specs/protocols/power_can2.md)と[上位CAN仕様](specs/protocols/remote_can2.md)で未確定として管理する。

## 6. 実機非依存のC制御コアと周辺アダプタ

<pre class="mermaid">
flowchart LR
    USB["操作入力と制御イベント"] --> AD["ハードウェア依存アダプタ（未実装）"]
    CAN["CAN／I2Cの受信値"] --> AD
    CLOCK["単調増加クロック"] --> AD
    AD --> F["更新鮮度の判定"]
    AD --> M["DISARMED・BALANCING・FAULTの遷移"]
    AD --> L["変化率制限と6状態LQR"]
    F --> M
    L --> W["倒立優先の左右トルク配分"]
    M --> G{"駆動許可と停止要求"}
    W --> G
    G --> OUTPUT["CAN送信アダプタ（未実装）"]
    subgraph CORE["ホストで検証する純粋なC11コア"]
        F
        M
        L
        W
        G
    end
</pre>

図のアダプタ側は未実装。**LQRの試験係数は実機用ではない。** 詳細は[制御コア設計](software-core.md)。

## 7. 後続開発：ジャンプの1つの状態機械

<pre class="mermaid">
stateDiagram-v2
    [*] --> PREPARE: 静止・両輪接地で押下イベント
    PREPARE --> TAKEOFF: 離陸準備完了
    TAKEOFF --> FLIGHT: 離陸を検出
    TAKEOFF --> RECOVERY: 離陸不成立で接地安定
    FLIGHT --> LANDING: 左右どちらかが接地
    LANDING --> RECOVERY: 両輪接地が安定
    RECOVERY --> [*]: 通常BALANCINGへ
    note right of TAKEOFF
      左右に同じ一定目標推力
      ガスシリンダ補償と個別トルク制限
    end note
    note right of LANDING
      接地した側：柔らかい脚長PD
      未接地側：空中用脚長PD
      機体全体の状態は1つ
    end note
</pre>

**後続の仕様のみ：** ジャンプはMOTOR-213の初期実装対象外。どの状態でも非常停止、DISARM、重大FAULTを優先する。離陸・着地誤判定とタイムアウトは[ジャンプ仕様](specs/behavior/jump.md)と[TODO](todo/jump.md)を参照。

## 図の管理

図はMermaid記法で編集できる。Doxygen 1.9.8ではMermaid記法の自動描画に対応していないため、`pre class="mermaid"`を用い、生成されたHTML上で**VM内のMermaid.js**により図に変換する。インターネット接続は不要。紙媒体向けのSVG・PNG書き出しは別途実施する。

文書中の未確定事項は図でも確定事項として表現しない。変更時には対応する仕様書と図を同じ作業内で更新する。