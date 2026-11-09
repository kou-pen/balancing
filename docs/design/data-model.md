# データ構造とモジュールの責務

**対象：** [実機非依存の制御コア](../software-core.md)／[状態推定仕様](../specs/control/state_estimation.md)／[LQR・VMC仕様](../specs/control/balance_vmc.md)／[動作仕様](../specs/behavior/robot_operation.md)

**設計方針：** STM32F446ではC11を使う。C++のクラス・継承・仮想関数は採用せず、「状態を持つ構造体＋それを処理する関数」をモジュールの基本単位にする。**データが使用される場所で意味と単位が分かること**を優先する。

## 1. データのまとまり

<pre class="mermaid">
flowchart LR
    observed["BObservedStamps：測定時刻"]
    freshness["BFreshness：個別の鮮度"]
    modeInput["BModeInput：操作と保護条件"]
    machine["BModeMachine：状態を保持"]
    modeOutput["BModeOutput：駆動許可と要求"]
    state["BControlState：推定6状態"]
    gains["BLqrConfig：LQR設定"]
    torque["BControlTorque：抽象トルク"]

    observed -->|鮮度判定| freshness
    freshness -->|周期ごとの入力| modeInput
    modeInput -->|状態更新| machine
    machine -->|判断結果| modeOutput
    state -->|状態と目標| torque
    gains -->|行列による演算| torque
</pre>

これは**データ型の関係を表す設計図**であり、クラス継承やポインタ所有権を示すものではない。

## 2. 何を誰が持つか

| 型 | 作る場所 | 生存期間 | 変更できる主体 | 単位・注意 |
|---|---|---|---|---|
| `BObservedStamps` | センサ・通信アダプタ | 制御周期のスナップショット | アダプタのみ | 単調増加時刻、µs |
| `BFreshness` | `b_check_freshness` | 制御周期のみ | 再計算で更新 | 個別のboolのみ。`all_fresh`を保存しない |
| `BModeInput` | アプリケーション層 | 制御周期のみ | アプリケーション層 | 有効化送信時刻と受信時刻は異なる |
| `BModeMachine` | 起動時に1個確保 | 起動から終了まで | `b_mode_init`と`b_mode_step`のみ | FAULTとARM待ちを保持 |
| `BModeOutput` | `b_mode_step`の戻り値 | 制御周期のみ | 上位層が読み取るだけ | CANフレームを含まない |
| `BControlState` | 状態推定器 | 制御周期のスナップショット | 状態推定器 | rad、rad/s、m、m/s |
| `BLqrConfig` | 設定／モデル検証層 | 設定が有効な期間 | DISARMED時に検証して反映 | 2×6行列、位置誤差上限m |
| `BControlTorque` | `b_lqr_compute` | 制御周期のみ | 制御関数の戻り出力 | 車輪合計・仮想股関節合計、N·m |
| `BWheelAllocation` | `b_wheel_allocate` | 制御周期のみ | トルク配分関数 | 左右N·m、飽和情報 |

`BModeInput`は一周期の**読み取り専用スナップショット**として渡す。デバイス状態を後から取得するグローバル関数を、この構造体の内部に持たせない。

`BModeOutput`の`request_rs_enable`／`request_rs_disable`は**出力イベント**であり、モータが本当に受理した事実ではない。送信成功の記録は通信アダプタが管理する。

## 3. LQRの状態と添字

人間が読むコードでは、無名の`state[4]`ではなく`state.pitch_rad`を使用する。配列は**行列演算の内部に限定**する。

| 添字定数 | 構造体フィールド | 単位 |
|---|---|---|
| `B_STATE_THETA` | `theta_rad` | rad |
| `B_STATE_THETA_RATE` | `theta_rate_rad_s` | rad/s |
| `B_STATE_POSITION` | `position_m` | m |
| `B_STATE_VELOCITY` | `velocity_m_s` | m/s |
| `B_STATE_PITCH` | `pitch_rad` | rad |
| `B_STATE_PITCH_RATE` | `pitch_rate_rad_s` | rad/s |
| `B_OUTPUT_WHEEL_TOTAL` | `wheel_total_nm` | N·m |
| `B_OUTPUT_HIP_TOTAL` | `hip_total_nm` | N·m |

行列のゲインを指定するときは、`B_OUTPUT_WHEEL_TOTAL`と`B_STATE_THETA`のような定数で行と列の意味を明示する。C構造体のメモリ配置をポインタの型変換によって配列扱いしない。

## 4. 人間が扱いやすくするための制限

- 型を作る基準は、**異なる関数間で意味を共有するデータのまとまり**があること。引数1個のためのラッパー型は作らない。
- `Manager`、`Factory`、`Service`、`Context`など、振る舞いのない汎用クラス／構造体を増殖させない。
- ハードウェア抽象化は、将来F446側で**実際に異なるデバイス実装が必要**になる箇所だけ用意する。共通コアからHALを直接呼ばない。
- 構造体に「値」と「計算済みの集約フラグ」を二重保存しない。`BFreshness`全体判定は`b_freshness_all`で導出する。
- boolの`verified`は数学的な有効性の宣言ではなく、**上位層の検証記録**。PC試験でtrueにすることと実機で検証済みであることを混同しない。
- ヒープ割当、独自イベントバス、依存性注入コンテナ、反射、抽象継承、過剰なラッパーは導入しない。
- 公開APIの名前・単位・所有者を変えるときはテストを同じ変更で更新する。不要な互換ラッパーを残さない。
- データ構造を追加する前に、既存型へ責務を自然に加えられないか確認する。ただし意味の異なる値を一つの巨大な`RobotContext`に集めない。

## 5. 変更前後の例

**避ける書き方**

```c
float state[6] = {0};
float torque[2] = {0};
state[4] = measured_pitch;
controller(state, torque);
```

**採用する書き方（概念例）**

```c
BControlState state = {0};
BControlState target = {0};
BControlTorque torque = {0};
state.pitch_rad = measured_pitch_rad;
bool ok = b_lqr_compute(&config, &state, &target, &torque);
```

これは関数の使用形を示す例であり、未知の`K_300`を生成するものではない。戻り値`ok=false`のときに実機の駆動を許可しないことは上位アプリケーションの責務。

## 6. 保留する型

これからの状態推定・VMC・指令配分で、**実際に必要と分かったものだけ**追加する。具体的には、2本の脚の幾何情報、センサ値を単位付きで受け渡す型、左右関節トルク、ロギング用の最小診断値が候補。未実装のため今は型だけ大量に作成しない。

**実機検証待ち：** モータ極性、関節原点、K_300、トルク換算、配線・CAN定義、物理保護限界。
