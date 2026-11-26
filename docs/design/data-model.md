# データ構造とモジュールの責務

現行F446はC++17の状態機械・観測経路を正本とする。数値演算の一部は移行中のC11型を使用する。[F446 公開API設計](public-api.md) を参照。

**対象：** [実機非依存の制御コア](../software-core.md)／[状態推定仕様](../specs/control/state_estimation.md)／[LQR・VMC仕様](../specs/control/balance_vmc.md)／[動作仕様](../specs/behavior/robot_operation.md)

**設計方針：** C++17の具象クラスに機能固有の状態を持たせ、型と所有者を明示する。実際に必要な境界以外の抽象化を増やさない。**データが使用される場所で意味と単位が分かること**を優先する。

## 1. データのまとまり

<pre class="mermaid">
flowchart LR
    sensors["ImuAttitudeSource / MotorService / PowerBoardService"] --> inputs["元時刻付き観測・安全判定"]
    inputs --> sequenceInput["RobotSequenceInput"]
    sequenceInput --> sequence["RobotSequence"]
    sequence --> result["RobotSequenceResult"]
    inputs --> estimator["StateEstimator"]
    estimator --> snapshot["ControlSnapshot"]
    snapshot --> control["BalanceControl"]
    control --> torque["機構トルク"]
</pre>

## 2. 何を誰が持つか

| 型 | 所有者 | 生存期間・責務 |
|---|---|---|
| `ImuAttitude` / `MotorFeedbackSnapshot` / `PowerBoardSnapshot` | 姿勢・モータ・電源の各機能 | 元の測定時刻と鮮度を持つ周期コピー |
| `RobotSequenceInput` | Application | 操作イベント、各機器の安全判断、ARMセッション付き応答 |
| `RobotSequence` | Application | DISARMED／ARMING／BALANCING／FAULT、ARM期限、故障記録 |
| `RobotSequenceResult` | RobotSequenceの戻り値 | 周期の状態・継続駆動要求・セッション。送信完了の証拠ではない |
| `ControlSnapshot` | StateEstimator | 元時刻を保持する6軸・姿勢の数学観測。成立した周期のみ |
| `BControlState` / `BLqrConfig` / `BControlTorque` | BalanceControlの数値演算 | 移行中のLQR状態・ゲイン・抽象トルク |
| `BWheelAllocation` | 車輪配分関数 | 左右N·m、飽和情報 |

通信鮮度は各機器の所有者が判定し、上位は周期コピーを使う。旧CのBObservedStamps、BFreshness、BModeMachine、観測ストアは廃止した。現行C++の期限ちょうどで無効となる判定やARMセッションを、旧Cの判定で置き換えない。

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
- 構造体に「値」と「計算済みの集約フラグ」を二重保存しない。各機器の受信鮮度は機器の所有者が評価し、RobotSequenceは周期の保護条件を評価する。
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
