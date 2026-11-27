# Raspberry Pi ソフトウェア構成

**仕様ID:** BOARD-PI-001  
**更新日:** 2026-11-27  
**状態:** PI-01〜07設計承認済み。初期CAN受信境界をレビュー版で追加。ROS接続・UI・実機は未実装。

根拠：[Issue #110](https://github.com/kou-pen/balancing/issues/110)。ワイヤ契約は[操作CAN](../protocols/remote_can2.md)・[電源CAN](../protocols/power_can2.md)、運転保護は[動作仕様](../behavior/robot_operation.md)を変更しない。

## 決定事項

| ID | 採用 |
|---|---|
| PI-01 | UIはPython/rclpy、その他はC++17/rclcppとROS 2。既存HAL非依存C++ CAN Codecを直接共用 |
| PI-02 | Python Web UIをPi上で提供。外部端末のブラウザで操作・監視。LiDAR・Nav2等の将来追加を想定し、初期範囲に含めない |
| PI-03 | ros2_socketcan＋SocketCAN、vcanによる試験。RMWはrmw_zenoh_cpp（ネイティブZenoh） |
| PI-04 | twist_muxで手動・自律の指令経路を分離。初期は手動のみ、明示選択、自動フォールバック禁止 |
| PI-05 | 独立した電源・運転操作＋状態に応じた操作制限。受理応答と状態成立を区別、自動ARM/FAULT解除/非停止要求の自動再送なし |
| PI-06 | Web UIで状態表示・受信失効表示、rosbag2の開始/停止・記録対象の動的選択・保存先/状態確認 |
| PI-07 | balancing_interfaces / balancing_can / balancing_web / balancing_bringup の機能別パッケージ、ROS 2 Launchで一括起動。初期コンテナ化は必須にしない |

## 責務と構成

<pre class="mermaid">
flowchart TD
    UI["balancing_web / Python"] -->|"手動速度・明示操作"| MUX["twist_mux / モード管理"]
    FUT["LiDAR・Nav2（将来）"] -.-> MUX
    MUX -->|"選択した速度"| CAN["balancing_can / C++"]
    UI -->|"明示電源・運転操作"| CAN
    CAN -->|"受信状態・操作結果"| UI
    CAN <-->|"CAN frame"| SOCK["ros2_socketcan"]
    SOCK <-->|"can0 / vcan0"| MCU["F446・F303"]
    UI -->|"記録開始・停止・トピック選択"| BAG["rosbag2 独立記録"]
    CAN --> BAG
</pre>

ROSノード間はZenohを使用する。安全機構はSTM32側で独立し、Piを倒立制御の周期ループへ入れない。UIが直接CANフレームを生成しない。電源投入・CLEAR_FAULT・ARMは独立した明示操作。F446側が選択元・中立・保護条件を最終判定する。Pi側でも状態鮮度を検査し、非有限・古い運動指令を拒否し、古い指令を周期再送しない。停止要求は状態不明・応答待ちでも妨げない。UI通信断とROS/CAN通信断は区別して扱い、F446既存の減速・復旧中立確認を保持する。物理非常停止の代替ではない。

## パッケージ

| パッケージ | 責務 | 現在 |
|---|---|---|
| balancing_interfaces | 状態・操作要求/結果のROSインターフェース | 未実装 |
| balancing_can | CAN v1変換、鮮度・操作応答照合、指令送信 | 初期受信境界のみ |
| balancing_web | ブラウザUI、ROS接続、記録管理 | 未実装 |
| balancing_bringup | Launch、Zenoh、twist_mux、SocketCAN接続設定 | 未実装 |

トピック・サービス名、Twist/TwistStamped選択、QoSは実装段階で決める。ROSの通信時刻とローカル単調時計を混同しない。CAN v1には指令生成時刻がなく、滞留・再送・切断後旧指令を避けること。運動20 ms/期限100 msは既存試験候補で実機値ではない。

rosbag2の記録プロセスは操作経路から分離し、ROS graphのトピック一覧から選択記録・全記録を扱う。任意シェル実行、無断削除は禁止。ストレージ容量不足・記録異常をUIで表示し、操作処理をブロックしない。電源/運転状態、操作元、FAULT、既存物理観測を対象とし、追加診断・設定書込み・追加IDは保留。

## 開発環境候補と検証

Ubuntu 24.04 LTS / ROS 2 Jazzy / arm64を基準候補とする（ユーザー未指定のため実機OS確定ではない）。[Zenoh Jazzyブランチ](https://github.com/ros2/rmw_zenoh/tree/jazzy)、[ros2_socketcan Jazzyリリース](https://github.com/autowarefoundation/ros2_socketcan-release)、[twist_mux](https://index.ros.org/p/twist_mux/)を参照。Pi内のZenoh routerを基本候補とし、RMW_IMPLEMENTATION=rmw_zenoh_cpp、rmw_zenohdで起動する。OS・ROSの実導入、依存バージョン固定、vcan、QoS、送信滞留、ブラウザセッション、アクセス制御、記録処理の実測は未検証。PiのCANアダプタ・ビットレートは実機確認後に設定。初期は実機操作を有効化しない。

受入条件：既存Codec共用の受信・送信候補のホスト試験、C++の状態鮮度・要求照合試験、ROS 2/Zenoh/vcanでF446/F303との結合、Web UIの明示操作・切断・rosbag記録、物理実機の通信・停止確認を区別して記録する。関連TODO：T-PROTO-05/06/11。
