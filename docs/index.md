# docs/

現行コードを調べる入口は、[architecture.md](architecture.md)（全体）→ [design/public-api.md](design/public-api.md)（フォルダ別）→ [diagrams.md](diagrams.md)（処理順序）です。Doxygenでは左側の実フォルダ名から各ソースの宣言・実装へ移動できます。

| フォルダ | 役割 |
|---|---|
| [design/](design/public-api.md) | 現行コードの役割・API、数学・通信の設計 |
| [F303電源基板の仕様](specs/boards/f303_power.md) | 承認済み10項目、回路境界、電源状態と復旧 |
| [Raspberry Piの仕様](specs/boards/pi.md) | PI-01〜07、ROS 2・Web UI・記録管理 |
| [specs/](spec-index.md) | 要求仕様の正本。実装済みとは限らない |
| [todo/](todo/roadmap.md) | 未完了の作業・受入条件 |
| [verification/](verification/index.md) | 日付・コミット付きの検証証拠。旧実装の記録も保持 |
| [development/](development/editor.md) | 開発環境・Git・運用 |
| [ai/](ai/context.md) | AI専用の参照順 |
| `assets/` | DoxygenとMermaidの表示設定 |

| 直下の文書 | 役割 |
|---|---|
| [architecture.md](architecture.md) | 実装の全体像と動作範囲 |
| [diagrams.md](diagrams.md) | 起動・周期・送受信・状態遷移の図 |
| [software-core.md](software-core.md) | HAL非依存の制御数学と状態判定 |
| [can-id-table.md](can-id-table.md) | CANの用途・ID一覧 |
| [spec-index.md](spec-index.md) | 固定仕様IDの索引 |

現行説明と要求仕様と検証記録を混ぜない方針です。細かな引数・戻り値・単位は実コードのDoxygenを正本とします。
