# docs/

| フォルダ | 内容 |
|---|---|
| `design/` | 設計の理由、実装方法、公開API |
| `specs/` | 仕様IDを持つ要求。配下の `boards/`、`behavior/`、`control/`、`protocols/` に分かれる |
| `todo/` | 作業順と受入条件 |
| `verification/` | 実施済みの試験、その結果と未検証事項 |
| `development/` | 開発環境、Git・GitHubの手順と運用規約 |
| `ai/` | AIの作業で参照する文書の順序 |
| `assets/` | Doxygenのフォルダ一覧とMermaid描画に使うファイル |

| ファイル | 内容 |
|---|---|
| [architecture.md](architecture.md) | F446とF303の構成、制御周期、実装・検証の現状 |
| [can-id-table.md](can-id-table.md) | CAN1・CAN2の用途、ID、データと物理単位 |
| [diagrams.md](diagrams.md) | 制御のデータフロー、状態遷移、停止と電源遮断の図 |
| [software-core.md](software-core.md) | 旧C11の状態機械、制御演算、ホスト回帰 |
| [spec-index.md](spec-index.md) | 仕様IDと仕様書の対応表 |

Doxygenでは、左のツリーからフォルダを開き、右の表で直下のファイル名と概要を確認できます。

文書やソースを追加・移動したときは `docs/assets/folder-navigation.dox` の該当フォルダを更新します。フォルダの追加・移動は `docs/assets/doxygen-layout.xml` のツリーにも反映します。

試験結果の読み方は [verification/index.md](verification/index.md)、AIの作業規約は [AGENTS.md](../AGENTS.md) を参照してください。
