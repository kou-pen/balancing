# Balancing — 二輪脚ロボット

STM32F446を中心に、脚機構と車輪を協調制御するロボットのファームウェアを開発するリポジトリです。最初の目標は**固定脚長での基本バランシング**。可変脚長とジャンプはその後に扱います。

## 現在地

| 領域 | 確認できていること | まだ確認できていないこと |
|---|---|---|
| F446のコード | C++17入口とC11モジュール。CubeIDE Debug/Releaseで10ソースのARMビルドPASS | 実機書込み、実時間500 Hz |
| 制御・通信の基礎 | 制御コア、CAN通信、IMU数学フィルタ、操作入力モックのホスト試験PASS | CAN/HAL・USB/I2Cの実機統合、停止性能 |
| ロボット全体 | 制御・通信・安全の設計と仕様IDを整備 | 状態推定・VMC等の結合、F303統合、実機倒立 |

**ホストPASSと実機PASSは別物です。** 今のビルド成果物は、モータを安全に動かせることを保証しません。

## 仕組みを理解する

操作・センサ入力をF446で受け取り、状態推定と制御からRS06脚モータ・C620車輪への指令を求める設計です。実際の駆動許可は安全状態機械が決めます。

[全体構成を図で見る](docs/architecture.md) · [実装順序を確認する](docs/todo/roadmap.md)

## 読み始める場所

| 知りたいこと | 最初に読むページ |
|---|---|
| 設計・コード・試験を理解する | [ドキュメント案内](docs/index.md) |
| 開発環境を整える | [CubeIDEとC++](docs/design/cubeide-cpp.md) |
| 仕様の確定値を調べる | [仕様ID索引](docs/spec-index.md) |
| 開発・PRの進め方を確認する | [開発手順](CONTRIBUTING.md) |

## PCで確認する

```sh
python3 tools/validate_specs.py
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
```

[試験結果の読み方と記録](docs/verification/index.md)に、ホスト検証と実機未検証の境界をまとめています。AIの作業ルールは [AGENTS.md](AGENTS.md) に分離しています。