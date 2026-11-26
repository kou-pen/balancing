# Balancing — 二輪脚ロボット

STM32F446で左右の脚（RS06 × 4）と車輪（C620 × 2）を制御します。最初の目標は固定脚長での基本バランシングです。

## プログラムを読む

| 知りたいこと | 文書 |
|---|---|
| ファームウェア全体の構造と現在の動作 | [全体構成](docs/architecture.md) |
| ファイルの配置・クラスの役割・コードへのリンク | [フォルダ別コード案内](docs/design/public-api.md) |
| 起動・割り込み・500 Hz周期の呼び出し順 | [処理順序の図](docs/diagrams.md) |
| どの型・クラスがデータを所有するか | [データ構造](docs/design/data-model.md) |
| 文書全体のフォルダ一覧 | [docs/](docs/index.md) |

## 現在地

自作AppのC++17移行は完了。通常ビルドは既定で受信・非駆動観測のみ。停止専用送信はPR #88で実装済みで、明示設定時に使用可能ですが、実機設定は未投入です。ARMINGからの実機有効化・非ゼロ駆動は未接続です。試運転は別ビルドBenchDebug、IWDGとUSB Hostの実機起動は既定無効です。

F303電源基板・Raspberry Piとの通信仕様とF446側Codecは存在します。対向ファームウェア・プログラムはこのリポジトリでは未確認です。

[実装・検証の状態](docs/architecture.md) · [検証記録](docs/verification/index.md) · [残作業](docs/todo/roadmap.md)

## 開発環境

[CubeIDEとC++](docs/design/cubeide-cpp.md) ／ [開発・PR手順](CONTRIBUTING.md) ／ [仕様索引](docs/spec-index.md)。

```sh
python3 tools/validate_specs.py
doxygen Doxyfile
python3 tools/run_host_tests.py
```

Doxygenの出力は `build/docs/html/index.html`。ホスト試験とARMクロスビルドは実機動作・物理安全性を保証しません。AIの規則は [AGENTS.md](AGENTS.md) に分離しています。
