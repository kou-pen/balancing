# STM32CubeIDEでC++を使用する（自動コード再生成対応）

**適用：** F446制御基板、`firmware/f446/balancing_f446`。2026-11-26更新。

CubeMX生成のCore/・Drivers/はCのまま保持する。自作App/Src/balancing以下はC++17、App/Incは対応する公開APIと生成Cから呼び出すC ABIブリッジ。生成ソースの編集はUSER CODE領域内に限る。

## ビルド構成

| 構成 | 出力ファイル | 起動経路 |
|---|---|---|
| Debug | Debug/balancing_f446.elf | 通常Application（現行非駆動観測） |
| Release | Release/balancing_f446.elf | 通常Application（現行非駆動観測） |
| BenchDebug | BenchDebug/balancing_f446_bench.elf | 停止確認用の独立試運転ランタイム |

BenchDebugだけC/C++両コンパイラにBALANCING_BENCH_BUILDを定義する。実行中のモード切替なし。既定の試運転設定は未検証のため起動拒否する。実機のパルス要求は未接続。別名のELFを生成するが、書込み権限を意味しない。

CubeIDEではプロジェクトをRefreshし、Project → Build Configurations → Set Activeで構成を選ぶ。C++はGNU++17、リンクはarm-none-eabi-g++、FLASHスクリプトを共有する。通常起動と試運転のCAN IRQ入口が混在しないことを生成ELFで確認する。

## CubeMX再生成後

```sh
python3 tools/sync_cubeide_cpp.py
python3 tools/sync_cubeide_cpp.py --fix
```

後者は必要時のみ実行し、.project/.cprojectのC++設定・BenchDebug構成を復元する。生成HALコードや.iocを変更しない。通常構成に試運転マクロが混ざっていた場合はエラーとする。KeepUserCode=trueを維持し、main.cとstm32f4xx_it.cのUSER CODEを確認する。

## 検証

```sh
python3 tools/run_host_tests.py
python3 tools/validate_specs.py
python3 tools/regenerate_cubeide_f446.py
```

最後のコマンドはVM上で3構成の独立CubeIDEワークスペースを使用し、管理ビルドによるARM ELFの生成・モード別シンボルを検証する。書込みや通電は行わない。

実装構造：[モータ制御レイヤーと単軸試運転](motor-layer.md)。検証：[BenchDebug起動・IRQ分離](../verification/f446-bench-startup.md)、[AppのC++統合](../verification/f446-all-cpp.md)。過去の5/6ソース構成の検証：[PR #16の検証](../verification/f446-source-layout.md)、[機能別配置](../verification/f446-module-layout.md)。旧設計のコード例はGit履歴に残す。
