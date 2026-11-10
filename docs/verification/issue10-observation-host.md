# Issue #10：時刻付きIMU・車輪スナップショットのホスト検証

**対象：** CTRL-EST-001、PROTO-C620-001、T-CTRL-04/05、T-F446-05  
**範囲：** C11 HAL非依存演算のみ。実機センサの測定成功や物理校正を意味しない。

## 何を確認したか

| 項目 | ホスト試験の条件 | 判定 |
|---|---|---|
| 500 Hz用の固定コピー | ストア更新後も過去のスナップショットは不変 | PASS |
| IMU時刻の保持 | 新規測定未証明、重複、過去、未来、NaN/Infは不受理 | PASS |
| IMUの50 ms鮮度境界 | 元の測定時刻が境界と超過で正しく判定される | PASS |
| C620デコーダとの接続 | 実C11 codecのID 0x201からロータ角・RPMを取得 | PASS |
| 左右車輪 | 8190→2／2→8190の折返し、鏡像方向、RPM換算 | PASS |
| 車輪20 ms鮮度境界 | コピー時刻ではなく受信時刻で判定 | PASS |
| 非一意・不正入力 | ±4096カウント、未校正設定、違う半径/減速比/方向 | PASS |
| RS06/F303・DualSense | 未実装（全体の有効性は通らない） | **NOT VERIFIED** |
| 実際のBNO055の更新証明・軸・偏差 | 実機未接続 | **NOT VERIFIED** |

### 実行結果（2026-11-10、開発VM）

| チェック | 結果 |
|---|---|
| 通常・UBSan | **PASS**：IMU/車輪と既存全モジュール |
| CMake/CTest | **7/7 PASS** |
| 仕様ID・文書リンク、Doxygen、Ruff | **PASS**（Doxygenの既知の日本語翻訳警告のみ） |
| CubeIDE Debug | **PASS**：8ソース・17:48:25、0エラー・0警告 |
| CubeIDE Release | **PASS**：8ソース・17:48:40、0エラー・0警告 |
| 実機センサ・受信割込み | **NOT RUN** |

CubeIDE管理ビルドは `python3 tools/regenerate_cubeide_f446.py` でクリーン再生成し、`python3 tools/verify_f446_build.py` で8ソースの生成Makefile、オブジェクト、ARM ELF32と鮮度を確認しました。ログはGit管理外の `build/cubeide-managed-pr16/{debug,release}.log` に保存されています。

### 実行方法

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/verify_f446_build.py
```

テスト数値の車輪半径0.1 m、ロータ／車輪比10:1、左右の符号は**数学fixture**です。実車輪径、M3508の内外減速比、左右極性・トルク定数を確認したことにはなりません。ハードウェアへの制御出力は一切追加していません。

## ソフトウェア上の残件

IMUの新規測定判定は外部から真偽値を渡すだけで、BNO055 I2Cデータレジスタや実HID入力の処理は実装していません。RS06×4・F303電源の受信値はこのストアでは保持しないため、既存 `BObservedStamps` の該当項目は無効のままです。全系統の鮮度が必要なARM状態へは接続していません。

将来のIRQ/制御ループ接続では同じストアを割込みと主ループから無同期で触らず、イベントの直列化・スナップショットのコピー一貫性を別途検証します。実際にサンプル間でロータが半回転以上する場合の巻数復元は、このアルゴリズムでは保証できません。

**本変更はIssue #10の部分実装です。** 操作元切替、BNO055補正・フィルタ、実車輪パラメータの同定を終えるまではIssueを閉じません。
