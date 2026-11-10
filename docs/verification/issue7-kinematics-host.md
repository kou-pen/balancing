# Issue #7：脚運動学の数学fixture・ホスト検証

**日付：** 2026-11-10  
**根拠：** CTRL-EST-001、T-CTRL-01/02、MOTOR-160〜162  
**検証範囲：** HAL非依存C11とホスト。実機CAD・原点・物理駆動は未検証。

## 実装と判定

| 項目 | 計算・テスト範囲 | 判定 |
|---|---|---|
| モータ→機構角 | 外部16:24比、2軸の原点・符号、速度換算 | ホストPASS |
| 平面閉リンク | 円交点の±分岐、脚長、仮想脚角、速度 | 数学fixture PASS |
| 解析ヤコビアン | 先端2本の幾何拘束を微分。中央有限差分で検証 | ホストPASS |
| 左右鏡像と周期 | 左右交換・反射、2π周回の一致 | ホストPASS |
| 到達不能・特異姿勢 | 円が交差しない、接する、ほぼ一直線、NaN/Inf | ホストPASS |
| 仮想仕事 | 軸力と仮想脚トルクからの機構軸トルク、理想外部比 | ホストPASS |
| 実機六節リンク | 節の接続、仮想足先、組立分岐、左右の原点・方向 | **未検証** |
| 新ソースのARMビルド | CubeIDEの正規Makefile再生成とDebug/Release | **PASS：7ソース、0エラー・0警告** |

このモデルは実機の六節リンクを数式化したものではありません。実接続が不明なため、別の数学fixtureとして二本の先端ロッドの円交点を解いています。旧来の脚長近似式を確定運動学として採用していません。

## 対象ファイル

| 責務 | ファイル |
|---|---|
| 公開API | firmware/f446/balancing_f446/App/Inc/balancing/control/estimator/balancing_leg_kinematics.h |
| C11実装 | firmware/f446/balancing_f446/App/Src/balancing/control/estimator/balancing_leg_kinematics.c |
| 数学テスト | tests/test_leg_kinematics.c |
| 人間向け説明 | docs/design/leg-kinematics.md |

## テスト条件と再実行

数学fixtureの入力：固定軸間隔0.20 m、左右クランク0.15 m、先端ロッド0.25 m、交差sin下限0.02、最小仮想脚長0.01 m。これらの値を実機のゲイン・トルク上限・可動域へ流用しません。

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

解析ヤコビアンは入力角2軸について中央有限差分と比較します。実機の六節リンクへの置換は、正確な拘束方程式と出力点が決まるまで実施しません。

## 実行結果（Balancing Development VM、2026-11-10）

| チェック | 結果 |
|---|---|
| ホスト通常・UBSan | **PASS**：運動学fixtureと既存の制御/CAN回帰 |
| CMake/CTest | **6/6 PASS**（新規運動学ターゲットを追加） |
| 仕様ID・文書リンク | **PASS** |
| Doxygen・Ruff | **PASS**（Doxygenの既知の日本語翻訳警告のみ） |
| CubeIDEメタデータ整合 | **PASS** |
| CubeIDE Debug/Release（7ソース） | **PASS：2026-11-10 12:44、両構成0エラー・0警告、ARM ELF32** |
| 実機書込み・動作 | **NOT RUN** |

CubeIDE本体のクリーンビルドは `python3 tools/regenerate_cubeide_f446.py` で実行。Debug 12:44:44、Release 12:44:59に両方 `Build Finished. 0 errors, 0 warnings.` を確認し、`python3 tools/verify_f446_build.py` で7ソース全件の生成Makefile・オブジェクト・ARM ELF32・成果物の鮮度をPASS。ログはGit管理外の `build/cubeide-managed-pr16/{debug,release}.log` に保存。

追加テストでは、13×13の角度格子と2組立分岐を走査し、各成功点のヤコビアンを中央有限差分に比較しています。失敗時は出力をゼロ化し、入力・特異・到達不能を呼出側へ明確に返します。

## CADと実機による残件

| 未確定 | 検証元 |
|---|---|
| 六節リンク各節・関節の接続、閉路、固定軸間隔 | CADアセンブリ |
| 出力点と仮想脚長・前後角の原点、左右の座標系 | CADと組立 |
| RS06四軸の符号、原点、電源再投入後の多回転角 | 実機校正 |
| 組立分岐・機械限界・特異点近傍の保護閾値 | CAD可動域と実機 |
| ガスシリンダの作用点・荷重、伝達効率・摩擦 | CADと測定 |

**このPRによるホストPASSは、実機での安全動作、物理的な伝達比・符号の正しさ、原点の再現性を意味しない。** 制御状態機械・実機ドライバには接続しない。
