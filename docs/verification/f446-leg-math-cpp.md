# 脚運動学・VMCのC++統合

基準 master 8c5b57a。CTRL-EST-001 / CTRL-LQR-001。App 96→94、Cソース2→1。新規Appファイルなし。

既存LegKinematics.hpp/.cppにモータ→機構角変換、2円交点の順運動学、解析J、仮想仕事J^T、理想伝達トルク換算を統合。旧C型とbalancing_leg_kinematics.h/.cを廃止。既存のLegMotorTransform、LegFixtureGeometry、LegEstimateを再利用。LegJointStateのみ追加し、既存LegEstimateの末尾に仮想足先座標2値を追加（両分岐・鏡映の独立検証に用いる）。BalanceControlから旧C中間型へのコピーを削除。これによるRelease bss増分16Bを確認。既存MotorCommandの換算・効率・安全判定は変更しない。

数式、float演算順、角度周期化、分岐、特異判定、失敗時ゼロ、校正拒否を保持。旧CのNULL引数APIは参照・固定長配列に置換。実機六節リンクCADを確定する変更ではない。fixtureOnlyと出力禁止を維持。

## 検証

旧C運動学試験をC++へ移行。両分岐、反射、周期性、有限差分J、13×13姿勢格子、仮想仕事率、校正・到達不能・特異性、異常入力の既存試験を保持。第2軸のVMCオーバーフロー時に両軸ゼロ、solveの未校正拒否を追加。

旧Cソースを基準Gitからbuild/leg-differentialへ抽出。seed446の20000反復×4演算で角・速度・到達状態・足先・脚長・角度・解析J・各トルクを直接比較。80000/80000一致、成功幾何8101件、通常・UBSan各同結果。比較コードはVM build/leg-differential/compare.cpp。

CTest48/48（970ee6359cd2453e）、通常/UBSan全件（50ef4fdc0a134858）、ARM Debug/Release全44 Appソース0 errors/0 warnings（98ee1a78d2534f17）。Release text69208/data2828/bss12232 B。実機書込み・通電・CAD校正・WCET未実施。
