# 姿勢フィルタのC++統合

基準 master 8c5b57a。CTRL-EST-001。App96→94、Cソース2→1（脚運動学PRと統合後は92、0）。新規Appファイルなし。

旧balancing_attitude_filter.h/.cを廃止し、ImuAttitudeSource.hpp/.cppへ取付軸変換、静止バイアス学習、重力方向補正、ジャイロ予測、最短角度差処理を移行。センサ値ImuSampleはBno055.hppに配置。I2cPortの前方宣言でHALへの間接依存を解消し、HAL非依存の数学試験を維持。固定長配列、参照引数、noexceptを使用。フィルタ状態の所有者は従来のImuAttitudeSource。既存SIフィールド名を保持する。

取付マップの右手系検証、新規静止証明、元時刻、バイアス平均のdouble演算、相補フィルタのfloat演算と角度範囲、重力範囲外のジャイロのみ予測、最大更新間隔、50ms鮮度、異常時のラッチ無効化を変更しない。純数学関数の失敗時は出力ゼロ・旧内部状態保持。実機静止証明、係数・軸方向、姿勢制御の実機接続は未確定。HAL/I2C設定、駆動許可は変更しない。

## 検証

旧C姿勢試験をC++へ移行し、右手系・鏡像拒否、静止と新規性、重複・未来・欠落時刻、加速度外乱、角度wrap、再初期化を保持。既存StateEstimator統合とIMU runtime試験も更新。初回ホストビルドでHALヘッダーへの間接依存を検出し、Bno055.hppの前方宣言で解消。

旧Cソースを基準Gitからbuild/attitude-differentialへ抽出し、seed446で2000セッション（設定、各8静止候補、32更新候補）、合計82000件を直接比較。更新成功41423件、通常・UBSanともに戻り値・推定・元時刻・バイアス一致。VM build/attitude-differential/compare.cpp。旧Cソースは製品コードへ再導入しない。

CTest48/48（7c53a366f625479c）。実機未操作。

通常/UBSan全件PASS（40b6f54da9d24d9e）、CubeIDE Debug/Release全44 Appソース、0 errors/0 warnings（0138b424960f4e52）。Release text69376/data2828/bss12216 B。仕様454 API参照、CAN監査19/19、CubeIDE設定22/22、Doxygen全sourceリンク、diff check PASS。
