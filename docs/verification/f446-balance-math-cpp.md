# BalanceControlのC++数値演算統合

基準 master 35d24c9。CTRL-LQR-001。

## 変更内容

既存BalanceControl.hpp/.cppに6状態LQR、左右車輪配分、固定脚長・Roll PD、Yaw PIの数値処理を集約。旧C11のbalancing_core、balancing_leg_pd、balancing_yaw_pi各.h/.cを廃止し、Appファイル102→96。Cソース5→2。新規Appファイル・クラスは追加しない。

- LqrConfigのゲイン・平衡値を固定長std::arrayへ変更。LqrStateIndex/LqrOutputIndexの名前付き添字を維持。型名から旧B接頭辞を除き、SIフィールド名は不要な差分を避けるため保持。
- balance_math名前空間内の純粋な計算関数（参照引数、noexcept、失敗時ゼロ）をBalanceControlと単体試験で共有する。実行時の脚長目標・Yaw積分器の所有者はBalanceControlだけ。ヌルポインタを受け取る旧APIは廃止。
- LQRはfloat状態差分、位置誤差のみのクリップ、double積和とfloat範囲検査を保持。倒立共通成分優先、残容量でYaw差動。
- 左右の初期脚長を平均と半差で保持し、PD微分項は測定速度のみ。Roll差動の符号、重力補償、double演算を保持。
- Yaw PIの積分上限、飽和悪化方向の積分凍結、符号反転による積分解消、非BALANCING・失敗時リセットを維持。
- VMC、脚運動学、姿勢フィルタは今回変更しない。制御周期、実機パラメータ、駆動許可、fixtureOnly、HAL/CAN出力経路も変更しない。

## 検証

旧C単体試験3本をC++へ移行。車輪配分10000ケース、状態添字・位置誤差、PD初期支持力、Roll符号、積分凍結と解除、無効入力を保持。LQR第2出力のオーバーフロー、NaN、状態差分オーバーフロー時の全出力ゼロを追加。Cのヌルポインタ試験は参照APIでは表現不能なので廃止。

旧Cソースを基準Gitからbuild/balance-differentialに抽出し、固定seed=446、20000反復×5演算（車輪配分、LQR、脚長取得、脚PD、Yaw PI）で新旧の戻り値・計算値・状態・飽和フラグを直接比較。NaN、Inf、最大float、負上限、未検証設定、非BALANCINGを含む。通常・UBSan各100000件一致。比較スクリプトの実体はVM build/balance-differential/compare.cpp。旧Cは製品コードへ再導入しない。

CTest48/48（c90ef4a12740428c、追加LQR試験を含む再ビルド・再実行も48/48）。実機通電・書込み・物理ゲイン・WCET実測は未実施。

通常/UBSan全件PASS（b2b2d987667641f6）。CubeIDE Debug/Release全45 Appソース、0 errors/0 warnings（a32964af115d4894）。Release text=69384, data=2828, bss=12216 B（textは基準から72 B減）。仕様454 API参照、CAN監査19/19、CubeIDE設定22/22、Doxygen全sourceリンク、diff check合格。両ELFに旧C制御演算・SIL・試運転シンボルなし。
