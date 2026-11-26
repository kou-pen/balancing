# F446 AppのC++移行統合検証

2026-11-26、master 8c5b57aを基準に#84 8b01d0cと#85 d725a21を検証専用ブランチへ統合。検証対象dcb79a0。masterおよび両PR本体は未変更。

Appの総ファイル数124→92（32減）、翻訳単位43（全.cpp）、.cは0本。C ABIのブリッジヘッダー、CubeMX Core、HAL/CMSIS、ST USB Host MiddlewareはC互換を維持する。C++の数学APIは固定長配列と参照引数を用いる。SIL・独立試運転は通常ELFにリンクしない。

競合4ファイル（CMakeLists、host runner、public-api、verification index）を解消。旧Cのコンパイル/リンクを両方廃止。旧API/ファイルへの現行コード・テスト・ビルド参照なし。

CTest48/48（34a13dd8702a47c8）、CubeIDE Debug/Release各43 Appソース0 errors/0 warnings（4f34a79141754eb4）、Release text69208/data2828/bss12232 B。仕様454 API参照、CubeIDE設定、Doxygen全sourceリンク、diff check合格。両ELFに旧C関数/SIL/試運転シンボルなし。実機未操作。残件は試運転専用起動/IRQ排他、実機入出力、物理モデル・駆動ゲイン・期限、IWDG、F303/PC連携。

通常/UBSan全件PASS（553ebb3a633046ca）、CAN監査19/19、CubeIDE設定22/22。旧数学との直接比較は#84の80000件、#85の82000件が各通常/UBSanで一致。
