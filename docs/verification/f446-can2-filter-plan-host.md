# CAN2受信フィルタのホスト検証

対象：BOARD-F446-001、PROTO-PWR-001、PROTO-REMOTE-001、T-PROTO-11。

## 構成

CAN1とCAN2の共有28バンクを14/14に分割する。CAN1 bank0/1は従来の32bit Mask（RS06 Type 2/24）。CAN2は32bit List（1バンクに2個のID）。

| Bank | ID 1 | ID 2 | 現行起動 |
|---|---|---|---|
| 14 | 標準0x201 | 標準0x202 | 有効 |
| 15 | 0x0840BC03 | 0x08408404 | 無効 |
| 16 | 0x0004BC01 | 0x0004FC01 | 無効 |
| 17 | 0x0880C401 | 0x0880C402 | 無効 |
| 18 | 0x0480C403 | 0x0880C404 | 無効 |
| 19 | 0x0880C405 | 0x0CC0C480 | 無効 |
| 20 | 0x0CC0C481 | 0x0D00C401 | 無効 |
| 21–27 | 未使用 | 未使用 | 無効 |

全自作IDは拡張データフレーム。標準IDはID<<21、拡張IDは(ID<<3)|IDE。RTR=0。HALの32bit ListではFilterMaskIdHigh/Lowはマスクではなく第2識別子のFR2へ格納する（リポジトリ内HAL実装を照合）。全フィルタはFIFO0に配送。

`Can2FilterPlan.hpp/.cpp` の `makeCan2FilterPlan(bool includeProtocol)` はHAL非依存の設定値生成。falseで1バンク、trueで7バンク。 `f446_can1_runtime.cpp` はfalse固定で起動し、未使用のCAN2バンクを明示的に無効化する。CAN1の割当は変更しない。自作通信を実際に有効化するには上位受信処理との結合が必要。

## 検証

- CTest 33/33 PASS。通常・UBSan全件PASS（job 9a2b5ec9bf224dc9、exit 0）。CAN静的監査19/19、CubeIDE設定22/22、validate_specs、sync_cubeide_cpp、Doxygen生成、diff check PASS。Doxygenは既知の日本語翻訳警告のみ。
- 標準全2048 ID、35拡張IDのうち12受信ID、RTR、IDE、各IDの1bit変異、予約ID、重複の有無をホスト上の比較模型で検査。
- HALスタブでbank14のFR1/FR2、bank15〜27の無効化、CAN1 bank0/1の維持、bank14/15/27設定失敗時の起動拒否を確認。
- 実機フィルタ・ARMクロスビルド未検証。trueの設定は生成試験のみでHALへ適用しない。現行C620用フィルタ方式は変更したが受信対象は同じ。DLCと内容は引き続きRouter/Codecで検査する。
- T-PROTO-11は未完了。サービス結合と通信負荷の実測は別工程。
