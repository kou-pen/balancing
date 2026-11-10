# CAN通信契約の静的照合（Issue #6）

**記録日:** 2026-11-10  
**対象:** PROTO-RS06-001 / PROTO-C620-001 / PROTO-PWR-001 / PROTO-REMOTE-001  
**判定:** ホストの静的照合と既知フレーム試験。実機との一致や物理停止は未検証。

## 照合結果

| バス・種別 | CAN IDまたはType | DLC | 方向と役割 | 証拠 |
|---|---|---|---|---|
| CAN1・29bit拡張 | Type 1 / 3 / 4 | 8 | F446→RS06：トルク／有効化／停止 | RS06通信仕様、C11 codec、既知フレーム |
| CAN1・29bit拡張 | Type 2 / 24 | 8 | RS06→F446：状態報告（Type 24は実機不明） | RS06通信仕様、C11 codec |
| CAN1・29bit拡張 | Type 24 | 8 | F446→RS06：定期報告切替（送信バイト未確定） | RS06通信仕様 |
| CAN2・標準11bit | 0x200 | 8 | F446→C620：ID1〜4共通電流指令、500 Hz | C620通信仕様、総覧、C11 codec |
| CAN2・標準11bit | 0x201 / 0x202 | 8 | 左右C620→F446：エンコーダ・rpm・電流生値・温度 | C620通信仕様、総覧、C11 codec |
| CAN2・標準11bit | 0x1FF | 8 | C620 ID5〜8向け、本機未使用 | C620通信仕様、総覧 |
| CAN2・形式未確定 | 未確定 | 未確定 | F303電源、Pi遠隔操作、非常停止の論理要求 | 電源・遠隔操作通信仕様 |

固定IDの監査対象はCAN2の4件です。**確認できた固定ID同士に、同一バス・同一フレーム形式での重複はありません。** RS06のTypeは拡張ID全体ではなく上位5bitの種別で、宛先・ホストIDなどはフレームごとに異なります。拡張IDの動的割当や、今後決定するF303/Pi IDとの競合が解消したわけではありません。

## 再現可能な静的検査

静的検証ツールは [check_can_contract.py](../../tools/check_can_contract.py) です。仕様書を直接読んで検証し、CAN IDの複製レジストリは持ちません。CAN IDの正本は [RS06](../specs/protocols/rs06_can1.md)、[C620](../specs/protocols/c620_can2.md)、[F303](../specs/protocols/power_can2.md)、[遠隔操作](../specs/protocols/remote_can2.md) の各仕様書です。

```sh
python3 tools/check_can_contract.py
python3 -m unittest discover -s tests -p test_can_contract.py -v
python3 tools/run_host_tests.py
python3 tools/validate_specs.py
```

**2026-11-10ホスト検証：** 静的照合PASS（CAN2の固定ID 4件・RS06の方向別Type 6件）、不整合検出テスト14/14 PASS。既存C11既知フレームの通常/UBSan、CTest 5/5、仕様リンク検査、DoxygenもPASS。

検査はCAN2の固定ID重複・標準11bit範囲、RS06の拡張29bit宣言・Typeの5bit範囲、DLC=8、RS06 Type/方向、仕様表と総覧とC11 codec中の既知形式を照合します。異常なID、DLC、旧文書との不一致、未確定IDへの仮の数値割当は回帰試験で拒否します。C11の既知フレーム実行試験は通常・UBSanの両方で行い、Type24を含めても現物の応答保証とはみなしません。

## 未確定・実機確認待ち

| 未確定項目 | 必要な確認 |
|---|---|
| RS06の主機ID、4台の実ID・FW版 | CANログで宛先・応答ID、モード・故障フラグを確認。既知フレームのホストID FDはテスト用 |
| RS06 Type24応答と送信DATA | Type2とのフィールド差、送信例の固定バイトと可変バイトを実測 |
| RS06の速度・トルクの換算 | 2025-11版の±50 rad/sと2026-04版の±15 rad/sの記載差を使用FWで照合 |
| C620実電流・左右ID・電源条件 | ±20 Aはプロトコル表現範囲。換算・許容電流・ID・24 V供給を実機で確認 |
| F303 / Pi のCAN ID・形式・DLC・payload | **F303側FWと上位Pi送受信ソースは現リポジトリに未収録。** ソース取得後に競合と権限を再照合 |
| 実時間性・通信故障と停止 | 250/500 Hz実送信、送信完了、受信鮮度、バス負荷、物理E-STOPを実機で確認 |

**判定の限界：** 静的監査は全ソースの全CAN送信経路を構文解析するものではありません。現行C11 codecの既知パターンとメーカー仕様の一致を検査するだけです。実CANフィルタ、Tx IRQ、停止時の実フレーム送達は検証していません。
