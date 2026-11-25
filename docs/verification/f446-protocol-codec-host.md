# CAN2 v1共通コーデックのホスト検証

対象：PROTO-PWR-001、PROTO-REMOTE-001、T-PROTO-11。2026-11-25。承認履歴 Issue #27。

## 実装範囲

- ProtocolId.hpp/.cpp：フィールド検査、29bit符号化と分解、35件の固定許可リスト、IDE/DLC検査。DJIの0x1FF～0x208、0x700は自作IDの数値と上位11bitの双方で禁止する。
- ProtocolPayload.hpp/.cpp：DLC内の指定位置をBig endianで読み書きする。uint16、uint32、IEEE754 binary32を扱い、NaN/Infと領域外を拒否。書込み失敗時は変更しない。
- 共通コーデックは許可された形式を判定するだけで、ロボット状態、物理量の上限、送信元の認証、送信許可を決定しない。HALや実機送信経路には未接続。

## 試験と結果

| 試験 | 結果 |
|---|---|
| CMake構成・ホストビルド | PASS |
| CTest | 30/30 PASS |
| tools/run_host_tests.py 通常・UBSan | 全件PASS（job 4f423a205b804ae8、exit 0） |
| CAN仕様静的監査・変異試験 | 19/19 PASS |
| CubeIDEメタデータ回帰 | 22/22 PASS |

35件の独立golden ID/DLC、予約領域の全標準ID走査、IDE/DLC、未登録ID、フィールド不正、Big endianの既知バイト列、境界offset、非有限値、書込み失敗時不変を検証した。静的監査は承認済み仕様表とgoldenを照合し、予約ID・方向・重複・周期の変異を検出する。旧C620/RS06の監査も維持した。

## 残る作業

個別電源・操作・診断コーデック、CAN2受信配送・フィルタ、配信予算、Pi側実装、旧FWとの互換性確認。未決定フラグ・物理限界を補完していない。ARMクロスビルド、実機CAN、物理動作は未検証。T-PROTO-11は完了扱いにしない。
