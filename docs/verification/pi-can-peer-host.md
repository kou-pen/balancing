# Pi CAN受信境界のホスト検証（2026-11-27）

対象：BOARD-PI-001、PROTO-PWR-001、PROTO-REMOTE-001、Issue #110。PI-01〜07決定後、初期のROS非依存実装。

| ファイル | 内容 |
|---|---|
| [CanPeer.hpp](../../raspberry_pi/src/balancing_can/include/balancing/pi/CanPeer.hpp) | Pi向け受信16型・送信候補の公開API |
| [CanPeer.cpp](../../raspberry_pi/src/balancing_can/src/CanPeer.cpp) | 既存Codecによる型検査、宛先・送信元・対象範囲の制限 |
| [CMakeLists.txt](../../raspberry_pi/src/balancing_can/CMakeLists.txt) | ament CMakeの初期パッケージ、既存Codecソース共用 |
| [test_pi_can_peer.cpp](../../tests/test_pi_can_peer.cpp) | 既知ID、16型、拒否条件の試験 |

PiはF446/F303の状態報告、Pi宛て操作結果、他基板の非常停止、既存物理観測、FAULTの3部分を受信する。標準ID（C620等）、RTR、エラーフレーム、異常DLC、未知値、他機器宛て応答、自機非常停止の反射、自機要求、保留中の設定/購読/周期診断の受信を拒否する。電源操作と非常停止の送信候補はPiの送信元に固定。既存RemoteControlCodecはそのまま再利用する。16型はデータ境界であり、FAULTの3部分再構成・操作応答照合・鮮度管理を実装したわけではない。

通常・UBSan単体試験PASS。全CTest 88/88 PASS（job 6da2c698c09843d4）。仕様整合13仕様/7 TODO/126 md/573 API参照・Doxygen生成とPi固有ソースリンク・差分検査PASS（既知の日本語翻訳警告のみ）。固定仕様ID検査へBOARD-PI-001を追加し、Piのsrcも検査対象に拡張した。実CANを開かず、送信候補は物理送信しない。製品STM32ソース・CAN仕様変更なし。

## 環境と未検証事項

Balancing VM Ubuntu 24.04.4 x86_64、G++、空き28Gを確認。/opt/ros、ros2、colcon、docker、podmanなし、ROS aptキャッシュにも該当パッケージなし。Jazzyのrmw_zenohブランチをGitで確認（5c4e4441e8944874c604cc26ac804bba8972fe7e）し、同READMEのrmw_zenohd・ルータ設定を確認。Pi実機OSは未確認。ROS未導入のためament/colconビルド、ROS/Zenoh/vcanの動作はNOT_RUN。既存VMのOS/apt/ネットワーク設定を変更していない。

package.xmlのmaintainerメール・licenseは未確定の暫定メタデータ（公開配布用ではない）。CMakeはリポジトリの既存Codecを相対パスで参照するため、Pi側へ配置するときはリポジトリ全体が必要。今回選んだ初期構成であり、将来共通Codecを移動する場合は別PRで行う。

次は時刻付き状態管理・要求照合、その後ROSメッセージとCANノード接続、Web UI、rosbag2、Launch・vcan結合試験。CubeMXと実機検証は保留。運転操作・電源保護の安全条件は変更しない。
