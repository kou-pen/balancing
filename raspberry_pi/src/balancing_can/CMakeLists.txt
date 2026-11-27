cmake_minimum_required(VERSION 3.16)
project(balancing_can LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
# リポジトリ内の既存HAL非依存Codecを共用。ワイヤ実装を複製しない。
set(SHARED_APP "${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/f446/balancing_f446/App")
add_library(balancing_pi_can_peer STATIC
    src/CanPeer.cpp
    ${SHARED_APP}/Src/balancing/communication/can/PowerBoardCodec.cpp
    ${SHARED_APP}/Src/balancing/communication/can/RemoteControlCodec.cpp
    ${SHARED_APP}/Src/balancing/communication/can/DiagnosticsCodec.cpp
    ${SHARED_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${SHARED_APP}/Src/balancing/communication/can/ProtocolPayload.cpp)
target_include_directories(balancing_pi_can_peer PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include" "${SHARED_APP}/Inc")
target_compile_features(balancing_pi_can_peer PUBLIC cxx_std_17)
target_compile_options(balancing_pi_can_peer PRIVATE -Wall -Wextra -Wpedantic -Werror)
# ノード・ROSインターフェースの接続は次段階。中間ライブラリはパッケージ内部で使用する。
ament_package()
