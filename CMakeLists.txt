cmake_minimum_required(VERSION 3.16)
project(balancing_core LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(F446_APP firmware/f446/balancing_f446/App)

add_library(balancing_core STATIC
    ${F446_APP}/Src/balancing/control/controller/balancing_core.c
    ${F446_APP}/Src/balancing/communication/balancing_motor_can.c
    ${F446_APP}/Src/balancing/app/balancing_motor_bench.c
    ${F446_APP}/Src/balancing/communication/balancing_tx_scheduler.c
    ${F446_APP}/Src/balancing/control/estimator/balancing_leg_kinematics.c
)
target_include_directories(balancing_core PUBLIC
    ${F446_APP}/Inc
)
target_compile_options(balancing_core PRIVATE
    -Wall -Wextra -Wpedantic
)

enable_testing()
add_executable(balancing_core_tests tests/test_core.c)
target_link_libraries(balancing_core_tests PRIVATE balancing_core m)
add_test(NAME balancing_core_tests COMMAND balancing_core_tests)
add_executable(balancing_motor_can_tests tests/test_motor_can.c)
target_link_libraries(balancing_motor_can_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_can_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_can_tests COMMAND balancing_motor_can_tests)

# HAL-independent issue #5 scheduler: 3 simulated CAN1 mailboxes + virtual time.
add_executable(balancing_tx_scheduler_tests tests/test_tx_scheduler.c)
target_link_libraries(balancing_tx_scheduler_tests PRIVATE balancing_core m)
target_compile_options(balancing_tx_scheduler_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_tx_scheduler_tests COMMAND balancing_tx_scheduler_tests)

# CAD-unverified mathematical 2-input closed-chain fixture (Issue #7).
add_executable(balancing_leg_kinematics_tests tests/test_leg_kinematics.c)
target_link_libraries(balancing_leg_kinematics_tests PRIVATE balancing_core m)
target_compile_options(balancing_leg_kinematics_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_leg_kinematics_tests COMMAND balancing_leg_kinematics_tests)

# F446 HAL adapter is host-compiled with an explicit test-only HAL shim.
# This does not replace a real STM32CubeMX build or an MCU hardware test.
add_executable(balancing_motor_hal_port_tests
    tests/test_motor_hal_port.c
    ${F446_APP}/Src/balancing/device/balancing_f446_motor_bench_port.c
)
target_include_directories(balancing_motor_hal_port_tests PRIVATE
    tests/stubs
    ${F446_APP}/Inc
)
target_link_libraries(balancing_motor_hal_port_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_hal_port_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_hal_port_tests COMMAND balancing_motor_hal_port_tests)
# C++ entry bridge is kept outside CubeMX-owned Core/Src. Host test only.
add_executable(balancing_cpp_app_tests
    tests/test_cpp_app.cpp
    ${F446_APP}/Src/balancing/app/application.cpp
)
target_include_directories(balancing_cpp_app_tests PRIVATE
    ${F446_APP}/Inc
)
target_compile_options(balancing_cpp_app_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_cpp_app_tests COMMAND balancing_cpp_app_tests)