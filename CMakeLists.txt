cmake_minimum_required(VERSION 3.16)
project(balancing_core LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(balancing_core STATIC
    firmware/core/src/balancing_core.c
    firmware/core/src/balancing_motor_can.c
    firmware/core/src/balancing_motor_bench.c
)
target_include_directories(balancing_core PUBLIC
    firmware/core/include
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
# F446 HAL adapter is host-compiled with an explicit test-only HAL shim.
# This does not replace a real STM32CubeMX build or an MCU hardware test.
add_executable(balancing_motor_hal_port_tests
    tests/test_motor_hal_port.c
    firmware/f446/src/balancing_f446_motor_bench_port.c
)
target_include_directories(balancing_motor_hal_port_tests PRIVATE
    tests/stubs
    firmware/f446/include
)
target_link_libraries(balancing_motor_hal_port_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_hal_port_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_hal_port_tests COMMAND balancing_motor_hal_port_tests)
# C++ entry bridge is kept outside CubeMX-owned Core/Src. Host test only.
add_executable(balancing_cpp_app_tests
    tests/test_cpp_app.cpp
    firmware/f446/balancing_f446/App/Src/application.cpp
)
target_include_directories(balancing_cpp_app_tests PRIVATE
    firmware/f446/balancing_f446/App/Inc
)
target_compile_options(balancing_cpp_app_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_cpp_app_tests COMMAND balancing_cpp_app_tests)
