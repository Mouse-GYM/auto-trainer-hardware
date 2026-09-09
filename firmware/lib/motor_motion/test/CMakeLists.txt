cmake_minimum_required(VERSION 3.20.0)

project(motor_motion_test)

set(CMAKE_C_FLAGS "-pedantic -Wall -Wextra")

add_executable(motor_motion_test)

add_compile_definitions(BENCH_TEST)

target_sources(motor_motion_test
    PRIVATE
        ../motor_math.c
        motor_math_test.c
)

target_include_directories(motor_motion_test
    PRIVATE
        "./"
        "../"
)

target_link_libraries(motor_motion_test
    PRIVATE
        m
)
