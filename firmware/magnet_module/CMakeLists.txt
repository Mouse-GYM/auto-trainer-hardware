cmake_minimum_required(VERSION 3.20.0)

# Generate a version file based on the git tag & hash at project configuration time
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
execute_process(
        COMMAND ${Python3_EXECUTABLE} ../gen_version.py
        OUTPUT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/VERSION
        ERROR_VARIABLE VERSION_GENERATION_ERROR
        RESULT_VARIABLE VERSION_GENERATION_RESULT
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

if(NOT VERSION_GENERATION_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to generate firmware VERSION:\n${VERSION_GENERATION_ERROR}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(magnet_module_firmware)

set(SRCS
        src/main.c
)

target_sources(app PRIVATE ${SRCS})
target_include_directories(app PRIVATE .)
