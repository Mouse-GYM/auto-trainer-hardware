# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: Apache-2.0
#
# zephyr/share/sysbuild/template/CMakeLists.txt, extended to pick up a per-board
# sysbuild fragment.

# Sysbuild has no boards/ lookup of its own: sysbuild.conf applies to every board.
# A board that has to differ in which images get built (nucleo_g474re, whose 512K of
# internal flash cannot hold two MCUboot slots) puts that in sysbuild_<board>.conf.
if(EXISTS "${APP_DIR}/sysbuild_${BOARD}.conf")
  list(APPEND SB_EXTRA_CONF_FILE "${APP_DIR}/sysbuild_${BOARD}.conf")
endif()

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)
