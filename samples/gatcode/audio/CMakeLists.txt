# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio)

target_sources(app PRIVATE
    src/main.c
    ../io_coder/src/io_coder.c
    ../hw_info/src/hw_info.c
)

zephyr_library_include_directories(../io_coder/include)
zephyr_library_include_directories(../hw_info/include)
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
