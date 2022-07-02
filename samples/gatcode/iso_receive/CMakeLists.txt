
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(iso_receive)

target_sources(app PRIVATE
    src/main.c
    src/sync_timer.c
    src/ble_hci_vsc.c
    ../io_coder/src/io_coder.c
)

zephyr_library_include_directories(include)
zephyr_library_include_directories(../io_coder/include)