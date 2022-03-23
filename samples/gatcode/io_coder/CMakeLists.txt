# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(io_coder)

target_sources(app PRIVATE
    src/main.c
    src/io_coder.c
)

zephyr_library_include_directories(include)