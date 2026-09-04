# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rp2350_zephyr_stepper)

target_sources(app PRIVATE
    src/main.c
    src/core1_launch.c
    src/stepper_shell.c
    src/core1_blob.c
    src/mdns_service.c
    src/eth_id.c
    src/bootloader_shell.c
    drivers/eth_lan9250/eth_lan9250.c
)

include(cmake/core1.cmake)
