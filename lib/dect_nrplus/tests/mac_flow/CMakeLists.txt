#
# Copyright (c) 2025 Manulytica Ltd
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mac_flow)

# Add test source files
target_sources(app PRIVATE
    src/main.c
    ../../mac/mock_nrf_modem_dect_phy.c
    ../../tests/utils/virtual_timer.c
    ../../tests/utils/test_harness_helpers.c
)

# Include directories for mocks and other dependencies
target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)
