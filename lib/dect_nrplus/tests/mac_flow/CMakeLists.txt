#
# Copyright (c) 2025 Manulytica Ltd
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_nrplus_shell)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE src/main.c)



# cmake_minimum_required(VERSION 3.21)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# project(mac_flow)


# # Enable Ztest for unit testing
# set(ENABLE_ZTEST TRUE)

# # Add test source files
# target_sources(app PRIVATE
#     src/main.c  # Main test file or entry point
#     src/mocks/mock_nrf_dect_phy.c
#     # Add other test source files as needed
# )

# # Include directories for mocks and other dependencies
# target_include_directories(app PRIVATE
#     src
#     ../mocks
#     ${ZEPHYR_BASE}/include
#     # Add other include paths as needed
# )

# # Enable test target
# zephyr_test_register()
