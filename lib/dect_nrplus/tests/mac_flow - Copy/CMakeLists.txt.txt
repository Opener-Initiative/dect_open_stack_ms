#
# Copyright (c) 2025 Manulytica Ltd
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_nrplus_ztest_mac_flow)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE src/main.c)