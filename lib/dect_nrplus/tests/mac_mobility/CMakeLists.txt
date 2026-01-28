# /* lib/dect_nrplus/tests/mac_mobility/CMakeLists.txt */
# // Overview: Build script for the new MAC mobility test suite.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_mobility_test)

target_sources(app PRIVATE
    src/main.c
    # The mock PHY that replaces the real one
    ../../mac/mock_nrf_modem_dect_phy.c
    # The shared test harness helper
    ../../tests/utils/test_harness_helpers.c
)

target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)

# target_link_libraries(app PRIVATE zephyr_interface)