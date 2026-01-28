# /* tests/mac_pdu/CMakeLists.txt */
# // Overview: Build script for the MAC PDU serialization/deserialization test suite.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_pdu_test)

# This test only needs to compile its own sources and the PDU module it is testing.
# It does not need the full MAC state machine or other layers.
target_sources(app PRIVATE
    src/main.c
    ../../mac/dect_mac_pdu.c
)

# Add include directories needed by the test and the PDU source file.
target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)