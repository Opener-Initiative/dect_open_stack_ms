cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_phy_ctrl_test)

target_sources(app PRIVATE
    src/main.c
    # Mock PHY that replaces the real one
    ../../mac/mock_nrf_modem_dect_phy.c
    # The virtual timer simulation clock
    ../../tests/utils/virtual_timer.c
    # The new shared test harness helper
    ../../tests/utils/test_harness_helpers.c 
        
    # # The actual code under test
    # ../../mac/dect_mac_phy_ctrl.c
    # ../../mac/dect_mac_phy_tbs_tables.c
    # ../../mac/dect_mac_timeline_utils.c
    # # Context and other needed stubs/sources
    # ../../mac/dect_mac.c
)

target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)
