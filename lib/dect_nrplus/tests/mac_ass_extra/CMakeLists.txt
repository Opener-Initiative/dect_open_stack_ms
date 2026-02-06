# /* tests/mac_ass_extra/CMakeLists.txt */
# // Overview: Corrects the test build script by removing all library source files. The test now only compiles its own main function and the mock PHY, and relies on Kconfig to link the main `dect_nrplus_driver` library.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_association_extra_test)

# The "app" target is created automatically for a Zephyr application.

# Add only the source files specific to this test:
# - The test's main entry point (containing the Ztest suite)
# - The mock PHY that replaces the real one's functions at link time.
target_sources(app PRIVATE
    src/main.c
    # The mock PHY that replaces the real one
    ../../mac/mock_nrf_modem_dect_phy.c
    # The virtual timer simulation clock
    ../../tests/utils/virtual_timer.c
    # The new shared test harness helper
    ../../tests/utils/test_harness_helpers.c    
    )

    
# Add the necessary include directories so the compiler can find headers
# like <mac/dect_mac_core.h> and <mocks/mock_nrf_modem_dect_phy.h>.
target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)

# NOTE: target_link_libraries is NOT needed here.
# Because prj.conf sets CONFIG_DECT_NRPLUS_STACK=y, the build system
# automatically finds and links the 'dect_nrplus_driver' library.
