# /* tests/mac_ass/CMakeLists.txt */
# // Overview: Corrects the test build script by removing all library source files. The test now only compiles its own main function and the mock PHY, and relies on Kconfig to link the main `dect_nrplus_driver` library.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_association_test)

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


#########################################################################################

# # /* tests/mac_ass/CMakeLists.txt */
# # // Overview: A correct, self-contained CMakeLists.txt for a Ztest application that compiles MAC sources directly.

# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_association_test)

# # The "app" target is created automatically for a Zephyr application.

# # Add all the necessary source files to be compiled for this test executable.
# target_sources(app PRIVATE
#     # The test's main entry point
#     src/main.c

#     # The mock PHY that replaces the real one
#     ../../mac/mock_nrf_modem_dect_phy.c

#     # --- MAC Layer Source Files ---
#     # We compile the MAC layer from source as part of this test.
#     # CRITICAL: We EXCLUDE dect_mac_phy_if.c, which calls the real modem functions.
#     ../../mac/dect_mac.c
#     ../../mac/dect_mac_api.c
#     ../../mac/dect_mac_core.c
#     ../../mac/dect_mac_data_path.c
#     ../../mac/dect_mac_main_dispatcher.c
#     ../../mac/dect_mac_mgmt.c
#     ../../mac/dect_mac_nvs.c
#     ../../mac/dect_mac_pdu.c
#     ../../mac/dect_mac_phy_ctrl.c
#     ../../mac/dect_mac_phy_if.c # <-- EXCLUDED
#     ../../mac/dect_mac_phy_tbs_tables.c
#     ../../mac/dect_mac_random.c
#     ../../mac/dect_mac_security.c
#     ../../mac/dect_mac_shell.c
#     ../../mac/dect_mac_sm_ft.c
#     ../../mac/dect_mac_sm_pt.c
#     ../../mac/dect_mac_timeline_utils.c

#     # --- Upper Layer Stubs (Needed for includes) ---
#     # The MAC layer includes headers from these files, so we must compile them
#     # to satisfy the linker, even though the test doesn't use their functions.
#     # ../../dect_cdd.c
#     # ../../dect_cvg.c
#     # ../../dect_dlc.c    
# )

# # Add the necessary include directories so the compiler can find headers
# # like <mac/dect_mac_core.h> and <dect_cvg.h>.
# # for all those pesky zehpur libraries
# target_include_directories(app PRIVATE
#     ${ZEPHYR_BASE}/include
#     ../../include
#     ../../include/mac
#     ../../include/mocks
#     ../../mac/include
# )



# #########################################################
