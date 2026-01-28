# /* tests/mac_ass/CMakeLists.txt */
# // Overview: A correct, self-contained CMakeLists.txt for a Ztest application that compiles MAC sources directly.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_association_test)

# The "app" target is created automatically for a Zephyr application.

# Add all the necessary source files to be compiled for this test executable.
target_sources(app PRIVATE
    # The test's main entry point
    src/main.c

    # The mock PHY that replaces the real one
    ../../mac/mock_nrf_modem_dect_phy.c

    # --- MAC Layer Source Files ---
    # We compile the MAC layer from source as part of this test.
    # CRITICAL: We EXCLUDE dect_mac_phy_if.c, which calls the real modem functions.
    ../../mac/dect_mac.c
    ../../mac/dect_mac_api.c
    ../../mac/dect_mac_core.c
    ../../mac/dect_mac_data_path.c
    ../../mac/dect_mac_main_dispatcher.c
    ../../mac/dect_mac_mgmt.c
    ../../mac/dect_mac_nvs.c
    ../../mac/dect_mac_pdu.c
    ../../mac/dect_mac_phy_ctrl.c
    # ../../mac/dect_mac_phy_if.c # <-- EXCLUDED
    ../../mac/dect_mac_phy_tbs_tables.c
    ../../mac/dect_mac_random.c
    ../../mac/dect_mac_security.c
    ../../mac/dect_mac_shell.c
    ../../mac/dect_mac_sm_ft.c
    ../../mac/dect_mac_sm_pt.c
    ../../mac/dect_mac_timeline_utils.c

    # --- Upper Layer Stubs (Needed for includes) ---
    # The MAC layer includes headers from these files, so we must compile them
    # to satisfy the linker, even though the test doesn't use their functions.
    # ../../dect_cdd.c
    # ../../dect_cvg.c
    # ../../dect_dlc.c    
)

# Add the necessary include directories so the compiler can find headers
# like <mac/dect_mac_core.h> and <dect_cvg.h>.
# for all those pesky zehpur libraries
target_include_directories(app PRIVATE
    ${ZEPHYR_BASE}/include
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../mac/include
)



#########################################################

# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_association_test)

# # The "app" target is created automatically for a Zephyr application.

# # Add all the necessary source files to be compiled for this test executable.
# target_sources(app PRIVATE
#     # The test's main entry point
#     src/main.c

#     # The mock PHY that replaces the real one
#     # ../../lib/dect_nrplus/mac/mock_nrf_modem_dect_phy.c
#     ../mocks/mock_nrf_modem_dect_phy.c

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
#     # ../../mac/dect_mac_phy_if.c # <-- EXCLUDED
#     ../../mac/dect_mac_phy_tbs_tables.c
#     ../../mac/dect_mac_random.c
#     ../../mac/dect_mac_security.c
#     ../../mac/dect_mac_shell.c
#     ../../mac/dect_mac_sm_ft.c
#     ../../mac/dect_mac_sm_pt.c
#     ../../mac/dect_mac_timeline_utils.c
# )

# # Add the necessary include directories so the compiler can find headers
# # like <mac/dect_mac_core.h> and <dect_cvg.h>.
# target_include_directories(app PRIVATE
#     ../../include
#     ../../mac/include
# )
#####################################################################################

# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_association_test)

# # The "app" target is created automatically for a Zephyr application.

# # Add the test's main source file and the mock PHY implementation.
# target_sources(app PRIVATE
#     src/main.c
#     ../../mac/mock_nrf_modem_dect_phy.c
#     # ../mocks/mock_nrf_modem_dect_phy.c
    
# )

# # Add the necessary include directories so the compiler can find headers
# # like <mac/dect_mac_core.h> and <dect_cvg.h>.
# target_include_directories(app PRIVATE
#     ../../include
#     ../../include/mac
#     ../../include/mocks
#     ../../mac/include
# )
# # Link the test executable against the necessary libraries:
# # 1. dect_mac_layer: Provides all our MAC functions.
# # 2. zephyr_interface: Provides all Zephyr kernel functions (k_sleep, k_timer_init, z_impl_*, etc.).
# target_link_libraries(app PRIVATE dect_mac_layer zephyr_interface)


#######################################################################################

# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_association_test)

# # The "app" target is created automatically for a Zephyr application.

# # Add the test's main source file. This contains the ZTEST_SUITE macro.
# target_sources(app PRIVATE src/main.c)

# # target_sources(app PRIVATE src/main.c ../../lib/dect_nrplus/mac/mock_nrf_modem_dect_phy.c)

# # Add the mock PHY implementation. This provides the nrf_modem_* symbols
# # that the MAC layer's object files will need at link time.
# # target_sources(app PRIVATE ../../mac/mock_nrf_modem_dect_phy.c)
# target_sources(app PRIVATE ../mocks/mock_nrf_modem_dect_phy.c)
# zephyr_include_directories(${ZEPHYR_BASE}/include)

# # Link the test executable against our complete DECT NR+ library.
# # This is the crucial step that brings in all the compiled code from
# # dect_mac.c, dect_cvg.c, dect_dlc.c, etc.
# # target_link_libraries(app PRIVATE dect_nrplus_driver)

##############################################################################


# Link the test executable against our complete DECT NR+ library.
# This brings in all the compiled code from dect_mac.c, dect_cvg.c, etc.
# target_link_libraries(app PRIVATE dect_nrplus_driver)

# cmake_minimum_required(VERSION 3.20)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_association_test)

# # # Add the source file for the test itself.
# # # The `zephyr_test_execute` command below creates a target named "app".
# # # We add our sources to this "app" target.
# target_sources(app PRIVATE src/main.c)
# target_sources(app PRIVATE src/mock_nrf_modem_dect_phy.c)

# # # Add the mock PHY implementation to the build for this test.
# # # This ensures it's compiled and linked, replacing the real PHY.
# # target_sources(app PRIVATE ../../mac/mock_nrf_dect_phy.c)

# # Add the driver header directory
# zephyr_library_include_directories(../../../drivers/net/dect_nrplus)

# Add the driver header directory
# zephyr_library_include_directories(
#     ../../mac/include/mocks
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include/mac
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include/mocks
#     )


# # target_sources(app PRIVATE
# #     src/main.c
# #     ../../mac/mock_nrf_dect_phy.c
# # )

# # Link the test executable against the main DECT NR+ library.
# # This makes all the functions from dect_mac.c, dect_dlc.c, etc., available to the test.
# # target_link_libraries(app PRIVATE dect_nrplus_driver)

# # Define the test and specify the board to run it on.
# # zephyr_test_execute(test BOARD native_sim)
