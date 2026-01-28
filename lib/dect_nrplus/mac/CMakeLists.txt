# SPDX-License-Identifier: Apache-2.0

# All these sources become part of the same dect_nrplus library
zephyr_library_sources(dect_mac.c)
zephyr_library_sources(dect_mac_api.c)
zephyr_library_sources(dect_mac_core.c)
zephyr_library_sources(dect_mac_data_path.c)
zephyr_library_sources(dect_mac_main_dispatcher.c)
zephyr_library_sources(dect_mac_mgmt.c)
zephyr_library_sources(dect_mac_nvs.c)
zephyr_library_sources(dect_mac_pdu.c)
zephyr_library_sources(dect_mac_phy_ctrl.c)
zephyr_library_sources(dect_mac_phy_if.c)
zephyr_library_sources(dect_mac_phy_tbs_tables.c)
zephyr_library_sources(dect_mac_random.c)
zephyr_library_sources(dect_mac_security.c)
zephyr_library_sources(dect_mac_shell.c)
zephyr_library_sources(dect_mac_sm_ft.c)
zephyr_library_sources(dect_mac_sm_pt.c)
zephyr_library_sources(dect_mac_timeline_utils.c)

# Include directories for MAC layer (internal use)
zephyr_library_include_directories(include)

# Export MAC public headers if any
target_include_directories(zephyr_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)














# # /* lib/dect_nrplus/mac/CMakeLists.txt */
# # // Overview: Corrects the MAC build script to export its source files to the parent library instead of creating its own. This resolves the cross-target linking issue.
# cmake_minimum_required(VERSION 3.20)

# # Create a list of all source files in this directory.
# # Using CMAKE_CURRENT_LIST_DIR ensures the paths are absolute.
# set(dect_mac_sources
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_api.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_core.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_data_path.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_main_dispatcher.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_mgmt.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_nvs.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_pdu.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_phy_ctrl.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_phy_if.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_phy_tbs_tables.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_random.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_security.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_shell.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_sm_ft.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_sm_pt.c
#     ${CMAKE_CURRENT_LIST_DIR}/dect_mac_timeline_utils.c
# )

# # Make this list of sources available to the parent CMakeLists.txt
# set(dect_mac_sources ${dect_mac_sources} PARENT_SCOPE)

# # Add this subdirectory's 'include' path to the parent's library target.
# # This allows includes like <mac/dect_mac_core.h> from the parent or other modules.
# zephyr_library_include_directories(include)

# target_include_directories(dect_nrplus_lib INTERFACE
#     ${CMAKE_CURRENT_LIST_DIR}/include
# )



# # /* lib/dect_nrplus/mac/CMakeLists.txt */
# # // Overview: Use the new test.cmake file to define the library sources.
# cmake_minimum_required(VERSION 3.20)

# # Define a library target for the MAC layer
# add_library(dect_mac_layer STATIC)

# # Add all MAC source files to this new library target
# target_sources(dect_mac_layer PRIVATE
#     dect_mac.c
#     dect_mac_api.c
#     dect_mac_core.c
#     dect_mac_data_path.c
#     dect_mac_main_dispatcher.c
#     dect_mac_mgmt.c
#     dect_mac_nvs.c
#     dect_mac_pdu.c
#     dect_mac_phy_ctrl.c
#     dect_mac_phy_if.c
#     dect_mac_phy_tbs_tables.c
#     dect_mac_random.c
#     dect_mac_security.c
#     dect_mac_shell.c
#     dect_mac_sm_ft.c
#     dect_mac_sm_pt.c
#     dect_mac_timeline_utils.c
# )

# # Make the MAC's public headers available to any target that links against it.
# # This allows includes like <mac/dect_mac_core.h>.
# target_include_directories(dect_mac_layer PUBLIC
#     include
# )

# # Also add the parent directory's include path as a PRIVATE dependency,
# # so that MAC source files can find headers like <dect_dlc.h>.
# target_include_directories(dect_mac_layer PRIVATE
#     ../include
# )

# # Link the MAC library to Zephyr's kernel library
# target_link_libraries(dect_mac_layer PUBLIC zephyr_interface)
