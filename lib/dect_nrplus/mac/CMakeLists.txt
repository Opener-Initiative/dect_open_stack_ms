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

