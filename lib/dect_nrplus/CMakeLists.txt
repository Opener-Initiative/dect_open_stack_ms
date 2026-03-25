#
# Copyright (c) 2026 Manulytica Ltd
#

# SPDX-License-Identifier: Apache-2.0

# DECT NR+ Stack Library
zephyr_library()

# Main stack API source - this implements the functions declared in dect_stack_api.h
zephyr_library_sources(dect_stack_api.c)

# Core layer sources
zephyr_library_sources(dect_cdd.c)
zephyr_library_sources_ifdef(CONFIG_DECT_CVG_LAYER dect_cvg.c)  
zephyr_library_sources_ifdef(CONFIG_DECT_DLC_LAYER dect_dlc.c)
zephyr_library_sources_ifdef(CONFIG_DECT_SECURITY_ENABLE dect_security_crypto.c)

# Forcefully expose the vanilla Mbed TLS PSA headers to the entire build 
# when Nordic's proprietary security wrapper is disabled (e.g., native_sim).
if(NOT CONFIG_NRF_SECURITY)
    zephyr_include_directories(${ZEPHYR_BASE}/../modules/crypto/mbedtls/include)
endif()

# Include directories for the library itself
zephyr_library_include_directories(include)

# CRITICAL: Export the public API headers to anyone who uses this library
# This makes dect_stack_api.h available throughout the Zephyr build system
target_include_directories(zephyr_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add MAC subdirectory (it will add to the same zephyr_library)
add_subdirectory(mac)

# # Link MbedTLS to this library to get psa/crypto.h headers
# # This is needed for native_sim where NRF_SECURITY is disabled
# # Link MbedTLS to this library to get psa/crypto.h headers
# if(CONFIG_MBEDTLS AND NOT CONFIG_NRF_SECURITY)
#     zephyr_library_link_libraries(mbedtls)
# endif()