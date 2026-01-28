# SPDX-License-Identifier: Apache-2.0

# DECT NR+ Stack Library
zephyr_library()

# Main stack API source - this implements the functions declared in dect_stack_api.h
zephyr_library_sources(dect_stack_api.c)

# Core layer sources
zephyr_library_sources(dect_cdd.c)
zephyr_library_sources(dect_cvg.c)  
zephyr_library_sources(dect_dlc.c)

# Include directories for the library itself
zephyr_library_include_directories(include)

# CRITICAL: Export the public API headers to anyone who uses this library
# This makes dect_stack_api.h available throughout the Zephyr build system
target_include_directories(zephyr_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add MAC subdirectory (it will add to the same zephyr_library)
add_subdirectory(mac)




# # /* lib/dect_nrplus/CMakeLists.txt */
# # // Overview: Corrects the main library's build script to include and compile the source files from the 'mac' subdirectory into a single, unified library.
# # Copyright (c) 2024 Manulytica Ltd
# # SPDX-License-Identifier: Apache-2.0
# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_nrplus_lib)

# # Create a regular Zephyr library
# zephyr_library()

# # Add the library's public include directory to its INTERFACE.
# zephyr_library_include_directories(INTERFACE include)

# # Add the library's source files conditionally based on Kconfig settings.
# # This ensures that code for a disabled layer is not compiled.
# zephyr_library_sources(dect_stack_api.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_CDD_LAYER dect_cdd.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_CVG_LAYER dect_cvg.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_DLC_LAYER dect_dlc.c)

# # Include directories for the main stack
# zephyr_library_include_directories(include)
# zephyr_library_include_directories(.)

# # Process the mac subdirectory. This will execute its CMakeLists.txt,
# # which populates the 'dect_mac_sources' variable in this scope.
# add_subdirectory(mac)
# zephyr_library_named(dect_nrplus_lib)
# zephyr_library_sources(${dect_mac_sources})
# target_include_directories(dect_stack_api INTERFACE 
#     include
#     ${ZEPHYR_CURRENT_SOURCE_DIR}/include
# )


# # Link the library to its dependencies.
# # Using PUBLIC for nrf_modem_lib ensures that any application linking
# # our library also gets the modem library's headers and link dependency.
# zephyr_library_link_libraries(
#     zephyr_interface
#     PUBLIC
#     nrf_modem_lib
# )


# # /* lib/dect_nrplus/CMakeLists.txt */
# # // Overview: Corrects the main library's build script to include and compile the source files from the 'mac' subdirectory into a single, unified library.
# # Copyright (c) 2024 Manulytica Ltd
# # SPDX-License-Identifier: Apache-2.0
# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_nrplus_lib)

# # Create a regular Zephyr library
# zephyr_library()

# # Add the library's source files conditionally based on Kconfig settings.
# # This ensures that code for a disabled layer is not compiled.
# zephyr_library_sources(dect_stack_api.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_CDD_LAYER dect_cdd.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_CVG_LAYER dect_cvg.c)
# zephyr_library_sources_ifdef(CONFIG_DECT_DLC_LAYER dect_dlc.c)

# # Process the mac subdirectory. This will execute its CMakeLists.txt,
# # which populates the 'dect_mac_sources' variable in this scope.
# add_subdirectory(mac)

# # Add the source files from the mac subdirectory to this library.
# zephyr_library_sources(${dect_mac_sources})

# # Make this library's public 'include' directory available.
# # The 'mac/include' directory is added by the mac/CMakeLists.txt.
# zephyr_library_include_directories(include)

# # Link the library to the Zephyr interface to make Kconfig-enabled
# # APIs (like PSA Crypto) available to the library's sources.
# zephyr_library_link_libraries(zephyr_interface)

# # Link this library to nrf_modem_lib and make its headers public to consumers.
# target_link_libraries(dect_nrplus_lib PUBLIC nrf_modem_lib)



######################################################################

# # lib/dect_nrplus/CMakeLists.txt
# # Copyright (c) 2024 Manulytica Ltd
# # SPDX-License-Identifier: Apache-2.0
# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# # This command creates the `dect_nrplus_driver` library target
# add_library(dect_nrplus_driver STATIC)

# # Add the source files for the upper layers (DLC, CVG, L2)
# target_sources(dect_nrplus_driver PRIVATE
#     dect_cdd.c
#     dect_cvg.c
#     dect_dlc.c
# )
# if(CONFIG_NETWORKING)
#     target_sources(dect_nrplus_driver PRIVATE dect_nrplus_l2.c)
# endif()

# # Add the MAC subdirectory, which defines the `dect_mac_layer` target
# add_subdirectory(mac)

# # Make the public headers (dect_cvg.h, dect_dlc.h) available
# target_include_directories(dect_nrplus_driver PUBLIC include)

# # Link this library to the MAC layer library.
# # PUBLIC makes the MAC's include directories available to targets that link this library.
# target_link_libraries(dect_nrplus_driver PUBLIC dect_mac_layer zephyr_interface)

