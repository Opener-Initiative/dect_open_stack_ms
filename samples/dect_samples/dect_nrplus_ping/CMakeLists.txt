# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dect_nrplus_ping)

# MANUALLY add the DECT module
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/dect_nrplus)
message(STATUS "Manually adding DECT module: ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/dect_nrplus")

# Application source
target_sources(app PRIVATE src/main.c)

# That's it! The DECT library is automatically available because:
# 1. It's a Zephyr module (module.yml)
# 2. It exports its headers via target_include_directories(zephyr_interface)
# 3. Zephyr automatically links all enabled modules





# # /* samples/dect_nrplus_ping/CMakeLists.txt */
# # // Overview: A minimal and correct build script for the ping sample. It only compiles main.c and relies on Kconfig to link the DECT NR+ library.

# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_nrplus_ping_sample)

# # The "app" target is created automatically.
# target_sources(app PRIVATE src/main.c)

# # target_include_directories(app PRIVATE
# # 	../../include
# # 	../../include/mac
# # 	../../include/mocks
# # 	../../tests
# # )

# # The `dect_nrplus_lib` is found and linked automatically by the Zephyr
# # build system because CONFIG_DECT_NRPLUS_STACK=y is set in prj.conf.
# # We only need to link zephyr_interface to satisfy the application's own needs.
# # target_link_libraries(app PRIVATE zephyr_interface)
# target_link_libraries(app PRIVATE dect_nrplus_lib)