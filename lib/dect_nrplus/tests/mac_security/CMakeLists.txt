# /* lib/dect_nrplus/tests/mac_security/CMakeLists.txt */
# // Overview: Build script for the MAC security test suite.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_mac_security_test)

target_sources(app PRIVATE
	src/main.c
	# The mock PHY that replaces the real one
	../../mac/mock_nrf_modem_dect_phy.c
	# The virtual timer simulation clock
	../../tests/utils/virtual_timer.c
	# The shared test harness helper
	../../tests/utils/test_harness_helpers.c
)


target_include_directories(app PRIVATE
    ../../include
    ../../include/mac
    ../../include/mocks
    ../../tests
)

# Link against the Zephyr interface library to pull in Kconfig-enabled
# dependencies like the PSA Crypto library.
target_link_libraries(app PRIVATE zephyr_interface)





# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(dect_mac_security_test)

# # The "app" target is created automatically for a Zephyr application.
# # Add only the source files specific to this test.
# target_sources(app PRIVATE
#     src/main.c
#     # Compile the specific library module under test directly
#     ../../mac/dect_mac_security.c
#     # Compile any dependencies of that module
#     ../../mac/dect_mac_random.c    
# )

# # Add the necessary include directories so the compiler can find headers
# # like <mac/dect_mac_security.h>.
# target_include_directories(app PRIVATE
#     ../../include
#     ../../include/mac
# )

# # Link against the Zephyr interface to pull in Kconfig-enabled dependencies
# # like the PSA Crypto library.
# target_link_libraries(app PRIVATE zephyr_interface)
