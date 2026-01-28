# /* lib/dect_nrplus/tests/stack_integration/CMakeLists.txt */
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_stack_integration_test)

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
	../../tests/utils
)

# Link against the Zephyr interface to pull in Kconfig-enabled dependencies
# like the PSA Crypto library.
target_link_libraries(app PRIVATE zephyr_interface)