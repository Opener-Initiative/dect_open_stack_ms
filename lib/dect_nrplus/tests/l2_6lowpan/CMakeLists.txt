# /* lib/dect_nrplus/tests/l2_6lowpan/CMakeLists.txt */
# // Overview: Build script for the full stack integration test. It compiles only the test's main function and mock PHY, linking against the main `dect_nrplus_driver` library.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_stack_integration_test)

target_sources(app PRIVATE
	src/main.c
	# The mock PHY that replaces the real one
	../../mac/mock_nrf_modem_dect_phy.c
	# The virtual timer simulation clock
	../../tests/utils/virtual_timer.c
	# The new shared test harness helper
	../../tests/utils/test_harness_helpers.c
)

target_include_directories(app PRIVATE
	../../include
	../../include/mac
	../../include/mocks
	../../tests
)