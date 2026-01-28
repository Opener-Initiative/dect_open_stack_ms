# This file is for the dect nr+ driver.

cmake_minimum_required(VERSION 3.21)	

zephyr_library()

zephyr_library_sources(dect_nrplus.c)

# The driver links to the library implicitly because its Kconfig
# will `select DECT_NRPLUS_STACK`.