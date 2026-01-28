# Install script for directory: /home/bob/ncs/v3-zlink-U/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/nrf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/cjson/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/azure-sdk-for-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/cirrus-logic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/suit-processor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/memfault-firmware-sdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/canopennode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/chre/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/hal_tdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/nrf_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/percepio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/nrfxlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/modules/connectedhomeip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/bob/ncs/v3-zlink-U/nrf/lib/dect_nrplus/tests/twister-out/native_sim_native_64/host/mac_ft_beacon.basic/zephyr/cmake/reports/cmake_install.cmake")
endif()

