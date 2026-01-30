# DECT 2020 NR+ Protocol Stack Implementation

**⚠️ EXPERIMENTAL RESEARCH CODE - NOT FOR PRODUCTION USE ⚠️**

## Important Legal Notice

This software is provided **STRICTLY FOR RESEARCH AND EDUCATIONAL PURPOSES ONLY**. 

### Copyright and Licensing
- **Copyright © 2025 Manulytica. All Rights Reserved.**
- This code represents proprietary intellectual property of Manulytica
- **All rights reserved** - no license is granted for use, reproduction, or distribution
- Unauthorized use, copying, modification, or distribution is strictly prohibited

### Usage Restrictions
This software implementation:
- Is provided **as-is** for experimental and research purposes only
- **Must not** be used in production systems or commercial products
- **Must not** be distributed, copied, or shared without express written permission from Manulytica
- Is intended solely for internal research and development within authorized organizations

## Technical Overview

This repository contains an experimental implementation of the **DECT 2020 NR+ (New Radio Plus)** protocol stack based on the **ETSI TS 103-636 Standard Version 2.1.1**.
This library covers Parts 4 & 5 of the specification.

### Architecture

The implementation includes:

- **DECT NR+ MAC Layer**
  - MAC state machines (Fixed Part & Portable Part)
  - Physical layer control and interface
  - Data path management
  - Security framework
  - Timeline and synchronization utilities
  - Management entity handlers

- **DECT NR+ Stack API**
  - Unified interface for application integration
  - State management and callbacks
  - Configuration and control interfaces

- **Network Driver**
  - Zephyr RTOS integration
  - Nordic Connect SDK compatibility

### Prerequisites

- Nordic Connect SDK v3.x
- Zephyr RTOS



### ZTest
west build -p always -b nrf9161dk/nrf9161/ns  mac_mobility -T mac_mobility.handover
west twister -p nrf9161dk/nrf9161/ns -T mac_mobility --clobber-output
west build -p always -b native_sim/native/64  mac_mobility -T mac_mobility.handover
west twister -p native_sim/native/64 -T mac_mobility --clobber-output

west build -p always -b nrf9161dk/nrf9161/ns  mac_security -T mac_security.crypto
west twister -p nrf9161dk/nrf9161/ns -T mac_security --clobber-output

west build -p always -b native_sim/native/64 mac_pdu -T mac_pdu.serialization
west twister -p native_sim/native/64 -T mac_pdu --clobber-output

west build -p always -b nrf9161dk/nrf9161/ns mac_ass -T mac_ass.basic
west build -p always -b native_sim/native/64 mac_ass -T mac_ass.basic
west twister -p native_sim/native/64 -T mac_ass --clobber-output

west build -p always -b native_sim/native/64 dlc_sar -T dlc.sar
west twister -p native_sim/native/64 -T dlc_sar --clobber-output

west build -p always -b native_sim/native/64 dlc_arq -T dlc.arq
west twister -p native_sim/native/64 -T dlc_arq --clobber-output


west build -p always -b native_sim/native/64 dlc_arq -T cvg.arq
west twister -p native_sim/native/64 -T cvg_arq --clobber-output


west build -p always -b native_sim/native/64 stack_integration -T stack.integration
west twister -p native_sim/native/64 -T stack_integration --clobber-output

west build -p always -b nrf9161dk/nrf9161/ns l2_cdd_6lowpan -T l2.cdd_6lowpan
west build -p always -b native_sim/native/64 l2_cdd_6lowpan -T l2.cdd_6lowpan
west twister -p native_sim/native/64 -T l2_cdd_6lowpan --clobber-output


west build -b native_sim/native/64 -d build_ft dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_ft.conf
west build -b nrf9161dk/nrf9161/ns -d build_ft dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_ft.conf

west build -p -b native_sim/native/64 -d build_pt lib/dect_nrplus/samples/dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_pt.conf
west build -p -b nrf9161dk/nrf9161/ns -d build_pt lib/dect_nrplus/samples/dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_pt.conf
