# DECT-2020 NR Protocol Stack Implementation

**⚠️ EXPERIMENTAL RESEARCH CODE - NOT FOR PRODUCTION USE ⚠️**

## Important Legal Notice

This software is provided **STRICTLY FOR RESEARCH AND EDUCATIONAL PURPOSES ONLY**. 

A proprietary DECT NR+ (ETSI TS 103 636 V2.1.1 / TS 103 874 / DECT-2020 NR) L2 driver library and protocol stack wholly owned, developed, and maintained by Manulytica Ltd for the Zephyr RTOS and Nordic nRF Connect SDK (NCS).

> [!IMPORTANT]
> **PROPRIETARY & CONFIDENTIAL**: This software is **NOT open source**. It is the exclusive intellectual property of Manulytica Ltd. No part of this code may be used, copied, compiled, modified, or distributed without prior express written permission and a valid license agreement from Manulytica Ltd.

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

This repository contains an experimental implementation of the **DECT-2020 NR (DECT NR+)** protocol stack based on the **ETSI TS 103 636 Standard Version 2.1.1**.
This library covers Parts 4 & 5 of the specification.

Targeted at Nordic Semiconductor's **nRF9161**, **nRF9151** and **nRF9131** System-in-Packages (SiPs) utilizing the Nordic modem PHY firmware (`mfw-nr+_nrf91x1_1.1.0`), this stack provides a carrier-grade MAC, DLC, Convergence (CVG) layer, and Zephyr L2 network driver with seamless IPv6 / 6LoWPAN integration, autonomous adaptive timing, closed-loop transmit power control, and hardware-accelerated PSA crypto.

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

```bash
west build -p always -b nrf9161dk/nrf9161/ns  mac_mobility -T mac_mobility.handover
west twister -p nrf9161dk/nrf9161/ns -T mac_mobility --clobber-output
west build -p always -b native_sim/native/64  mac_mobility -T mac_mobility.handover
west twister -p native_sim/native/64 -T mac_mobility --clobber-output
```

```bash
west build -p always -b nrf9161dk/nrf9161/ns  mac_security -T mac_security.crypto
west twister -p nrf9161dk/nrf9161/ns -T mac_security --clobber-output
```

```bash
west build -p always -b native_sim/native/64 mac_pdu -T mac_pdu.serialization
west twister -p native_sim/native/64 -T mac_pdu --clobber-output
```

```bash
west build -p always -b nrf9161dk/nrf9161/ns mac_ass -T mac_ass.basic
west build -p always -b native_sim/native/64 mac_ass -T mac_ass.basic
west twister -p native_sim/native/64 -T mac_ass --clobber-output
```

```bash
west build -p always -b native_sim/native/64 dlc_sar -T dlc.sar
west twister -p native_sim/native/64 -T dlc_sar --clobber-output
```

```bash
west build -p always -b native_sim/native/64 dlc_arq -T dlc.arq
west twister -p native_sim/native/64 -T dlc_arq --clobber-output
```

```bash
west build -p always -b native_sim/native/64 dlc_arq -T cvg.arq
west twister -p native_sim/native/64 -T cvg_arq --clobber-output
```

```bash
west build -p always -b native_sim/native/64 stack_integration -T stack.integration
west twister -p native_sim/native/64 -T stack_integration --clobber-output
```

```bash
west build -p always -b nrf9161dk/nrf9161/ns l2_cdd_6lowpan -T l2.cdd_6lowpan
west build -p always -b native_sim/native/64 l2_cdd_6lowpan -T l2.cdd_6lowpan
west twister -p native_sim/native/64 -T l2_cdd_6lowpan --clobber-output
```

```bash
west build -b native_sim/native/64 -d build_ft dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_ft.conf
west build -b nrf9161dk/nrf9161/ns -d build_ft dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_ft.conf
```

```bash
west build -p -b native_sim/native/64 -d build_pt lib/dect_nrplus/samples/dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_pt.conf
west build -p -b nrf9161dk/nrf9161/ns -d build_pt lib/dect_nrplus/samples/dect_nrplus_ping -- -DCONF_FILE=boards/native_sim_pt.conf
```
