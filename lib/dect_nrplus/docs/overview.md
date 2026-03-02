# DECT NR+ Protocol Stack Architecture with Zephyr IP Stack

## 1. Updated Architectural Overview

This document describes the data flow through the DECT NR+ protocol stack integrated with Zephyr's networking subsystem, with 6LoWPAN compression now handled by the CVG layer.

## 2. Data Flow Description

### 2.1 Transmission Path (Application → RF)

1. **Application Sends IPv6 Data**
   - The Zephyr application calls `send()` or `sendto()` on a standard UDP or TCP socket.

2. **Zephyr IP Stack**
   - The Zephyr IP stack processes the packet and determines the outgoing interface as the `dect_nrplus` network interface via its routing table.

3. **Zephyr L2 Network Driver (dect_nrplus.c)**
   - The IP stack invokes the `send` function pointer of the registered `net_if_api`, specifically `dect_nrplus_iface_send`.
   - The full IPv6 packet is passed to the CVG layer.

4. **Convergence Layer – 6LoWPAN Compression (dect_cvg.c)**
   - The CVG layer receives the full IPv6 packet and:
     - Invokes Zephyr's `sixlowpan_compress()` to compress IPv6/UDP/TCP headers into a compact 6LoWPAN format.
     - Adds CVG-specific headers (sequencing, SAR, security) to form a CVG PDU.
   - Passes the compressed CVG PDU to the DLC layer.

5. **DLC Layer (dect_dlc.c)**
   - Adds routing and segmentation headers to create one or more DLC PDUs.

6. **MAC Layer API (dect_mac_api.c)**
   - The DLC layer transmits DLC PDUs to the MAC layer via `dect_mac_api_send()` or `dect_mac_api_ft_send_to_pt()`.

7. **MAC Layer Core (dect_mac_data_path.c)**
   - The MAC scheduler constructs the final MAC PDU (adding MAC headers, IEs, and MIC) and calls the PHY control layer.

8. **PHY Control (dect_mac_phy_ctrl.c)**
   - Constructs Physical Control Channel (PCC) parameters and invokes the Nordic PHY API.

9. **Nordic Modem (Net CPU)**
   - The modem's PHY firmware performs the actual radio transmission.

### 2.2 Reception Path (RF → Application)

1. **Nordic Modem**
   - Receives RF transmission and passes data up to PHY control.

2. **PHY Control**
   - Processes physical layer data and delivers it to the MAC layer.

3. **MAC Layer**
   - Validates, decrypts, and reassembles MAC PDUs, then forwards them to the DLC layer.

4. **DLC Layer**
   - Processes DLC headers and routes data to the corresponding CVG instance.

5. **Convergence Layer – 6LoWPAN Decompression (dect_cvg.c)**
   - Removes CVG headers and extracts the compressed 6LoWPAN payload.
   - Invokes Zephyr's `sixlowpan_decompress()` to reconstruct the full IPv6 packet.
   - Passes the decompressed IPv6 packet to the L2 driver.

6. **L2 Network Driver (dect_nrplus.c)**
   - Receives the decompressed IPv6 packet and passes it to Zephyr's IP stack using `net_recv_data()`.

7. **Zephyr IP Stack**
   - Processes the IPv6 packet and delivers it to the appropriate socket.

8. **Application**
   - Receives data via `recv()` or `recvfrom()`.

## 3. Updated Stack Architecture Diagram

```
+-----------------------------------------------------------+
|         User Application (e.g., UDP/TCP Sockets)          |
+-----------------------------------------------------------+
|      Zephyr Networking Stack (IP/UDP/TCP Protocols)       |
+-----------------------------------------------------------+
|        DECT NR+ L2 Network Driver (dect_nrplus.c)         |
| (Implements Zephyr net_if_api, passes full IPv6)          |
+-----------------------------+-----------------------------+
| DECT NR+ Convergence Layer  | (CVG)                       |
| (6LoWPAN Compression, SAR)  | (lib/dect_nrplus/)          |
| • sixlowpan_compress()      |                             |
| • sixlowpan_decompress()    |                             |
+-----------------------------+-----------------------------+
| DECT NR+ DLC Layer          | (DLC)                       |
| (Routing, Segmentation)     | (lib/dect_nrplus/)          |
+-----------------------------+-----------------------------+
| DECT NR+ MAC Layer          | (MAC)                       |
| (Association, Scheduling)   | (lib/dect_nrplus/mac/)      |
+-----------------------------+-----------------------------+
| Nordic nRF91x1 DECT NR+     |                             |
| PHY API (nrfxlib/nrf_modem) | (Nordic's Library)          |
+-----------------------------+-----------------------------+
| nRF91x1 Modem Firmware      |                             |
| (Runs on Net CPU)           | (Nordic's Pre-built Binary) |
+-----------------------------+-----------------------------+
|            DECT NR+ RF Hardware (nRF91x1 SiP)             |
+-----------------------------------------------------------+
```

## 4. Key Architectural Changes

### 4.1 6LoWPAN Compression Location

- **Previous:** Handled in L2 driver (`dect_nrplus.c`)
- **Current:** Moved to CVG layer (`dect_cvg.c`)

### 4.2 Benefits of This Approach

- **Layer Separation** – CVG layer handles all convergence-specific processing.
- **Protocol Flexibility** – Easier to support non-IP protocols alongside 6LoWPAN.
- **Maintainability** – 6LoWPAN logic co-located with other convergence functions.
- **Standard Compliance** – Aligns with DECT NR+ layer responsibilities.

## 5. Interface Changes

### 5.1 CVG Layer API

```c
/* dect_cvg.h */
int dect_cvg_send_ipv6(struct net_pkt *pkt);  /* For uncompressed IPv6 */
int dect_cvg_send_6lowpan(struct net_buf *buf); /* For pre-compressed data */
```

### 5.2 L2 Driver Simplification

```c
/* dect_nrplus.c - send function */
static int dect_nrplus_iface_send(struct net_if *iface, struct net_pkt *pkt)
{
    /* Simply pass full IPv6 packet to CVG layer */
    return dect_cvg_send_ipv6(pkt);
}
```

## 6. Implementation Notes

- The CVG layer must include Zephyr's 6LoWPAN headers.
- Compression context is maintained per CVG connection.
- Fallback to uncompressed format if compression fails.
- Supports both IPv6 and 6LoWPAN packet types in the receive path.

## 7. Summary

This updated architecture maintains a clean separation between Zephyr's standard networking stack and the proprietary DECT NR+ protocol layers while optimizing for efficient wireless transmission. By moving 6LoWPAN compression into the CVG layer, the system gains modularity, scalability, and alignment with standard DECT NR+ layer roles.

---

Here is the corrected, accurate description of the data flow:

1. **Application sends IPv6 data:** Your Zephyr application calls `send()` or `sendto()` on a standard UDP or TCP socket.

2. **Zephyr IP Stack:** The Zephyr IP stack processes the packet and, via its routing table, determines the outgoing interface is the `dect_nrplus` network interface.

3. **Zephyr L2 Network Driver (dect_nrplus.c):** The IP stack invokes the `send` function pointer of the registered `net_if_api` for this interface, which is `dect_nrplus_iface_send`.

4. **6LoWPAN Compression:** Inside `dect_nrplus_iface_send`, the full IPv6 packet is passed to the Zephyr `sixlowpan_compress()` function. This function compresses the IPv6/UDP/TCP headers into a compact 6LoWPAN format.

5. **Your Convergence Layer (dect_cvg.c):** The `dect_nrplus_iface_send` function then calls `dect_cvg_send()`, passing the compressed 6LoWPAN packet as the payload. The CVG layer adds its own headers (e.g., for sequencing, SAR, or security) to this payload, creating a CVG PDU.

6. **Your DLC Layer (dect_dlc.c):** The CVG layer passes the CVG PDU to the DLC layer. The DLC layer adds its headers (e.g., for routing and segmentation) to create one or more DLC PDUs.

7. **Your MAC Layer (dect_mac_api.c):** The DLC layer passes the DLC PDU(s) to the MAC layer via the `dect_mac_api_send()` or `dect_mac_api_ft_send_to_pt()` functions.

8. **Your MAC Layer Core (dect_mac_data_path.c):** The MAC scheduler picks up the packet, builds the final MAC PDU (with MAC headers, IEs, and MIC), and calls the PHY control layer.

9. **Your PHY Control (dect_mac_phy_ctrl.c):** This module constructs the Physical Control Channel (PCC) parameters and calls the Nordic PHY API.

10. **Nordic Modem (Net CPU):** The modem's PHY firmware receives the command and data, and performs the actual radio transmission.

The key difference is that the L2 driver (`dect_nrplus.c`) acts as the "glue" between the standard Zephyr IP/6LoWPAN stack and your custom DECT NR+ stack (CVG/DLC/MAC). The IP stack does not know about the CVG layer directly.

```
+-----------------------------------------------------------+
|         User Application (e.g., UDP/TCP Sockets)          |
+-----------------------------------------------------------+
|      Zephyr Networking Stack (IP/UDP/TCP Protocols)       |
+-----------------------------------------------------------+
|              Zephyr 6LoWPAN Adaptation Layer              |
|            (Header Compression/Decompression)             |
+-----------------------------------------------------------+
|        DECT NR+ L2 Network Driver (dect_nrplus.c)         |
| (Implements Zephyr net_if_api, translates IP to CVG)      |
+-----------------------------+-----------------------------+
| DECT NR+ Convergence Layer  | (CVG)                       |
| (End-to-End Services, SAR)  | (lib/dect_nrplus/)          |
+-----------------------------+-----------------------------+
| DECT NR+ DLC Layer          | (DLC)                       |
| (Routing, Segmentation)     | (lib/dect_nrplus/)          |
+-----------------------------+-----------------------------+
| DECT NR+ MAC Layer          | (MAC)                       |
| (Association, Scheduling)   | (lib/dect_nrplus/mac/)      |
+-----------------------------+-----------------------------+
| Nordic nRF91x1 DECT NR+     |                             |
| PHY API (nrfxlib/nrf_modem) | (Nordic's Library)          |
+-----------------------------+-----------------------------+
| nRF91x1 Modem Firmware      |                             |
| (Runs on Net CPU)           | (Nordic's Pre-built Binary) |
+-----------------------------+-----------------------------+
|            DECT NR+ RF Hardware (nRF91x1 SiP)             |
+-----------------------------------------------------------+
```
