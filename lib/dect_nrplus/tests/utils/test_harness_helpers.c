/* lib/dect_nrplus/tests/utils/test_harness_helpers.c */
// Overview: A new source file implementing the shared test utilities.
#include <zephyr/kernel.h>
#include "test_harness_helpers.h"

#include <zephyr/ztest.h>
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac_pdu.h>
#include <zephyr/sys/byteorder.h>

/* --- Test Capture Buffer (defined in each test's main.c) --- */
extern uint8_t g_last_tx_pdu_capture[];
extern uint16_t g_last_tx_pdu_len_capture;


bool tx_capture_is_from_long_id(uint32_t expected_tx_long_id)
{
    printk("\n=== PACKET DEBUG ===\n");
    printk("Expected TX Long ID: 0x%08X\n", expected_tx_long_id);
    printk("Captured PDU length: %u bytes\n", g_last_tx_pdu_len_capture);
    
    if (g_last_tx_pdu_len_capture < sizeof(dect_mac_header_type_octet_t)) {
        printk("ERROR: PDU too short for header type detection\n");
        return false;
    }

    // Read the header type
    dect_mac_header_type_octet_t hdr_type;
    memcpy(&hdr_type, g_last_tx_pdu_capture, sizeof(hdr_type));
    
    printk("Header type: 0x%02X\n", *(uint8_t *)&hdr_type);

    // Handle different header types
    switch (hdr_type.mac_header_type) {
    case MAC_COMMON_HEADER_TYPE_BEACON: {
        printk("Packet type: BEACON\n");
        const dect_mac_beacon_header_t *beacon_hdr =
            (const dect_mac_beacon_header_t *)(g_last_tx_pdu_capture +
                              sizeof(dect_mac_header_type_octet_t));
        
        // Debug beacon header
        printk("Beacon header bytes: ");
        for (int i = 0; i < sizeof(dect_mac_beacon_header_t); i++) {
            printk("%02X ", ((uint8_t *)beacon_hdr)[i]);
        }
        printk("\n");
        
        uint32_t transmitter_id = sys_be32_to_cpu(beacon_hdr->transmitter_long_rd_id_be);
        printk("Beacon transmitter ID: 0x%08X\n", transmitter_id);
        printk("Match: %s\n", (transmitter_id == expected_tx_long_id) ? "YES" : "NO");
        
        return (transmitter_id == expected_tx_long_id);
    }
    
    case MAC_COMMON_HEADER_TYPE_UNICAST: {
        printk("Packet type: UNICAST\n");
        const dect_mac_unicast_header_t *unicast_hdr =
            (const dect_mac_unicast_header_t *)(g_last_tx_pdu_capture +
                               sizeof(dect_mac_header_type_octet_t));
        
        // Debug unicast header
        printk("Unicast header bytes: ");
        for (int i = 0; i < sizeof(dect_mac_unicast_header_t); i++) {
            printk("%02X ", ((uint8_t *)unicast_hdr)[i]);
        }
        printk("\n");
        
        uint32_t transmitter_id = sys_be32_to_cpu(unicast_hdr->transmitter_long_rd_id_be);
        printk("Unicast transmitter ID: 0x%08X\n", transmitter_id);
        printk("Match: %s\n", (transmitter_id == expected_tx_long_id) ? "YES" : "NO");
        
        return (transmitter_id == expected_tx_long_id);
    }
    
    case MAC_COMMON_HEADER_TYPE_DATA_PDU: {
        printk("Packet type: DATA PDU\n");
        // Handle data PDU if needed
        return false;
    }
    
    default:
        printk("Unknown header type: 0x%02X\n", hdr_type.mac_header_type);
        return false;
    }
}


const uint8_t *get_ie_payload_from_pdu(const uint8_t *pdu, uint16_t pdu_len,
					      uint8_t target_ie_type, uint16_t *payload_len)
{
	if (!pdu || !payload_len || pdu_len < sizeof(dect_mac_header_type_octet_t)) {
		return NULL;
	}

	dect_mac_header_type_octet_t hdr_type;
	memcpy(&hdr_type, pdu, sizeof(hdr_type));

	size_t common_hdr_len = 0;
	switch (hdr_type.mac_header_type) {
	case MAC_COMMON_HEADER_TYPE_UNICAST:
		common_hdr_len = sizeof(dect_mac_unicast_header_t);
		break;
	case MAC_COMMON_HEADER_TYPE_DATA_PDU:
		common_hdr_len = sizeof(dect_mac_data_pdu_header_t);
		break;
	case MAC_COMMON_HEADER_TYPE_BEACON:
		common_hdr_len = sizeof(dect_mac_beacon_header_t);
		break;
	default:
		return NULL;
	}

	if (pdu_len < (sizeof(dect_mac_header_type_octet_t) + common_hdr_len)) {
		return NULL;
	}

	const uint8_t *sdu_area = pdu + sizeof(dect_mac_header_type_octet_t) + common_hdr_len;
	size_t sdu_area_len = pdu_len - (sizeof(dect_mac_header_type_octet_t) + common_hdr_len);

	const uint8_t *current_ie_ptr = sdu_area;
	size_t remaining_len = sdu_area_len;

	while (remaining_len > 0) {
		uint8_t ie_type;
		const uint8_t *ie_payload_ptr;
		int mux_hdr_len = parse_mac_mux_header(current_ie_ptr, remaining_len, &ie_type,
						       payload_len, &ie_payload_ptr);

		if (mux_hdr_len <= 0 || (remaining_len < (size_t)mux_hdr_len + *payload_len)) {
			return NULL;
		}

		if (ie_type == target_ie_type) {
			return ie_payload_ptr;
		}

		size_t consumed = mux_hdr_len + *payload_len;
		if (consumed == 0) {
			return NULL;
		}
		remaining_len -= consumed;
		current_ie_ptr += consumed;
	}

	return NULL;
}

/**
 * @brief A safe, test-only MAC state change callback that only logs the new state.
 */
void test_mac_state_change_cb(dect_mac_public_state_t new_state)
{
	printk("[TEST_CALLBACK] MAC Public State changed to: %d\n", new_state);
}

// const uint8_t *get_ie_payload_from_pdu(const uint8_t *pdu_data, uint16_t pdu_len,
// 				       uint8_t target_ie_type, uint16_t *out_ie_len)
// {
// 	if (!pdu_data || pdu_len < (sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_unicast_header_t))) {
// 		return NULL;
// 	}

// 	const uint8_t *sdu_area_start = pdu_data + sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_unicast_header_t);
// 	size_t sdu_area_len = pdu_len - (sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_unicast_header_t));

// 	const uint8_t *p = sdu_area_start;
// 	size_t remaining = sdu_area_len;

// 	while (remaining > 0) {
// 		uint8_t ie_type;
// 		uint16_t ie_len;
// 		const uint8_t *ie_payload;
// 		int mux_len = parse_mac_mux_header(p, remaining, &ie_type, &ie_len, &ie_payload);

// 		if (mux_len <= 0 || (remaining < (size_t)mux_len + ie_len)) {
// 			break;
// 		}

// 		if (ie_type == target_ie_type) {
// 			if (out_ie_len) {
// 				*out_ie_len = ie_len;
// 			}
// 			return ie_payload;
// 		}

// 		size_t consumed = mux_len + ie_len;
// 		if (consumed == 0) break;
// 		p += consumed;
// 		remaining -= consumed;
// 	}

// 	return NULL;
// }
