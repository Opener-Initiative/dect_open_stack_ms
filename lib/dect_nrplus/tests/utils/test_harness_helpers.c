/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/tests/utils/test_harness_helpers.c */
// Overview: Shared test utilities for DECT NR+ simulation tests. 
//           Contains node registration and PDU capture analysis helpers.
#include <zephyr/kernel.h>
#include "test_harness_helpers.h"

#include <zephyr/ztest.h>
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac_pdu.h>
#include <zephyr/sys/byteorder.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac.h>
#include <mocks/mock_nrf_modem_dect_phy.h>

/* --- Registry for simulation nodes --- */
#define MAX_SIM_NODES 32
static struct {
	dect_mac_context_t *mac;
	mock_phy_context_t *phy;
} g_sim_nodes[MAX_SIM_NODES];
static int g_num_sim_nodes = 0;

void dect_sim_register_node(void *mac, void *phy)
{
	if (g_num_sim_nodes < MAX_SIM_NODES) {
		g_sim_nodes[g_num_sim_nodes].mac = (dect_mac_context_t *)mac;
		g_sim_nodes[g_num_sim_nodes].phy = (mock_phy_context_t *)phy;
		g_num_sim_nodes++;
	}
}

void dect_sim_reset(void)
{
	g_num_sim_nodes = 0;
}

/* --- Capture Helpers --- */
extern uint8_t g_last_tx_pdu_capture[];
extern uint16_t g_last_tx_pdu_len_capture;

bool tx_capture_is_from_long_id(uint32_t expected_tx_long_id)
{
	if (g_last_tx_pdu_len_capture == 0) {
		return false;
	}

	if (g_last_tx_pdu_len_capture < sizeof(dect_mac_header_type_octet_t)) {
		return false;
	}

	dect_mac_header_type_octet_t hdr_type;
	memcpy(&hdr_type, g_last_tx_pdu_capture, sizeof(hdr_type));

	uint32_t transmitter_id = 0;
	switch (hdr_type.mac_header_type) {
	case MAC_COMMON_HEADER_TYPE_BEACON: {
		const dect_mac_beacon_header_t *beacon_hdr =
			(const dect_mac_beacon_header_t *)(g_last_tx_pdu_capture +
							  sizeof(dect_mac_header_type_octet_t));
		transmitter_id = sys_be32_to_cpu(beacon_hdr->transmitter_long_rd_id_be);
		break;
	}
	case MAC_COMMON_HEADER_TYPE_UNICAST: {
		const dect_mac_unicast_header_t *unicast_hdr =
			(const dect_mac_unicast_header_t *)(g_last_tx_pdu_capture +
							   sizeof(dect_mac_header_type_octet_t));
		transmitter_id = sys_be32_to_cpu(unicast_hdr->transmitter_long_rd_id_be);
		break;
	}
	default:
		return false;
	}

	return (transmitter_id == expected_tx_long_id);
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

void test_mac_state_change_cb(dect_mac_public_state_t new_state)
{
	printk("[TEST_CALLBACK] MAC Public State changed to: %d\n", new_state);
}
