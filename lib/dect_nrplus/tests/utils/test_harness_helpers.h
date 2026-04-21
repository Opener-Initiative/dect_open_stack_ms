/*
 * Copyright (c) 2026 Manulytica Ltd
 */

#ifndef TEST_HARNESS_HELPERS_H
#define TEST_HARNESS_HELPERS_H

#include <zephyr/kernel.h>
#include <mac/dect_mac_types.h>

/**
 * @brief Structure to represent a node in the simulation.
 */
typedef struct {
	void *mac;
	void *phy;
} dect_sim_node_t;

/**
 * @brief Registers a node (MAC + PHY context) for simulation services.
 */
void dect_sim_register_node(void *mac, void *phy);

/**
 * @brief Resets the simulation engine registry.
 */
void dect_sim_reset(void);

/**
 * @brief Checks if the last captured PDU is from a specific long ID.
 */
bool tx_capture_is_from_long_id(uint32_t expected_tx_long_id);

/**
 * @brief Extracts an IE payload from an encoded PDU.
 */
const uint8_t *get_ie_payload_from_pdu(const uint8_t *pdu, uint16_t pdu_len,
					      uint8_t target_ie_type, uint16_t *payload_len);

/**
 * @brief Standard state change callback for tests.
 */
void test_mac_state_change_cb(dect_mac_public_state_t new_state);

#endif /* TEST_HARNESS_HELPERS_H */