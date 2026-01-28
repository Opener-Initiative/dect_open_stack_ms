/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Mock nRF Modem DECT PHY library.
 */
#ifndef MOCK_NRF_MODEM_DECT_PHY_H__
#define MOCK_NRF_MODEM_DECT_PHY_H__

#include <stdbool.h>
#include <zephyr/kernel.h> // For k_fifo, k_timeout_t
#include <stdint.h>        // For uintxx_t types
#include <stddef.h>        // For size_t
#include <mac/nrf_modem_dect_phy.h>

/* --- Public Mock Data Structures --- */

/** @brief Defines the type of a scheduled mock operation. */
typedef enum {
	MOCK_OP_TYPE_TX,
	MOCK_OP_TYPE_RX,
	MOCK_OP_TYPE_RSSI,
} mock_op_type_t;

/** @brief Represents a scheduled operation on the mock PHY's timeline. */
typedef struct {
	bool active;
	uint32_t handle;
	mock_op_type_t type;
	uint64_t start_time_us;
	uint64_t end_time_us;
	uint8_t pdu_data[2048]; // Stores the full PDU for TX ops
	uint16_t pdu_len;
} mock_scheduled_operation_t;

/** @brief Represents a packet to be "received" by the mock PHY. */
typedef struct {
	bool active;
	uint64_t reception_time_us;
	struct nrf_modem_dect_phy_pcc_event pcc_data;
	uint8_t pdc_payload[2048];
	size_t pdc_len;
} mock_rx_packet_t;

/* --- Public Mock Control Functions --- */

/** @brief Resets the mock PHY to its initial state. */
void mock_phy_reset(void);

/** @brief Advances the mock's internal time, processing any events that occur. */
void mock_phy_advance_time_us(uint32_t time_to_advance_us);

/** @brief Sets the mock's internal time to a specific target, processing events. */
void mock_phy_advance_time_to_us(uint64_t target_time_us);

/** @brief Queues a packet to be delivered to the MAC layer at a future time. */
int mock_phy_queue_rx_packet(const mock_rx_packet_t *packet);

/** @brief Gets the mock's current internal time. */
uint64_t mock_phy_get_time_us(void);

/** @brief Gets a pointer to the last operation scheduled on the timeline. */
mock_scheduled_operation_t *mock_phy_get_last_scheduled_op(void);

/** @brief Captures the PDU data from the last scheduled TX operation. */
void mock_phy_capture_last_tx_pdu(uint8_t *buf, uint16_t *len);

#endif /* MOCK_NRF_MODEM_DECT_PHY_H__ */