/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Mock nRF Modem DECT PHY library.
 */
#ifndef MOCK_NRF_DECT_PHY_H__
#define MOCK_NRF_DECT_PHY_H__
#ifndef MOCK_NRF_MODEM_DECT_PHY_H__
#define MOCK_NRF_MODEM_DECT_PHY_H__
#include "../mac/nrf_modem_dect_phy.h"
#include <zephyr/kernel.h> // For k_fifo, k_timeout_t
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/util.h>

/* --- Public Mock Data Structures --- */

/** @brief Defines the internal states of the mock PHY. */
typedef enum {
	PHY_STATE_DEINITIALIZED,
	PHY_STATE_INITIALIZED,
	PHY_STATE_ACTIVE,
} mock_phy_internal_state_t;


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
	bool completed; /* Flag to indicate the op complete event has been sent */
	union nrf_modem_dect_phy_hdr pcc_header; /* Store the PCC header used for this TX */
	uint8_t phy_type;                        /* Store the PHY type (0 or 1) */
	uint8_t pdu_data[2048]; // Stores the full PDU for TX ops
	uint16_t pdu_len;
	uint16_t carrier; /* For RSSI and RX ops */
	bool intermediate_event_sent; /* For multi-event ops like RSSI */
	void *context; /* Pointer to the MAC context that scheduled this op */
} mock_scheduled_operation_t;

/** @brief Represents a packet to be "received" by the mock PHY. */
typedef struct {
    bool active;
    uint64_t reception_time_us;
	uint16_t carrier;
    bool force_pcc_crc_error;
    bool force_pdc_crc_error;
	struct nrf_modem_dect_phy_pcc_event pcc_data;
	uint8_t pdc_payload[2048];
	size_t pdc_len;
} mock_rx_packet_t;


/* --- Public Mock Control Functions --- */

/** @brief Resets the mock PHY to its initial state. */
void mock_phy_reset(void);

/**
 * @brief Finds the next chronological event, advances time to it, and processes it.
 *
 * This is the main function for driving the simulation in an event-driven test.
 *
 * @return The timestamp of the event that was processed, or 0 if no events were pending.
 */
uint64_t mock_phy_run_until_next_event(void);

/** @brief Synchronizes the mock's internal time with the Zephyr kernel's uptime and processes any events that would have occurred. */
void mock_phy_sync_time(void);

/** @brief Advances the mock's internal time, processing any events that occur. */
void mock_phy_advance_time_us(uint64_t time_to_advance_us);

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

/** @brief Gets the time of the next scheduled event on the timeline. */
uint64_t mock_phy_get_next_event_time_us(void);

/** @brief Gets a pointer to the last operation of a specific type scheduled on the timeline. */
mock_scheduled_operation_t *mock_phy_get_last_op_by_type(mock_op_type_t type);


/* --- Test Capture Buffer (Declared Here, Defined in test main.c) --- */
extern uint8_t g_last_tx_pdu_capture[];
extern uint16_t g_last_tx_pdu_len_capture;


/* --- Test Context Override (Defined in mock_nrf_modem_dect_phy.c) --- */
/* This is a test-only mechanism to allow the mock to force a specific context
 * for events that don't have an active operation handle (e.g., FT RACH receive).
 */
extern struct dect_mac_context *g_mock_phy_context_override;


/** @brief Gets a pointer to the last TX operation scheduled for a specific peer. */
mock_scheduled_operation_t *mock_phy_get_last_tx_op_for_peer(uint16_t peer_short_id);

void mock_phy_process_events(void);

#endif /* MOCK_NRF_MODEM_DECT_PHY_H__ */
#endif /* MOCK_NRF_DECT_PHY_H__ */