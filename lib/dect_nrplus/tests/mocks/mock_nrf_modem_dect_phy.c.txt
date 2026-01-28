/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mock implementation of the nrf_modem_dect_phy.h API for Ztest.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "mock_nrf_modem_dect_phy.h"

/* -----------------------------------------------------------------------------
 * Mock-specific definitions and state
 * -----------------------------------------------------------------------------
 */

#define MOCK_TIMELINE_MAX_EVENTS 32
#define MOCK_RX_QUEUE_MAX_PACKETS 16
#define MOCK_PDC_MAX_SIZE 2048

typedef enum {
	PHY_STATE_DEINITIALIZED,
	PHY_STATE_INITIALIZED,
	PHY_STATE_ACTIVE,
} mock_phy_internal_state_t;

typedef struct {
	mock_phy_internal_state_t state;
	uint64_t current_time_us;
	nrf_modem_dect_phy_event_handler_t event_handler;
	mock_scheduled_operation_t timeline[MOCK_TIMELINE_MAX_EVENTS];
	mock_rx_packet_t rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];
	uint16_t transaction_id_counter;
} mock_phy_state_t;

static mock_phy_state_t g_mock_phy;

/* -----------------------------------------------------------------------------
 * Forward declarations for internal functions
 * -----------------------------------------------------------------------------
 */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event);
static int find_free_timeline_slot(void);
static void process_timeline(uint64_t new_time_us);
static void process_rx_queue(uint64_t new_time_us);
static uint32_t mock_calculate_tx_duration_us(size_t pdu_len);

/* -----------------------------------------------------------------------------
 * Mock control functions (for the test harness to use)
 * -----------------------------------------------------------------------------
 */

void mock_phy_reset(void)
{
	memset(&g_mock_phy, 0, sizeof(g_mock_phy));
	g_mock_phy.state = PHY_STATE_DEINITIALIZED;
	printf("[PHY Mock] State reset.\n");
}

void mock_phy_advance_time_us(uint32_t time_to_advance_us)
{
	if (g_mock_phy.state == PHY_STATE_DEINITIALIZED)
		return;
	uint64_t new_time = g_mock_phy.current_time_us + time_to_advance_us;
	process_timeline(new_time);
	process_rx_queue(new_time);
	g_mock_phy.current_time_us = new_time;
}

void mock_phy_advance_time_to_us(uint64_t target_time_us)
{
	if (g_mock_phy.state == PHY_STATE_DEINITIALIZED ||
	    target_time_us <= g_mock_phy.current_time_us)
		return;
	process_timeline(target_time_us);
	process_rx_queue(target_time_us);
	g_mock_phy.current_time_us = target_time_us;
}

int mock_phy_queue_rx_packet(const mock_rx_packet_t *packet)
{
	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
		if (!g_mock_phy.rx_queue[i].active) {
			g_mock_phy.rx_queue[i] = *packet;
			g_mock_phy.rx_queue[i].active = true;
			return 0;
		}
	}
	return -1;
}

uint64_t mock_phy_get_time_us(void)
{
	return g_mock_phy.current_time_us;
}

mock_scheduled_operation_t *mock_phy_get_last_scheduled_op(void)
{
	uint64_t latest_start_time = 0;
	mock_scheduled_operation_t *last_op = NULL;
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; i++) {
		if (g_mock_phy.timeline[i].active &&
		    g_mock_phy.timeline[i].start_time_us >= latest_start_time) {
			latest_start_time = g_mock_phy.timeline[i].start_time_us;
			last_op = &g_mock_phy.timeline[i];
		}
	}
	return last_op;
}

void mock_phy_capture_last_tx_pdu(uint8_t *buf, uint16_t *len)
{
	mock_scheduled_operation_t *op = mock_phy_get_last_scheduled_op();
	if (op && op->type == MOCK_OP_TYPE_TX) {
		*len = op->pdu_len;
		memcpy(buf, op->pdu_data, op->pdu_len);
	} else {
		*len = 0;
	}
}

/* -----------------------------------------------------------------------------
 * Mock implementation of the nrf_modem_dect_phy.h API
 * -----------------------------------------------------------------------------
 */

int nrf_modem_dect_phy_event_handler_set(nrf_modem_dect_phy_event_handler_t handler)
{
	g_mock_phy.event_handler = handler;
	return 0;
}

int nrf_modem_dect_phy_init(void)
{
	g_mock_phy.state = PHY_STATE_INITIALIZED;
	return 0;
}

int nrf_modem_dect_phy_activate(enum nrf_modem_dect_phy_radio_mode mode)
{
	g_mock_phy.state = PHY_STATE_ACTIVE;
	return 0;
}

int nrf_modem_dect_phy_tx(const struct nrf_modem_dect_phy_tx_params *params)
{
	if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
	int slot = find_free_timeline_slot();
	if (slot < 0) return -1;

	uint64_t duration_us = mock_calculate_tx_duration_us(params->data_size);
	g_mock_phy.timeline[slot] = (mock_scheduled_operation_t){
		.active = true,
		.handle = params->handle,
		.type = MOCK_OP_TYPE_TX,
		.start_time_us = params->start_time,
		.end_time_us = params->start_time + duration_us,
		.pdu_len = params->data_size,
	};
	if (params->data_size > 0) {
		memcpy(g_mock_phy.timeline[slot].pdu_data, params->data, params->data_size);
	}
	return 0;
}

int nrf_modem_dect_phy_rx(const struct nrf_modem_dect_phy_rx_params *params)
{
	if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
	int slot = find_free_timeline_slot();
	if (slot < 0) return -1;
	g_mock_phy.timeline[slot] = (mock_scheduled_operation_t){
		.active = true,
		.handle = params->handle,
		.type = MOCK_OP_TYPE_RX,
		.start_time_us = params->start_time,
		.end_time_us = params->start_time + params->duration
	};
	return 0;
}

int nrf_modem_dect_phy_cancel(uint32_t handle)
{
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
		if (g_mock_phy.timeline[i].active && g_mock_phy.timeline[i].handle == handle) {
			g_mock_phy.timeline[i].active = false;
			struct nrf_modem_dect_phy_event op_event = {
				.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
				.time = g_mock_phy.current_time_us,
				.op_complete = {.handle = handle, .err = NRF_MODEM_DECT_PHY_ERR_OP_CANCELED}
			};
			mock_phy_send_event(&op_event);
			break;
		}
	}
	return 0;
}

/* Dummy implementations for other functions */
int nrf_modem_dect_phy_deinit(void) { return 0; }
int nrf_modem_dect_phy_deactivate(void) { return 0; }
int nrf_modem_dect_phy_configure(const struct nrf_modem_dect_phy_config_params *params) { return 0; }
int nrf_modem_dect_phy_tx_harq(const struct nrf_modem_dect_phy_tx_params *params) { return nrf_modem_dect_phy_tx(params); }
int nrf_modem_dect_phy_tx_rx(const struct nrf_modem_dect_phy_tx_rx_params *params) { return 0; }
int nrf_modem_dect_phy_rssi(const struct nrf_modem_dect_phy_rssi_params *params) { return 0; }
int nrf_modem_dect_phy_latency_get(void) { return 0; }

/* -----------------------------------------------------------------------------
 * Internal mock helper functions
 * -----------------------------------------------------------------------------
 */

static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event)
{
	if (g_mock_phy.event_handler) {
		g_mock_phy.event_handler(event);
	}
}

static int find_free_timeline_slot(void)
{
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
		if (!g_mock_phy.timeline[i].active) {
			return i;
		}
	}
	return -1;
}

static void process_timeline(uint64_t new_time_us)
{
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
		mock_scheduled_operation_t *op = &g_mock_phy.timeline[i];
		if (!op->active) continue;
		if (op->end_time_us > g_mock_phy.current_time_us && op->end_time_us <= new_time_us) {
			struct nrf_modem_dect_phy_event event = {
				.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
				.time = op->end_time_us,
				.op_complete = {.handle = op->handle, .err = NRF_MODEM_DECT_PHY_SUCCESS}
			};
			mock_phy_send_event(&event);
			op->active = false;
		}
	}
}

static void process_rx_queue(uint64_t new_time_us)
{
	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
		mock_rx_packet_t *pkt = &g_mock_phy.rx_queue[i];
		if (!pkt->active) continue;
		if (pkt->reception_time_us > g_mock_phy.current_time_us && pkt->reception_time_us <= new_time_us) {
			for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
				mock_scheduled_operation_t *rx_op = &g_mock_phy.timeline[j];
				if (rx_op->active && rx_op->type == MOCK_OP_TYPE_RX &&
				    pkt->reception_time_us >= rx_op->start_time_us &&
				    pkt->reception_time_us <= rx_op->end_time_us) {
					
					struct nrf_modem_dect_phy_event pcc_evt = {
						.id = NRF_MODEM_DECT_PHY_EVT_PCC,
						.time = pkt->reception_time_us,
						.pcc = pkt->pcc_data,
					};
					pcc_evt.pcc.handle = rx_op->handle;
					pcc_evt.pcc.transaction_id = g_mock_phy.transaction_id_counter++;
					mock_phy_send_event(&pcc_evt);

					if (pkt->pdc_len > 0) {
						struct nrf_modem_dect_phy_event pdc_evt = {
							.id = NRF_MODEM_DECT_PHY_EVT_PDC,
							.time = pkt->reception_time_us,
							.pdc = {
								.handle = rx_op->handle,
								.transaction_id = pcc_evt.pcc.transaction_id,
								.data = pkt->pdc_payload,
								.len = pkt->pdc_len
							}
						};
						mock_phy_send_event(&pdc_evt);
					}
					pkt->active = false;
					break;
				}
			}
		}
	}
}

static uint32_t mock_calculate_tx_duration_us(size_t pdu_len)
{
	/* Simplified calculation assuming a fixed bitrate for testing */
	uint32_t bits_per_second = 1000000; // 1 Mbps
	uint64_t total_bits = (uint64_t)pdu_len * 8;
	return (total_bits * 1000000) / bits_per_second;
}