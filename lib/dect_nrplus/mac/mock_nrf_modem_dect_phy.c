/* lib/dect_nrplus/mac/mock_nrf_modem_dect_phy.c */
// Overview: This is the complete, corrected mock PHY. It is based on the new architecture that uses the Zephyr kernel clock as the single source of truth, ensuring correct time synchronization for all tests.
// --- REPLACE ENTIRE FILE ---
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_core.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include <mac/dect_mac_timeline_utils.h>

/* --- Test Capture Buffer (defined in test main.c) --- */
extern uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
extern uint16_t g_last_tx_pdu_len_capture;

/* --- Public flags for controlling mock behavior from tests --- */
bool g_force_phy_schedule_failure = false;
enum nrf_modem_dect_phy_err g_phy_schedule_failure_code = NRF_MODEM_DECT_PHY_ERR_OP_SCHEDULING_CONFLICT;
bool g_strict_scheduling_mode = false;


/* --- Mock State --- */
static mock_phy_context_t *g_active_phy_ctx = NULL;
static nrf_modem_dect_phy_event_handler_t g_event_handler = NULL;

/* --- Forward declarations for internal functions --- */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event);
static int find_free_timeline_slot(mock_phy_context_t *ctx);
static uint64_t get_timeline_end_time(mock_phy_context_t *ctx);

/* --- Public Mock Control Functions --- */

void mock_phy_complete_reset(mock_phy_context_t *ctx)
{
    /* Clear all timeline slots */
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; i++) {
        ctx->timeline[i].active = false;
        ctx->timeline[i].running = false;
        ctx->timeline[i].handle = 0;
        ctx->timeline[i].start_time_us = 0;
        ctx->timeline[i].end_time_us = 0;
        ctx->timeline[i].type = 0;
        ctx->timeline[i].pdu_len = 0;
        ctx->timeline[i].context = NULL;
    }
    
    /* Clear all RX queue slots */
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
        ctx->rx_queue[i].active = false;
        ctx->rx_queue[i].reception_time_us = 0;
        ctx->rx_queue[i].carrier = 0;
        ctx->rx_queue[i].pdc_len = 0;
    }
    
    /* Reset other state */
    ctx->transaction_id_counter = 123;
    ctx->state = PHY_STATE_DEINITIALIZED;
}


// void mock_phy_init_context(mock_phy_context_t *ctx, struct dect_mac_context *mac_ctx,
// 			   mock_phy_context_t *peer_ctx)
// {
// 	memset(ctx, 0, sizeof(mock_phy_context_t));
// 	ctx->transaction_id_counter = 123;
// 	ctx->mac_ctx = mac_ctx;
// 	ctx->peer_ctx = peer_ctx;
// 	ctx->state = PHY_STATE_DEINITIALIZED;
// }
void mock_phy_init_context(mock_phy_context_t *ctx, struct dect_mac_context *mac_ctx,
			   mock_phy_context_t **peers, size_t num_peers)
{
	memset(ctx, 0, sizeof(mock_phy_context_t));
	ctx->transaction_id_counter = 123;
	ctx->mac_ctx = mac_ctx;
	ctx->peers = peers;
	ctx->num_peers = num_peers;
	ctx->num_active_rx_ops = 0;
	ctx->state = PHY_STATE_DEINITIALIZED;
}

void mock_phy_set_active_context(mock_phy_context_t *ctx)
{
	g_active_phy_ctx = ctx;
}

int mock_phy_queue_rx_packet(mock_phy_context_t *dest_ctx, const mock_rx_packet_t *packet)
{
	if (!dest_ctx || !packet) {
		return -EINVAL;
	}

	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
		if (!dest_ctx->rx_queue[i].active) {
			dest_ctx->rx_queue[i] = *packet;
			dest_ctx->rx_queue[i].active = true;

			// size_t len = 8; /* Adjust this based on your needs */
			// printk("[MOCK_PHY] PHY_QUEUE_RX_PACKET Payload Hexdump (first %zu bytes): \t", len);
			// for (size_t j = 0; j < len && j < packet->pdc_len; j++) {
			// 	printk("%02x ", packet->pdc_payload[j]);
			// }
			// printk("\n");

			// printk("[MOCK_PHY] PHY_QUEUE_RX_QUEUE Payload Hexdump (first %zu bytes): \t", len);
			
			// for (size_t j = 0; j < len && j < packet->pdc_len; j++) {
			// 	printk("%02x ", dest_ctx->rx_queue[i].pdc_payload[j]);
			// }			
			// printk("\n");

			return 0;
		}
	}
	return -ENOMEM;
}

uint64_t mock_phy_get_next_event_time(mock_phy_context_t *const phy_contexts[],
				      size_t num_contexts)
{
	uint64_t next_event_time = UINT64_MAX;
	uint64_t current_time = k_ticks_to_us_floor64(k_uptime_ticks());
	// printk("Calling mock_phy_get_next_event_time - current_time:%lluus \n", current_time);

	for (size_t i = 0; i < num_contexts; i++) {
		mock_phy_context_t *ctx = phy_contexts[i];

		/* Check timeline events */
		for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; j++) {
			if (ctx->timeline[j].active){
				// printk("ctx->timeline[%d].active:%s ctx->timeline[%d].end_time_us:%lluus \n", 
				// 	j, ctx->timeline[j].active ?"Active":"Inactive",j, ctx->timeline[j].end_time_us);
			}
			
			if (ctx->timeline[j].active && ctx->timeline[j].end_time_us > current_time) {
				next_event_time = MIN(next_event_time, ctx->timeline[j].end_time_us);
			}
		}

		/* Check RX queue events */
		for (int j = 0; j < MOCK_RX_QUEUE_MAX_PACKETS; j++) {
			// if (ctx->rx_queue[j].active){
			// 	printk("ctx->rx_queue[%d].active:%s ctx->rx_queue[%d].reception_time_us:%lluus \n", 
			// 		j, ctx->rx_queue[j].active ?"Active":"Inactive", j, ctx->rx_queue[j].reception_time_us);
			// }
			
			if (ctx->rx_queue[j].active &&
			    ctx->rx_queue[j].reception_time_us > current_time) {
				next_event_time =
					MIN(next_event_time, ctx->rx_queue[j].reception_time_us);
			}
		}
	}
	// printk("next_event_time %lluus \n", next_event_time);
	if (next_event_time == UINT64_MAX){
		// next_event_time = current_time + 100;
	}
	return next_event_time;
}


// void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us)
// {
// 	/* Process timeline events */
// 	// printk("/* Process timeline events */ \n");
// 	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
// 		mock_scheduled_operation_t *op = &ctx->timeline[i];

// 		/* Mark operations as running when their start time is reached */
// 		if (op->active && !op->running && op->start_time_us <= current_time_us) {
// 			op->running = true;
// 		}

// 		if (op->active && op->running && op->end_time_us <= current_time_us) {
// 			mock_phy_set_active_context(ctx);

// 			printk("[MOCK_EVENT_GEN_DBG] Generating OP_COMPLETE event for handle %u.\n", op->handle);

// 			/*
// 			 * A real PHY sends specific events (like RSSI results) BEFORE the
// 			 * generic OP_COMPLETE. We simulate that here.
// 			 */
// 			// if (op->type == MOCK_OP_TYPE_RSSI) {
// 			// 	/* Create a dummy RSSI result indicating a quiet channel */
// 			// 	printk("op->type == MOCK_OP_TYPE_RSSI : Create a dummy RSSI result indicating a quiet channel */ \n");
// 			// 	static int8_t dummy_rssi_meas[] = { -89, -91, -95, -89, -35, -87 };
// 			// 	struct nrf_modem_dect_phy_event rssi_evt = {
// 			// 		.id = NRF_MODEM_DECT_PHY_EVT_RSSI,
// 			// 		.time = op->end_time_us,
// 			// 		.rssi = { .handle = op->handle,
// 			// 			  .carrier = op->carrier,
// 			// 			  .meas_len = ARRAY_SIZE(dummy_rssi_meas),
// 			// 			  .meas = dummy_rssi_meas }
// 			// 	};
// 			// 	mock_phy_send_event(&rssi_evt);
// 			// }

// 			/* Now send the generic completion event for all op types */
// 			struct nrf_modem_dect_phy_event complete_evt = {
// 				.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
// 				.time = op->end_time_us,
// 				.op_complete = { .handle = op->handle,
// 						.err = NRF_MODEM_DECT_PHY_SUCCESS }
//             };
// 			mock_phy_send_event(&complete_evt);
// 			op->active = false;

// 			/* If this was an RX operation, remove it from the active list */
// 			if (op->type == MOCK_OP_TYPE_RX || op->type == MOCK_OP_TYPE_RSSI) {
// 				for (size_t k = 0; k < ctx->num_active_rx_ops; k++) {
// 					// if (ctx->active_rx_ops[k].handle == op->handle) {
// 					// 	/* Remove by swapping with the last element */
// 					// 	ctx->active_rx_ops[k] = ctx->active_rx_ops[ctx->num_active_rx_ops - 1];
// 					// 	printk("ctx->num_active_rx_ops:%d \n", ctx->num_active_rx_ops);
// 					// 	ctx->num_active_rx_ops--;
// 					// 	printk("ctx->num_active_rx_ops:%d \n", ctx->num_active_rx_ops);
// 					// 	break;
// 					// }
// 					if (ctx->active_rx_ops[k]->handle == op->handle) {
// 						/* Remove by swapping with the last element */
// 						ctx->active_rx_ops[k] =
// 							ctx->active_rx_ops[ctx->num_active_rx_ops - 1];
// 						printk("ctx->num_active_rx_ops:%d \n", ctx->num_active_rx_ops);
// 						ctx->num_active_rx_ops--;
// 						printk("ctx->num_active_rx_ops:%d \n", ctx->num_active_rx_ops);
// 						break;
// 					}					
// 				}
// 			}
// 		}
// 	}

// 	/* Process RX queue events */
// 	// printk("/* Process RX queue events */ \n");
// 	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
// 		mock_rx_packet_t *pkt = &ctx->rx_queue[i];

// 		if (pkt->active && pkt->reception_time_us <= current_time_us) {
//             /* Find matching RX operation */
// 			// for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
// 			// 	mock_scheduled_operation_t *rx_op = &ctx->timeline[j];

// 			// 	if (rx_op->active && rx_op->running &&
// 			// 	    rx_op->type == MOCK_OP_TYPE_RX &&
// 			// 	    pkt->carrier == rx_op->carrier) {
// 			// 		uint16_t tid = ctx->transaction_id_counter++;

//             //         /* Send PCC event */
// 			// 		struct nrf_modem_dect_phy_event pcc_evt = {
// 			// 			.id = NRF_MODEM_DECT_PHY_EVT_PCC,
// 			// 			.time = pkt->reception_time_us,
// 			// 		};
// 			// 		pcc_evt.pcc = pkt->pcc_data;
// 			// 		pcc_evt.pcc.handle = rx_op->handle;
// 			// 		pcc_evt.pcc.transaction_id = tid;
// 			// 		mock_phy_set_active_context(ctx);
// 			// 		mock_phy_send_event(&pcc_evt);

// 			// 		if (pkt->pdc_len > 0) {
// 			// 			struct nrf_modem_dect_phy_event pdc_evt = {
// 			// 				.id = NRF_MODEM_DECT_PHY_EVT_PDC,
// 			// 				.time = pkt->reception_time_us,
// 			// 				.pdc = {.handle = rx_op->handle,
// 			// 					.transaction_id = tid,
// 			// 					.data = pkt->pdc_payload,
// 			// 					.len = pkt->pdc_len}
//             //             };
// 			// 			mock_phy_set_active_context(ctx);
// 			// 			mock_phy_send_event(&pdc_evt);
// 			// 		}

// 			// 		pkt->active = false;
// 			// 		break;
// 			// 	}
// 			// }
// 			int active_slot = 0;
// 			bool rx_op_found = false;
// 			printk("[MOCK_PHY_RX_DBG] Processing queued packet for carrier %u at time %llu\n",
// 			       pkt->carrier, current_time_us);

// 			for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
// 				mock_scheduled_operation_t *rx_op = &ctx->timeline[j];

// 				if (rx_op->handle == 1604170158) { /* Hardcoded handle for debug */
// 					printk("[MOCK_PROCESS_DBG] Checking timeline slot %d for handle %u. Active: %d, Running: %d, Type: %d, Carrier: %u\n",
// 					       j, rx_op->handle, rx_op->active, rx_op->running, rx_op->type, rx_op->carrier);
// 				}

// 				active_slot = j;
// 				if (rx_op->active){
// 					printk("timeline[%d] -> rx_op->active:%s && rx_op->running:%s && rx_op->type[%d] == [1]or[2] && pkt->carrier[%d] == rx_op->carrier[%d] \n", 
// 						active_slot, rx_op->active?"Active":"Inactive", rx_op->running ?"Running":"Stopped", rx_op->type, pkt->carrier, rx_op->carrier);
// 				}

// 				// if (rx_op->active && rx_op->running &&
// 				//     (rx_op->type == MOCK_OP_TYPE_RX || rx_op->type == MOCK_OP_TYPE_RSSI)&&
// 				//     pkt->carrier == rx_op->carrier) {
// 				if (rx_op->active && rx_op->running &&
// 				    (rx_op->type == MOCK_OP_TYPE_RX || rx_op->type == MOCK_OP_TYPE_RSSI)) {				
// 					uint16_t tid = ctx->transaction_id_counter++;

// 					printk("  -> Checking against active RX op: handle=%u, carrier=%u, type=%d\n",
// 					       rx_op->handle, rx_op->carrier, rx_op->type);
						   
// 					/* Send PCC event */
// 					struct nrf_modem_dect_phy_event pcc_evt = {
// 						.id = NRF_MODEM_DECT_PHY_EVT_PCC,
// 						.time = pkt->reception_time_us,
// 					};
// 					pcc_evt.pcc = pkt->pcc_data;
// 					pcc_evt.pcc.handle = rx_op->handle;
// 					pcc_evt.pcc.transaction_id = tid;
// 					mock_phy_set_active_context(ctx);
// 					mock_phy_send_event(&pcc_evt);

// 					/* Send PDC event if there is a payload */
// 					if (pkt->pdc_len > 0) {
// 						struct nrf_modem_dect_phy_event pdc_evt = {
// 							.id = NRF_MODEM_DECT_PHY_EVT_PDC,
// 							.time = pkt->reception_time_us,
// 							.pdc = { .handle = rx_op->handle,
// 								 .transaction_id = tid,
// 								 .data = pkt->pdc_payload,
// 								 .len = pkt->pdc_len }
// 						};
// 						mock_phy_set_active_context(ctx);
// 						mock_phy_send_event(&pdc_evt);
// 					}

// 					pkt->active = false;
// 					rx_op_found = true;
// 					break;
// 				}
// 			}
// 			if (!rx_op_found) {
// 				printk("MOCK_PHY: RX packet for carrier %u dropped, no active RX operation final timeline.\n",
// 					pkt->carrier);
// 				// pkt->active = false; /* Discard the packet */
// 			}
// 		}
// 	}
// }
// Overview: Refactors the mock PHY's event processing to be a true discrete-event simulator. It now finds and processes only the single, next chronological event instead of batch-processing, which fixes the event queue overflow bug.
// --- REPLACE ENTIRE FUNCTION: mock_phy_process_events ---
// // <<START REPLACEMENT CODE>>
void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us)
{
	/* Process timeline for completed operations */
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
		mock_scheduled_operation_t *op = &ctx->timeline[i];
		if (op->active && !op->running && op->start_time_us <= current_time_us) {
			op->running = true;
		}
		if (op->active && op->running && op->end_time_us <= current_time_us) {
			mock_phy_set_active_context(ctx);
			struct nrf_modem_dect_phy_event complete_evt = {
				.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
				.time = op->end_time_us,
				.op_complete = {.handle = op->handle, .err = NRF_MODEM_DECT_PHY_SUCCESS}};
			mock_phy_send_event(&complete_evt);
			op->active = false;
		}
	}

	/* Find the single next RX packet to process */
	int next_pkt_idx = -1;
	uint64_t earliest_time = UINT64_MAX;
	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
		if (ctx->rx_queue[i].active) {
			printk("[MOCK_PROCESS_RX_DBG] Found active packet in slot %d for time %llu (current time: %llu)\n",
			       i, ctx->rx_queue[i].reception_time_us, current_time_us);
		}

		if (ctx->rx_queue[i].active && ctx->rx_queue[i].reception_time_us <= current_time_us) {
			if (ctx->rx_queue[i].reception_time_us < earliest_time) {
				earliest_time = ctx->rx_queue[i].reception_time_us;
				next_pkt_idx = i;
			}
		}
	}

	if (next_pkt_idx == -1) {
		printk("[MOCK_PROCESS_RX_DBG] No RX packets to process at this time.\n");
		return; /* No packets to process at this time */
	}

	mock_rx_packet_t *pkt = &ctx->rx_queue[next_pkt_idx];

	printk("[MOCK_PROCESS_RX_DBG] Processing packet from slot %d. Searching for matching RX op...\n", next_pkt_idx);

	/* Find a matching, active RX operation on the timeline */
	for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
		mock_scheduled_operation_t *rx_op = &ctx->timeline[j];

		if (ctx->mac_ctx->role == MAC_ROLE_FT) {
			printk("[FT_RX_MATCH_DBG] Checking timeline slot %d: active=%d, running=%d, type=%d, carrier=%u\n",
			       j, rx_op->active, rx_op->running, rx_op->type, rx_op->carrier);
		}
		if (rx_op->active && rx_op->running && rx_op->type == MOCK_OP_TYPE_RX) {
			printk("[CARRIER_CHECK_DBG] Comparing packet carrier %u with RX op carrier %u.\n",
			       pkt->carrier, rx_op->carrier);
		}

		if (rx_op->active && rx_op->running && rx_op->type == MOCK_OP_TYPE_RX &&
		    pkt->carrier == rx_op->carrier) {
			mock_phy_set_active_context(ctx);

			/* Send PCC event */
			struct nrf_modem_dect_phy_event pcc_evt = {
				.id = NRF_MODEM_DECT_PHY_EVT_PCC,
				.time = pkt->reception_time_us,
				.pcc = pkt->pcc_data};
			// pcc_evt.pcc.hdr.handle = rx_op->handle;
			pcc_evt.pcc.handle= rx_op->handle;
			mock_phy_send_event(&pcc_evt);

			/* Send PDC event */
			struct nrf_modem_dect_phy_event pdc_evt = {
				.id = NRF_MODEM_DECT_PHY_EVT_PDC,
				.time = pkt->reception_time_us,
				.pdc = {.handle = rx_op->handle, .data = pkt->pdc_payload, .len = pkt->pdc_len}};
			mock_phy_send_event(&pdc_evt);

			break; /* Processed one packet, exit */
		}
	}

	/* Deactivate the processed packet */
	pkt->active = false;
}
// <<END REPLACEMENT CODE>>




/* --- Mock implementation of the nrf_modem_dect_phy.h API --- */
int nrf_modem_dect_phy_event_handler_set(nrf_modem_dect_phy_event_handler_t handler)
{
	g_event_handler = handler;
	return 0;
}

int nrf_modem_dect_phy_init(void)
{
	g_active_phy_ctx->state = PHY_STATE_INITIALIZED;
	struct nrf_modem_dect_phy_event event = {
		.id = NRF_MODEM_DECT_PHY_EVT_INIT,
		.init = {.err = NRF_MODEM_DECT_PHY_SUCCESS}};
	mock_phy_send_event(&event);
	return 0;
}

int nrf_modem_dect_phy_tx(const struct nrf_modem_dect_phy_tx_params *params)
{
	if (g_force_phy_schedule_failure) {
		printk("MOCK_PHY: Forcing PHY schedule failure with code %d",
			g_phy_schedule_failure_code);
		return g_phy_schedule_failure_code;
	}

	if (g_strict_scheduling_mode) {
		for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; i++) {
			if (g_active_phy_ctx->timeline[i].active &&
			    g_active_phy_ctx->timeline[i].start_time_us < params->start_time + params->data_size * 8 * 10 &&
			    g_active_phy_ctx->timeline[i].end_time_us > params->start_time) {
				printk("MOCK_PHY: Strict scheduling violation. New TX conflicts with existing operation.");
				return NRF_MODEM_DECT_PHY_ERR_OP_SCHEDULING_CONFLICT;
			}
		}
	}

	if (g_active_phy_ctx->state != PHY_STATE_ACTIVE) {
		return -1;
	}
	int slot = find_free_timeline_slot(g_active_phy_ctx);
	if (slot < 0) {
		return -ENOMEM;
	}

	uint64_t start_time_us =
		(params->start_time == 0) ?
			      MAX(k_ticks_to_us_floor64(k_uptime_ticks()),
				  get_timeline_end_time(g_active_phy_ctx)) :
			      params->start_time;

	/* Simplified duration based on payload size for simulation */
	uint64_t duration_us =
		modem_ticks_to_us(params->data_size * 8 * 10, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	g_active_phy_ctx->timeline[slot] = (mock_scheduled_operation_t){
		.active = true,
		.running = false,
		.handle = params->handle,
		.type = MOCK_OP_TYPE_TX,
		.start_time_us = start_time_us,
		.end_time_us = start_time_us + duration_us,
		.pdu_len = params->data_size,
		.context = g_active_phy_ctx->mac_ctx,
	};
	if (params->data) {
		memcpy(g_active_phy_ctx->timeline[slot].pdu_data, params->data, params->data_size);
	}
	g_last_tx_pdu_len_capture = params->data_size;
	memcpy(g_last_tx_pdu_capture, params->data, params->data_size);

	// if (g_active_phy_ctx->peer_ctx) {
	// 	mock_rx_packet_t rx_pkt = {
	// 		.active = true,
	// 		.reception_time_us = start_time_us + 100, /* 100us prop delay */
	// 		.carrier = params->carrier,
	// 		.pcc_data =
	// 			{
	// 				.phy_type = params->phy_type,
	// 				.hdr = *params->phy_header,
	// 				.rssi_2 = -120, /* -60 dBm in Q7.1 */
	// 				.snr = 80,	/* 20 dB in Q13.2 */
	// 			},
	// 		.pdc_len = params->data_size,
	// 	};
	// 	if (params->data) {
	// 		memcpy(rx_pkt.pdc_payload, params->data, params->data_size);
	// 	}
	// 	mock_phy_queue_rx_packet(g_active_phy_ctx->peer_ctx, &rx_pkt);
	// }

	printk("\n--- DEBUG_PROBE: nrf_modem_dect_phy_tx ---\n");
	printk("  - Transmitter is: %s\n", (g_active_phy_ctx->mac_ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
	printk("  - Number of configured peers: %zu\n", g_active_phy_ctx->num_peers);

	/* Simulate air: queue this packet for all potential peers that are listening */
	printk("[MOCK_PHY] Simulate air: queue this packet for all potential peers that are listening \n");

	for (size_t i = 0; i < g_active_phy_ctx->num_peers; i++) {
		mock_phy_context_t *peer_ctx = g_active_phy_ctx->peers[i];
		printk("  - Packet to Queuue for peer %d (context %p)\n", (int)i, (void *)peer_ctx);

		if (peer_ctx) {
			mock_rx_packet_t rx_pkt = {
				.active = true,
				.reception_time_us = start_time_us + 100, /* 100us prop delay */
				.carrier = params->carrier,
				.pcc_data = {
					.phy_type = params->phy_type,
					.hdr = *params->phy_header,
					.rssi_2 = -120, /* -60 dBm in Q7.1 */
					.snr = 80,      /* 20 dB in Q13.2 */
				},
				.pdc_len = params->data_size,
			};
			if (params->data) {
				// memcpy(&rx_pkt.pcc_data, &pcc, sizeof(pcc));
				memcpy(rx_pkt.pdc_payload, params->data, params->data_size);
			}

			printk("[MOCK_TX_DBG]   -> Queuing packet for peer.\n");
			mock_phy_queue_rx_packet(peer_ctx, &rx_pkt);
		} else {
			printk("[MOCK_TX_DBG]   -> NO PEER for packet.\n");
		}

	}

	return 0;
}



int nrf_modem_dect_phy_rx(const struct nrf_modem_dect_phy_rx_params *params)
{
	if (g_active_phy_ctx->state != PHY_STATE_ACTIVE) {
		return -1;
	}
	int slot = find_free_timeline_slot(g_active_phy_ctx);
	if (slot < 0) {
		return -ENOMEM;
	}

	uint64_t start_time_us =
		(params->start_time == 0) ?
			      MAX(k_ticks_to_us_floor64(k_uptime_ticks()),
				  get_timeline_end_time(g_active_phy_ctx)) :
			      params->start_time;

	uint64_t end_time_us =
		(params->duration == 0) ?
			      UINT64_MAX :
			      (start_time_us + modem_ticks_to_us(params->duration,
								NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ));

	// uint64_t end_time_us = 
	// 	(params->duration == 0) ? 
	// 				(start_time_us + 1000000000) : /* 1000 seconds */
	// 				(start_time_us + modem_ticks_to_us(params->duration, 
	// 							NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ));

	g_active_phy_ctx->timeline[slot] = (mock_scheduled_operation_t){
		.active = true,
		.running = false,
		.handle = params->handle,
		.type = MOCK_OP_TYPE_RX,
		.start_time_us = start_time_us,
		.end_time_us = end_time_us,
		.carrier = params->carrier,
		.context = g_active_phy_ctx->mac_ctx,
	};

	if (g_active_phy_ctx->num_active_rx_ops < MOCK_TIMELINE_MAX_EVENTS) {
		g_active_phy_ctx->active_rx_ops[g_active_phy_ctx->num_active_rx_ops++] = &g_active_phy_ctx->timeline[slot];
	}

	printk("[MOCK_PHY] RX operation scheduled. Handle: %u, Start: %llu us, End: %llu us\n",
	       params->handle, start_time_us, end_time_us);

	return 0;
}

int nrf_modem_dect_phy_rssi(const struct nrf_modem_dect_phy_rssi_params *params)
{
	printk("[MOCK_PHY_RSSI_DBG] Entering mock nrf_modem_dect_phy_rssi g_active_phy_ctx = %p\n", (void *)g_active_phy_ctx);

	if (g_active_phy_ctx->state != PHY_STATE_ACTIVE) {
		printk("[MOCK_PHY_RSSI_DBG] g_active_phy_ctx->state != PHY_STATE_ACTIVE..\n");
		return -1;
	}

	// printk("[MOCK_PHY_RSSI_DBG] Calling find_free_timeline_slot...\n");
	int slot = find_free_timeline_slot(g_active_phy_ctx);
	if (slot < 0) {
		printk("[MOCK_PHY_RSSI_DBG] find_free_timeline_slot < 0 [%d].\n",slot);
		return -ENOMEM;
	}

	printk("[MOCK_PHY_RSSI_DBG] find_free_timeline_slot returned: %d\n", slot);

	uint64_t start_time_us =
		(params->start_time == 0) ?
			      MAX(k_ticks_to_us_floor64(k_uptime_ticks()),
				  get_timeline_end_time(g_active_phy_ctx)) :
			      params->start_time;

	printk("[MOCK_PHY_RSSI_DBG] Calculating start time...\n");

	uint64_t duration_us =
		modem_ticks_to_us(params->duration, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	g_active_phy_ctx->timeline[slot] = (mock_scheduled_operation_t){
		.active = true,
		.running = false,
		.handle = params->handle,
		.type = MOCK_OP_TYPE_RSSI,
		.start_time_us = start_time_us,
		.end_time_us = start_time_us + duration_us,
		.carrier = params->carrier,
		.context = g_active_phy_ctx->mac_ctx,
	};

	// printk("[MOCK_PHY_RSSI_DBG] Exiting mock nrf_modem_dect_phy_rssi.\n");
	return 0;
}

int nrf_modem_dect_phy_activate(enum nrf_modem_dect_phy_radio_mode mode)
{
	g_active_phy_ctx->state = PHY_STATE_ACTIVE;
	struct nrf_modem_dect_phy_event event = {
		.id = NRF_MODEM_DECT_PHY_EVT_ACTIVATE,
		.activate = {.err = NRF_MODEM_DECT_PHY_SUCCESS}};
	mock_phy_send_event(&event);
	return 0;
}

int nrf_modem_dect_phy_cancel(uint32_t handle)
{
	printk("[MOCK_PHY_CANCEL_DBG] Entering mock nrf_modem_dect_phy_cancel for handle %u.\n", handle);

	if (g_active_phy_ctx->state != PHY_STATE_ACTIVE) {
		return -1;
	}

	bool found = false;

	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
		if (g_active_phy_ctx->timeline[i].active &&
		    g_active_phy_ctx->timeline[i].handle == handle) {

			printk("[MOCK_PHY_CANCEL_DBG] Found handle %u in timeline slot %d. Deactivating.\n", handle, i);

			g_active_phy_ctx->timeline[i].active = false;
			found = true;

			struct nrf_modem_dect_phy_event op_event = {
				.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
				.time = k_ticks_to_us_floor64(k_uptime_ticks()),
				.op_complete = {.handle = handle,
						.err = NRF_MODEM_DECT_PHY_ERR_OP_CANCELED}};
			mock_phy_set_active_context(g_active_phy_ctx);
			mock_phy_send_event(&op_event);
			break;
		}
	}

	struct nrf_modem_dect_phy_event cancel_event = {
		.id = NRF_MODEM_DECT_PHY_EVT_CANCELED,
		.time = k_ticks_to_us_floor64(k_uptime_ticks()),
		.cancel = {.handle = handle,
			   .err = found ? NRF_MODEM_DECT_PHY_SUCCESS :
					  NRF_MODEM_DECT_PHY_ERR_NOT_FOUND}};
	mock_phy_set_active_context(g_active_phy_ctx);
	mock_phy_send_event(&cancel_event);

	return 0;
}

/* --- Other dummy implementations --- */
int nrf_modem_dect_phy_deinit(void)
{
	g_active_phy_ctx->state = PHY_STATE_DEINITIALIZED;
	return 0;
}
int nrf_modem_dect_phy_deactivate(void)
{
	g_active_phy_ctx->state = PHY_STATE_INITIALIZED;
	return 0;
}
int nrf_modem_dect_phy_configure(const struct nrf_modem_dect_phy_config_params *params)
{
	return 0;
}
int nrf_modem_dect_phy_tx_harq(const struct nrf_modem_dect_phy_tx_params *params)
{
	return nrf_modem_dect_phy_tx(params);
}
int nrf_modem_dect_phy_tx_rx(const struct nrf_modem_dect_phy_tx_rx_params *params)
{
	return 0;
}
int nrf_modem_dect_phy_capability_get(void)
{
	return 0;
}
int nrf_modem_dect_phy_band_get(void)
{
	return 0;
}
int nrf_modem_dect_phy_time_get(void)
{
	return 0;
}
int nrf_modem_dect_phy_radio_config(
	const struct nrf_modem_dect_phy_radio_config_params *params)
{
	return 0;
}
int nrf_modem_dect_phy_link_config(const struct nrf_modem_dect_phy_link_config_params *params)
{
	return 0;
}
int nrf_modem_dect_phy_stf_cover_seq_control(bool rx_enable, bool tx_enable)
{
	return 0;
}
int nrf_modem_dect_phy_latency_get(void)
{
	return 0;
}

/* --- Internal mock helper functions --- */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event)
{
	if (g_event_handler) {
		g_event_handler(event);
	}
}

// static int find_free_timeline_slot(mock_phy_context_t *ctx)
// {
// 	printk("[MOCK_HELPER_DBG] Entering find_free_timeline_slot...\n");
	
// 	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i)
// 		if (!ctx->timeline[i].active)
// 			return i;
// 	return -1;
// }

static int find_free_timeline_slot(mock_phy_context_t *ctx)
{
    if (!ctx) {
        printk("NULL context passed to find_free_timeline_slot\n");
        return -EINVAL;
    }
    
    // printk("[MOCK_HELPER_DBG] Entering find_free_timeline_slot...\n");
    
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        if (!ctx->timeline[i].active) {
            return i;
        }
    }
	printk("[MOCK_HELPER_DBG] Exiting find_free_timeline_slot (no free slot).\n");
    return -1;
}

static uint64_t get_timeline_end_time(mock_phy_context_t *ctx)
{
	// printk("[MOCK_HELPER_DBG] Entering get_timeline_end_time..\n");

	uint64_t latest_end = 0;
	for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; i++) {
		if (ctx->timeline[i].active && ctx->timeline[i].end_time_us != UINT64_MAX) {
			latest_end = MAX(latest_end, ctx->timeline[i].end_time_us);
		}
	}
	// printk("[MOCK_HELPER_DBG] Exiting get_timeline_end_time. latest_end: %llu\n", latest_end);
	printk("    -  latest_end:%lluus \n", latest_end);
	return latest_end;
}

