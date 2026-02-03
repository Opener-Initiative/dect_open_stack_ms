/* lib/dect_nrplus/mac/mock_nrf_modem_dect_phy.c */
// Overview: Complete, corrected mock PHY with advanced RF simulation (Noise, Loss, Collisions).
// Based on Zephyr kernel clock for time synchronization.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h> /* Added for packet loss probability */

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

/* --- Mock Configuration (New) --- */
static int8_t g_mock_carrier_noise_dbm[MOCK_MAX_CARRIERS];
static uint8_t g_mock_packet_loss_rate_percent = 0;
static bool g_mock_collisions_enabled = false;

/* --- RSSI Result Buffer (Static to avoid stack overflow) --- */
#define MOCK_RSSI_MAX_SAMPLES 10
static int8_t g_mock_rssi_buffer[MOCK_RSSI_MAX_SAMPLES];

/* --- Mock State --- */
static mock_phy_context_t *g_active_phy_ctx = NULL;
static nrf_modem_dect_phy_event_handler_t g_event_handler = NULL;

/* --- Forward declarations for internal functions --- */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event);
static int find_free_timeline_slot(mock_phy_context_t *ctx);
static uint64_t get_timeline_end_time(mock_phy_context_t *ctx);
static void init_rf_environment(void);

/* ========================================================================
 * PUBLIC MOCK CONFIGURATION API
 * ======================================================================== */

void mock_phy_test_set_noise_floor(uint16_t carrier, int8_t dbm)
{
    if (carrier < MOCK_MAX_CARRIERS) {
        g_mock_carrier_noise_dbm[carrier] = dbm;
        printk("[MOCK_CONFIG] Carrier %u noise floor set to %d dBm\n", carrier, dbm);
    }
}

void mock_phy_test_config_error_injection(uint8_t loss_rate_percent)
{
    g_mock_packet_loss_rate_percent = (loss_rate_percent > 100) ? 100 : loss_rate_percent;
    printk("[MOCK_CONFIG] Packet loss rate set to %u%%\n", g_mock_packet_loss_rate_percent);
}

void mock_phy_test_config_collisions(bool enabled)
{
    g_mock_collisions_enabled = enabled;
    printk("[MOCK_CONFIG] Collision detection: %s\n", enabled ? "ENABLED" : "DISABLED");
}

static void init_rf_environment(void)
{
    /* 1. Set a "Noisy" baseline for all carriers (-60 dBm) */
    for (int i = 0; i < MOCK_MAX_CARRIERS; i++) {
        g_mock_carrier_noise_dbm[i] = -60; 
    }

    /* 2. Set the Default Channel to be "Clean" (-100 dBm) */
    /* This ensures the FT DCS algorithm will prefer this channel if RSSI is valid. */
    int default_carrier = 0;
#ifdef CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL
    default_carrier = CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL;
#endif
    
    if (default_carrier < MOCK_MAX_CARRIERS) {
        g_mock_carrier_noise_dbm[default_carrier] = -100;
        printk("[MOCK_INIT] Configured Default Carrier %d to Clean RSSI (-100 dBm)\n", default_carrier);
    }
}
// // static void init_rf_environment(void)
// // {
// //     for (int i = 0; i < MOCK_MAX_CARRIERS; i++) {
// //         g_mock_carrier_noise_dbm[i] = -60; /* Default noise floor */
// //     }
// // #ifdef CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL
// //     if (CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL < MOCK_MAX_CARRIERS) {
// //         g_mock_carrier_noise_dbm[CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL] = -100;
// //     }
// // #endif
// }

/* ========================================================================
 * PUBLIC MOCK CONTROL FUNCTIONS
 * ======================================================================== */

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
    ctx->num_active_rx_ops = 0;
    
    init_rf_environment();
}

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
    
    static bool env_init_done = false;
    if (!env_init_done) {
        init_rf_environment();
        env_init_done = true;
    }
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
			
			size_t len = 8; /* Adjust this based on your needs */
			printk("[MOCK_PHY] PHY_QUEUE_RX_PACKET Payload Hexdump (first %zu bytes): \t", len);
			for (size_t j = 0; j < len && j < packet->pdc_len; j++) {
				printk("%02x ", packet->pdc_payload[j]);
			}
			printk("\n");

			printk("[MOCK_PHY] PHY_QUEUE_RX_QUEUE Payload Hexdump (first %zu bytes): \t", len);
			
			for (size_t j = 0; j < len && j < packet->pdc_len; j++) {
				printk("%02x ", dest_ctx->rx_queue[i].pdc_payload[j]);
			}			
			printk("\n");

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

    for (size_t i = 0; i < num_contexts; i++) {
        mock_phy_context_t *ctx = phy_contexts[i];

        /* Check timeline events */
        for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; j++) {
			// printk("[MOCK_GET_NEXT] ctx->timeline[%d].active:%s ctx->timeline[%d].end_time_us:%lluus \n", 
			// 	j, ctx->timeline[j].active ?"Active":"Inactive",j, ctx->timeline[j].end_time_us);

            if (ctx->timeline[j].active && ctx->timeline[j].end_time_us > current_time) {
                next_event_time = MIN(next_event_time, ctx->timeline[j].end_time_us);
            }
        }

        /* Check RX queue events */
        for (int j = 0; j < MOCK_RX_QUEUE_MAX_PACKETS; j++) {
			if (ctx->rx_queue[j].active){
				printk("[MOCK_GET_NEXT] ctx->rx_queue[%d].active:%s ctx->rx_queue[%d].reception_time_us:%lluus \n", 
					j, ctx->rx_queue[j].active ?"Active":"Inactive", j, ctx->rx_queue[j].reception_time_us);
			}

            if (ctx->rx_queue[j].active &&
                ctx->rx_queue[j].reception_time_us > current_time) {
                next_event_time = MIN(next_event_time, ctx->rx_queue[j].reception_time_us);
            }
        }
    }
	// printk("next_event_time %lluus \n", next_event_time);
	if (next_event_time == UINT64_MAX){
		// next_event_time = current_time + 100;
	}

    return next_event_time;
}

/* Overview: Refactors the mock PHY's event processing to be a true discrete-event simulator. 
** It now finds and processes only the single, next chronological event instead of batch-processing.
** UPDATED: Includes packet loss simulation and collision detection.
*/
void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us)
{
    /* --- Phase 1: Start Scheduled Operations --- */
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        mock_scheduled_operation_t *op = &ctx->timeline[i];
        if (op->active && !op->running && op->start_time_us <= current_time_us) {
            op->running = true;
        }
    }
    
    /* --- Phase 2: Complete Scheduled Operations --- */
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        mock_scheduled_operation_t *op = &ctx->timeline[i];
        if (op->active && op->running && op->end_time_us <= current_time_us) {
            mock_phy_set_active_context(ctx);
            
            /* Handle RSSI completion specially */
            if (op->type == MOCK_OP_TYPE_RSSI) {
				int8_t rssi_val = (op->carrier < MOCK_MAX_CARRIERS) ? g_mock_carrier_noise_dbm[op->carrier] : -60;
                
                /* Fill the static buffer with the measured value */
                memset(g_mock_rssi_buffer, rssi_val, MOCK_RSSI_MAX_SAMPLES);
                
                printk("[MOCK_RSSI_REPORT] Handle: %u, Carrier: %u, RSSI: %d dBm (Simulating %d samples)\n", 
                       op->handle, op->carrier, rssi_val, MOCK_RSSI_MAX_SAMPLES);
                // int8_t dummy_meas[1];
                // dummy_meas[0] = (op->carrier < MOCK_MAX_CARRIERS) ? g_mock_carrier_noise_dbm[op->carrier] : -60;
                
                // printk("[MOCK_RSSI_REPORT] Handle: %u, Carrier: %u, RSSI: %d dBm\n", 
                //        op->handle, op->carrier, dummy_meas[0]);
                
                struct nrf_modem_dect_phy_event rssi_evt = {
                    .id = NRF_MODEM_DECT_PHY_EVT_RSSI,
                    .time = op->end_time_us,
                    .rssi = { 
                        .handle = op->handle, 
                        .meas_start_time = op->start_time_us, 
                        .carrier = op->carrier,
						.meas_len = MOCK_RSSI_MAX_SAMPLES, /* Return multiple samples */
                        .meas = g_mock_rssi_buffer         /* Pointer to full buffer */
                        // .meas_len = 1, 
                        // .meas = dummy_meas 
                    }
                };
                mock_phy_send_event(&rssi_evt);
            }
            
            /* General completion event */
            struct nrf_modem_dect_phy_event complete_evt = {
                .id = NRF_MODEM_DECT_PHY_EVT_COMPLETED,
                .time = op->end_time_us,
                .op_complete = {.handle = op->handle, .err = NRF_MODEM_DECT_PHY_SUCCESS}
            };
            mock_phy_send_event(&complete_evt);
            op->active = false;
        }
    }

    /* --- Phase 3: Process Incoming RX Packets --- */
    /* Find the single next RX packet to process */
    int next_pkt_idx = -1;
    uint64_t earliest_time = UINT64_MAX;
    
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
        if (ctx->rx_queue[i].active && ctx->rx_queue[i].reception_time_us <= current_time_us) {
            if (ctx->rx_queue[i].reception_time_us < earliest_time) {
                earliest_time = ctx->rx_queue[i].reception_time_us;
                next_pkt_idx = i;
            }
        }
    }

    if (next_pkt_idx == -1) {
        return; /* No packets to process at this time */
    }

    mock_rx_packet_t *pkt = &ctx->rx_queue[next_pkt_idx];

    /* --- Feature: Packet Loss Simulation --- */
    if (g_mock_packet_loss_rate_percent > 0 && (sys_rand32_get() % 100 < g_mock_packet_loss_rate_percent)) {
        printk("[MOCK_RF_LOSS] Probability drop. Time: %llu, Carrier: %u\n", pkt->reception_time_us, pkt->carrier);
        pkt->active = false; /* Silently drop */
        return; /* Will pick up next packet in next iteration */
    }

    /* --- Feature: Collision Detection --- */
    bool collision = false;
    if (g_mock_collisions_enabled) {
        for (int k = 0; k < MOCK_RX_QUEUE_MAX_PACKETS; k++) {
            if (k != next_pkt_idx && ctx->rx_queue[k].active && ctx->rx_queue[k].carrier == pkt->carrier) {
                bool overlaps = (pkt->reception_time_us < (ctx->rx_queue[k].reception_time_us + ctx->rx_queue[k].duration_us)) &&
                                (ctx->rx_queue[k].reception_time_us < (pkt->reception_time_us + pkt->duration_us));
                if (overlaps) { 
                    collision = true; 
                    printk("[MOCK_RF_COLLISION] Detected overlap between slot %d and %d on carrier %u\n", next_pkt_idx, k, pkt->carrier);
                    break; 
                }
            }
        }
    }

    /* Find a matching, active RX operation on the timeline */
    bool rx_op_found = false;
    for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
        mock_scheduled_operation_t *rx_op = &ctx->timeline[j];

        if (rx_op->active && rx_op->running && rx_op->type == MOCK_OP_TYPE_RX) {
            if (ctx->mac_ctx->role == MAC_ROLE_FT) {
                printk("[FT_RX_MATCH_DBG] Checking timeline slot %d: active=%d, running=%d, type=%d, carrier=%u\n",
                       j, rx_op->active, rx_op->running, rx_op->type, rx_op->carrier);
                printk("[CARRIER_CHECK_DBG] Comparing packet carrier %u with RX op carrier %u.\n",
                       pkt->carrier, rx_op->carrier);
            }

            if (pkt->carrier == rx_op->carrier) {
                mock_phy_set_active_context(ctx);
                rx_op_found = true;

                if (collision) {
                    /* Collision Logic: Send PCC Error */
                    printk("[MOCK_RF_COLLISION] Reporting PCC_ERROR for Handle %u at %llu.\n", rx_op->handle, pkt->reception_time_us);
                    
                    /* Generate unique transaction ID for error reporting */
                    ctx->transaction_id_counter++; 
                    if (ctx->transaction_id_counter == 0) ctx->transaction_id_counter = 1;
                    
                    struct nrf_modem_dect_phy_event pcc_err = {
                        .id = NRF_MODEM_DECT_PHY_EVT_PCC_ERROR,
                        .time = pkt->reception_time_us,
                        .pcc_crc_err = { .handle = rx_op->handle, .transaction_id = ctx->transaction_id_counter }
                    };
                    mock_phy_send_event(&pcc_err);
                } else {
                    /* Success Logic: Send PCC then PDC */
                    printk("[MOCK_PROCESS_RX_DBG] Delivering packet from slot %d. Handle: %u\n", next_pkt_idx, rx_op->handle);
                    
                    struct nrf_modem_dect_phy_event pcc_evt = {
                        .id = NRF_MODEM_DECT_PHY_EVT_PCC,
                        .time = pkt->reception_time_us,
                        .pcc = pkt->pcc_data};
                    pcc_evt.pcc.handle = rx_op->handle;
                    mock_phy_send_event(&pcc_evt);

                    struct nrf_modem_dect_phy_event pdc_evt = {
                        .id = NRF_MODEM_DECT_PHY_EVT_PDC,
                        .time = pkt->reception_time_us,
                        .pdc = {.handle = rx_op->handle, .data = pkt->pdc_payload, .len = pkt->pdc_len}};
                    mock_phy_send_event(&pdc_evt);
                }
                break; /* Processed one packet, exit loop */
            }
        }
    }
    
    if (!rx_op_found && !collision) {
        printk("[MOCK_PROCESS_RX_DBG] No matching RX Op found for packet on carrier %u\n", pkt->carrier);
    }

    /* Deactivate the processed packet */
    pkt->active = false;
}

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
        printk("MOCK_PHY: Forcing PHY schedule failure with code %d", g_phy_schedule_failure_code);
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

    uint64_t start_time_us = (params->start_time == 0) ?
                  MAX(k_ticks_to_us_floor64(k_uptime_ticks()), get_timeline_end_time(g_active_phy_ctx)) :
                  params->start_time;

    /* Simplified duration based on payload size for simulation */
    uint64_t duration_us = modem_ticks_to_us(params->data_size * 8 * 10, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
    if (duration_us < 100) duration_us = 100; /* Minimum duration */

    g_active_phy_ctx->timeline[slot] = (mock_scheduled_operation_t){
        .active = true,
        .running = false,
        .handle = params->handle,
        .type = MOCK_OP_TYPE_TX,
        .start_time_us = start_time_us,
        .end_time_us = start_time_us + duration_us,
        .duration_us = duration_us,
        .pdu_len = params->data_size,
        .carrier = params->carrier,
        .context = g_active_phy_ctx->mac_ctx,
    };
    if (params->data) {
        memcpy(g_active_phy_ctx->timeline[slot].pdu_data, params->data, params->data_size);
    }
    g_last_tx_pdu_len_capture = params->data_size;
    memcpy(g_last_tx_pdu_capture, params->data, params->data_size);

    printk("\n--- DEBUG_PROBE: nrf_modem_dect_phy_tx ---\n");
    printk("  - Transmitter is: %s\n", (g_active_phy_ctx->mac_ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
    printk("  - Number of configured peers: %zu\n", g_active_phy_ctx->num_peers);

    /* Simulate air: queue this packet for all potential peers that are listening */
    for (size_t i = 0; i < g_active_phy_ctx->num_peers; i++) {
        mock_phy_context_t *peer_ctx = g_active_phy_ctx->peers[i];
        
        if (peer_ctx) {
            printk("  - Packet to Queue for peer %d (context %p)\n", (int)i, (void *)peer_ctx);
            
            mock_rx_packet_t rx_pkt = {
                .active = true,
                .reception_time_us = start_time_us + 100, /* 100us prop delay */
                .duration_us = duration_us,               /* Stored for collision detection */
                .carrier = params->carrier,
                .pcc_data = {
                    .phy_type = params->phy_type,
                    .hdr = *params->phy_header,
                    .rssi_2 = -60, /* Default strong signal */
                    .snr = 80,     /* 20 dB in Q13.2 */
                },
                .pdc_len = params->data_size,
            };
            if (params->data) {
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

    uint64_t start_time_us = (params->start_time == 0) ?
                  MAX(k_ticks_to_us_floor64(k_uptime_ticks()), get_timeline_end_time(g_active_phy_ctx)) :
                  params->start_time;

    uint64_t end_time_us = (params->duration == 0) ? UINT64_MAX :
                  (start_time_us + modem_ticks_to_us(params->duration, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ));

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
        return -1;
    }

    int slot = find_free_timeline_slot(g_active_phy_ctx);
    if (slot < 0) {
        return -ENOMEM;
    }

    uint64_t start_time_us = (params->start_time == 0) ?
                  MAX(k_ticks_to_us_floor64(k_uptime_ticks()), get_timeline_end_time(g_active_phy_ctx)) :
                  params->start_time;

    uint64_t duration_us = modem_ticks_to_us(params->duration, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

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
int nrf_modem_dect_phy_deinit(void) { g_active_phy_ctx->state = PHY_STATE_DEINITIALIZED; return 0; }
int nrf_modem_dect_phy_deactivate(void) { g_active_phy_ctx->state = PHY_STATE_INITIALIZED; return 0; }
int nrf_modem_dect_phy_configure(const struct nrf_modem_dect_phy_config_params *params) { return 0; }
int nrf_modem_dect_phy_tx_harq(const struct nrf_modem_dect_phy_tx_params *params) { return nrf_modem_dect_phy_tx(params); }
int nrf_modem_dect_phy_tx_rx(const struct nrf_modem_dect_phy_tx_rx_params *params) { return 0; }
int nrf_modem_dect_phy_capability_get(void) { return 0; }
int nrf_modem_dect_phy_band_get(void) { return 0; }
int nrf_modem_dect_phy_time_get(void) { return 0; }
int nrf_modem_dect_phy_radio_config(const struct nrf_modem_dect_phy_radio_config_params *params) { return 0; }
int nrf_modem_dect_phy_link_config(const struct nrf_modem_dect_phy_link_config_params *params) { return 0; }
int nrf_modem_dect_phy_stf_cover_seq_control(bool rx_enable, bool tx_enable) { return 0; }
int nrf_modem_dect_phy_latency_get(void) { return 0; }

/* --- Internal mock helper functions --- */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event)
{
    if (g_event_handler) {
        g_event_handler(event);
    }
}

static int find_free_timeline_slot(mock_phy_context_t *ctx)
{
    if (!ctx) return -EINVAL;
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        if (!ctx->timeline[i].active) return i;
    }
    return -1;
}

static uint64_t get_timeline_end_time(mock_phy_context_t *ctx)
{
    uint64_t latest_end = 0;
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; i++) {
        if (ctx->timeline[i].active && ctx->timeline[i].end_time_us != UINT64_MAX) {
            latest_end = MAX(latest_end, ctx->timeline[i].end_time_us);
        }
    }
    printk("    -  latest_end:%lluus \n", latest_end);
    return latest_end;
}