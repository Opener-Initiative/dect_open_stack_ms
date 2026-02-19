/* dect_mac/dect_mac_sm_pt.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/sys/util.h>

#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_ctrl.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac.h>
#include <dect_cdd.h>
#include <mac/dect_mac_security.h>
#include <mac/dect_mac_timeline_utils.h>
#include <mac/dect_mac_random.h>

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
#include <psa/crypto.h>
#endif


LOG_MODULE_REGISTER(dect_mac_sm_pt, CONFIG_DECT_MAC_SM_PT_LOG_LEVEL);

#ifndef HPC_RX_WINDOW_SIZE
#define HPC_RX_WINDOW_SIZE 64
#endif
#ifndef HPC_RX_FORWARD_WINDOW_MAX_ADVANCE
// Allow a jump of up to 1024
#define HPC_RX_FORWARD_WINDOW_MAX_ADVANCE 1024
#endif

static void print_pcc_cache_state(const char *log_prefix)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	printk("[%s] PCC Cache State:\n", log_prefix);
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		pcc_transaction_t *t = &ctx->pcc_transaction_cache[i];
		if (t->is_valid) {
			printk("  Slot[%d]: VALID | TID: %5u | Time: %llu\n", i, t->transaction_id,
			       t->reception_time_us);

			/* Add hexdump of the raw PCC header */
			const uint8_t *hdr_bytes = (const uint8_t *)&t->pcc_data.hdr;
			/* nRF PHY Type 0 is ETSI Type 1 (5 bytes), Type 1 is ETSI Type 2 (10 bytes) */
			size_t hdr_len = (t->pcc_data.phy_type == 0) ? 5 : 10;
			printk("    -> HDR: ");
			for (size_t j = 0; j < hdr_len; j++) {
				printk("%02x ", hdr_bytes[j]);
			}
			printk("\n");

		} else {
			// printk("  Slot[%d]: INVALID\n", i);
		}
	}
}

// --- Static Helper Function Prototypes ---
static void pt_handle_phy_op_complete_internal(const struct nrf_modem_dect_phy_op_complete_event *event, pending_op_type_t completed_op_type);
static void pt_handle_phy_pcc_internal(const struct nrf_modem_dect_phy_pcc_event *event, uint64_t pcc_event_time);
static void pt_handle_phy_pdc_internal(const struct nrf_modem_dect_phy_pdc_event *pdc_event, const struct nrf_modem_dect_phy_pcc_event *assoc_pcc_event, uint64_t pcc_reception_modem_time);

static void pt_update_mobility_candidate(uint16_t carrier, int16_t rssi, uint32_t long_id, uint16_t short_id);

static void pt_process_identified_beacon_and_attempt_assoc(dect_mac_context_t *ctx,
                                                           const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
                                                           const dect_mac_rach_info_ie_fields_t *rach_fields,
                                                           uint32_t ft_long_id, uint16_t ft_short_id, int16_t rssi,
                                                           uint16_t beacon_rx_carrier, uint64_t beacon_pcc_rx_time);
static void pt_send_association_request_action(void);
static void pt_process_association_response_pdu(const uint8_t *mac_sdu_area_data, size_t mac_sdu_area_len,
                                                uint32_t ft_tx_long_rd_id, uint64_t assoc_resp_pcc_rx_time);
static void pt_send_keep_alive_action(void);
static void pt_process_page_indication(void);
static void pt_paging_cycle_timer_expired_action(void);
// static void pt_beacon_listen_timer_expiry_fn(struct k_timer *timer_id);

static void pt_authentication_complete_action(dect_mac_context_t* ctx, bool success); // Called after key derivation	

// static void pt_initiate_background_mobility_scan_action(void); // TODO
/***************************************  UNUSED WARNING ***************************************/
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
// static void pt_handle_phy_rssi_internal(const struct nrf_modem_dect_phy_rssi_event *event);
// static void pt_start_authentication_with_ft_action(dect_mac_context_t *ctx); // Simplified for PSK
// static void pt_initiate_authentication_handshake(
	// dect_mac_context_t *ctx, const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
	// const dect_mac_rach_info_ie_fields_t *rach_fields, uint32_t ft_long_id,
	// uint16_t ft_short_id, int16_t rssi_q7_1, uint16_t beacon_rx_carrier,
	// uint64_t beacon_pcc_rx_time);


static void pt_send_auth_response_action(void);
static void pt_process_auth_success(dect_mac_context_t *ctx, const uint8_t *ft_mac);
#endif

static void pt_send_auth_initiate_action(void);
static void pt_process_group_assignment_ie(const uint8_t *payload, uint16_t len);
static void pt_evaluate_mobility_candidate(dect_mac_context_t *ctx,
					   const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
					   uint32_t ft_long_id, uint16_t ft_short_id,
					   int16_t rssi_q7_1, uint16_t beacon_rx_carrier);
static void pt_initiate_handover_action(dect_mac_context_t *ctx,
					dect_mobility_candidate_t *candidate);
void pt_send_association_release_action(dect_mac_peer_info_t *old_ft_info);


// Helpers from data_path or utils (ensure they are accessible)
extern uint32_t get_subslot_duration_ticks(dect_mac_context_t *ctx);
extern uint64_t modem_us_to_ticks(uint64_t us, uint32_t tick_rate_khz);
extern void update_next_occurrence(dect_mac_context_t *ctx, dect_mac_schedule_t *schedule, uint64_t current_modem_time, uint8_t link_mu_code);
extern uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time, uint8_t sfn_of_anchor_relevance, uint8_t target_sfn_val, uint16_t target_subslot_idx, uint8_t link_mu_code, uint8_t link_beta_code);

// --- PT Timer Expiry Action Functions (called by dispatcher) ---
void pt_rach_backoff_timer_expired_action(void) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    LOG_INF("PT SM: RACH Backoff timer expired.");
    if (ctx->role_ctx.pt.target_ft.is_valid && ctx->role_ctx.pt.target_ft.is_fully_identified) {
        LOG_INF("PT SM: Retrying Association Request to FT ShortID 0x%04X.", ctx->role_ctx.pt.target_ft.short_rd_id);
        // State should already be PT_RACH_BACKOFF, send_association_request changes to PT_ASSOCIATING
        pt_send_association_request_action();
    } else {
        LOG_ERR("PT SM: RACH backoff expired, but no valid/fully_identified target FT. Restarting scan.");
        dect_mac_sm_pt_start_operation();
    }
}


bool pt_get_next_tx_opportunity(uint64_t *out_start_time, uint16_t *out_carrier,
				dect_mac_schedule_t *out_schedule)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

    printk("[DEBUG_PROBE] pt_get_next_tx_opportunity called. Current state: %s(%d)\n",
           dect_mac_state_to_str(ctx->state), ctx->state);
    // printk("  - Checking UL schedule: is_active = %s\n",
    //        ctx->role_ctx.pt.ul_schedule.is_active ? "Active":"Idle");

	if (ctx->state != MAC_STATE_ASSOCIATED && ctx->state != MAC_STATE_PT_RELEASING) {
		printk("[ERROR] pt_get_next_tx_opportunity called. Current state: %s(%d)\n",
           dect_mac_state_to_str(ctx->state), ctx->state);
		return false;
	}

	dect_mac_schedule_t *sched = &ctx->role_ctx.pt.ul_schedule;

	printk("  - Checking UL schedule: is_active = %d, validity_value = %u ul_duration_subslots=%u\n",
	       sched->is_active, sched->validity_value, sched->ul_duration_subslots);

	if (!sched->is_active || sched->ul_duration_subslots == 0) {
		printk("  - (!sched->is_active || sched->ul_duration_subslots == 0)  \n");
		return false;
	}

	uint8_t peer_mu = ctx->role_ctx.pt.associated_ft.peer_mu;
	// update_next_occurrence(ctx, sched, ctx->last_known_modem_time, peer_mu);
	update_next_occurrence(ctx, sched, k_ticks_to_us_floor64(k_uptime_ticks()), peer_mu);	

	if (!sched->is_active) {
		printk("  - !sched->is_active \n");
		return false;
	}

	uint32_t transition_ticks = modem_us_to_ticks(
		ctx->phy_latency.scheduled_operation_transition_us,
		NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
	uint32_t prep_latency_ticks = modem_us_to_ticks(
		ctx->phy_latency.idle_to_active_tx_us +
			ctx->phy_latency.scheduled_operation_startup_us,
		NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	uint64_t earliest_start = k_ticks_to_us_floor64(k_uptime_ticks()) + 
		modem_ticks_to_us(prep_latency_ticks, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	/* If another operation just ended or will end, ensure transition guard band */
	if (ctx->last_phy_op_end_time > 0) {
		uint64_t transition_earliest = ctx->last_phy_op_end_time + 
			modem_ticks_to_us(transition_ticks, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
		if (transition_earliest > earliest_start) {
			earliest_start = transition_earliest;
		}
	}

	if (sched->next_occurrence_modem_time >= earliest_start) {
		*out_start_time = sched->next_occurrence_modem_time;
		*out_carrier = sched->channel;
		*out_schedule = *sched;
		printk("  - out_start_time:%lluus \n", sched->next_occurrence_modem_time);
		return true;
	}
	printk("[ERROR] pt_get_next_tx_opportunity return FALSE (earliest %llu)\n", earliest_start);
	return false;
}

static void pt_requeue_held_packets(dect_mac_context_t *ctx)
{
	mac_sdu_t *sdu;
	int count = 0;

	while ((sdu = k_queue_get(&ctx->role_ctx.pt.handover_tx_holding_queue, K_NO_WAIT)) != NULL) 
	{
        /* Prepend to the reliable queue to send them out first */
		k_queue_prepend(mac_tx_queues[MAC_FLOW_RELIABLE_DATA], sdu);
		count++;
	}

	if (count > 0) {
		LOG_INF("MOBILITY: Re-queued %d held packets for transmission to new/reverted FT.", count);
	}
}


static void pt_revert_to_old_ft_after_handover_failure(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	LOG_ERR("MOBILITY: Handover to FT 0x%04X failed. Reverting to old FT 0x%04X.",
		ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.associated_ft.short_rd_id);

    pt_requeue_held_packets(ctx);

	/* Clear the failed target */
	memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
	ctx->role_ctx.pt.target_ft.is_valid = false;

	/* Cancel any active HARQ processes that were for the failed handover attempt.
	 * Re-queue their SDUs to be sent to the original FT.
	 */
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];

		if (harq_p->is_active) {
			LOG_WRN("MOBILITY: Cancelling active HARQ process %d (for PSN %u) due to handover failure.",
				i, harq_p->original_psn);
			k_timer_stop(&harq_p->retransmission_timer);
			if (harq_p->sdu) {
				/* Prepend to the reliable queue to ensure it's sent first */
				k_queue_prepend(mac_tx_queues[MAC_FLOW_RELIABLE_DATA], harq_p->sdu);
								harq_p->sdu = NULL;
			}
			/* Reset the HARQ process */
			memset(harq_p, 0, sizeof(dect_harq_tx_process_t));
			k_timer_init(&harq_p->retransmission_timer,
				     dect_mac_data_path_harq_timer_expired, NULL);
			harq_p->retransmission_timer.user_data = (void *)((uintptr_t)i);
		}
	}

	/* Revert state and restart timers for the old (and now current) FT */
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	printk("MOBILITY: State -> ASSOCIATED. Data TX resumed to old FT.\n");
	// LOG_INF("MOBILITY: State -> ASSOCIATED. Data TX resumed to old FT.");

	k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
		      K_MSEC(ctx->config.keep_alive_period_ms),
		      K_MSEC(ctx->config.keep_alive_period_ms));
	if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
		k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
			      K_MSEC(ctx->config.mobility_scan_interval_ms),
			      K_MSEC(ctx->config.mobility_scan_interval_ms));
	}
}


static void pt_paging_cycle_timer_expired_action(void)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (ctx->state != MAC_STATE_PT_PAGING) {
        LOG_WRN("PT_PAGING: Paging timer fired in unexpected state %s. Stopping timer.",
                dect_mac_state_to_str(ctx->state));
        k_timer_stop(&ctx->role_ctx.pt.paging_cycle_timer);
        return;
    }

    // if (ctx->pending_op_type != PENDING_OP_NONE) {
    //     LOG_WRN("PT_PAGING: Paging listen time, but op %s pending. Will retry shortly.",
    //             dect_pending_op_to_str(ctx->pending_op_type));
    //     k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer, K_MSEC(100), K_NO_WAIT); // Quick retry
    //     return;
    // }

    LOG_INF("PT_PAGING: Waking up to listen for page (on FT carrier %u).",
            ctx->role_ctx.pt.associated_ft.operating_carrier);

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));
    uint32_t listen_duration_modem_units = get_subslot_duration_ticks(ctx) *
                                           SUB_SLOTS_PER_ETSI_SLOT * 2;

    int ret = dect_mac_phy_ctrl_start_rx(
        ctx->role_ctx.pt.associated_ft.operating_carrier,
        listen_duration_modem_units,
        NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
        phy_op_handle,
        0xFFFF,
        PENDING_OP_PT_PAGING_LISTEN);

    if (ret != 0) {
        LOG_ERR("PT_PAGING: Failed to schedule RX for paging listen: %d. Retrying shortly.", ret);
        k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer, K_MSEC(200), K_NO_WAIT);
    }
}


void pt_rach_response_window_timer_expired_action(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	k_timer_stop(&ctx->rach_context.rach_response_window_timer);

	bool was_handover = (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING);

	if (ctx->state != MAC_STATE_PT_WAIT_ASSOC_RESP && !was_handover) {
		LOG_WRN("PT_RACH_RESP_TIMEOUT: Timer expired in unexpected state %s. Ignoring.",
			dect_mac_state_to_str(ctx->state));
		return;
	}

	LOG_WRN("PT_RACH_RESP_TIMEOUT: No Association Response from target FT 0x%04X (L:0x%08X).",
		ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.target_ft.long_rd_id);

	/* Cancel the listening operation that was started */
	if (ctx->pending_op_type == PENDING_OP_PT_WAIT_ASSOC_RESP) {
		/* CRITICAL: Clear the pending operation state synchronously before attempting
		 * to schedule a new operation. The cancel call is best-effort.
		 */
		// dect_mac_core_clear_pending_op();		
		dect_mac_phy_ctrl_cancel_op(ctx->pending_op_handle);
	}

	if (was_handover) {
		pt_revert_to_old_ft_after_handover_failure();
		return;
	}

	ctx->role_ctx.pt.current_assoc_retries++;
	if (ctx->role_ctx.pt.target_ft.is_valid &&
	    ctx->role_ctx.pt.current_assoc_retries < ctx->config.max_assoc_retries) {
		LOG_INF("PT_RACH_RESP_TIMEOUT: Retrying association to FT 0x%04X (attempt %u / %u).",
			ctx->role_ctx.pt.target_ft.short_rd_id,
			ctx->role_ctx.pt.current_assoc_retries + 1, ctx->config.max_assoc_retries);

		uint8_t ft_cwmin_code =
			ctx->role_ctx.pt.current_ft_rach_params
				.advertised_beacon_ie_fields.cwmin_sig_code;
		uint8_t ft_cwmax_code =
			ctx->role_ctx.pt.current_ft_rach_params
				.advertised_beacon_ie_fields.cwmax_sig_code;

		if (ctx->rach_context.rach_cw_current_idx < ft_cwmin_code) {
			ctx->rach_context.rach_cw_current_idx = ft_cwmin_code;
		}

		if (ctx->rach_context.rach_cw_current_idx < ft_cwmax_code) {
			ctx->rach_context.rach_cw_current_idx++;
			LOG_DBG("PT_RACH_RESP_TIMEOUT: Increased CW index to %u for next attempt.",
				ctx->rach_context.rach_cw_current_idx);
		} else {
			LOG_DBG("PT_RACH_RESP_TIMEOUT: CW index already at max (%u) from FT.",
				ft_cwmax_code);
		}
		pt_send_association_request_action();

	} else {
		LOG_ERR("PT_RACH_RESP_TIMEOUT: Max association retries (%u) for FT 0x%04X or no valid target. Restarting scan.",
			ctx->config.max_assoc_retries, ctx->role_ctx.pt.target_ft.short_rd_id);
		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.target_ft.is_valid = false;
		dect_mac_sm_pt_start_operation();
	}
}

void dect_mac_sm_pt_keep_alive_timer_expired_action(void) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (ctx->state == MAC_STATE_ASSOCIATED) {
        if (ctx->pending_op_type == PENDING_OP_NONE) {
            pt_send_keep_alive_action();
        } else {
            LOG_WRN("PT SM: Keep-alive time, but op %s pending. Will retry on next expiry.",
                    dect_pending_op_to_str(ctx->pending_op_type));
            // Periodic timer will fire again.
        }
    }
}

void dect_mac_sm_pt_mobility_scan_timer_expired_action(void)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    if (ctx->state != MAC_STATE_ASSOCIATED) { // Only scan for mobility if associated
        LOG_DBG("PT SM: Mobility scan timer fired but not associated. Restarting general scan.");
        dect_mac_sm_pt_start_operation();
        return;
    }

    // if (ctx->pending_op_type != PENDING_OP_NONE) {
    //     printk("PT SM: Mobility scan time, but op %s pending. Deferring scan.",
    //             dect_pending_op_to_str(ctx->pending_op_type));
    //     // LOG_WRN("PT SM: Mobility scan time, but op %s pending. Deferring scan.",
    //     //         dect_pending_op_to_str(ctx->pending_op_type));				
    //     // The timer will fire again later.
    //     return;
    // }

    // Simple channel selection logic: scan the next channel.
    // TODO: A production system would use a more sophisticated channel hopping sequence.
    uint32_t current_carrier = ctx->role_ctx.pt.associated_ft.operating_carrier;
    uint32_t scan_carrier = (current_carrier != 0) ? (current_carrier ) : DEFAULT_DECT_CARRIER;
    // TODO: Add logic to wrap around the valid channel range.

    LOG_INF("PT SM: Starting mobility background RSSI scan on carrier %u.", scan_carrier);

    // uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
    // A short scan, e.g., for one or two slots duration.
    uint32_t scan_duration_modem_units = get_subslot_duration_ticks(ctx) * SUB_SLOTS_PER_ETSI_SLOT * 2;

    int ret = dect_mac_phy_ctrl_start_rssi_scan(
        scan_carrier,
        scan_duration_modem_units,
        NRF_MODEM_DECT_PHY_RSSI_INTERVAL_24_SLOTS, // Get one report for this short scan
        phy_op_handle,
        PENDING_OP_PT_MOBILITY_SCAN);

    if (ret != 0) {
        LOG_ERR("PT SM: Failed to start mobility RSSI scan: %d.", ret);
        // The periodic timer will try again on its next cycle.
    }
}


// --- PT Public Functions ---
void dect_mac_sm_pt_start_operation(void)
{
	printk("[PT_START_OP_DBG] Entering dect_mac_sm_pt_start_operation...\n");

	dect_mac_context_t *ctx = dect_mac_get_active_context();

	/* Hard reset of the context before starting PT operations */
	dect_mac_reset_context(ctx);

	dect_mac_change_state(MAC_STATE_PT_SCANNING);
	uint16_t scan_carrier = CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL;
	printk("PT_SM_START_DBG: Initializing scan with carrier value: %u (0x%X)\n", scan_carrier, scan_carrier);
	LOG_INF("PT SM: Starting scan for FT beacons on carrier %u.", scan_carrier);

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));

	int ret = dect_mac_phy_ctrl_start_rx(scan_carrier, 0,
					   NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
					   phy_op_handle, 0xFFFF,
					   PENDING_OP_PT_SCAN);
	if (ret != 0) {
		dect_mac_enter_error_state("Failed to start initial PT scan");
	}
	// printk("[PT_START_OP_DBG] Exiting dect_mac_sm_pt_start_operation.\n");
}


static void pt_handle_phy_pdc_pt(const struct nrf_modem_dect_phy_pdc_event *pdc_event,
				 uint64_t event_modem_time)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	int pcc_slot = -1;


	/* First, extract the FT's Long ID from the MAC header to check against the reject timer */
	if (pdc_event->len < (sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_beacon_header_t))) {
		return; /* Not a valid beacon */
	}
	const dect_mac_beacon_header_t *beacon_hdr = (const dect_mac_beacon_header_t *)((uint8_t *)pdc_event->data + sizeof(dect_mac_header_type_octet_t));
	uint32_t ft_long_id = sys_be32_to_cpu(beacon_hdr->transmitter_long_rd_id_be);

	/* Check if we are in a reject backoff period for this FT */
	if (ctx->role_ctx.pt.target_ft.long_rd_id == ft_long_id &&
	    k_timer_remaining_get(&ctx->role_ctx.pt.reject_timer) > 0) {
		LOG_DBG("PT_SCAN: Ignoring beacon from FT 0x%08X due to active reject timer.",
			ft_long_id);
		return;
	}

	/*
	 * This is the point of synchronization. The PT adopts the FT's SFN and
	 * time anchor as its own to ensure all future schedule calculations are correct.
	 */
	const dect_mac_header_type_octet_t *hdr_type = (const dect_mac_header_type_octet_t *)pdc_event->data;
	if (hdr_type->mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
		const uint8_t *sdu_area = (const uint8_t *)pdc_event->data + sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_beacon_header_t);
		uint16_t sdu_area_len = pdc_event->len - (sizeof(dect_mac_header_type_octet_t) + sizeof(dect_mac_beacon_header_t));
		uint8_t ie_type;
		uint16_t ie_len;
		const uint8_t *ie_payload;
		int mux_hdr_len = parse_mac_mux_header(sdu_area, sdu_area_len, &ie_type, &ie_len, &ie_payload);
		if (mux_hdr_len > 0 && ie_type == IE_TYPE_CLUSTER_BEACON) {
			dect_mac_cluster_beacon_ie_fields_t cb_fields;
			if (parse_cluster_beacon_ie_payload(ie_payload, ie_len, &cb_fields) == 0) {
				ctx->ft_sfn_zero_modem_time_anchor = event_modem_time - k_us_to_ticks_ceil64(cb_fields.sfn * FRAME_DURATION_MS_NOMINAL * 1000);
				ctx->current_sfn_at_anchor_update = cb_fields.sfn;
				ctx->role_ctx.ft.sfn = cb_fields.sfn;
				LOG_INF("PT_SYNC: Synchronized to FT. SFN is now %u, anchor time is %llu.",
					ctx->role_ctx.ft.sfn, ctx->ft_sfn_zero_modem_time_anchor);
			}
		}
	}	

	printk("[PT_PDC_HANDLER] Received PDC with TID %u. Searching cache...\n",
	       pdc_event->transaction_id);

	print_pcc_cache_state("PT_PDC_SEARCH");

	/* Find the matching PCC in the cache by transaction ID */
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		if (ctx->pcc_transaction_cache[i].is_valid &&
		    ctx->pcc_transaction_cache[i].transaction_id == pdc_event->transaction_id) {
			pcc_slot = i;
			printk("[PT_PDC_HANDLER] Found matching PCC with TID %u in slot %d.\n",
			       ctx->pcc_transaction_cache[i].transaction_id, i);
			break;
		}
	}

	if (pcc_slot == -1) {
		printk("***FAILURE***: No matching PCC found for TID %u. Discarding PDC.\n",
		       pdc_event->transaction_id);

		LOG_WRN("PT_SM_PDC_WRAP: PDC (TID %u) but no matching valid PCC stored. Discarding.",
			pdc_event->transaction_id);
		return;
	}

	/* We found a match. Process it and then invalidate the cache entry. */
	pcc_transaction_t *transaction = &ctx->pcc_transaction_cache[pcc_slot];

	k_timer_stop(&transaction->timeout_timer);

	const struct nrf_modem_dect_phy_pcc_event *assoc_pcc_event = &transaction->pcc_data;
	uint64_t pcc_reception_modem_time = transaction->reception_time_us;

	pt_handle_phy_pdc_internal(pdc_event, assoc_pcc_event, pcc_reception_modem_time);

	/* Invalidate the cache entry now that it has been fully processed */
	transaction->is_valid = false;
}


void dect_mac_sm_pt_handle_event(const struct dect_mac_event_msg *msg)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
// printk("[PT_SM] dect_mac_sm_pt_handle_event: ctx->state:%s(%d):: msg->type:%s(%d) \n", dect_mac_state_to_str(ctx->state), ctx->state, dect_mac_event_to_str(msg->type), msg->type);

	switch (ctx->state) {
	case MAC_STATE_IDLE:
		/* In IDLE, the only action is to start scanning, which is not event-driven */
		break;

	case MAC_STATE_PT_SCANNING:
	case MAC_STATE_PT_BEACON_PDC_WAIT:
		if (msg->type == MAC_EVENT_PHY_PCC) {
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
		} else if (msg->type == MAC_EVENT_PHY_PDC) {
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
		} else if (msg->type == MAC_EVENT_PHY_OP_COMPLETE) {
			printk("[PT_SM] state->MAC_STATE_PT_BEACON_PDC_WAIT && msg->type == MAC_EVENT_PHY_OP_COMPLETE \n");
			pending_op_type_t op =
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete);
			if (op == PENDING_OP_PT_SCAN) {
				pt_handle_phy_op_complete_internal(&msg->data.op_complete, op);
			}
		}
		break;

	case MAC_STATE_PT_CANCELLING_SCAN:
		if (msg->type == MAC_EVENT_PHY_OP_COMPLETE) {
		printk("[PT_SM] state->MAC_STATE_PT_CANCELLING_SCAN && msg->type == MAC_EVENT_PHY_OP_COMPLETE \n");
			pt_handle_phy_op_complete_internal(
				&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
		}
		break;

	case MAC_STATE_PT_ASSOCIATING:
	case MAC_STATE_PT_RACH_BACKOFF:
	case MAC_STATE_PT_AUTHENTICATING:
	case MAC_STATE_PT_WAIT_AUTH_CHALLENGE:
	case MAC_STATE_PT_WAIT_AUTH_SUCCESS:
	case MAC_STATE_PT_HANDOVER_ASSOCIATING:
	case MAC_STATE_PT_WAIT_ASSOC_RESP:	
	// printk("[PT_SM] MAC_STATE_PT_WAIT_ASSOC_RESP: ctx->state:%s(%d):: msg->type:%s(%d) \n", dect_mac_state_to_str(ctx->state), ctx->state, dect_mac_event_to_str(msg->type), msg->type);
		switch (msg->type) {
		case MAC_EVENT_PHY_OP_COMPLETE:
			// printk("[PT_SM] state->MAC_STATE_PT_ASSOCIATING && msg->type == MAC_EVENT_PHY_OP_COMPLETE ****************************************\n");
			printk("[PT_SM] dect_mac_sm_pt_handle_event: ctx->state:%s(%d):: msg->type:%s(%d) \n", dect_mac_state_to_str(ctx->state), ctx->state, dect_mac_event_to_str(msg->type), msg->type);
			pt_handle_phy_op_complete_internal(
				&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
			// printk("[PT_SM] state->MAC_STATE_PT_ASSOCIATING && msg->type == MAC_EVENT_PHY_OP_COMPLETE ****************************************\n");

			break;
		case MAC_EVENT_PHY_PCC:
		printk("MAC_STATE_PT_WAIT_ASSOC_RESP-> MAC_EVENT_PHY_PCC\n");
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PDC:
		printk("MAC_STATE_PT_WAIT_ASSOC_RESP-> MAC_EVENT_PHY_PDC\n");
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF:
			pt_rach_backoff_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW:
			pt_rach_response_window_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_AUTH_TIMEOUT:
			LOG_ERR("PT_SM: Auth handshake timed out in state %s. Aborting.", dect_mac_state_to_str(ctx->state));
			dect_mac_sm_pt_start_operation(); /* Restart scan/association */
			break;
		default:
			break; /* Ignore other events */
		}
		break;

	case MAC_STATE_ASSOCIATED:
		printk("[PT_SM] state->MAC_STATE_ASSOCIATED : msg->type:%d  \n", msg->type);
		switch (msg->type) {
		case MAC_EVENT_PHY_OP_COMPLETE:
			pt_handle_phy_op_complete_internal(
				&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
			break;
		case MAC_EVENT_PHY_PCC:
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PDC:
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_TIMER_EXPIRED_KEEPALIVE:
			dect_mac_sm_pt_keep_alive_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN:
			dect_mac_sm_pt_mobility_scan_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_HARQ:
			dect_mac_data_path_handle_harq_nack_action(msg->data.timer_data.id);
			break;
		case MAC_EVENT_CMD_RELEASE_LINK:
			LOG_INF("PT_SM: Received command to release link.");
			pt_send_association_release_action(&ctx->role_ctx.pt.associated_ft);
			dect_mac_service();
			/* Now, change the state to wait for the TX to complete.
			 * The simulation loop will call dect_mac_service() to schedule the TX.
			 */
			dect_mac_change_state(MAC_STATE_PT_RELEASING);	
			break;		
		case MAC_EVENT_CMD_ENTER_PAGING_MODE:
			dect_mac_change_state(MAC_STATE_PT_PAGING);
			k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
			k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);
			k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer,
				      K_MSEC(CONFIG_DECT_MAC_PT_PAGING_CYCLE_MS),
				      K_MSEC(CONFIG_DECT_MAC_PT_PAGING_CYCLE_MS));
			break;
		default:
			break;
		}
		break;

	case MAC_STATE_PT_RELEASING:
		if (msg->type == MAC_EVENT_PHY_OP_COMPLETE) {
			pending_op_type_t op = dect_mac_phy_ctrl_handle_op_complete(
				&msg->data.op_complete);
			if (op == PENDING_OP_GENERIC_UNICAST_TX) {
				LOG_INF("PT_SM:(dect_mac_sm_pt_handle_event) Association Release TX complete. Restarting scan.");
				dect_mac_core_clear_pending_op();
				dect_mac_sm_pt_start_operation();
			}
		}
		break;
	case MAC_STATE_PT_PAGING:
		printk("[PT_SM] dect_mac_sm_pt_handle_event: msg->type:%s \n", dect_mac_event_to_str(msg->type));	
		// dect_mac_event_to_str(msg->type), dect_mac_state_to_str(ctx->state));

		switch (msg->type) {
		case MAC_EVENT_PHY_OP_COMPLETE:
			pt_handle_phy_op_complete_internal(
				&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
			break;
		case MAC_EVENT_PHY_PCC:
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PDC:
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_TIMER_EXPIRED_PAGING_CYCLE:
			pt_paging_cycle_timer_expired_action();
			break;
		default:
			break;
		}
		break;

	default:
		LOG_WRN("PT SM: Event %s received in unhandled state %s. Discarding.",
			dect_mac_event_to_str(msg->type), dect_mac_state_to_str(ctx->state));
		break;
	}
}

// All static helper functions (pt_handle_phy_op_complete_internal, pt_handle_phy_pcc_internal,
// pt_handle_phy_pdc_internal, pt_process_identified_beacon_and_attempt_assoc,
// pt_send_association_request_action, pt_process_association_response_pdu,
// pt_send_keep_alive_action, pt_start_authentication_with_ft_action,
// pt_authentication_complete_action) are included below with their full implementations.

// --- PT Static Helper Implementations ---
static void pt_handle_phy_op_complete_internal(const struct nrf_modem_dect_phy_op_complete_event *event,
                                               pending_op_type_t completed_op_type) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("[PT_SM] pt_handle_phy_op_complete_internal: completed_op_type:%s(%d) Started... \n", dect_pending_op_to_str(completed_op_type),completed_op_type);


    switch (completed_op_type) {
		
        case PENDING_OP_PT_MOBILITY_SCAN:
            printk("PT SM: Mobility scan op completed (err %d).", event->err);
			// LOG_DBG("PT SM: Mobility scan op completed (err %d).", event->err);
            break;
		case PENDING_OP_GENERIC_UNICAST_TX:
			if (ctx->state == MAC_STATE_PT_RELEASING) {
				LOG_INF("PT_SM:(PENDING_OP_GENERIC_UNICAST_TX) Association Release TX complete. Restarting scan.");
				k_timer_stop(&ctx->role_ctx.pt.beacon_listen_timer);
				dect_mac_core_clear_pending_op();
				dect_mac_sm_pt_start_operation();
			} else {
				LOG_DBG("PT_SM: Generic Unicast TX complete in state %s. Clearing pending op.", dect_mac_state_to_str(ctx->state));
				dect_mac_core_clear_pending_op();
			}
			break;
        case PENDING_OP_PT_PAGING_LISTEN:
			printk("[PT_SM] completed_op_type->PENDING_OP_PT_PAGING_LISTEN \n");
            if (ctx->state == MAC_STATE_PT_PAGING) {
                LOG_DBG("PT_PAGING: Paging listen RX window complete (err %d).", event->err);
            }
            break;
        case PENDING_OP_PT_SCAN:
			printk("[PT_SM] completed_op_type->PENDING_OP_PT_SCAN \n");
			if (event->err == NRF_MODEM_DECT_PHY_ERR_OP_CANCELED) {
				LOG_INF("PT SM: Scan successfully canceled (Hdl %u).", event->handle);
				if (ctx->state == MAC_STATE_PT_CANCELLING_SCAN) {
					printk("PT SM: Scan cancelled for association. Sending Association Request.\n");
					// LOG_INF("PT SM: Scan cancelled for association. Sending Association Request now.");
					/* CRITICAL FIX: Clear the pending state of the completed scan
					* BEFORE attempting to schedule the new TX operation.
					*/
					dect_mac_core_clear_pending_op();
					pt_send_association_request_action();

					/* Start an RX operation to listen for the response */
					uint32_t phy_rx_op_handle;
					dect_mac_rand_get((uint8_t *)&phy_rx_op_handle, sizeof(phy_rx_op_handle));
					uint32_t resp_win_ms = ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us / 1000;
					if (resp_win_ms == 0) { resp_win_ms = 200; }
					uint32_t rx_duration_modem_units = modem_us_to_ticks(
						(resp_win_ms + 50) * 1000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

					dect_mac_phy_ctrl_start_rx(
						ctx->role_ctx.pt.target_ft.operating_carrier,
						rx_duration_modem_units,
						NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
						phy_rx_op_handle, ctx->own_short_rd_id,
						PENDING_OP_PT_WAIT_ASSOC_RESP);
				} else {
					printk("PT SM: Scan cancelled in unexpected state %s. Restarting scan.",
						dect_mac_state_to_str(ctx->state));
					dect_mac_sm_pt_start_operation();
					// LOG_WRN("PT SM: Scan cancelled in unexpected state %s. Restarting scan.",
					// 	dect_mac_state_to_str(ctx->state));
					// dect_mac_sm_pt_start_operation();
				}
			} else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                printk("PT_SM: Scan PHY op failed (err %d, %s). Restarting scan after delay.\n", event->err, nrf_modem_dect_phy_err_to_str(event->err));
				// LOG_ERR("PT_SM: Scan PHY op failed (err %d, %s). Restarting scan after delay.", event->err, nrf_modem_dect_phy_err_to_str(event->err));
                // k_sleep(K_MSEC(1000 + (sys_rand32_get() % 1000)));
				uint8_t random_byte;
				sys_rand_get(&random_byte, sizeof(random_byte));
				k_sleep(K_MSEC(1000 + (random_byte % 100)));  // 0-99ms additional delay				
                dect_mac_sm_pt_start_operation();
            } else {
                printk("PT_SM: Scan PHY op completed (Hdl %u), but no suitable FT found during PDC parsing. Restarting scan.\n", event->handle);
				// LOG_INF("PT_SM: Scan PHY op completed (Hdl %u), but no suitable FT found during PDC parsing. Restarting scan.", event->handle);
                dect_mac_sm_pt_start_operation();
            }
            break;

       case PENDING_OP_PT_RACH_ASSOC_REQ:
	   		printk("[PT_SM] completed_op_type->PENDING_OP_PT_RACH_ASSOC_REQ \n");
            if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
                printk("[PT_SM]: Association Request TX successful (Hdl %u). Waiting for Response.\n", event->handle);
				// LOG_INF("PT_SM: Association Request TX successful (Hdl %u). Waiting for Response.", event->handle);
                dect_mac_change_state(MAC_STATE_PT_WAIT_ASSOC_RESP);

                uint32_t resp_win_ms = ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us / 1000;
                if (resp_win_ms < 10) resp_win_ms = 50;
                if (resp_win_ms == 0 && ctx->config.rach_response_window_ms > 0) {
                    resp_win_ms = ctx->config.rach_response_window_ms;
                } else if (resp_win_ms == 0) {
                    resp_win_ms = 200;
                }
                if (resp_win_ms > 5000) resp_win_ms = 5000;

                k_timer_start(&ctx->rach_context.rach_response_window_timer, K_MSEC(resp_win_ms), K_NO_WAIT);

				uint32_t phy_rx_op_handle;
				dect_mac_rand_get((uint8_t *)&phy_rx_op_handle, sizeof(phy_rx_op_handle));
                uint32_t rx_duration_modem_units = modem_us_to_ticks( (resp_win_ms + 50) * 1000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ );
				/* CRITICAL FIX: Clear the pending state from the completed TX operation
				* BEFORE attempting to schedule the new RX operation.
				*/
				printk("[PT_SM] case PENDING_OP_PT_RACH_ASSOC_REQ: Calling dect_mac_phy_ctrl_start_rx() \n");
				dect_mac_core_clear_pending_op();
				
                int ret = dect_mac_phy_ctrl_start_rx(
                    ctx->role_ctx.pt.target_ft.operating_carrier,
                    rx_duration_modem_units,
                    NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
                    phy_rx_op_handle,
                    ctx->own_short_rd_id,
                    PENDING_OP_PT_WAIT_ASSOC_RESP);
                if (ret != 0) {
                    printk("PT_SM: Failed to schedule RX for AssocResp: %d. Resp timer will timeout.\n", ret);
					// LOG_ERR("PT_SM: Failed to schedule RX for AssocResp: %d. Resp timer will timeout.", ret);
                } else {
                    printk("PT_SM: RX scheduled (Hdl %u) for Association Response from FT 0x%04X.\n",
                            phy_rx_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
                    // LOG_INF("PT_SM: RX scheduled (Hdl %u) for Association Response from FT 0x%04X.",
                    //         phy_rx_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);							
                }
                /* The original TX op is complete and the next RX op is scheduled.
                 * It is now safe to clear the pending state from the TX op.
                 */
				printk("[PT_SM] case PENDING_OP_PT_RACH_ASSOC_REQ: The original TX op is complete and the next RX op is scheduled, clear the pending state from the TX op. POINT 2 \n");
                // dect_mac_core_clear_pending_op();				

			} else if (event->err == NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY) {
				/* 
				 * P1.4.1.1: Implement RACH Backoff Pause/Resume.
				 * If the channel is busy (LBT failure), we do NOT increase the Contention Window (CW).
				 * We explicitly pause the backoff (or rather, just retry after a short, fixed interval)
				 * effectively resuming the attempt when the channel might be free.
				 */
				printk("PT_SM: RACH TX failed (LBT BUSY). Pausing/Retrying without increasing CW.\n");
				dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);

				/* Retry relatively quickly, e.g., next frame or shortly after. 
				 * ETSI TS 103 636-4 implies staying in same backoff stage.
				 */
				k_timer_start(&ctx->rach_context.rach_backoff_timer,
					      K_MSEC(10), K_NO_WAIT);

			} else {
				/* Any other error (or collision implied by lack of Ack later) triggers backoff adjustment */
				printk("PT_SM: RACH TX failed (err %d, %s). Increasing CW and backing off.",
				       event->err, nrf_modem_dect_phy_err_to_str(event->err));
				// LOG_WRN("PT_SM: RACH TX failed (err %d, %s). Increasing CW and backing off.",
				//        event->err, nrf_modem_dect_phy_err_to_str(event->err));
				dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);

				if (ctx->role_ctx.pt.current_assoc_retries >=
				    ctx->config.max_assoc_retries) {
					printk("PT_SM_RACH_FAIL: Max association retries reached for FT 0x%04X after TX failure. Restarting scan.",
					       ctx->role_ctx.pt.target_ft.short_rd_id);
					// LOG_ERR("PT_SM_RACH_FAIL: Max association retries reached for FT 0x%04X after TX failure. Restarting scan.",
					//         ctx->role_ctx.pt.target_ft.short_rd_id);
					dect_mac_sm_pt_start_operation();
					return;
				}

				uint8_t ft_cwmin_code =
					ctx->role_ctx.pt.current_ft_rach_params
						.advertised_beacon_ie_fields.cwmin_sig_code;
				uint8_t ft_cwmax_code =
					ctx->role_ctx.pt.current_ft_rach_params
						.advertised_beacon_ie_fields.cwmax_sig_code;

				if (ctx->rach_context.rach_cw_current_idx < ft_cwmin_code) {
					ctx->rach_context.rach_cw_current_idx = ft_cwmin_code;
				}

				if (ctx->rach_context.rach_cw_current_idx < ft_cwmax_code) {
					ctx->rach_context.rach_cw_current_idx++;
					LOG_DBG("PT_RACH_FAIL: Increased CW index to %u for next attempt.",
						ctx->rach_context.rach_cw_current_idx);
				}

				uint16_t current_cw_value =
					8 * (1U << ctx->rach_context.rach_cw_current_idx);
				uint32_t backoff_slots_to_wait = 0;

				if (current_cw_value > 0) {
					dect_mac_rand_get((uint8_t *)&backoff_slots_to_wait,
							  sizeof(backoff_slots_to_wait));
					backoff_slots_to_wait %= current_cw_value;
				}

				uint8_t ft_mu_code =
					ctx->role_ctx.pt.current_ft_rach_params
						.advertised_beacon_ie_fields.mu_value_for_ft_beacon;
				uint32_t rach_contention_slot_ticks =
					get_subslot_duration_ticks_for_mu(ft_mu_code);
				if (rach_contention_slot_ticks == 0) {
					rach_contention_slot_ticks = NRF_MODEM_DECT_LBT_PERIOD_MIN;
				}

				uint32_t backoff_duration_ticks =
					backoff_slots_to_wait * rach_contention_slot_ticks;
				uint32_t backoff_ms =
					(backoff_duration_ticks * 1000U) /
					NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ;
				if (backoff_ms == 0 && backoff_slots_to_wait > 0) {
					backoff_ms = 1;
				}
				if (backoff_ms == 0 && backoff_slots_to_wait == 0) {
					backoff_ms = 2;
				}

				k_timer_start(&ctx->rach_context.rach_backoff_timer,
					      K_MSEC(MAX(10, backoff_ms)), K_NO_WAIT);
			}
            break;

        case PENDING_OP_PT_WAIT_ASSOC_RESP:
            printk("[PT_SM] Handling PHY_OP_COMPLETE for PENDING_OP_PT_WAIT_ASSOC_RESP, Handle=%u\n",
                   event->handle);
            if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
				printk("PT SM: RX window for %s closed without receiving a valid response. Triggering timeout logic.\n",
				dect_pending_op_to_str(completed_op_type));
			// LOG_WRN("PT SM: RX window for %s closed without receiving a valid response. Triggering timeout logic.",
			// 	dect_pending_op_to_str(completed_op_type));
			/* This completion means the listen window timed out without a valid PDU.
			 * We must treat it as if the response timer itself fired.
			 */
			pt_rach_response_window_timer_expired_action();
			break;
            } else {
                printk("[PT_SM] RX operation failed (err %d, %s) in PT_WAIT_ASSOC_RESP. Initiating backoff.\n",
                       event->err, nrf_modem_dect_phy_err_to_str(event->err));
                dect_mac_core_clear_pending_op(); // Clear pending_op on failure
                dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);

                if (ctx->role_ctx.pt.current_assoc_retries >= ctx->config.max_assoc_retries) {
                    printk("[PT_SM] Max association retries reached for FT 0x%04X after RX failure. Restarting scan.\n",
                           ctx->role_ctx.pt.target_ft.short_rd_id);
                    dect_mac_sm_pt_start_operation();
                    return;
                }

                // Reuse backoff logic from PENDING_OP_PT_RACH_ASSOC_REQ
                uint8_t ft_cwmin_code = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.cwmin_sig_code;
                uint8_t ft_cwmax_code = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.cwmax_sig_code;

                if (ctx->rach_context.rach_cw_current_idx < ft_cwmin_code) {
                    ctx->rach_context.rach_cw_current_idx = ft_cwmin_code;
                }

                if (ctx->rach_context.rach_cw_current_idx < ft_cwmax_code) {
                    ctx->rach_context.rach_cw_current_idx++;
                    printk("[PT_SM] Increased CW index to %u for next attempt.\n",
                           ctx->rach_context.rach_cw_current_idx);
                }

                uint16_t current_cw_value = 8 * (1U << ctx->rach_context.rach_cw_current_idx);
                uint32_t backoff_slots_to_wait = 0;

                if (current_cw_value > 0) {
                    dect_mac_rand_get((uint8_t *)&backoff_slots_to_wait, sizeof(backoff_slots_to_wait));
                    backoff_slots_to_wait %= current_cw_value;
                }

                uint32_t backoff_ms = 10; // Default backoff
                k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(backoff_ms), K_NO_WAIT);
            }
            break;
		
        case PENDING_OP_PT_WAIT_AUTH_CHALLENGE:
        case PENDING_OP_PT_WAIT_AUTH_SUCCESS:
			printk("PT SM: RX op for %s completed (Hdl %u, err %d). If no PDC, timer will expire.",
			dect_pending_op_to_str(completed_op_type), event->handle, event->err);
            // LOG_DBG("PT SM: RX op for %s completed (Hdl %u, err %d). If no PDC, timer will expire.",
            //         dect_pending_op_to_str(completed_op_type), event->handle, event->err);
			/* The listen window is over. The logical timeout is handled by the k_timer.
			 * We just need to clear the pending PHY operation so the SM can schedule
			 * a new one if the k_timer fires.
			 */
			dect_mac_core_clear_pending_op();
			break;
            break;

        case PENDING_OP_PT_AUTH_MSG_TX:
            if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_INF("PT_SM: Auth message TX successful (Hdl %u). Waiting for response.",
                        event->handle);
                // uint32_t phy_rx_op_handle = sys_rand32_get();
				uint32_t phy_rx_op_handle;
				sys_rand_get(&phy_rx_op_handle, sizeof(uint32_t));				
                uint32_t rx_duration_modem_units = modem_us_to_ticks(
                    (ctx->config.rach_response_window_ms + 50) * 1000,
                    NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

                pending_op_type_t next_op = PENDING_OP_NONE;

                if (ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE) {
                    next_op = PENDING_OP_PT_WAIT_AUTH_CHALLENGE;
                } else if (ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS) {
                    next_op = PENDING_OP_PT_WAIT_AUTH_SUCCESS;
                }

                if (next_op != PENDING_OP_NONE) {
                    dect_mac_phy_ctrl_start_rx(
                        ctx->role_ctx.pt.target_ft.operating_carrier,
                        rx_duration_modem_units, NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
                        phy_rx_op_handle, ctx->own_short_rd_id, next_op);
                }
            } else {
                LOG_ERR("PT_SM: Auth message TX failed (err %d). Restarting scan.",
                        event->err);
                dect_mac_sm_pt_start_operation();
            }
            break;
        case PENDING_OP_PT_KEEP_ALIVE:
			LOG_DBG("PT SM: Keep-alive TX op completed (err %d).", event->err);
			/* The pending op is cleared by the caller (handle_op_complete), nothing more to do. */
            if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_ERR("PT_SM: Keep Alive TX failed (Hdl %u, err %d, %s).",
                        event->handle, event->err, nrf_modem_dect_phy_err_to_str(event->err));
            } else {
                LOG_DBG("PT_SM: Keep Alive TX successful (Hdl %u).", event->handle);
            }
            break;
        case PENDING_OP_PT_DATA_TX_HARQ0:
        case PENDING_OP_PT_DATA_TX_HARQ_MAX:
            {
                int harq_idx = completed_op_type - PENDING_OP_PT_DATA_TX_HARQ0;
                if (harq_idx >= 0 && harq_idx < MAX_HARQ_PROCESSES) {
                    if (event->err == NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY) {
                        LOG_WRN("PT SM: Data TX HARQ %d LBT busy. Data Path will re-TX.", harq_idx);
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                        LOG_ERR("PT SM: Data TX HARQ %d failed (err %d, %s). Data Path will re-TX/discard.",
                                harq_idx, event->err, nrf_modem_dect_phy_err_to_str(event->err));
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else {
                        LOG_DBG("PT SM: Data TX HARQ %d PHY op complete. Awaiting feedback.", harq_idx);
						if (ctx->state == MAC_STATE_PT_RELEASING) {
							printk("PT_SM:(pt_handle_phy_op_complete_internal) Association Release TX complete. Restarting scan.\n");
							LOG_INF("PT_SM: Association Release TX complete. Restarting scan.");
							dect_mac_core_clear_pending_op();
							dect_mac_sm_pt_start_operation();
						}
                    }
                } else {
                     LOG_ERR("PT SM: OP_COMPLETE for invalid PT_DATA_TX_HARQ op type: %d", completed_op_type);
                }
            }
            break;
			
        default:
             LOG_WRN("PT SM: Case Ended OP_COMPLETE for unhandled PT op type: %s (Hdl %u), err %d (%s)",
                    dect_pending_op_to_str(completed_op_type), event->handle,
                    event->err, nrf_modem_dect_phy_err_to_str(event->err));
            break;
    }
}

static void pt_handle_phy_pcc_internal(const struct nrf_modem_dect_phy_pcc_event *pcc_event,
				       uint64_t pcc_event_time)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("[PT_SM] pt_handle_phy_pcc_internal Started...\n");

	if (pcc_event->header_status != NRF_MODEM_DECT_PHY_HDR_STATUS_VALID) {
		return; /* Ignore invalid PCCs */
	}

	/* Find a free slot or evict the oldest one */
	int free_slot = -1;
	uint64_t oldest_time = UINT64_MAX;
	int oldest_slot = 0;

	printk("[PT_PCC_CACHE]: Adding to cache");
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		printk("PT_PCC_CACHE: (TID %u) in slot %d.\n",
		       ctx->pcc_transaction_cache[i].transaction_id, i);
		if (!ctx->pcc_transaction_cache[i].is_valid) {
			free_slot = i;
			break;
		}
		if (ctx->pcc_transaction_cache[i].reception_time_us < oldest_time) {
			oldest_time = ctx->pcc_transaction_cache[i].reception_time_us;
			oldest_slot = i;
		}
	}

	if (free_slot == -1) {
		free_slot = oldest_slot;
		printk("PT_PCC_CACHE: Cache full, evicting oldest entry (TID %u) from slot %d.\n",
		       ctx->pcc_transaction_cache[free_slot].transaction_id, free_slot);
		k_timer_stop(&ctx->pcc_transaction_cache[free_slot].timeout_timer);
	}

	/* Store the new PCC transaction and start its expiry timer */
	pcc_transaction_t *transaction = &ctx->pcc_transaction_cache[free_slot];

	transaction->is_valid = true;
	transaction->transaction_id = pcc_event->transaction_id;
	transaction->reception_time_us = pcc_event_time;
	memcpy(&transaction->pcc_data, pcc_event, sizeof(struct nrf_modem_dect_phy_pcc_event));

	k_timer_start(&transaction->timeout_timer, K_MSEC(200), K_NO_WAIT);

	print_pcc_cache_state("PT_PCC_ADD");

	printk("PT_PCC_CACHE: Stored PCC with TID %u in slot %d. \n", transaction->transaction_id,
	       free_slot);

	/* HARQ feedback processing can still happen immediately */
	if (pcc_event->phy_type == 1 && ctx->state == MAC_STATE_ASSOCIATED) {
		uint16_t pcc_tx_short_id = dect_mac_phy_ctrl_get_transmitter_id((const union nrf_modem_dect_phy_hdr *)&pcc_event->hdr, pcc_event->phy_type);
		dect_mac_data_path_process_harq_feedback(&pcc_event->hdr.hdr_type_2.feedback,
							 pcc_tx_short_id);
	}
}



// Overview: Phase 3 of rebuilding the PT's PDC handler. This change re-integrates the full security stack (decryption, MIC verification) and the HARQ feedback generation logic, making the PT capable of reliable, secure data reception.
static void pt_handle_phy_pdc_internal(const struct nrf_modem_dect_phy_pdc_event *pdc_event,
				       const struct nrf_modem_dect_phy_pcc_event *assoc_pcc_event,
				       uint64_t pcc_reception_modem_time)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("\n--- DEBUG_PROBE: pt_handle_phy_pdc_internal ---\n");
	printk("\n--- PT PDC HANDLER(ctx->state) (State: %s) ---\n", dect_mac_state_to_str(ctx->state));
	printk("  - Received PDC for TID %u\n", pdc_event->transaction_id);
	printk("  - Current PT State:(dect_mac_get_active_context()->state) %s\n", dect_mac_state_to_str(dect_mac_get_active_context()->state));


	if (pdc_event->len < sizeof(dect_mac_header_type_octet_t)) {
		LOG_ERR("PT_SM_PDC: PDU too short for header type octet.");
		return;
	}

	uint8_t mac_pdc_payload_copy[CONFIG_DECT_MAC_PDU_MAX_SIZE];
	memset(mac_pdc_payload_copy, 0, sizeof(mac_pdc_payload_copy));
	uint16_t pdc_payload_len = pdc_event->len;

	const uint8_t *data = pdc_event->data;
	size_t len = 16; /* Adjust this based on your needs */
	printk("[PT_SM] RX_QUEUE:PDC Payload Hexdump (first %zu bytes): ", len);
	for (size_t i = 0; i < len && i < pdc_payload_len; i++) {
		printk("%02x ", data[i]);
	}
	printk("\n");

	if (pdc_payload_len > sizeof(mac_pdc_payload_copy)) {
		LOG_ERR("PT_SM_PDC: PDC payload from PHY (%u bytes) too large for copy buffer (%zu). Discarding.",
			pdc_payload_len, sizeof(mac_pdc_payload_copy));
		return;
	}

	memcpy(mac_pdc_payload_copy, pdc_event->data, pdc_payload_len);
	// printk("[PT_SM] First byte of received PDC payload: 0x%02X(%d)\n", mac_pdc_payload_copy[0], mac_pdc_payload_copy[0]);

	dect_mac_header_type_octet_t mac_hdr_type_octet;
	memcpy(&mac_hdr_type_octet, &mac_pdc_payload_copy[0], sizeof(dect_mac_header_type_octet_t));

	uint8_t *common_hdr_start_in_payload = mac_pdc_payload_copy + sizeof(dect_mac_header_type_octet_t);
	size_t common_hdr_actual_len = 0;
	uint8_t *sdu_area_after_common_hdr = NULL;
	size_t sdu_area_plus_mic_len_in_payload = 0;
	uint16_t ft_sender_short_id_from_pcc = 0;

	printk("ctx->state:%s[%d] phy_type:%s header_type:%d \n",dect_mac_state_to_str(ctx->state), ctx->state, assoc_pcc_event->phy_type == 0? "Beacon":"Unicast", mac_hdr_type_octet.mac_header_type );

	ft_sender_short_id_from_pcc = dect_mac_phy_ctrl_get_transmitter_id((const union nrf_modem_dect_phy_hdr *)&assoc_pcc_event->hdr, assoc_pcc_event->phy_type);

	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
		common_hdr_actual_len = sizeof(dect_mac_beacon_header_t);
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		common_hdr_actual_len = sizeof(dect_mac_unicast_header_t);
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU) {
		common_hdr_actual_len = sizeof(dect_mac_data_pdu_header_t);
	} else {
		LOG_WRN("PT_SM_PDC: Received PDC with unhandled MAC Hdr Type %u. Discarding.",
			mac_hdr_type_octet.mac_header_type);
		return;
	}

	if (pdc_payload_len < (sizeof(dect_mac_header_type_octet_t) + common_hdr_actual_len)) {
		LOG_ERR("PT_SM_PDC: PDU too short for its Common Hdr. Len %u, HdrLen %zu",
			pdc_payload_len, common_hdr_actual_len);
		return;
	}
	sdu_area_after_common_hdr = common_hdr_start_in_payload + common_hdr_actual_len;
	sdu_area_plus_mic_len_in_payload = pdc_payload_len - sizeof(dect_mac_header_type_octet_t) - common_hdr_actual_len;

	/* --- Beacon Processing Logic --- */
	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
		/* Trap for stay beacon messages from other FT nodes*/
		if (ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP) {
			LOG_DBG("PT_SM_PDC: Ignoring beacon from FT 0x%04X while waiting for association response.",
			ft_sender_short_id_from_pcc);
			/*
				* CRITICAL: The RX operation was likely single-shot and is now complete.
				* We must immediately restart it to continue listening for the real response.
				*/
			// uint32_t phy_rx_op_handle;
			// dect_mac_rand_get((uint8_t *)&phy_rx_op_handle, sizeof(phy_rx_op_handle));
			// uint32_t resp_win_ms = ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us / 1000;
			// if (resp_win_ms == 0) { resp_win_ms = 200; }
			// uint32_t rx_duration_modem_units = modem_us_to_ticks(
			// 	(resp_win_ms + 50) * 1000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

			// dect_mac_phy_ctrl_start_rx(
			// 	ctx->role_ctx.pt.target_ft.operating_carrier,
			// 	rx_duration_modem_units,
			// 	NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
			// 	phy_rx_op_handle, ctx->own_short_rd_id,
			// 	PENDING_OP_PT_WAIT_ASSOC_RESP);
			return;
		}			
		const dect_mac_beacon_header_t *bch = (const dect_mac_beacon_header_t *)common_hdr_start_in_payload;
		uint32_t ft_long_id = sys_be32_to_cpu(bch->transmitter_long_rd_id_be);

	// printk("\n\n\n\n\n\n***********************************HERE*************MAC_COMMON_HEADER_TYPE_BEACON*******************************\n\n\n\n");	
	// printk("ctx->state:%d \n",ctx->state );
	printk("[DEBUG_PROBE] PT received a beacon from FT 0x%08X Connected to 0x%08X\n", ft_long_id, ctx->role_ctx.pt.associated_ft.long_rd_id);
	printk("[DEBUG_PROBE] PT's PDC handler received a BEACON PDC from FT 0x%08X. Current PT state: %s\n",
               ft_long_id, dect_mac_state_to_str(ctx->state));
		
		if (ctx->state == MAC_STATE_ASSOCIATED && ft_long_id != ctx->role_ctx.pt.associated_ft.long_rd_id) {
			/* This is a beacon from another FT. Evaluate for mobility. */
			printk("/* This is a beacon from another FT. Evaluate for mobility. */ \n");
			printk("[DEBUG_PROBE] PT in ASSOCIATED state received a beacon from FT 0x%08X.\n", ft_long_id);
			dect_mac_cluster_beacon_ie_fields_t cb_fields_parsed = {0};
			bool cb_found = false;
			const uint8_t *sdu_area_ptr = sdu_area_after_common_hdr;
			size_t sdu_area_len = sdu_area_plus_mic_len_in_payload;

			while (sdu_area_len > 0) {
				uint8_t ie_type;
				uint16_t ie_len;
				const uint8_t *ie_payload;
				int mux_len = parse_mac_mux_header(sdu_area_ptr, sdu_area_len, &ie_type, &ie_len, &ie_payload);
				if (mux_len <= 0 || (sdu_area_len < (size_t)mux_len + ie_len)) {
					break;
				}
				if (ie_type == IE_TYPE_CLUSTER_BEACON) {
					if (parse_cluster_beacon_ie_payload(ie_payload, ie_len, &cb_fields_parsed) == 0) {
						cb_found = true;
					}
					break;
				}
				size_t consumed = mux_len + ie_len;
				if (consumed == 0) {
					break;
				}
				sdu_area_ptr += consumed;
				sdu_area_len -= consumed;
			}

			if (cb_found) {
				printk("[DEBUG_PROBE] PT sending pt_evaluate_mobility_candidate() messaged from FT 0x%08X.\n", ft_long_id);
				pt_evaluate_mobility_candidate(ctx, &cb_fields_parsed, ft_long_id,
								ft_sender_short_id_from_pcc,
								assoc_pcc_event->rssi_2,
								ctx->current_rx_op_carrier);
			}
			return;
		}




		dect_mac_cluster_beacon_ie_fields_t cb_fields_parsed = {0};
		dect_mac_rach_info_ie_fields_t rach_fields_parsed = {0};
		dect_mac_rd_capability_ie_t ft_caps_parsed = {0};
		dect_mac_resource_alloc_ie_fields_t res_alloc_fields = {0};
		bool cb_found = false, rach_found = false, ft_cap_found = false, res_alloc_found = false;

		const uint8_t *sdu_area_ptr = sdu_area_after_common_hdr;
		size_t sdu_area_len = sdu_area_plus_mic_len_in_payload;

		/* Reconstruct the full 32-bit Network ID from the FT's beacon */
		uint32_t net_id_ms24 = ((uint32_t)bch->network_id_ms24[0] << 24) |
					   ((uint32_t)bch->network_id_ms24[1] << 16) |
					   ((uint32_t)bch->network_id_ms24[2] << 8);
		uint8_t net_id_ls8 = assoc_pcc_event->hdr.hdr_type_1.short_network_id;
		uint32_t ft_network_id = net_id_ms24 | net_id_ls8;

		printk("[[PT_SM]] Reconstructed FT Network ID: 0x%08X\n", ft_network_id);

		/* First pass: Find RD Capability IE to determine the FT's numerology (mu) */
		const uint8_t *p = sdu_area_ptr;
		size_t l = sdu_area_len;
		while (l > 0) {
			uint8_t ie_type; uint16_t ie_len; const uint8_t *ie_payload;
			int mux_len = parse_mac_mux_header(p, l, &ie_type, &ie_len, &ie_payload);
			if (mux_len <= 0 || (l < (size_t)mux_len + ie_len)) break;
			if (ie_type == IE_TYPE_RD_CAPABILITY) {
				if (parse_rd_capability_ie_payload(ie_payload, ie_len, &ft_caps_parsed) == 0) {
					ft_cap_found = true;
				}
				break; /* Found it, no need to look further in this pass */
			}
			size_t consumed = mux_len + ie_len;
			if (consumed == 0) break;
			p += consumed;
			l -= consumed;
		}

		/* Second pass: Parse all other IEs, using the discovered mu if needed */
		p = sdu_area_ptr;
		l = sdu_area_len;
		while (l > 0) {
			uint8_t ie_type; uint16_t ie_len; const uint8_t *ie_payload;
			int mux_len = parse_mac_mux_header(p, l, &ie_type, &ie_len, &ie_payload);
			if (mux_len <= 0 || (l < (size_t)mux_len + ie_len)) break;

			if (ie_type == IE_TYPE_CLUSTER_BEACON) {
				if (parse_cluster_beacon_ie_payload(ie_payload, ie_len, &cb_fields_parsed) == 0) cb_found = true;
			} else if (ie_type == IE_TYPE_RACH_INFO) {
				uint8_t ft_mu_code = ft_cap_found ? ft_caps_parsed.phy_variants[0].mu_value : 0;
				if (parse_rach_info_ie_payload(ie_payload, ie_len, ft_mu_code, &rach_fields_parsed) == 0) rach_found = true;
			} else if (ie_type == IE_TYPE_RES_ALLOC) {
				uint8_t ft_mu_code = ft_cap_found ? ft_caps_parsed.phy_variants[0].mu_value : 0;
				if (parse_resource_alloc_ie_payload(ie_payload, ie_len, ft_mu_code, &res_alloc_fields) == 0) {
					if (res_alloc_fields.repeat_val == RES_ALLOC_REPEAT_FRAMES_GROUP || res_alloc_fields.repeat_val == RES_ALLOC_REPEAT_SUBSLOTS_GROUP) {
						res_alloc_found = true;
						/* Update template immediately so subsequent GroupAssignment in same beacon can use it */
						dect_mac_schedule_t *group_sched = &ctx->role_ctx.pt.group_schedule;
						group_sched->is_active = true;
						group_sched->alloc_type = res_alloc_fields.alloc_type_val;
						group_sched->ul_start_subslot = res_alloc_fields.start_subslot_val_res1;
						group_sched->ul_duration_subslots = res_alloc_fields.length_val_res1 + 1;
						group_sched->ul_length_is_slots = res_alloc_fields.length_type_is_slots_res1;
						group_sched->repeat_type = res_alloc_fields.repeat_val;
						group_sched->repetition_value = res_alloc_fields.repetition_value;
						group_sched->validity_value = res_alloc_fields.validity_value;
						group_sched->channel = res_alloc_fields.channel_present ? res_alloc_fields.channel_val : ctx->current_rx_op_carrier;
					}
				}
			} else if (ie_type == IE_TYPE_BROADCAST_IND) {
				if (ctx->state == MAC_STATE_PT_PAGING && ie_len >= 2) {
					uint16_t paged_short_id = sys_be16_to_cpu(*((const uint16_t*)ie_payload));
					if (paged_short_id == ctx->own_short_rd_id) {
						pt_process_page_indication();
					}
				}
			} else if (ie_type == IE_TYPE_GROUP_ASSIGNMENT) {
				if (ctx->state == MAC_STATE_ASSOCIATED) {
					pt_process_group_assignment_ie(ie_payload, ie_len);
				}
			}

			size_t consumed = mux_len + ie_len;
			if (consumed == 0) {
				LOG_WRN("PT_PDC_BEACON: Consumed 0 bytes, breaking loop.");
				break;
			}
			p += consumed;
			l -= consumed;
		}

		uint16_t beacon_rx_carrier = ctx->current_rx_op_carrier;

		if (cb_found && rach_found) {
			/* The PT must adopt the Network ID of the FT it is associating with. */
			ctx->network_id_32bit = ft_network_id;
			printk("[PT_BEACON_PROC] PT Network ID adopted: 0x%08X\n", ctx->network_id_32bit);

			/*
			 * This is the point of synchronization. The PT adopts the FT's SFN and
			 * time anchor as its own to ensure all future schedule calculations are correct.
			 */
			uint64_t frame_duration_ticks = k_us_to_ticks_ceil64(FRAME_DURATION_MS_NOMINAL * 1000);
			ctx->ft_sfn_zero_modem_time_anchor = pcc_reception_modem_time - (cb_fields_parsed.sfn * frame_duration_ticks);
			ctx->current_sfn_at_anchor_update = cb_fields_parsed.sfn;
			ctx->role_ctx.ft.sfn = cb_fields_parsed.sfn;
			LOG_INF("PT_SYNC: Synchronized to FT. SFN is now %u, anchor time is %llu.",
				ctx->role_ctx.ft.sfn, ctx->ft_sfn_zero_modem_time_anchor);
				

			if (ctx->state == MAC_STATE_PT_SCANNING || ctx->state == MAC_STATE_PT_BEACON_PDC_WAIT) {
				pt_process_identified_beacon_and_attempt_assoc(
					ctx, &cb_fields_parsed, &rach_fields_parsed, ft_long_id,
					ft_sender_short_id_from_pcc, assoc_pcc_event->rssi_2,
					beacon_rx_carrier, pcc_reception_modem_time);
			} else if (ctx->state == MAC_STATE_ASSOCIATED) {
				pt_evaluate_mobility_candidate(ctx, &cb_fields_parsed, ft_long_id,
								ft_sender_short_id_from_pcc,
								assoc_pcc_event->rssi_2, beacon_rx_carrier);
			}
		}
		return;
	}

	/* --- Unicast PDU Processing Logic --- */
	printk("/* --- Unicast PDU Processing Logic --- */ -> ctx->state[%d]  \n", ctx->state);
	dect_mac_peer_info_t *active_ft_peer_ctx = NULL;
	if ((ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP ||
	     ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE ||
	     ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS ||
	     ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING) &&
		 ctx->role_ctx.pt.target_ft.is_valid &&
		 ft_sender_short_id_from_pcc == ctx->role_ctx.pt.target_ft.short_rd_id) {
		/* This block handles all messages from a NEW FT we are trying to associate with. */
		active_ft_peer_ctx = &ctx->role_ctx.pt.target_ft;
		} else if (ctx->state >= MAC_STATE_ASSOCIATED && ctx->role_ctx.pt.associated_ft.is_valid &&
			ft_sender_short_id_from_pcc == ctx->role_ctx.pt.associated_ft.short_rd_id) {
		/* This block handles all messages from our CURRENTLY associated FT.
		* This is the crucial path for data, keep-alives, and CDD updates.
		*/
		if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
			/* This is a beacon from our own FT. We need to process it for CDD updates. */
			LOG_DBG("PT_SM_PDC: Processing beacon from own FT for CDD update check.");
			const uint8_t *sdu_area = common_hdr_start_in_payload + common_hdr_actual_len;
			// size_t sdu_area_len = pdc_payload_len - sizeof(mac_hdr_type_octet) - common_hdr_actual_len;
			size_t sdu_area_len = pdc_payload_len - sizeof(dect_mac_header_type_octet_t) - common_hdr_actual_len;
			uint8_t ie_type;
			uint16_t ie_len;
			const uint8_t *ie_payload;
			int mux_hdr_len = parse_mac_mux_header(sdu_area, sdu_area_len, &ie_type, &ie_len, &ie_payload);

			if (mux_hdr_len > 0 && ie_type == IE_TYPE_ROUTE_INFO) {
				dect_mac_route_info_ie_t route_info;
				if (parse_route_info_ie_payload(ie_payload, ie_len, &route_info) == 0) {
					dect_cdd_pt_process_beacon_info(sys_be32_to_cpu(route_info.sink_address_be), route_info.app_sequence_number);
				}
			}
			/* Beacon processed for CDD, no further action needed for this PDU */
			return;
		}
		printk("====================================== 3 NEW LOGIC PT==================================================\n\n\n\n\n\n\n");
		/* This block handles all messages from our CURRENTLY associated FT. */
		active_ft_peer_ctx = &ctx->role_ctx.pt.associated_ft;
	} else if (ctx->state == MAC_STATE_PT_SCANNING && ctx->role_ctx.pt.target_ft.is_valid &&
		   ft_sender_short_id_from_pcc == ctx->role_ctx.pt.target_ft.short_rd_id) {
		/* This can happen if an AssocResp arrives after a retry/rescan has been initiated */
		printk("PT_SM_PDC: Received unicast from target FT while in SCANNING state. Processing.");
		active_ft_peer_ctx = &ctx->role_ctx.pt.target_ft;
	} else {
		printk("[PT_PDC_DBG] Unicast discard details:\n");
		printk("  - Sender's Short ID: 0x%04X\n", ft_sender_short_id_from_pcc);
		printk("  - PT State: %s\n", dect_mac_state_to_str(ctx->state));
		printk("  - Associated FT is_valid: %d\n", ctx->role_ctx.pt.associated_ft.is_valid);
		printk("  - Associated FT Short ID: 0x%04X\n", ctx->role_ctx.pt.associated_ft.short_rd_id);
		printk("  - Target FT is_valid: %d\n", ctx->role_ctx.pt.target_ft.is_valid);
		printk("  - Target FT Short ID: 0x%04X\n", ctx->role_ctx.pt.target_ft.short_rd_id);
		LOG_WRN("PT_SM_PDC: Unicast from FT 0x%04X in unexpected state %s or from unexpected FT. Discarding.",
			ft_sender_short_id_from_pcc, dect_mac_state_to_str(ctx->state));
		return;
	}

	bool pdc_process_ok_for_feedback = true;
	uint8_t *sdu_area_for_data_path = sdu_area_after_common_hdr;
	size_t sdu_area_len_for_data_path = sdu_area_plus_mic_len_in_payload;

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	bool security_applied_by_sender = (mac_hdr_type_octet.mac_security != MAC_SECURITY_NONE);
	bool link_is_expected_to_be_secure = (active_ft_peer_ctx && active_ft_peer_ctx->is_secure && ctx->keys_provisioned);

	if (security_applied_by_sender) {
		if (!link_is_expected_to_be_secure) {
			LOG_WRN("PT_SM_PDC_SEC: Secured PDU from FT 0x%04X, but link is not secure. Discarding.", ft_sender_short_id_from_pcc);
			return;
		}
		if (sdu_area_plus_mic_len_in_payload < 5) {
			LOG_ERR("PT_SM_PDC_SEC: Secured PDU from FT 0x%04X too short for MIC. Discarding.", ft_sender_short_id_from_pcc);
			return;
		}

		const dect_mac_unicast_header_t *uch_ptr = (const dect_mac_unicast_header_t *)common_hdr_start_in_payload;
		uint16_t received_psn = ((uch_ptr->sequence_num_high_reset_rsv >> 4) & 0x0F) << 8 | uch_ptr->sequence_num_low;
		uint32_t ft_tx_long_id_from_hdr = sys_be32_to_cpu(uch_ptr->transmitter_long_rd_id_be);

		if (ft_tx_long_id_from_hdr != active_ft_peer_ctx->long_rd_id) {
			LOG_WRN("PT_SM_PDC_SEC: Secured PDU LongID 0x%08X mismatch for FT 0x%04X. Discarding.", ft_tx_long_id_from_hdr, ft_sender_short_id_from_pcc);
			return;
		}

		uint8_t *payload_to_decrypt_start = NULL;
		size_t payload_to_decrypt_len = 0;
		size_t cleartext_sec_ie_mux_len = 0;
		uint32_t hpc_for_iv = active_ft_peer_ctx->hpc;

		if (mac_hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) {
			uint8_t ie_type_sec; uint16_t ie_len_sec; const uint8_t *ie_payload_sec;
			int mux_hdr_len_sec = parse_mac_mux_header(sdu_area_after_common_hdr, sdu_area_plus_mic_len_in_payload, &ie_type_sec, &ie_len_sec, &ie_payload_sec);

			if (mux_hdr_len_sec > 0 && ie_type_sec == IE_TYPE_MAC_SECURITY_INFO) {
				cleartext_sec_ie_mux_len = mux_hdr_len_sec + ie_len_sec;
				uint8_t ver, kidx, secivtype_from_ie;
				uint32_t hpc_from_ie;

				if (parse_mac_security_info_ie_payload(ie_payload_sec, ie_len_sec, &ver, &kidx, &secivtype_from_ie, &hpc_from_ie) == 0) {
					uint32_t forward_diff = (hpc_from_ie >= active_ft_peer_ctx->highest_rx_peer_hpc) ? (hpc_from_ie - active_ft_peer_ctx->highest_rx_peer_hpc) : ((UINT32_MAX - active_ft_peer_ctx->highest_rx_peer_hpc) + hpc_from_ie + 1);
					if (forward_diff <= HPC_RX_FORWARD_WINDOW_MAX_ADVANCE) {
						active_ft_peer_ctx->highest_rx_peer_hpc = hpc_from_ie;
						hpc_for_iv = hpc_from_ie;
					} else {
						pdc_process_ok_for_feedback = false;
					}
				} else {
					pdc_process_ok_for_feedback = false;
				}
				payload_to_decrypt_start = sdu_area_after_common_hdr + cleartext_sec_ie_mux_len;
				payload_to_decrypt_len = sdu_area_plus_mic_len_in_payload - cleartext_sec_ie_mux_len;
			} else {
				pdc_process_ok_for_feedback = false;
			}
		} else {
			cleartext_sec_ie_mux_len = 0;
			payload_to_decrypt_start = sdu_area_after_common_hdr;
			payload_to_decrypt_len = sdu_area_plus_mic_len_in_payload;
		}

		if (!pdc_process_ok_for_feedback || payload_to_decrypt_len < 5) {
			goto process_feedback;
		}

		uint8_t iv[16];
		security_build_iv(iv, ft_tx_long_id_from_hdr, ctx->own_long_rd_id, hpc_for_iv, received_psn);

		if (security_crypt_payload(payload_to_decrypt_start, payload_to_decrypt_len, ctx->cipher_key, iv, false) != 0) {
			pdc_process_ok_for_feedback = false;
			goto process_feedback;
		}

		uint8_t *cleartext_mic_ptr = payload_to_decrypt_start + payload_to_decrypt_len - 5;
		uint8_t calculated_mic[5];
		size_t data_for_mic_len = common_hdr_actual_len + (payload_to_decrypt_len - 5) + cleartext_sec_ie_mux_len;

		if (security_calculate_mic(common_hdr_start_in_payload, data_for_mic_len, ctx->integrity_key, calculated_mic) != 0) {
			pdc_process_ok_for_feedback = false;
			goto process_feedback;
		}

		if (memcmp(cleartext_mic_ptr, calculated_mic, 5) != 0) {
			active_ft_peer_ctx->consecutive_mic_failures++;
			if (active_ft_peer_ctx->consecutive_mic_failures >= MAX_MIC_FAILURES_BEFORE_HPC_RESYNC) {
				active_ft_peer_ctx->self_needs_to_request_hpc_from_peer = true;
			}
			pdc_process_ok_for_feedback = false;
		} else {
			active_ft_peer_ctx->consecutive_mic_failures = 0;
			active_ft_peer_ctx->hpc = hpc_for_iv;
			sdu_area_for_data_path = sdu_area_after_common_hdr + cleartext_sec_ie_mux_len;
			sdu_area_len_for_data_path = payload_to_decrypt_len - 5;
		}
	}
process_feedback:;
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

	if (active_ft_peer_ctx && ctx->state == MAC_STATE_ASSOCIATED) {
		uint8_t harq_proc_in_ft_tx = assoc_pcc_event->hdr.hdr_type_2.df_harq_process_num;
		if (active_ft_peer_ctx->num_pending_feedback_items < 2) {
			int fb_idx = active_ft_peer_ctx->num_pending_feedback_items++;
			active_ft_peer_ctx->pending_feedback_to_send[fb_idx].valid = true;
			active_ft_peer_ctx->pending_feedback_to_send[fb_idx].is_ack = pdc_process_ok_for_feedback;
			active_ft_peer_ctx->pending_feedback_to_send[fb_idx].harq_process_num_for_peer = harq_proc_in_ft_tx;
			LOG_DBG("PT_SM_HARQ_RX: Stored %s for FT's HARQ_Proc %u.",
				pdc_process_ok_for_feedback ? "ACK" : "NACK",
				harq_proc_in_ft_tx);
		} else {
			LOG_WRN("PT_SM_PDC: Feedback buffer full for FT 0x%04X",
				ft_sender_short_id_from_pcc);
		}
	}

	if (!pdc_process_ok_for_feedback) {
		return;
	}

	uint32_t sender_long_id_final = 0;
	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		const dect_mac_unicast_header_t *uch = (const dect_mac_unicast_header_t *)common_hdr_start_in_payload;
		sender_long_id_final = sys_be32_to_cpu(uch->transmitter_long_rd_id_be);

		uint8_t ie_type; uint16_t ie_len; const uint8_t *ie_payload;
		int mux_hdr_len = parse_mac_mux_header(sdu_area_for_data_path, sdu_area_len_for_data_path, &ie_type, &ie_len, &ie_payload);

		if (mux_hdr_len > 0) {
			switch (ie_type) {
			case IE_TYPE_ASSOC_RESP:
				printk("  - Found Association Response IE in PDU. State: %s\n",
				       dect_mac_state_to_str(ctx->state));
				if (ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP) {
					pt_process_association_response_pdu(
						sdu_area_for_data_path,
						sdu_area_len_for_data_path,
						sender_long_id_final,
						pcc_reception_modem_time);
				}
				break;
			case IE_TYPE_ASSOC_RELEASE:
				if (ctx->state == MAC_STATE_PT_RELEASING) {
					dect_mac_assoc_release_ie_t release_fields;
					if (parse_assoc_release_ie_payload(ie_payload, ie_len, &release_fields) == 0) {
						LOG_INF("PT SM: Received Association Release confirmation from FT (cause: %u). Disconnecting.", release_fields.cause);
						dect_mac_sm_pt_start_operation(); /* Restart full scan */
					}
				}
				break;
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
			case IE_TYPE_AUTH_CHALLENGE:
				if (ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE && ie_len == sizeof(dect_mac_auth_challenge_ie_t)) {
					const dect_mac_auth_challenge_ie_t *chal = (const dect_mac_auth_challenge_ie_t *)ie_payload;
					active_ft_peer_ctx->ft_nonce = sys_be32_to_cpu(chal->ft_nonce_be);
					pt_send_auth_response_action();
				}
				break;
			case IE_TYPE_AUTH_SUCCESS:
				if (ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS && ie_len == sizeof(dect_mac_auth_success_ie_t)) {
					const dect_mac_auth_success_ie_t *succ = (const dect_mac_auth_success_ie_t *)ie_payload;
					pt_process_auth_success(ctx, succ->ft_mac);
				}
				break;
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */
			default:
				if (active_ft_peer_ctx && active_ft_peer_ctx->is_valid) {
					dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path, sdu_area_len_for_data_path, sender_long_id_final);
				}
				break;
			}
		}
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU && active_ft_peer_ctx && active_ft_peer_ctx->is_valid) {
		sender_long_id_final = active_ft_peer_ctx->long_rd_id;
		dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path, sdu_area_len_for_data_path, sender_long_id_final);
	} else {
		LOG_WRN("PT_SM_PDC: Received PDC with unhandled/unexpected MAC Common Header Type %u.",
			mac_hdr_type_octet.mac_header_type);
	}
}


/***************************************  UNUSED WARNING ***************************************/
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
// static void pt_initiate_authentication_handshake(
// 	dect_mac_context_t *ctx, const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
// 	const dect_mac_rach_info_ie_fields_t *rach_fields, uint32_t ft_long_id,
// 	uint16_t ft_short_id, int16_t rssi_q7_1, uint16_t beacon_rx_carrier,
// 	uint64_t beacon_pcc_rx_time)
// {
// 	/* Store target FT info and RACH params, similar to legacy assoc */
// 	pt_process_identified_beacon_and_attempt_assoc(ctx, cb_fields, rach_fields, ft_long_id,
// 						       ft_short_id, rssi_q7_1, beacon_rx_carrier,
// 						       beacon_pcc_rx_time);

// 	/* Generate and store PT nonce */
// 	dect_mac_rand_get((uint8_t *)&ctx->role_ctx.pt.target_ft.pt_nonce,
// 			    sizeof(ctx->role_ctx.pt.target_ft.pt_nonce));

// 	/* Cancel scan and send Auth Initiate */
// 	if (ctx->pending_op_type == PENDING_OP_PT_SCAN && ctx->pending_op_handle != 0) {
// 		dect_mac_phy_ctrl_cancel_op(ctx->pending_op_handle);
// 		/* The actual send will happen in the OP_COMPLETE handler for the cancelled scan */
// 	} else {
// 		pt_send_auth_initiate_action();
// 	}
// }
#endif

static void pt_process_identified_beacon_and_attempt_assoc(dect_mac_context_t *ctx,
                                                           const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
                                                           const dect_mac_rach_info_ie_fields_t *rach_fields,
                                                           uint32_t ft_long_id,
                                                           uint16_t ft_short_id,
                                                           int16_t rssi_q7_1,
                                                           uint16_t beacon_rx_carrier,
                                                           uint64_t beacon_pcc_rx_time)
{
	printk("[PT_SM] In pt_process_identified_beacon_and_attempt_assoc.\n");
    if (!ctx || !cb_fields || !rach_fields) {
		printk("[PT_SM] PT_BEACON_PROC: NULL arguments.");
        LOG_ERR("PT_BEACON_PROC: NULL arguments.");
        return;
    }

	printk("[PT_BEACON_PROC] PT Network ID before processing beacon: 0x%08X\n", ctx->network_id_32bit);


    bool select_this_ft = false;
	/***************************************  UNUSED WARNING ***************************************/
    // uint8_t new_ft_cost = 255;

    /* TODO: Find the Route Info IE to get the cost */
    /* This logic is now inside , which needs to be updated */
    /* For now, assume route info is passed in or looked up differently */
    /* Simplified logic: assume mesh is not primary factor for now */
    if (!ctx->role_ctx.pt.target_ft.is_valid) {
        select_this_ft = true;
        printk("PT_BEACON_PROC: No current target. Selecting FT 0x%04X (RSSI:%d).\n", ft_short_id, rssi_q7_1 );
		LOG_INF("PT_BEACON_PROC: No current target. Selecting FT 0x%04X (RSSI:%d).", ft_short_id, rssi_q7_1 );
    } else if (rssi_q7_1 > (ctx->role_ctx.pt.target_ft.rssi_2 + (RSSI_HYSTERESIS_DB * 2))) {
        select_this_ft = true;
        printk("PT_BEACON_PROC: New FT 0x%04X has better RSSI (%.1f > %d). Switching.\n",
                ft_short_id, rssi_q7_1 / 2.0, ctx->role_ctx.pt.target_ft.rssi_2);
        LOG_INF("PT_BEACON_PROC: New FT 0x%04X has better RSSI (%.1f > %d). Switching.",
                ft_short_id, rssi_q7_1 / 2.0, ctx->role_ctx.pt.target_ft.rssi_2);				
    }


    if (select_this_ft) {
        printk("PT_BEACON_PROC: Selected FT LongID:0x%08X, ShortID:0x%04X, BeaconRxCarrier:%u, RSSI2:%d dBm \n",
                ft_long_id, ft_short_id, beacon_rx_carrier, rssi_q7_1);
        LOG_INF("PT_BEACON_PROC: Selected FT LongID:0x%08X, ShortID:0x%04X, BeaconRxCarrier:%u, RSSI2:%d dBm",
                ft_long_id, ft_short_id, beacon_rx_carrier, rssi_q7_1);

		/* --- DEBUG LOGGING: Verify parsed beacon period code --- */
		LOG_DBG("PT_BEACON_PROC: Parsed Cluster Beacon IE -> cluster_beacon_period_code: %u",
			cb_fields->cluster_beacon_period_code);

		/* Store beacon period from the parsed beacon IE */
		uint32_t clus_period_ms_val[] = {10, 50, 100, 500, 1000, 1500, 2000, 4000, 8000, 16000, 32000};
		if (cb_fields->cluster_beacon_period_code < ARRAY_SIZE(clus_period_ms_val)) {
			ctx->role_ctx.pt.target_ft.beacon_period_ms =
				clus_period_ms_val[cb_fields->cluster_beacon_period_code];
		}

		/* --- DEBUG LOGGING: Verify stored millisecond value --- */
		LOG_DBG("PT_BEACON_PROC: Stored beacon_period_ms in target_ft context: %u ms",
			ctx->role_ctx.pt.target_ft.beacon_period_ms);
			
        dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);
        
        ctx->role_ctx.pt.target_ft.is_valid = true;
        ctx->role_ctx.pt.target_ft.is_fully_identified = (ft_long_id != 0);
        ctx->role_ctx.pt.target_ft.long_rd_id = ft_long_id;
        ctx->role_ctx.pt.target_ft.short_rd_id = ft_short_id;
        ctx->role_ctx.pt.target_ft.rssi_2 = rssi_q7_1;

        if (cb_fields->next_channel_present && cb_fields->next_cluster_channel_val != 0 &&
            cb_fields->next_cluster_channel_val != 0xFFFF) {
            ctx->role_ctx.pt.target_ft.operating_carrier = cb_fields->next_cluster_channel_val;
        } else {
            ctx->role_ctx.pt.target_ft.operating_carrier = beacon_rx_carrier;
        }
        printk("PT_BEACON_PROC: Target FT operating_carrier set to: %u \n", ctx->role_ctx.pt.target_ft.operating_carrier);
		LOG_INF("PT_BEACON_PROC: Target FT operating_carrier set to: %u", ctx->role_ctx.pt.target_ft.operating_carrier);

        uint32_t frame_duration_ticks = (uint32_t)FRAME_DURATION_MS_NOMINAL *
                                        (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);

        uint64_t new_sfn0_estimate = beacon_pcc_rx_time -
                                     ((uint64_t)cb_fields->sfn * frame_duration_ticks);

        if (ctx->ft_sfn_zero_modem_time_anchor == 0 || select_this_ft) {
            ctx->ft_sfn_zero_modem_time_anchor = new_sfn0_estimate;
        } else {
            ctx->ft_sfn_zero_modem_time_anchor = (ctx->ft_sfn_zero_modem_time_anchor + new_sfn0_estimate) / 2;
        }
        ctx->current_sfn_at_anchor_update = cb_fields->sfn;
        LOG_DBG("PT_BEACON_PROC: FT SFN0 Anchor: %llu (Beacon SFN %u @ %llu)",
                ctx->ft_sfn_zero_modem_time_anchor, cb_fields->sfn, beacon_pcc_rx_time);

        memcpy(&ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields,
               rach_fields, sizeof(dect_mac_rach_info_ie_fields_t));

        if (rach_fields->channel_field_present &&
            rach_fields->channel_abs_num != 0 &&
            rach_fields->channel_abs_num != 0xFFFF) {
            ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel = rach_fields->channel_abs_num;
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel = ctx->role_ctx.pt.target_ft.operating_carrier;
        }

        if (rach_fields->cwmin_sig_code <= 7) {
             ctx->role_ctx.pt.current_ft_rach_params.cw_min_val = 8 * (1U << rach_fields->cwmin_sig_code);
        } else {
             ctx->role_ctx.pt.current_ft_rach_params.cw_min_val = 8 * (1U << ctx->config.rach_cw_min_idx);
        }
        if (rach_fields->cwmax_sig_code <= 7) {
            ctx->role_ctx.pt.current_ft_rach_params.cw_max_val = 8 * (1U << rach_fields->cwmax_sig_code);
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.cw_max_val = 8 * (1U << ctx->config.rach_cw_max_idx);
        }

        uint32_t resp_win_subslots_actual = rach_fields->response_window_subslots_val_minus_1 + 1;
        uint8_t ft_mu_code_from_beacon = rach_fields->mu_value_for_ft_beacon;
        if (ft_mu_code_from_beacon > 7) {
            ft_mu_code_from_beacon = 0;
        }
        uint32_t ft_subslot_duration_ticks = get_subslot_duration_ticks_for_mu(ft_mu_code_from_beacon);

        if (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ > 0 && ft_subslot_duration_ticks > 0) {
            ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us =
                (resp_win_subslots_actual * ft_subslot_duration_ticks * 1000U) / NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ;
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us = ctx->config.rach_response_window_ms * 1000;
        }

        ctx->role_ctx.pt.current_assoc_retries = 0;
        ctx->rach_context.rach_cw_current_idx = ctx->config.rach_cw_min_idx;

		if (ctx->pending_op_type == PENDING_OP_PT_SCAN && ctx->pending_op_handle != 0) {
			printk("PT_BEACON_PROC: Cancelling ongoing scan (handle %u) to associate with FT 0x%04X.\n",
				ctx->pending_op_handle, ft_short_id);
			// LOG_INF("PT_BEACON_PROC: Cancelling ongoing scan (handle %u) to associate with FT 0x%04X.",
			// 	ctx->pending_op_handle, ft_short_id);
			// printk("[PT_SM] dect_mac_change_state Called \n");
			dect_mac_change_state(MAC_STATE_PT_CANCELLING_SCAN);
			// printk("[PT_SM_EVIDENCE] Returned from dect_mac_change_state. Now calling cancel op.\n");
			// printk("[PT_SM] dect_mac_phy_ctrl_cancel_op Called \n");
			dect_mac_phy_ctrl_cancel_op(ctx->pending_op_handle);
			/* The actual association request will be sent from the OP_COMPLETE handler for the cancelled scan */
		} else {
			LOG_INF("PT_BEACON_PROC: No active scan to cancel. Directly attempting association with FT 0x%04X.", ft_short_id);
			pt_send_association_request_action();
		}
    } else {
        printk("PT_BEACON_PROC: Beacon from FT 0x%04X (L:0x%08X, RSSI:%.1f) not better than current target 0x%04X (RSSI:%.1f). Continuing scan.",
                ft_short_id, ft_long_id, rssi_q7_1 / 2.0,
                ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.target_ft.rssi_2 / 2.0);
        LOG_DBG("PT_BEACON_PROC: Beacon from FT 0x%04X (L:0x%08X, RSSI:%.1f) not better than current target 0x%04X (RSSI:%.1f). Continuing scan.",
                ft_short_id, ft_long_id, rssi_q7_1 / 2.0,
                ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.target_ft.rssi_2 / 2.0);				
    }
}

static void pt_send_auth_initiate_action(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (!ctx->role_ctx.pt.target_ft.is_valid) {
		LOG_ERR("PT_AUTH_INIT: No valid target FT. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_change_state(MAC_STATE_PT_WAIT_AUTH_CHALLENGE);

	/* Start auth timeout timer (e.g., 2 seconds) */
	k_timer_start(&ctx->role_ctx.pt.auth_timeout_timer, K_SECONDS(2), K_NO_WAIT);

	uint8_t sdu_area_buf[32];
	int sdu_area_len = build_auth_initiate_ie_muxed(
		sdu_area_buf, sizeof(sdu_area_buf), ctx->role_ctx.pt.target_ft.pt_nonce);

	if (sdu_area_len < 0) {
		LOG_ERR("PT_AUTH_INIT: Failed to build Auth Initiate IE: %d", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t hdr_type_octet_byte = 0;
	hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F);
	hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;

	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(ctx->role_ctx.pt.target_ft.long_rd_id);

	mac_sdu_t *pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("PT_AUTH_INIT: Failed to alloc PDU buffer.");
		return;
	}

	uint16_t pdu_len;
	int ret;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						   hdr_type_octet_byte, &common_hdr, sizeof(common_hdr),
						   sdu_area_buf, (size_t)sdu_area_len, &pdu_len);

	if (ret != 0) {
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_sm_pt_start_operation();
		return;
	}


	printk("[PT_TX_EVIDENCE] Assembled Association Request PDU (len %u) for FT 0x%04X:\n",
	       pdu_len, ctx->role_ctx.pt.target_ft.short_rd_id);
	for (int i = 0; i < pdu_len; i++) {
		printk("%02x ", pdu_sdu->data[i]);
	}
	printk("\n");
	

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));
	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, pdu_sdu->data, pdu_len,
		ctx->role_ctx.pt.target_ft.short_rd_id, false, phy_op_handle,
		PENDING_OP_PT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);

	// dect_mac_buffer_free(pdu_sdu);

	if (ret != 0) {
		LOG_ERR("PT_AUTH_INIT: Failed to schedule Auth Initiate TX: %d. Entering backoff.", ret);
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);
		// uint32_t backoff_ms = 20 + (sys_rand32_get() % 50);
		uint32_t random_value;
		sys_rand_get(&random_value, sizeof(random_value));
		uint32_t backoff_ms = 20 + (random_value % 50);  // Range: 20-69ms		
		k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(backoff_ms), K_NO_WAIT);
	} else {
		LOG_INF("PT_AUTH_INIT: Auth Initiate TX scheduled (Hdl %u) to FT 0x%04X.",
			phy_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
	}
}


// Brief Overview: Enhances pt_send_association_request_action to fully populate
// the dect_mac_assoc_req_ie_t structure with all relevant ETSI fields,
// including HARQ parameters, and placeholder logic for requested Flow IDs and FT Mode parameters.
// Also populates the PT's RD Capability IE more completely.
static void pt_send_association_request_action(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("[PT_SM_ASSOC_REQ] Action started with context %p (Role: %s)\n",
	       (void *)ctx, (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
	printk("[PT_SM] In pt_send_association_request_action. State: %s\n", dect_mac_state_to_str(ctx->state));

	if (!ctx->role_ctx.pt.target_ft.is_valid || !ctx->role_ctx.pt.target_ft.is_fully_identified) {
		LOG_ERR("PT_SM_ASSOC_REQ: No valid or not fully identified target FT. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	// TODO: The correct channal needs to be collected from the beacon.
	if (ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0 ||
	    ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0xFFFF) {
			LOG_ERR("PT_SM_ASSOC_REQ: Updating Target FT RACH operating channel (0x%04X).", ctx->role_ctx.pt.target_ft.operating_carrier);
			ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel = ctx->role_ctx.pt.target_ft.operating_carrier;
	}
	
	
	if (ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0 ||
	    ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0xFFFF) {
		LOG_ERR("PT_SM_ASSOC_REQ: Target FT RACH operating channel invalid(0x%04X). Restarting scan.", ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t ft_max_rach_len_actual_units = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units + 1;
	if (ft_max_rach_len_actual_units == 0 || ft_max_rach_len_actual_units > 128) { /* Max N-1 is 127 for 7 bits */
		LOG_ERR("PT_SM_ASSOC_REQ: Target FT RACH max PDU length invalid (%u units). Cannot send. Restarting scan.", ft_max_rach_len_actual_units);
		dect_mac_sm_pt_start_operation();
		return;
	}


#if IS_ENABLED(CONFIG_ZTEST)
// TODO: Fix as the field is not populatied correctly in some tests
	// if (ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units = 0 ){
		// ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units = 7;
	if (ft_max_rach_len_actual_units < 2){
		ft_max_rach_len_actual_units = 7;
		LOG_ERR("PT_SM_ASSOC_REQ: Target FT RACH max PDU length invalid, ASSIGNED DEFAULT VALUE (%u units).", ft_max_rach_len_actual_units);
	}
#endif

	dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);

	LOG_INF("PT_SM_ASSOC_REQ: Attempting Association Request to FT 0x%04X on RACH carrier %u (Attempt %u).",
		ctx->role_ctx.pt.target_ft.short_rd_id,
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel,
		ctx->role_ctx.pt.current_assoc_retries + 1);

	uint8_t sdu_area_buf[128];
	dect_mac_assoc_req_ie_t assoc_req_fields;
	memset(&assoc_req_fields, 0, sizeof(assoc_req_fields));
	dect_mac_rd_capability_ie_t rd_cap_fields; /* PT's own capabilities */
	memset(&rd_cap_fields, 0, sizeof(rd_cap_fields));

	/* --- Populate Association Request IE Fields (dect_mac_assoc_req_ie_t) --- */
	assoc_req_fields.setup_cause_val = ASSOC_CAUSE_INITIAL_ASSOCIATION;
	assoc_req_fields.power_const_active = false;
	assoc_req_fields.ft_mode_capable = IS_ENABLED(CONFIG_DECT_MAC_PT_CAN_BE_FT);

	assoc_req_fields.number_of_flows_val = 0;

	assoc_req_fields.harq_params_present = true;
	assoc_req_fields.harq_processes_tx_val = CONFIG_DECT_MAC_PT_HARQ_TX_PROC_CODE & 0x07;
	assoc_req_fields.max_harq_re_tx_delay_code = CONFIG_DECT_MAC_PT_HARQ_RETX_DELAY_PT_CODE & 0x1F;
	assoc_req_fields.harq_processes_rx_val = CONFIG_DECT_MAC_PT_HARQ_RX_PROC_CODE & 0x07;
	assoc_req_fields.max_harq_re_rx_delay_code = CONFIG_DECT_MAC_PT_HARQ_RERX_DELAY_PT_CODE & 0x1F;

	// Use preprocessor directives to conditionally compile the code
	// that depends on the conditional Kconfig options.
#if IS_ENABLED(CONFIG_DECT_MAC_PT_CAN_BE_FT)
	assoc_req_fields.ft_mode_capable = true;
		assoc_req_fields.ft_beacon_periods_octet_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_SIGNAL_PERIODS);
		if (assoc_req_fields.ft_beacon_periods_octet_present) {
			assoc_req_fields.ft_network_beacon_period_code = CONFIG_DECT_MAC_PT_FT_MODE_NET_BEACON_PERIOD_CODE & 0x0F;
			assoc_req_fields.ft_cluster_beacon_period_code = CONFIG_DECT_MAC_PT_FT_MODE_CLUS_BEACON_PERIOD_CODE & 0x0F;
		}

		assoc_req_fields.ft_next_channel_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_NEXT_CHAN_PRESENT);
		assoc_req_fields.ft_time_to_next_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_TIME_TO_NEXT_PRESENT);
		assoc_req_fields.ft_current_channel_present = false;

		if (assoc_req_fields.ft_next_channel_present || assoc_req_fields.ft_time_to_next_present || assoc_req_fields.ft_current_channel_present) {
			assoc_req_fields.ft_param_flags_octet_present = true;
		} else {
			assoc_req_fields.ft_param_flags_octet_present = false;
		}

		if (assoc_req_fields.ft_next_channel_present) {
			assoc_req_fields.ft_next_cluster_channel_val = CONFIG_DECT_MAC_PT_FT_MODE_NEXT_CLUSTER_CHANNEL_VAL & 0x1FFF;
		}
		if (assoc_req_fields.ft_time_to_next_present) {
			assoc_req_fields.ft_time_to_next_us_val = CONFIG_DECT_MAC_PT_FT_MODE_TIME_TO_NEXT_US_VAL;
		}
#else
		// This is the "PT-only" case.
		assoc_req_fields.ft_mode_capable = false;
		assoc_req_fields.ft_beacon_periods_octet_present = false;
		assoc_req_fields.ft_param_flags_octet_present = false;
#endif

	/* --- Populate PT's RD Capability IE Fields from common Kconfig values --- */
	rd_cap_fields.release_version = CONFIG_DECT_MAC_CAP_RELEASE_VERSION;
	rd_cap_fields.num_phy_capabilities = 1; /* We are sending one explicit set */

	rd_cap_fields.supports_group_assignment = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_GROUP_ASSIGNMENT);
	rd_cap_fields.supports_paging = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_PAGING);
	rd_cap_fields.operating_modes_code = assoc_req_fields.ft_mode_capable
						 ? DECT_MAC_OP_MODE_BOTH
						 : DECT_MAC_OP_MODE_PT_ONLY;
	rd_cap_fields.supports_mesh = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_MESH);
	rd_cap_fields.supports_sched_data = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_SCHED_DATA);
	rd_cap_fields.mac_security_modes_code = IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
						    ? DECT_MAC_SECURITY_SUPPORT_MODE1
						    : DECT_MAC_SECURITY_SUPPORT_NONE;

	dect_mac_phy_capability_set_t *pt_phy_set0 = &rd_cap_fields.phy_variants[0];
	pt_phy_set0->dlc_service_type_support_code = CONFIG_DECT_MAC_CAP_DLC_SERVICE_SUPPORT_CODE;
	pt_phy_set0->rx_for_tx_diversity_code = CONFIG_DECT_MAC_CAP_RX_TX_DIVERSITY_CODE;
	pt_phy_set0->mu_value = ctx->own_phy_params.mu;
	pt_phy_set0->beta_value = ctx->own_phy_params.beta;
	pt_phy_set0->max_nss_for_rx_code = CONFIG_DECT_MAC_CAP_MAX_NSS_RX_CODE;
	pt_phy_set0->max_mcs_code = CONFIG_DECT_MAC_CAP_MAX_MCS_CODE;
	pt_phy_set0->harq_soft_buffer_size_code = CONFIG_DECT_MAC_CAP_HARQ_BUFFER_CODE;
	pt_phy_set0->num_harq_processes_code = CONFIG_DECT_MAC_CAP_NUM_HARQ_PROC_CODE;
	pt_phy_set0->harq_feedback_delay_code = CONFIG_DECT_MAC_CAP_HARQ_FEEDBACK_DELAY_CODE;
	pt_phy_set0->supports_dect_delay = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_DECT_DELAY);
	pt_phy_set0->supports_half_duplex = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_HALF_DUPLEX);

	/* --- Build SDU Area and MAC PDU --- */
	int sdu_area_len = build_assoc_req_ies_area(sdu_area_buf, sizeof(sdu_area_buf),
                                               &assoc_req_fields, &rd_cap_fields);
	if (sdu_area_len < 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to build Association Request SDU area: %d. Restarting scan.", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t hdr_type_octet_byte = 0;
	hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F) << 0;
	hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;
	
	dect_mac_unicast_header_t common_hdr;
	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1 /*reset*/);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(ctx->role_ctx.pt.target_ft.long_rd_id);

	uint8_t *full_mac_pdu_for_phy_slab = NULL;
	/* Use the public API to allocate the buffer */
	mac_sdu_t *pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("DATA_TX_INT: Failed to alloc full MAC PDU TX buffer");
		return;
	}
	full_mac_pdu_for_phy_slab = pdu_sdu->data;
	uint8_t * const full_mac_pdu_for_phy = full_mac_pdu_for_phy_slab;

	uint16_t pdu_len;
	int ret = dect_mac_phy_ctrl_assemble_final_pdu(full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE,
                                         hdr_type_octet_byte, &common_hdr, sizeof(common_hdr),
                                         sdu_area_buf, (size_t)sdu_area_len,
                                         &pdu_len);


		printk("[PT_CREATE_EVIDENCE] Assembled Association Request PDU (len %u) before sending to PHY control:\n", pdu_len);
		for (int i = 0; i < pdu_len; i++) {
			printk("%02x ", pdu_sdu->data[i]);
		}
		printk("\n");

	if (ret != 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to assemble Association Request PDU: %d. Restarting scan.", ret);
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/***************************************  UNUSED WARNING ***************************************/
	uint8_t pcc_pkt_len_f, pcc_pkt_len_type_f; // , pcc_mcs_f_ignored
	uint8_t rach_tx_mcs = 0;
	uint8_t ft_mu_for_rach_timing = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.mu_value_for_ft_beacon;
	if (ft_mu_for_rach_timing > 7) ft_mu_for_rach_timing = 0;
	uint8_t ft_beta_for_rach_timing = 0;

	dect_mac_phy_ctrl_calculate_pcc_params(pdu_len - sizeof(dect_mac_header_type_octet_t),
                                           ft_mu_for_rach_timing,
                                           ft_beta_for_rach_timing,
                                           &pcc_pkt_len_f, &rach_tx_mcs, &pcc_pkt_len_type_f);
	uint32_t assoc_req_tx_duration_actual_units = pcc_pkt_len_f + 1;
	if (pcc_pkt_len_type_f == 1) {
		assoc_req_tx_duration_actual_units *= get_subslots_per_etsi_slot_for_mu(ft_mu_for_rach_timing);
	}

	if (assoc_req_tx_duration_actual_units > ft_max_rach_len_actual_units) {
		LOG_ERR("PT_SM_ASSOC_REQ: Assembled AssocReq PDU needs %u units, but FT RACH max is %u. Cannot send. Restarting scan.",
			assoc_req_tx_duration_actual_units, ft_max_rach_len_actual_units);
		// k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab);
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));
	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, full_mac_pdu_for_phy,
		pdu_len, ctx->role_ctx.pt.target_ft.short_rd_id, false, phy_op_handle,
		PENDING_OP_PT_RACH_ASSOC_REQ, true, 0, ctx->role_ctx.pt.target_ft.peer_mu, NULL);

	/* If the TX was successfully scheduled, the HARQ process now owns the buffer.
	 * If it failed, we must free it here.
	 */
	if (ret != 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to schedule AssocReq TX: %d. Freeing buffer.", ret);
		// dect_mac_buffer_free((mac_sdu_t *)full_mac_pdu_for_phy_slab);
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);
		uint32_t random_value;
		dect_mac_rand_get((uint8_t *)&random_value, sizeof(random_value));
		uint32_t backoff_ms = 20 + (random_value % 50);
		k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(backoff_ms), K_NO_WAIT);
	} else {
		LOG_INF("PT_SM_ASSOC_REQ: Association Request TX scheduled (Hdl %u) to FT 0x%04X.",
			phy_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
		/* The buffer is now owned by the HARQ process and will be freed on ACK/timeout. */
	}
}




static void pt_process_association_response_pdu(const uint8_t *mac_sdu_area_data,
						size_t mac_sdu_area_len,
						uint32_t ft_tx_long_rd_id,
						uint64_t assoc_resp_pcc_rx_time)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	k_timer_stop(&ctx->rach_context.rach_response_window_timer);
printk("[PT_SM] PT ASSOC RESP PROCESSOR pt_process_association_response_pdu. State: %s\n", dect_mac_state_to_str(ctx->state));

	if (ft_tx_long_rd_id != ctx->role_ctx.pt.target_ft.long_rd_id) {
		printk("PT_SM_ASSOC_RESP: From unexpected FT L:0x%08X (expected L:0x%08X). Ignoring.",
			ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.long_rd_id);
		// LOG_WRN("PT_SM_ASSOC_RESP: From unexpected FT L:0x%08X (expected L:0x%08X). Ignoring.",
		// 	ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.long_rd_id);
		return;
	}

	dect_mac_assoc_resp_ie_t resp_fields;
	dect_mac_rd_capability_ie_t ft_cap_fields;
	dect_mac_resource_alloc_ie_fields_t res_alloc_fields;
	bool resp_ie_found = false;
	bool ft_cap_found = false;
	bool res_alloc_found = false;

	const uint8_t *current_ie_ptr = mac_sdu_area_data;
	size_t remaining_len = mac_sdu_area_len;

	printk("PT_SM_ASSOC_RESP: Processing AssocResp from FT L:0x%08X (S:0x%04X) \n",
		ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.short_rd_id);
	// LOG_INF("PT_SM_ASSOC_RESP: Processing AssocResp from FT L:0x%08X (S:0x%04X)",
	// 	ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.short_rd_id);

	while (remaining_len > 0) {
		uint8_t ie_type;
		uint16_t ie_payload_len;
		const uint8_t *ie_payload_ptr;
		
        // The parse_mac_mux_header function now correctly handles all length calculations.
		int mux_hdr_len = parse_mac_mux_header(current_ie_ptr, remaining_len, &ie_type,
						       &ie_payload_len, &ie_payload_ptr);

		if (mux_hdr_len <= 0 || (remaining_len < (size_t)mux_hdr_len + ie_payload_len)) {
			break;
		}

		if (ie_type == IE_TYPE_ASSOC_RESP) {
			if (parse_assoc_resp_ie_payload(ie_payload_ptr, ie_payload_len,
							&resp_fields) == 0) {
				resp_ie_found = true;
			}		
		} else if (ie_type == IE_TYPE_RD_CAPABILITY) {
			if (parse_rd_capability_ie_payload(ie_payload_ptr, ie_payload_len,
							   &ft_cap_fields) == 0) {
				ft_cap_found = true;
			}
		} else if (ie_type == IE_TYPE_RES_ALLOC) {
			if (ft_cap_found && ft_cap_fields.num_phy_capabilities >= 1) {
				uint8_t ft_mu_code = ft_cap_fields.phy_variants[0].mu_value;
				if (parse_resource_alloc_ie_payload(ie_payload_ptr, ie_payload_len,
								    ft_mu_code,
								    &res_alloc_fields) == 0) {
					res_alloc_found = true;
				}
			} else {
				LOG_WRN("PT_ASSOC_RESP: Found Resource Allocation IE but no preceding RD Capability IE. Cannot parse correctly.");
			}
		}

		size_t consumed = mux_hdr_len + ie_payload_len;
		if (consumed == 0) {
			LOG_WRN("PT_ASSOC_RESP: Consumed 0 bytes, breaking loop.");
			break;
		}
		if (remaining_len >= consumed) {
			remaining_len -= consumed;
			current_ie_ptr += consumed;
		} else {
			remaining_len = 0;
		}
	}

	if (!resp_ie_found) {
		LOG_ERR("PT_SM_ASSOC_RESP: Association Response IE missing. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	/*
	 * CRITICAL: The PDU has been received, so the RX operation that was listening for it
	 * is now obsolete. We must cancel it to clear the pending_op state before we
	 * attempt to schedule a new operation (like a new scan).
	 */
	if (ctx->pending_op_type == PENDING_OP_PT_WAIT_ASSOC_RESP) {
		printk("[PT_ASSOC_RESP_DBG] PDU processed. Clearing pending op and cancelling obsolete RX op (Hdl: %u).\n",
		       ctx->pending_op_handle);
		uint32_t handle_to_cancel = ctx->pending_op_handle;
		dect_mac_core_clear_pending_op(); /* Clear state synchronously */
		dect_mac_phy_ctrl_cancel_op(handle_to_cancel); /* Tell PHY to stop */
	}


	if (resp_fields.ack_nack) {
		printk("  - Association ACCEPTED by FT. (ctx->state:%d)\n", ctx->state);
		if (!ft_cap_found || !res_alloc_found) {
			LOG_ERR("PT_SM_ASSOC_RESP: ACK received but mandatory IEs missing (Cap:%d, Res:%d). Restarting scan.",
				ft_cap_found, res_alloc_found);
			dect_mac_sm_pt_start_operation();
			return;
		} else {
			/* --- Association ACCEPTED Logic --- */
			printk("[PT_ASSOC_RESP_DBG] Association was ACCEPTED. Proceeding to secure link.\n");
		}

		if (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING) {
			printk("MOBILITY: Handover association to FT 0x%04X ACCEPTED.",
				ctx->role_ctx.pt.target_ft.short_rd_id);
			// LOG_INF("MOBILITY: Handover association to FT 0x%04X ACCEPTED.",
			// 	ctx->role_ctx.pt.target_ft.short_rd_id);				
			dect_mac_peer_info_t old_ft_info = ctx->role_ctx.pt.associated_ft;

			memcpy(&ctx->role_ctx.pt.associated_ft, &ctx->role_ctx.pt.target_ft,
			       sizeof(dect_mac_peer_info_t));

			if (old_ft_info.is_valid) {
				pt_send_association_release_action(&old_ft_info);
			}
			pt_requeue_held_packets(ctx);
		} else {
			printk("PT_SM: Association ACCEPTED by FT L:0x%08X (S:0x%04X).",
				ctx->role_ctx.pt.target_ft.long_rd_id,
				ctx->role_ctx.pt.target_ft.short_rd_id);
			// LOG_INF("PT_SM: Association ACCEPTED by FT L:0x%08X (S:0x%04X).",
			// 	ctx->role_ctx.pt.target_ft.long_rd_id,
			// 	ctx->role_ctx.pt.target_ft.short_rd_id);				
			memcpy(&ctx->role_ctx.pt.associated_ft, &ctx->role_ctx.pt.target_ft,
			       sizeof(dect_mac_peer_info_t));
		}
		ctx->role_ctx.pt.associated_ft.is_valid = true;

		if (ft_cap_found) {
			ctx->role_ctx.pt.associated_ft.num_phy_variants = ft_cap_fields.actual_num_phy_variants_parsed;
			if (ctx->role_ctx.pt.associated_ft.num_phy_variants > 0) {
				for (int i = 0; i < ctx->role_ctx.pt.associated_ft.num_phy_variants; i++) {
					memcpy(&ctx->role_ctx.pt.associated_ft.phy_variants[i], &ft_cap_fields.phy_variants[i],
					       sizeof(dect_mac_phy_capability_set_t));
				}
				ctx->role_ctx.pt.associated_ft.peer_mu = ft_cap_fields.phy_variants[0].mu_value;
				ctx->role_ctx.pt.associated_ft.peer_beta = ft_cap_fields.phy_variants[0].beta_value;
				ctx->role_ctx.pt.associated_ft.peer_phy_params_known = true;
			}
		}

		/* Store the negotiated parameters from the FT's response */
		if (resp_fields.harq_mod_present) {
			LOG_INF("PT_SM: FT negotiated HARQ params -> PT_TX(FT_RX) Procs: %u, Delay: %u; PT_RX(FT_TX) Procs: %u, Delay: %u",
				resp_fields.harq_processes_rx_val_ft, /* FT's RX is our TX */
				resp_fields.max_harq_re_rx_delay_code_ft,
				resp_fields.harq_processes_tx_val_ft, /* FT's TX is our RX */
				resp_fields.max_harq_re_tx_delay_code_ft);

			ctx->role_ctx.pt.associated_ft.pt_req_harq_procs_tx = resp_fields.harq_processes_rx_val_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_max_harq_retx_delay = resp_fields.max_harq_re_rx_delay_code_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_harq_procs_rx = resp_fields.harq_processes_tx_val_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_max_harq_rerx_delay = resp_fields.max_harq_re_tx_delay_code_ft;
		}

		if (resp_fields.group_assignment_active) {
			ctx->role_ctx.pt.associated_ft.group_id = resp_fields.group_id_val;
			ctx->role_ctx.pt.associated_ft.resource_tag = resp_fields.resource_tag_val;
			LOG_INF("PT_SM: Assigned to Group ID %u with Resource Tag %u by FT.",
				resp_fields.group_id_val, resp_fields.resource_tag_val);
		}

		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.target_ft.is_valid = false;

		if (res_alloc_found) {
			LOG_INF("PT_SM: Storing schedule from FT 0x%04X.",
				ctx->role_ctx.pt.associated_ft.short_rd_id);

			printk("[DEBUG_PROBE] SFN at Assoc Resp time: FT's SFN (from beacon)=%u, PT's current SFN=%u\n",
			       ctx->current_sfn_at_anchor_update, ctx->role_ctx.ft.sfn);
			
			// /* Synchronize the PT's SFN with the FT's SFN from the beacon */
			// ctx->role_ctx.ft.sfn = ctx->current_sfn_at_anchor_update;
			
			uint32_t frame_duration_ticks_val =
				(uint32_t)FRAME_DURATION_MS_NOMINAL *
				(NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
			if (frame_duration_ticks_val == 0) {
				LOG_ERR("PT_SCHED: Frame duration ticks is 0!");
				return;
			}

			uint8_t ft_mu_code = ctx->role_ctx.pt.associated_ft.peer_phy_params_known
						     ? ctx->role_ctx.pt.associated_ft.peer_mu
						     : 0;
			uint8_t subslots_per_ft_slot = get_subslots_per_etsi_slot_for_mu(ft_mu_code);
			uint16_t schedule_channel =
				res_alloc_fields.channel_present
					? res_alloc_fields.channel_val
					: ctx->role_ctx.pt.associated_ft.operating_carrier;

			/* Populate Downlink Schedule */
			dect_mac_schedule_t *dl_sched = &ctx->role_ctx.pt.dl_schedule;
			memset(dl_sched, 0, sizeof(dect_mac_schedule_t));
			dl_sched->is_active = true;
			dl_sched->alloc_type = RES_ALLOC_TYPE_DOWNLINK;
			dl_sched->res1_is_9bit_subslot = res_alloc_fields.res1_is_9bit_subslot;
			dl_sched->dl_start_subslot = res_alloc_fields.start_subslot_val_res1;
			dl_sched->dl_length_is_slots = res_alloc_fields.length_type_is_slots_res1;
			dl_sched->dl_duration_subslots = res_alloc_fields.length_val_res1 + 1;
			if (dl_sched->dl_length_is_slots) {
				dl_sched->dl_duration_subslots *= subslots_per_ft_slot;
			}

			dl_sched->repeat_type = res_alloc_fields.repeat_val;
			dl_sched->repetition_value = res_alloc_fields.repetition_value;
			dl_sched->validity_value = res_alloc_fields.validity_value;
			dl_sched->channel = schedule_channel;
			dl_sched->schedule_init_modem_time = assoc_resp_pcc_rx_time;

			if (res_alloc_fields.sfn_present) {
				dl_sched->sfn_of_initial_occurrence = res_alloc_fields.sfn_val;
				dl_sched->next_occurrence_modem_time = calculate_target_modem_time(
					ctx, ctx->ft_sfn_zero_modem_time_anchor,
					ctx->current_sfn_at_anchor_update, res_alloc_fields.sfn_val,
					dl_sched->dl_start_subslot, ft_mu_code,
					ctx->role_ctx.pt.associated_ft.peer_beta);
			} else {
				dl_sched->sfn_of_initial_occurrence =
					ctx->current_sfn_at_anchor_update;
				uint64_t now_plus_processing_delay =
					assoc_resp_pcc_rx_time +
					modem_us_to_ticks(
						CONFIG_DECT_MAC_SCHEDULE_PROCESSING_DELAY_US,
						NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
				uint64_t current_frame_start_approx =
					(now_plus_processing_delay / frame_duration_ticks_val) *
					frame_duration_ticks_val;
				if (current_frame_start_approx < ctx->ft_sfn_zero_modem_time_anchor) {
					current_frame_start_approx = ctx->ft_sfn_zero_modem_time_anchor;
				}
				uint32_t ft_subslot_duration = get_subslot_duration_ticks_for_mu(ft_mu_code);
				uint64_t candidate_time =
					current_frame_start_approx +
					(uint64_t)dl_sched->dl_start_subslot * ft_subslot_duration;
				while (candidate_time <= now_plus_processing_delay) {
					candidate_time += frame_duration_ticks_val;
				}
				dl_sched->next_occurrence_modem_time = candidate_time;
			}
			update_next_occurrence(ctx, dl_sched, ctx->last_known_modem_time, ft_mu_code);
			LOG_INF("PT_SM: DL Schedule Init: NextOcc @ %llu, StartSS %u, Dur %u subslots",
				dl_sched->next_occurrence_modem_time, dl_sched->dl_start_subslot,
				dl_sched->dl_duration_subslots);

			/* Populate Uplink Schedule */
			if (res_alloc_fields.alloc_type_val == RES_ALLOC_TYPE_BIDIR) {
				dect_mac_schedule_t *ul_sched = &ctx->role_ctx.pt.ul_schedule;
				memset(ul_sched, 0, sizeof(dect_mac_schedule_t));
				ul_sched->is_active = true;
				ul_sched->alloc_type = RES_ALLOC_TYPE_UPLINK;
				ul_sched->res1_is_9bit_subslot = res_alloc_fields.res2_is_9bit_subslot;
				ul_sched->ul_start_subslot = res_alloc_fields.start_subslot_val_res2;
				ul_sched->ul_length_is_slots =
					res_alloc_fields.length_type_is_slots_res2;
				ul_sched->ul_duration_subslots = res_alloc_fields.length_val_res2 + 1;
				if (ul_sched->ul_length_is_slots) {
					ul_sched->ul_duration_subslots *= subslots_per_ft_slot;
				}
				ul_sched->repeat_type = res_alloc_fields.repeat_val;
				ul_sched->repetition_value = res_alloc_fields.repetition_value;
				ul_sched->validity_value = res_alloc_fields.validity_value;
				ul_sched->channel = schedule_channel;
				ul_sched->schedule_init_modem_time = assoc_resp_pcc_rx_time;

				if (res_alloc_fields.sfn_present) {
					ul_sched->sfn_of_initial_occurrence = res_alloc_fields.sfn_val;
					ul_sched->next_occurrence_modem_time = calculate_target_modem_time(
						ctx, ctx->ft_sfn_zero_modem_time_anchor,
						ctx->current_sfn_at_anchor_update,
						res_alloc_fields.sfn_val, ul_sched->ul_start_subslot,
						ft_mu_code, ctx->role_ctx.pt.associated_ft.peer_beta);
				} else {
					ul_sched->sfn_of_initial_occurrence =
						ctx->current_sfn_at_anchor_update;
					uint64_t now_plus_processing_delay =
						assoc_resp_pcc_rx_time +
						modem_us_to_ticks(
							CONFIG_DECT_MAC_SCHEDULE_PROCESSING_DELAY_US,
							NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
					uint64_t current_frame_start_approx =
						(now_plus_processing_delay /
						 frame_duration_ticks_val) *
						frame_duration_ticks_val;
					if (current_frame_start_approx <
					    ctx->ft_sfn_zero_modem_time_anchor) {
						current_frame_start_approx =
							ctx->ft_sfn_zero_modem_time_anchor;
					}
					uint32_t ft_subslot_duration = get_subslot_duration_ticks_for_mu(ft_mu_code);
					uint64_t candidate_time =
						current_frame_start_approx +
						(uint64_t)ul_sched->ul_start_subslot *
							ft_subslot_duration;
					while (candidate_time <= now_plus_processing_delay) {
						candidate_time += frame_duration_ticks_val;
					}
					ul_sched->next_occurrence_modem_time = candidate_time;
				}
				update_next_occurrence(ctx, ul_sched, ctx->last_known_modem_time, ft_mu_code);
				LOG_INF("PT_SM: UL Schedule Init: NextOcc @ %llu, StartSS %u, Dur %u subslots",
					ul_sched->next_occurrence_modem_time,
					ul_sched->ul_start_subslot, ul_sched->ul_duration_subslots);
			} else {
				ctx->role_ctx.pt.ul_schedule.is_active = false;
			}
			/* Store the FT's beacon period for the listen timer */
			ctx->role_ctx.pt.associated_ft.beacon_period_ms =
				ctx->role_ctx.pt.target_ft.beacon_period_ms;
		}

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
		dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);
		dect_mac_sm_pt_initiate_authentication_protocol();
#else
		/* Security is disabled. A successful response means we are now associated. */
		ctx->keys_provisioned = false;
		ctx->role_ctx.pt.associated_ft.is_secure = false;
		printk("[SM PT]: pt_process_association_response_pdu ->dect_mac_change_state Started... MAC_STATE_ASSOCIATED \n");
		// dect_mac_change_state(MAC_STATE_ASSOCIATED);
		// printk("PT_SM: Association successful. Link is UNSECURE.");
		// printk("  - State transitioned to ASSOCIATED.\n");


		dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);
		pt_authentication_complete_action(ctx, false); /* Proceed with unsecure link setup */
		printk("PT_SM: Association Link is UNSECURE.");
		printk("  - State transitioned to MAC_STATE_PT_AUTHENTICATING.\n");

		/* Start periodic timers for an active link */
		k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
			      K_MSEC(ctx->config.keep_alive_period_ms),
			      K_MSEC(ctx->config.keep_alive_period_ms));
		if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
			k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
				      K_MSEC(ctx->config.mobility_scan_interval_ms),
				      K_MSEC(ctx->config.mobility_scan_interval_ms));
		}
		/* Start the beacon listen timer based on the FT's advertised period */
		uint32_t beacon_period_ms = ctx->role_ctx.pt.associated_ft.beacon_period_ms;
		if (beacon_period_ms > 0) {
			LOG_INF("PT_SM: Starting beacon listen timer with period %u ms.", beacon_period_ms);
			k_timer_start(&ctx->role_ctx.pt.beacon_listen_timer, K_MSEC(beacon_period_ms), K_MSEC(beacon_period_ms));
		} else {
			LOG_WRN("PT_SM: FT beacon period is 0. Cannot start beacon listen timer.");
		}

		ctx->role_ctx.pt.current_assoc_retries = 0;
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */
	} else {
		/* --- Association REJECTED Logic --- */
		printk("[PT_ASSOC_RESP_DBG] Association was REJECTED by FT.\n");
		printk("  - Reject Cause: %u, Reject Timer Code: %u\n",
		       resp_fields.reject_cause, resp_fields.reject_timer_code);

		/* Start the reject timer to prevent immediate re-association with this FT */
		uint32_t backoff_ms = resp_fields.reject_timer_code * 100; /* Timer code is in 100ms units */
		if (backoff_ms > 0) {
			LOG_INF("PT_SM: Starting reject backoff timer for %u ms for FT 0x%08X",
				backoff_ms, ctx->role_ctx.pt.target_ft.long_rd_id);
			k_timer_start(&ctx->role_ctx.pt.reject_timer, K_MSEC(backoff_ms), K_NO_WAIT);
		}

		if (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING) {
			printk("  - Handling as a FAILED HANDOVER.\n");
			pt_revert_to_old_ft_after_handover_failure();
		} else {
			printk("  - Handling as a FAILED INITIAL ASSOCIATION. Restarting scan.\n");
			memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
			ctx->role_ctx.pt.target_ft.is_valid = false;
			dect_mac_sm_pt_start_operation();
		}
	}
	printk("--- END PT ASSOC RESP PROCESSOR ---\n");	
}

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)	
static void pt_send_auth_response_action(void)
{

	dect_mac_context_t *ctx = dect_mac_get_active_context();
	dect_mac_peer_info_t *target_ft = &ctx->role_ctx.pt.target_ft;
	uint8_t pt_mac[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t k_auth[16];
	int ret;

	LOG_INF("PT_AUTH_RESP: Generating Auth Response for FT 0x%04X.", target_ft->short_rd_id);

	ret = security_derive_auth_key(ctx->master_psk, k_auth);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to derive K_auth: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	ret = security_generate_auth_mac(k_auth, target_ft->pt_nonce, target_ft->ft_nonce,
					 ctx->own_long_rd_id, target_ft->long_rd_id, pt_mac);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to generate PT MAC: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t sdu_area_buf[32];
	int sdu_area_len =
		build_auth_response_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf), pt_mac);
	if (sdu_area_len < 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to build Auth Response IE: %d", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t hdr_type_octet_byte = 0;
	hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F);
	hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;

	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(target_ft->long_rd_id);

	mac_sdu_t *pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("PT_AUTH_RESP: Failed to alloc PDU buffer.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint16_t pdu_len;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						   hdr_type_octet_byte, &common_hdr, sizeof(common_hdr),
						   sdu_area_buf, (size_t)sdu_area_len, &pdu_len);

	if (ret != 0) {
		dect_mac_buffer_free(pdu_sdu);
		return;
	}

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, pdu_sdu->data, pdu_len,
		target_ft->short_rd_id, false, phy_op_handle, PENDING_OP_PT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);


	if (ret == 0) {
		dect_mac_change_state(MAC_STATE_PT_WAIT_AUTH_SUCCESS);
		/* Restart auth timer for the next leg (Auth Success) */
		k_timer_start(&ctx->role_ctx.pt.auth_timeout_timer, K_SECONDS(2), K_NO_WAIT);
	} else {
		/* TODO: Handle RACH busy/backoff */
		dect_mac_buffer_free(pdu_sdu);
		dect_mac_sm_pt_start_operation();
	}
	
}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
static void pt_process_auth_success(dect_mac_context_t *ctx, const uint8_t *ft_mac)
{

	dect_mac_peer_info_t *target_ft = &ctx->role_ctx.pt.target_ft;
	uint8_t k_auth[16];
	uint8_t expected_ft_mac[DECT_MAC_AUTH_MAC_SIZE];
	int ret;

	LOG_INF("PT_AUTH_SUCC: Received Auth Success from FT 0x%04X.", target_ft->short_rd_id);

	ret = security_derive_auth_key(ctx->master_psk, k_auth);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Failed to derive K_auth: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/* Generate the expected FT MAC for verification */
	ret = security_generate_auth_mac(k_auth, target_ft->ft_nonce, target_ft->pt_nonce,
					 target_ft->long_rd_id, ctx->own_long_rd_id,
					 expected_ft_mac);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Failed to generate expected FT MAC: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/* Stop the auth timer now that we have a response */
	k_timer_stop(&ctx->role_ctx.pt.auth_timeout_timer);

	// Use constant-time comparison
	if (!dect_mac_security_timing_safe_equal(ft_mac, expected_ft_mac, DECT_MAC_AUTH_MAC_SIZE)) {
		LOG_ERR("PT_AUTH_SUCC: FT MAC verification FAILED! Aborting association.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	LOG_INF("PT_AUTH_SUCC: FT MAC verified. Deriving session keys.");

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	/* Derive final session keys using the now-trusted K_auth */
	ret = security_derive_session_keys(k_auth, ctx->own_long_rd_id, target_ft->long_rd_id,
					   ctx->integrity_key, ctx->cipher_key);
#endif

	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Session key derivation failed: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/* Authentication is fully complete */
	pt_authentication_complete_action(ctx, true);
	
}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

static void pt_process_group_assignment_ie(const uint8_t *payload, uint16_t len)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	dect_mac_peer_info_t *ft = &ctx->role_ctx.pt.associated_ft;

	if (!ft->is_valid || ft->group_id == 0) {
		return; /* Not part of a group */
	}

	uint8_t group_id = (payload[0] >> 0) & 0x3F;

	if (group_id != ft->group_id) {
		LOG_DBG("GROUP_ASSIGN: Group ID mismatch (rcvd %u, own %u)", group_id, ft->group_id);
		return; /* Not for our group */
	}

	uint8_t num_tags = len - 1;
	LOG_INF("GROUP_ASSIGN: Processing Group %u Assignment with %u tags. Own tag: %u", group_id, num_tags, ft->resource_tag);

	for (int i = 0; i < num_tags; i++) {
		uint8_t tag = payload[i + 1] & 0x7F;
		LOG_DBG("GROUP_ASSIGN: Checking tag[%d] = %u", i, tag);

		if (tag == ft->resource_tag) {
			/* This is our slot repetition */
			dect_mac_schedule_t *ul_sched = &ctx->role_ctx.pt.ul_schedule;

			if (ctx->role_ctx.pt.group_schedule.is_active &&
			    (ctx->role_ctx.pt.group_schedule.repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP ||
			     ctx->role_ctx.pt.group_schedule.repeat_type == RES_ALLOC_REPEAT_SUBSLOTS_GROUP)) {
				
				/* Activate the main UL schedule from the group template */
				memcpy(ul_sched, &ctx->role_ctx.pt.group_schedule, sizeof(dect_mac_schedule_t));
				ul_sched->is_active = true;

				uint64_t repetition_ticks = 0;
				uint32_t frame_ticks = (uint32_t)FRAME_DURATION_MS_NOMINAL * (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
				if (ul_sched->repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP) {
					repetition_ticks = (uint64_t)ul_sched->repetition_value * frame_ticks;
				} else {
					repetition_ticks = (uint64_t)ul_sched->repetition_value *
						get_subslot_duration_ticks_for_mu(ft->peer_mu);
				}
				/* Our specific slot is the i-th repetition */
				ul_sched->next_occurrence_modem_time =
					ul_sched->schedule_init_modem_time + (i * repetition_ticks);
				
				update_next_occurrence(ctx, ul_sched, ctx->last_known_modem_time, ft->peer_mu);

				LOG_INF("GROUP_ASSIGN: Found our slot! Tag %u is repetition %d. Activating UL schedule. New UL time: %llu",
					tag, i, ul_sched->next_occurrence_modem_time);
			}
			break;
		}
	}
}


// Overview: Restores the full security and buffer management functionality to the Keep-Alive action, adapting it to the refactored codebase. 
// This fixes a critical memory leak and re-enables essential HPC synchronization logic.
static void pt_send_keep_alive_action(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->state != MAC_STATE_ASSOCIATED || !ctx->role_ctx.pt.associated_ft.is_valid) {
		LOG_WRN("PT_SM_KA: Cannot send Keep Alive, not in associated state or no valid FT.");
		k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
		return;
	}

	LOG_INF("PT_SM_KA: Sending Keep Alive to associated FT 0x%04X.",
		ctx->role_ctx.pt.associated_ft.short_rd_id);

	uint8_t sdu_area_buf[20];
	size_t current_sdu_area_len = 0;
	int ie_len_written_val;

	bool secure_this_pdu = ctx->role_ctx.pt.associated_ft.is_secure && ctx->keys_provisioned;
	bool include_mac_sec_info_ie_for_ka = false;
	
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	size_t len_of_muxed_sec_ie_for_crypto_calc = 0;
	uint8_t sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;

	if (secure_this_pdu) {
		dect_mac_peer_info_t *assoc_ft_ctx = &ctx->role_ctx.pt.associated_ft;

		if (assoc_ft_ctx->self_needs_to_request_hpc_from_peer) {
			include_mac_sec_info_ie_for_ka = true;
			sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE;
		} else if (assoc_ft_ctx->peer_requested_hpc_resync ||
			   ctx->send_mac_sec_info_ie_on_next_tx) {
			include_mac_sec_info_ie_for_ka = true;
			sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
		}
	}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

	uint8_t hdr_type_octet_byte = 0;

	hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F);
	if (secure_this_pdu) {
		hdr_type_octet_byte |= ((include_mac_sec_info_ie_for_ka ? MAC_SECURITY_USED_WITH_IE
								: MAC_SECURITY_USED_NO_IE) &
					0x03)
				       << 4;
	} else {
		hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;
	}

	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	uint16_t current_psn_for_tx = ctx->psn;

	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((current_psn_for_tx >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = current_psn_for_tx & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be =
		sys_cpu_to_be32(ctx->role_ctx.pt.associated_ft.long_rd_id);

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	if (include_mac_sec_info_ie_for_ka) {
		ie_len_written_val = build_mac_security_info_ie_muxed(
			sdu_area_buf + current_sdu_area_len,
			sizeof(sdu_area_buf) - current_sdu_area_len, 0, ctx->current_key_index,
			sec_iv_type_for_ka_ie, ctx->hpc);
		if (ie_len_written_val < 0) {
			LOG_ERR("PT_SM_KA: Build MAC Sec Info IE failed: %d", ie_len_written_val);
			return;
		}
		current_sdu_area_len += ie_len_written_val;
		len_of_muxed_sec_ie_for_crypto_calc = ie_len_written_val;
	}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

	ie_len_written_val = build_keep_alive_ie_muxed(
		sdu_area_buf + current_sdu_area_len, sizeof(sdu_area_buf) - current_sdu_area_len);
	if (ie_len_written_val < 0) {
		LOG_ERR("PT_SM_KA: Build Keep Alive IE failed: %d", ie_len_written_val);
		return;
	}
	current_sdu_area_len += ie_len_written_val;

	mac_sdu_t *pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("PT_SM_KA: Failed to alloc PDU buf for Keep Alive.");
		return;
	}

	uint16_t assembled_pdu_len_pre_mic;
	int ret = dect_mac_phy_ctrl_assemble_final_pdu(
		pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE, hdr_type_octet_byte, &common_hdr,
		sizeof(common_hdr), sdu_area_buf, current_sdu_area_len,
		&assembled_pdu_len_pre_mic);

	if (ret != 0) {
		LOG_ERR("PT_SM_KA: Assemble PDU failed: %d", ret);
		dect_mac_buffer_free(pdu_sdu);
		return;
	}

	uint16_t final_tx_pdu_len = assembled_pdu_len_pre_mic;

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	uint32_t current_hpc_for_iv = ctx->hpc;

	if (secure_this_pdu) {
		uint8_t iv[16];
		security_build_iv(iv, ctx->own_long_rd_id,
				  ctx->role_ctx.pt.associated_ft.long_rd_id, current_hpc_for_iv,
				  current_psn_for_tx);

		uint8_t *mic_calculation_start_ptr = pdu_sdu->data + sizeof(uint8_t);
		size_t mic_calculation_length = sizeof(common_hdr) + current_sdu_area_len;

		if ((assembled_pdu_len_pre_mic + 5) > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
			LOG_ERR("PT_SM_KA: No space for MIC in PDU.");
			dect_mac_buffer_free(pdu_sdu);
			return;
		}
		uint8_t *mic_location_ptr = pdu_sdu->data + assembled_pdu_len_pre_mic;
		ret = security_calculate_mic(mic_calculation_start_ptr, mic_calculation_length,
					   ctx->integrity_key, mic_location_ptr);
		if (ret != 0) {
			LOG_ERR("PT_SM_KA: MIC calculation failed: %d", ret);
			dect_mac_buffer_free(pdu_sdu);
			return;
		}
		final_tx_pdu_len = assembled_pdu_len_pre_mic + 5;

		uint8_t *encryption_start_ptr;
		size_t encryption_length;

		if (include_mac_sec_info_ie_for_ka) {
			encryption_start_ptr = pdu_sdu->data + sizeof(uint8_t) +
					   sizeof(common_hdr) + len_of_muxed_sec_ie_for_crypto_calc;
			encryption_length = (current_sdu_area_len -
					     len_of_muxed_sec_ie_for_crypto_calc) +
					    5;
		} else {
			encryption_start_ptr = pdu_sdu->data + sizeof(uint8_t) + sizeof(common_hdr);
			encryption_length = current_sdu_area_len + 5;
		}

		if (encryption_length > 0) {
			ret = security_crypt_payload(encryption_start_ptr, encryption_length,
						   ctx->cipher_key, iv, true);
			if (ret != 0) {
				LOG_ERR("PT_SM_KA: Encryption failed: %d", ret);
				dect_mac_buffer_free(pdu_sdu);
				return;
			}
		}
	}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

	pdu_sdu->len = final_tx_pdu_len;

	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(uint32_t));
	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.associated_ft.operating_carrier, pdu_sdu->data, pdu_sdu->len,
		ctx->role_ctx.pt.associated_ft.short_rd_id, false, phy_op_handle,
		PENDING_OP_PT_KEEP_ALIVE, true, 0, ctx->own_phy_params.mu, NULL);

	if (ret != 0) {
		LOG_ERR("PT_SM_KA: Failed to schedule Keep Alive TX: %d", ret);
		dect_mac_buffer_free(pdu_sdu);
	} else {
		LOG_INF("PT_SM_KA: Keep Alive TX scheduled (Hdl %u).", phy_op_handle);
		/* The PHY control layer now owns the buffer, so we do not free it here.
		 * It will be freed by the HARQ process upon ACK or final failure.
		 * This fixes the memory leak.
		 */
	}
}


/***************************************  UNUSED WARNING ***************************************/
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
// static void pt_start_authentication_with_ft_action(dect_mac_context_t *ctx) {
//     if (!ctx) {
//         LOG_ERR("PT_AUTH_START: NULL context provided.");
//         return;
//     }

//     if (!ctx->role_ctx.pt.associated_ft.is_valid) {
//         LOG_ERR("PT_AUTH_START: No valid associated FT to authenticate with. Aborting auth.");
//         // This state should not normally be reached if called correctly after processing AssocResp(ACK)
//         dect_mac_change_state(MAC_STATE_IDLE); // Go back to idle to rescan
//         dect_mac_sm_pt_start_operation();      // Trigger rescan
//         return;
//     }

//     LOG_INF("PT SM: Starting Authentication (PSK-based key derivation) with FT 0x%04X (L:0x%08X).",
//             ctx->role_ctx.pt.associated_ft.short_rd_id,
//             ctx->role_ctx.pt.associated_ft.long_rd_id);

//     dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);

//     // For PSK-based "authentication", the main action is local key derivation.
//     // No PDUs are exchanged for this simplified model.
//     // A real authentication would involve sending an Auth Request, receiving Challenge, sending Response etc.

//     if (ctx->master_psk_provisioned) {
// #if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
//         LOG_DBG("PT_AUTH_START: Master PSK is provisioned. Deriving session keys.");
//         int kdf_err = security_derive_session_keys(
//             ctx->master_psk,
//             ctx->own_long_rd_id,                                // PT is local
//             ctx->role_ctx.pt.associated_ft.long_rd_id,          // FT is peer
//             ctx->integrity_key,
//             ctx->cipher_key);
// #endif
//         if (kdf_err == 0) {
//             LOG_INF("PT_AUTH_START: Session keys successfully derived from PSK.");
//             // Call the completion action with success
//             pt_authentication_complete_action(ctx, true);
//         } else {
//             LOG_ERR("PT_AUTH_START: Failed to derive session keys (err %d). Authentication failed.", kdf_err);
//             // Call the completion action with failure
//             pt_authentication_complete_action(ctx, false);
//         }
//     } else {
//         LOG_WRN("PT_AUTH_START: Master PSK not provisioned. Cannot perform PSK-based authentication. Authentication 'fails' (link remains unsecure).");
//         // Call the completion action with failure for security establishment,
//         // but the MAC link might still be considered associated (unsecure).
//         pt_authentication_complete_action(ctx, false);
//     }
// }
#endif

/**
 * @brief pt_authentication_complete_action
 * 
 * @param ctx 
 * @param success 
 */
static void pt_authentication_complete_action(dect_mac_context_t* ctx, bool success) {
	printk("[AUTH_COMPLETE_DBG] pt_authentication_complete_action called with success: %d\n", success);

    if (!ctx) {
        LOG_ERR("PT_AUTH_COMPLETE: NULL context provided.");
        return;
    }

    if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
        LOG_WRN("PT_AUTH_COMPLETE: Called in unexpected state %s. Current FT ShortID: 0x%04X",
                dect_mac_state_to_str(ctx->state), ctx->role_ctx.pt.associated_ft.short_rd_id);
        // If not in authenticating, perhaps an old/stale completion.
        // If already associated, do nothing more.
        // If in another state, it might be an error. For now, just log.
        if (ctx->state == MAC_STATE_ASSOCIATED && ctx->role_ctx.pt.associated_ft.is_valid) {
            return; // Already successfully associated.
        }
        // Otherwise, if some error led here, might need to reset.
    }

    if (success && ctx->role_ctx.pt.associated_ft.is_valid) {
        // This 'success' specifically means keys were derived and security context is ready.
        ctx->keys_provisioned = true; // PT's global session keys are now set for this FT
        ctx->role_ctx.pt.associated_ft.is_secure = true;

        // Reset/Initialize HPCs for this new secure session:
        // PT's own transmit HPC for communication with this FT.
        ctx->hpc = 1; // Start own HPC at 1 (or a random value, but 1 is fine for new session)
        // PT's tracking of the FT's transmit HPC. Assume FT also starts/resets its HPC for PT.
        ctx->role_ctx.pt.associated_ft.hpc = 1; // Initial assumption of FT's TX HPC
        // PT should send its HPC in the first secured PDU to the FT.
        ctx->send_mac_sec_info_ie_on_next_tx = true;
		LOG_INF("PT_AUTH_COMPLETE: Authentication successful. Link is SECURE.");

        dect_mac_change_state(MAC_STATE_ASSOCIATED);
        LOG_INF("PT_AUTH_COMPLETE: Authentication successful with FT 0x%04X (L:0x%08X). Link is SECURE.",
                ctx->role_ctx.pt.associated_ft.short_rd_id, ctx->role_ctx.pt.associated_ft.long_rd_id);
        LOG_INF("PT_AUTH_COMPLETE: OwnTX_HPC=%u, Tracking FT_TX_HPC=%u. Will send SecIE.",
                ctx->hpc, ctx->role_ctx.pt.associated_ft.hpc);

        // Start periodic timers for an active link
        k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
                      K_MSEC(ctx->config.keep_alive_period_ms),  // Initial delay
                      K_MSEC(ctx->config.keep_alive_period_ms)); // Period

        if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) { // Enable via Kconfig
			printk("[AUTH_COMPLETE_DBG] Starting mobility scan timer with period %u ms.\n", ctx->config.mobility_scan_interval_ms);

            k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
                          K_MSEC(ctx->config.mobility_scan_interval_ms),
                          K_MSEC(ctx->config.mobility_scan_interval_ms));
        }
        // Reset association attempt counter for any future (re-)associations
        ctx->role_ctx.pt.current_assoc_retries = 0;

        // After successful association, notify the CDD service about the new link.
        // The CDD service will then decide if a configuration request is needed.
        // We assume the initial App Sequence Number from the beacon was 0 or unknown.
        LOG_INF("PT_AUTH_COMPLETE: Notifying CDD service of new association.");
        dect_cdd_pt_process_beacon_info(ctx->role_ctx.pt.associated_ft.long_rd_id, 0);

    } else { // Authentication failed or was skipped (e.g. no PSK)
        if (ctx->role_ctx.pt.associated_ft.is_valid) { // Still associated, but unsecure
            ctx->keys_provisioned = false;
            ctx->role_ctx.pt.associated_ft.is_secure = false;
            dect_mac_change_state(MAC_STATE_ASSOCIATED); // Proceed to associated state, but unsecure
            // LOG_WRN("PT_AUTH_COMPLETE: Authentication failed or skipped for FT 0x%04X. Link is UNSECURE.",
            //         ctx->role_ctx.pt.associated_ft.short_rd_id);
			if (IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)) {
				LOG_WRN("PT_AUTH_COMPLETE: Authentication FAILED for FT 0x%04X. Link is UNSECURE.",
					ctx->role_ctx.pt.associated_ft.short_rd_id);
			} else {
				LOG_INF("PT_AUTH_COMPLETE: Authentication for FT 0x%04X Security is disabled. Link is UNSECURE.",
					ctx->role_ctx.pt.associated_ft.short_rd_id);
			}
			LOG_INF("    - BEFORE ctx->config.keep_alive_period_ms: %u", ctx->config.keep_alive_period_ms);
            k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
                          K_MSEC(ctx->config.keep_alive_period_ms),
                          K_MSEC(ctx->config.keep_alive_period_ms));
            if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
				printk("[AUTH_COMPLETE_DBG] Starting mobility scan timer with period %u ms.\n", ctx->config.mobility_scan_interval_ms);
				LOG_INF("    - BEFORE ctx->config.mobility_scan_interval_ms: %u", ctx->config.mobility_scan_interval_ms);
                k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
                              K_MSEC(ctx->config.mobility_scan_interval_ms),
                              K_MSEC(ctx->config.mobility_scan_interval_ms));
            }
			/* Start a continuous RX to listen for DL data and mobility candidates */
			LOG_INF("PT_SM: Association complete. Starting continuous RX.");
			uint32_t phy_rx_op_handle;

			dect_mac_rand_get((uint8_t *)&phy_rx_op_handle, sizeof(phy_rx_op_handle));
			int ret = dect_mac_phy_ctrl_start_rx(
				ctx->role_ctx.pt.associated_ft.operating_carrier, 0, /* Continuous */
				NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, phy_rx_op_handle,
				ctx->own_short_rd_id, PENDING_OP_PT_DATA_RX); /* Generic data RX op */
			if (ret != 0) {
				dect_mac_enter_error_state("Failed to start continuous PT RX");
			}
			/* Do NOT start a continuous RX. The PT is now in a connected-idle state,
			* waiting for its scheduled UL/DL slots or for timers to fire.
			* The data path service will handle scheduling TX/RX ops in those slots.
			*/
			LOG_INF("PT_SM: Association complete. Entering connected-idle state.");
			printk("============================================PT============================================\n\n\n\n\n\n\n");


        } else { // No valid associated FT (e.g. if assoc was rejected prior to auth attempt)
            LOG_ERR("PT_AUTH_COMPLETE: Authentication failed and no valid associated FT. Restarting scan.");
            dect_mac_sm_pt_start_operation(); // Go back to scanning
        }
    }
}


void dect_mac_sm_pt_initiate_authentication_protocol(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (!ctx->role_ctx.pt.associated_ft.is_valid) {
		LOG_ERR("PT_AUTH_INIT: No valid associated FT to authenticate with. Aborting.");
		dect_mac_change_state(MAC_STATE_IDLE);
		dect_mac_sm_pt_start_operation();
		return;
	}

	if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
		LOG_WRN("PT_AUTH_INIT: Called in state %s, expected AUTHENTICATING. Transitioning.", dect_mac_state_to_str(ctx->state));
		dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);
	}

	LOG_INF("PT_AUTH_INIT: Initiating authentication handshake with FT 0x%04X.",
		ctx->role_ctx.pt.associated_ft.short_rd_id);

	pt_send_auth_initiate_action();
}

void dect_mac_sm_pt_handle_auth_pdu(const uint8_t *pdu_data, size_t pdu_len)
{
    ARG_UNUSED(pdu_data);
    ARG_UNUSED(pdu_len);
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
        LOG_WRN("PT_AUTH_HANDLE_PDU: Received Auth PDU in unexpected state %s. Ignoring.", dect_mac_state_to_str(ctx->state));
        return;
    }

    LOG_INF("PT_AUTH_HANDLE_PDU: Received (stubbed) Auth PDU from FT 0x%04X (len %zu). No action for current PSK model.",
            ctx->role_ctx.pt.associated_ft.is_valid ? ctx->role_ctx.pt.associated_ft.short_rd_id : 0xFFFF,
            pdu_len);

    // For a real multi-step protocol, this would parse pdu_data.
    // If it's the final confirmation from FT:
    // pt_authentication_complete_action(ctx, true_if_auth_ok_else_false);
}




static void pt_update_mobility_candidate(uint16_t carrier, int16_t rssi, uint32_t long_id,
					 uint16_t short_id)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	int free_slot = -1;
	int existing_slot = -1;

	/* Check if this candidate (by Long ID) already exists */
	for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
		if (ctx->role_ctx.pt.mobility_candidates[i].is_valid) {
			if (ctx->role_ctx.pt.mobility_candidates[i].long_rd_id == long_id) {
				existing_slot = i;
				break;
			}
		} else if (free_slot == -1) {
			free_slot = i;
		}
	}

	int target_slot = -1;

	if (existing_slot != -1) {
		target_slot = existing_slot;
		LOG_DBG("MOBILITY: Updating existing candidate in slot %d.", target_slot);
	} else if (free_slot != -1) {
		target_slot = free_slot;
		LOG_INF("MOBILITY: Adding new candidate in slot %d.", target_slot);
	} else {
		/* No free slots. Find the weakest candidate to replace. */
		int16_t weakest_rssi = 0; /* RSSI is negative, so 0 is very strong */
		int weakest_slot = 0;

		for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
			if (ctx->role_ctx.pt.mobility_candidates[i].rssi_2 < weakest_rssi) {
				weakest_rssi = ctx->role_ctx.pt.mobility_candidates[i].rssi_2;
				weakest_slot = i;
			}
		}
		if (rssi > weakest_rssi) {
			target_slot = weakest_slot;
			LOG_INF("MOBILITY: Evicting weakest candidate (slot %d, RSSI %.1f) for new one (RSSI %.1f).",
				target_slot, weakest_rssi / 2.0, rssi / 2.0);
		} else {
			LOG_DBG("MOBILITY: New candidate (RSSI %.1f) not stronger than weakest in full list (RSSI %.1f). Ignoring.",
				rssi / 2.0, weakest_rssi / 2.0);
			return;
		}
	}

	/* Update the target slot with the new information */
	dect_mobility_candidate_t *cand = &ctx->role_ctx.pt.mobility_candidates[target_slot];

	cand->is_valid = true;
	cand->long_rd_id = long_id;
	cand->short_rd_id = short_id;
	cand->operating_carrier = carrier;
	cand->rssi_2 = rssi;
	
	/* Reset trigger count when a candidate is first added or updated */
	cand->trigger_count_remaining = ctx->role_ctx.pt.initial_count_to_trigger;
}


static void pt_process_page_indication(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	LOG_INF("PT_PAGING: Page received from FT! Responding and transitioning to Associated state.");

	/* Stop the paging cycle timer */
	k_timer_stop(&ctx->role_ctx.pt.paging_cycle_timer);

	/*
	 * Per ETSI 5.12.3, PT shall transmit a "Paging response".
	 * If config is ok, this is a Keep alive IE.
	 * If config needs modification, it's an Association Request.
	 * We implement the Keep alive IE response here.
	 */
	pt_send_keep_alive_action();

	/* Transition back to the normal connected state */
	dect_mac_change_state(MAC_STATE_ASSOCIATED);

	/* Restart normal link supervision */
	k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
		      K_MSEC(ctx->config.keep_alive_period_ms),
		      K_MSEC(ctx->config.keep_alive_period_ms));
	if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
		k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
			      K_MSEC(ctx->config.mobility_scan_interval_ms),
			      K_MSEC(ctx->config.mobility_scan_interval_ms));
	}
}

/***************************************  UNUSED WARNING ***************************************/
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
// static void pt_handle_phy_rssi_internal(const struct nrf_modem_dect_phy_rssi_event *event)
// {
//     if (event == NULL || event->meas_len == 0) {
//         return;
//     }

//     // For mobility, we are just interested in the average channel energy.
//     // If we find a quiet channel, we could schedule a brief RX on it to listen for beacons.
//     int32_t rssi_sum = 0;
//     int valid_count = 0;
//     for (uint16_t i = 0; i < event->meas_len; ++i) {
//         if (event->meas[i] != NRF_MODEM_DECT_PHY_RSSI_NOT_MEASURED) {
//             rssi_sum += event->meas[i];
//             valid_count++;
//         }
//     }

//     if (valid_count > 0) {
//         int16_t avg_rssi = rssi_sum / valid_count;
//         LOG_DBG("MOBILITY: Scan on carrier %u result: avg RSSI %.1f dBm.", event->carrier, avg_rssi / 2.0);

//         // TODO: Here you would decide if this channel is "interesting" enough to
//         // do a follow-up RX listen for a beacon. For now, this RSSI scan is just a placeholder.
//     }
// }
#endif


static void pt_evaluate_mobility_candidate(dect_mac_context_t *ctx,
					   const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
					   uint32_t ft_long_id, uint16_t ft_short_id,
					   int16_t rssi_q7_1, uint16_t beacon_rx_carrier)
{
	printk("[DEBUG_PROBE] pt_evaluate_mobility_candidate called.\n");
	printk("[MOBILITY_DBG] Evaluating candidate FT 0x%04X. PT state: %s\n",
	       ft_short_id, dect_mac_state_to_str(ctx->state));

	if (ft_long_id == ctx->role_ctx.pt.associated_ft.long_rd_id) {
		/* This is a beacon from our current FT, update its info */
		ctx->role_ctx.pt.associated_ft.rssi_2 = rssi_q7_1;
		/* Update the trigger count based on the latest beacon from our FT */
		ctx->role_ctx.pt.initial_count_to_trigger = cb_fields->count_to_trigger_code;
		return;
	}

	pt_update_mobility_candidate(beacon_rx_carrier, rssi_q7_1, ft_long_id, ft_short_id);

	/* Find the updated candidate to evaluate it */
	for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
		dect_mobility_candidate_t *cand = &ctx->role_ctx.pt.mobility_candidates[i];

		if (cand->is_valid && cand->long_rd_id == ft_long_id) {
			/* RelQuality code (0-7) maps to 0,3,6,9... dB. Multiply by 2 for Q7.1 format. */
			int16_t rssi_hysteresis = (int16_t)(cb_fields->rel_quality_code * 3 * 2);

			if (IS_ENABLED(CONFIG_ZTEST)){
				// FORCE THE RSSI TO BE HIGHER
				cand->rssi_2 = cand->rssi_2 + 100;
			}

			/* Convert Q7.1 RSSI values to dBm for logging */
			int16_t candidate_rssi_dbm = cand->rssi_2 / 2;  // Q7.1 to dBm
			int16_t current_rssi_dbm = ctx->role_ctx.pt.associated_ft.rssi_2 / 2;  // Q7.1 to dBm
			int16_t hysteresis_dbm = rssi_hysteresis / 2;  // Q7.1 to dBm

			/* Add your debug logging here */
			printk("[DEBUG_PROBE] Evaluating handover decision:\n");
			printk("  - Slot %d. RACH channel_abs_num: %u\n", i, cand->rach_params_from_beacon.channel_abs_num);
			printk("  - Candidate FT2 RSSI: %d dBm\n", candidate_rssi_dbm);
			printk("  - Current FT1 RSSI:   %d dBm\n", current_rssi_dbm);
			printk("  - Hysteresis:         %d dB\n", hysteresis_dbm);
			printk("  - Condition Check: (%d > (%d + %d)) -> %s\n",
				candidate_rssi_dbm,
				current_rssi_dbm,
				hysteresis_dbm,
				(cand->rssi_2 > (ctx->role_ctx.pt.associated_ft.rssi_2 + rssi_hysteresis)) ? "TRUE" : "FALSE");

			if (cand->rssi_2 > (ctx->role_ctx.pt.associated_ft.rssi_2 + rssi_hysteresis)) {
				printk("[DEBUG_PROBE] Handover decision triggered:\n");
				if (cand->trigger_count_remaining > 0) {
					cand->trigger_count_remaining--;
					LOG_INF("MOBILITY: Candidate FT 0x%04X is stronger. Trigger count now %u.",
						cand->short_rd_id, cand->trigger_count_remaining);
				}

				if (cand->trigger_count_remaining == 0) {
					LOG_INF("MOBILITY: Handover triggered for candidate FT 0x%04X!",
						cand->short_rd_id);
					pt_initiate_handover_action(ctx, cand);
				}
			} else {
				/* Not significantly better, reset its trigger count */
				if (cand->trigger_count_remaining !=
				    ctx->role_ctx.pt.initial_count_to_trigger) {
					LOG_DBG("MOBILITY: Candidate FT 0x%04X not strong enough. Resetting trigger count.",
						cand->short_rd_id);
					cand->trigger_count_remaining =
						ctx->role_ctx.pt.initial_count_to_trigger;
				}
			}
			break;
		}
	}
}


static void pt_initiate_handover_action(dect_mac_context_t *ctx,
					dect_mobility_candidate_t *candidate)
{
	if (ctx->state != MAC_STATE_ASSOCIATED) {
		LOG_WRN("MOBILITY: Handover initiated in unexpected state %s",
			dect_mac_state_to_str(ctx->state));
		return;
	}

    /* 1. Release the current connection */
    pt_send_association_release_action(&ctx->role_ctx.pt.associated_ft);

	/* Set the new target FT from the winning candidate */
	ctx->role_ctx.pt.target_ft.is_valid = true;
	ctx->role_ctx.pt.target_ft.is_fully_identified = true;
	ctx->role_ctx.pt.target_ft.long_rd_id = candidate->long_rd_id;
	ctx->role_ctx.pt.target_ft.short_rd_id = candidate->short_rd_id;
	ctx->role_ctx.pt.target_ft.operating_carrier = candidate->operating_carrier;
	ctx->role_ctx.pt.target_ft.rssi_2 = candidate->rssi_2;
	memcpy(&ctx->role_ctx.pt.current_ft_rach_params, &candidate->rach_params_from_beacon,
	       sizeof(dect_ft_rach_params_t));

printk("[MOBILITY_CANDIDATE_DBG] Stored new candidate in slot. Parsed RACH channel_abs_num: %u\n",
		       candidate->rach_params_from_beacon.channel_abs_num);

	/* Invalidate the candidate so we don't try to switch to it again immediately */
	candidate->is_valid = false;

	/* Stop timers related to the old FT before attempting to switch */
	k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
	k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);

	dect_mac_change_state(MAC_STATE_PT_HANDOVER_ASSOCIATING);
	LOG_INF("MOBILITY: State -> HANDOVER_ASSOCIATING. Data TX paused.");

	/* Start the association process with the new target */
	pt_send_association_request_action();
}



 void pt_send_association_release_action(dect_mac_peer_info_t *old_ft_info)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (!old_ft_info || !old_ft_info->is_valid) {
		return;
	}

	LOG_INF("PT_SM_RELEASE: Sending Association Release to FT 0x%08X via RACH.",
		old_ft_info->long_rd_id);

	uint8_t sdu_area_buf[16];
	dect_mac_assoc_release_ie_t release_fields = { .cause =
							       ASSOC_RELEASE_CAUSE_CONN_TERMINATION };

	int sdu_area_len =
		build_assoc_release_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf), &release_fields);
	if (sdu_area_len < 0) {
		LOG_ERR("PT_SM_RELEASE: Failed to build Assoc Release IE: %d", sdu_area_len);
		return;
	}

	uint8_t hdr_type_octet_byte = 0;

	hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F);
	hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;

	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(old_ft_info->long_rd_id);

	mac_sdu_t *pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("PT_SM_RELEASE: Failed to alloc PDU buffer.");
		return;
	}

	uint16_t pdu_len;
	int ret;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						   hdr_type_octet_byte, &common_hdr,
						   sizeof(common_hdr), sdu_area_buf,
						   (size_t)sdu_area_len, &pdu_len);

	if (ret != 0) {
		dect_mac_buffer_free(pdu_sdu);
		return;
	}

	/* Schedule the transmission on the RACH, bypassing the data path service */
	uint32_t phy_op_handle;
	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));

printk("[DEBUG_PROBE] Scheduling Release TX with op_type: %s (%d)\n",
       dect_pending_op_to_str(PENDING_OP_GENERIC_UNICAST_TX), PENDING_OP_GENERIC_UNICAST_TX);

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, pdu_sdu->data, pdu_len,
		old_ft_info->short_rd_id, false, phy_op_handle,
		PENDING_OP_GENERIC_UNICAST_TX, /* Use a simple, non-HARQ op type */
		true, 0, ctx->own_phy_params.mu, NULL);

	if (ret != 0) {
		LOG_ERR("PT_SM_RELEASE: Failed to schedule Release TX on RACH: %d", ret);
		dect_mac_buffer_free(pdu_sdu);
	} else {
		LOG_INF("PT_SM_RELEASE: Association Release TX scheduled on RACH (Hdl %u).",
			phy_op_handle);

		ctx->pending_op_type = PENDING_OP_GENERIC_UNICAST_TX;
		ctx->pending_op_handle = phy_op_handle;

		/* Start a listening window for the FT's confirmation */
		// uint32_t phy_rx_op_handle;
		// dect_mac_rand_get((uint8_t *)&phy_rx_op_handle, sizeof(phy_rx_op_handle));
		// dect_mac_phy_ctrl_start_rx(
		// 	ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel,
		// 	0, /* Continuous RX */
		// 	NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
		// 	phy_rx_op_handle,
		// 	ctx->own_short_rd_id,
		// 	PENDING_OP_PT_WAIT_ASSOC_RESP); /* Reuse this op type for the listen */
		// 	// PENDING_OP_GENERIC_UNICAST_TX); /* Reuse this op type for the listen */

		/* The HARQ process now owns the buffer */
	}
	// /* This is a fire-and-forget message, so we free the buffer immediately. */
	// dect_mac_buffer_free(pdu_sdu);
}

void dect_mac_sm_pt_beacon_listen_timer_expired_action(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->state != MAC_STATE_ASSOCIATED) {
		LOG_WRN("PT_BEACON_LSN: Timer action in unexpected state %s. Stopping timer.",
			dect_mac_state_to_str(ctx->state));
		k_timer_stop(&ctx->role_ctx.pt.beacon_listen_timer);
		return;
	}

	if (ctx->pending_op_type != PENDING_OP_NONE) {
		LOG_DBG("PT_BEACON_LSN: Beacon listen time, but op %s is pending. Deferring.",
			dect_pending_op_to_str(ctx->pending_op_type));
		return;
	}

	// /* Calculate the expected arrival time of the next beacon */
	// uint64_t next_beacon_time = calculate_target_modem_time(
	// 	ctx, ctx->ft_sfn_zero_modem_time_anchor, ctx->current_sfn_at_anchor_update,
	// 	(ctx->role_ctx.ft.sfn + 1) & 0xFF, /* Target next SFN */
	// 	0, /* Beacons are at subslot 0 */
	// 	ctx->role_ctx.pt.associated_ft.peer_mu, ctx->role_ctx.pt.associated_ft.peer_beta);

	/* Schedule a short RX window around the expected beacon time */
	uint32_t listen_duration_ticks = get_subslot_duration_ticks_for_mu(ctx->role_ctx.pt.associated_ft.peer_mu) * 4;
	uint32_t phy_op_handle;

	dect_mac_rand_get((uint8_t *)&phy_op_handle, sizeof(phy_op_handle));

	LOG_INF("PT_BEACON_LSN: Scheduling RX to listen for next beacon from FT 0x%04X",
		ctx->role_ctx.pt.associated_ft.short_rd_id);

	int ret = dect_mac_phy_ctrl_start_rx(
		ctx->role_ctx.pt.associated_ft.operating_carrier, listen_duration_ticks,
		NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT, phy_op_handle,
		0xFFFF, /* Beacons are broadcast */
		PENDING_OP_PT_BEACON_LISTEN);

	if (ret != 0) {
		LOG_ERR("PT_BEACON_LSN: Failed to schedule RX for beacon listen: %d", ret);
	}
}