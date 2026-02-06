/* dect_mac/dect_mac_data_path.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/random/random.h>

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>      // For get_mac_context(), increment_psn_and_hpc()
#include <mac/dect_mac_context.h>   // For dect_mac_context_t access and constants
#include <mac/dect_mac_phy_ctrl.h>  // For dect_mac_phy_ctrl_start_tx_assembled, _assemble_final_pdu, calculate_pcc_params
#include <mac/dect_mac_pdu.h>       // For IE_TYPE_USER_DATA_FLOW_1, MAC Common Headers, MAC Hdr Type, parse_mac_mux_header
#include <mac/dect_mac_main_dispatcher.h> // For string utils for logging, mac_event_msgq
#include <mac/dect_mac_security.h>  // For security_build_iv, _calculate_mic, _crypt_payload
#include <mac/dect_mac_sm_pt.h>     // For PT timer callback function extern declarations
#include <mac/dect_mac_sm_ft.h>     // For FT timer callback function extern declarations
#include <mac/dect_mac_phy_tbs_tables.h> // For TBS lookup tables
#include <mac/dect_mac_timeline_utils.h>

LOG_MODULE_REGISTER(dect_mac_data_path, CONFIG_DECT_MAC_DATA_PATH_LOG_LEVEL);


// Declare the pointer as external. The linker will resolve this to the
// variable defined in dect_mac_api.c.
extern sys_dlist_t *g_dlc_rx_sdu_dlist_ptr;

// Function pointer to the DLC's callback for reporting final TX status.
static dlc_tx_status_cb_t g_dlc_status_callback = NULL;



/**
 * @brief Registers the DLC's callback for TX status reports.
 *
 * The DLC layer calls this during its initialization to provide the MAC data
 * path with a function to call when a reportable SDU transmission is complete.
 *
 * @param cb The DLC's callback handler function.
 */
void dect_mac_data_path_register_dlc_callback(dlc_tx_status_cb_t cb)
{
	g_dlc_status_callback = cb;
	if (cb) {
		LOG_INF("DLC TX status callback registered with MAC Data Path.");
	} else {
		LOG_WRN("DLC TX status callback unregistered (set to NULL).");
	}
}


uint32_t get_tbs_for_schedule(uint8_t num_subslots, uint8_t mcs_code, uint8_t mu_code,
				     uint8_t beta_code)
{
	if (num_subslots == 0 || num_subslots > TBS_MAX_SUB_SLOTS_J) {
		LOG_WRN("TBS_GET: Invalid num_subslots %u", num_subslots);
		return 0;
	}
	if (mcs_code > TBS_MAX_MCS_INDEX) {
		LOG_WRN("TBS_GET: Invalid mcs_code %u", mcs_code);
		return 0;
	}

	const uint32_t (*selected_tbs_table)[TBS_MAX_SUB_SLOTS_J];

	if (beta_code != 0) {
		LOG_WRN("TBS_GET: TBS lookup for beta_code %u not yet implemented. Using beta=1 table.",
			beta_code);
	}
    
    printk("TBS_GET: mu_code:%d \n", mu_code);
	switch (mu_code) {
	case 0: /* mu = 1 */
		selected_tbs_table = tbs_single_slot_mu1_beta1;
		break;
	case 1: /* mu = 2 */
		selected_tbs_table = tbs_single_slot_mu2_beta1;
		break;
	case 2: /* mu = 4 */
		selected_tbs_table = tbs_single_slot_mu4_beta1;
		break;
	default:
		LOG_ERR("TBS_GET: Unsupported mu_code=%u. Using default mu=1 table.", mu_code);
		selected_tbs_table = tbs_single_slot_mu1_beta1;
		break;
	}

	uint32_t tbs_bits = selected_tbs_table[mcs_code][num_subslots - 0];

    printk("TBS_GET: [mcs_code]%d[num_subslots - 1]%d - Returning tbs_bits/8:%d \n", mcs_code, num_subslots - 1, tbs_bits/8);
	return tbs_bits / 8;
}

/**
 * @brief Checks if a given SDU will fit into a scheduled allocation.
 *
 * @param ctx Pointer to the MAC context.
 * @param sdu Pointer to the SDU to check.
 * @param schedule Pointer to the schedule for the allocation.
 * @param peer_slot_idx For FT role, the index of the target peer. -1 for PT role.
 * @return True if the SDU fits, false otherwise.
 */
bool does_sdu_fit_schedule(dect_mac_context_t *ctx, mac_sdu_t *sdu,
				  dect_mac_schedule_t *schedule, int peer_slot_idx)
{
    /* Estimate the total PDU size */
	size_t overhead = 0;
    printk("[FIT_CHECK_DBG] SDU len: %u\n", sdu->len);
	overhead += sizeof(dect_mac_header_type_octet_t);
    printk("[FIT_CHECK_DBG] After Hdr Type (size %zu): overhead = %zu\n", sizeof(dect_mac_header_type_octet_t), overhead);
    printk("[FIT_CHECK_DBG] Size of Unicast Hdr: %zu, Size of Data PDU Hdr: %zu\n",
	       sizeof(dect_mac_unicast_header_t), sizeof(dect_mac_data_pdu_header_t));
	// overhead += sizeof(dect_mac_unicast_header_t);
    overhead += sizeof(dect_mac_data_pdu_header_t);
    printk("[FIT_CHECK_DBG] After Unicast Hdr (size %zu): overhead = %zu\n", sizeof(dect_mac_unicast_header_t), overhead);
	overhead += 3; /* Max MUX header size for user data IE */
    printk("[FIT_CHECK_DBG] After MUX Hdr (size 3): overhead = %zu\n", overhead);

	bool is_secure = false;
	dect_mac_peer_info_t *peer_ctx = NULL;

	if (ctx->role == MAC_ROLE_PT) {
		peer_ctx = &ctx->role_ctx.pt.associated_ft;
		is_secure = peer_ctx->is_secure && ctx->keys_provisioned;
	} else if (peer_slot_idx != -1) {
		peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];
		is_secure = peer_ctx->is_secure &&
			    ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx];
	}

	if (is_secure) {
		overhead += 5; /* MIC */
        printk("[FIT_CHECK_DBG] After MIC (size 5): overhead = %zu\n", overhead);
        /* Simplified: Assume MAC Sec Info IE is not sent for this check. */
	}

	size_t total_pdu_size = sdu->len + overhead;
    
    // 2. Get available TBS from schedule
	uint8_t num_subslots = (schedule->alloc_type == RES_ALLOC_TYPE_UPLINK ||
				(ctx->role == MAC_ROLE_PT && schedule->alloc_type == RES_ALLOC_TYPE_BIDIR))
				       ? schedule->ul_duration_subslots
				       : schedule->dl_duration_subslots;

	uint8_t mcs_code = ctx->config.default_data_mcs_code;
	uint8_t mu_code = 2;
	uint8_t beta_code = 0;

	if (peer_ctx && peer_ctx->peer_phy_params_known) {
		mu_code = peer_ctx->peer_mu;
		beta_code = peer_ctx->peer_beta;
	} else {
		LOG_WRN("FIT_CHECK: Peer PHY params unknown. Using own params as fallback.");
		mu_code = ctx->own_phy_params.mu;
		beta_code = ctx->own_phy_params.beta;
	}

	uint32_t available_bytes = get_tbs_for_schedule(num_subslots, mcs_code, mu_code, beta_code);

    printk("[FIT_CHECK_DBG] Final Check: total_pdu_size (%zu) vs available_bytes (%u)\n", total_pdu_size, available_bytes);

	if (total_pdu_size > available_bytes) {
		LOG_WRN("FIT_CHECK: SDU (len %u, total ~%zu) does NOT fit in schedule (slots %u, mcs %u, tbs %u bytes)",
			sdu->len, total_pdu_size, num_subslots, mcs_code, available_bytes);
		return false;
	}

	LOG_DBG("FIT_CHECK: SDU (len %u, total ~%zu) fits in schedule (slots %u, mcs %u, tbs %u bytes)",
		sdu->len, total_pdu_size, num_subslots, mcs_code, available_bytes);
	return true;
}




// External FIFOs and slab (defined in dect_mac_api.c)
extern struct k_fifo * const mac_tx_fifos[]; // Generic TX FIFOs (used by PT for UL)
extern struct k_fifo *g_dlc_rx_sdu_fifo_ptr; // Pointer to DLC's RX FIFO
extern struct k_mem_slab g_mac_sdu_slab;     // For SDU buffers used by MAC API and internal PDU construction
extern struct k_msgq mac_event_msgq;         // For HARQ timer expiry events


/* --- Helper Functions --- */

uint32_t get_subslot_duration_ticks(dect_mac_context_t *ctx) {
    // ETSI TS 103 636-3, section 4: 1 subslot = 5 OFDM symbols.
    // NRF_MODEM_DECT_SYMBOL_DURATION is duration of 1 symbol in modem ticks (defined in nrf_modem_dect_phy.h)
    // This should be mu-independent as subslot is defined in terms of OFDM symbols,
    // and NRF_MODEM_DECT_SYMBOL_DURATION should be for the base numerology symbol.
    // If NRF_MODEM_DECT_SYMBOL_DURATION changes with mu, this needs ctx->phy_caps.mu.
    // For now, assuming NRF_MODEM_DECT_SYMBOL_DURATION is fixed for the base.
    ARG_UNUSED(ctx); // ctx might be needed if mu affects symbol duration reporting by PHY lib.
    return NRF_MODEM_DECT_SYMBOL_DURATION * 5;
}


// --- Initialization and HARQ Management Functions ---
void dect_mac_data_path_init(void) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (!ctx) {
        LOG_ERR("DATA_PATH_INIT: MAC Context is NULL. Cannot initialize HARQ.");
        return;
    }
    for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
        k_timer_init(&ctx->harq_tx_processes[i].retransmission_timer,
                     dect_mac_data_path_harq_timer_expired, NULL); // Expiry function
        // Store HARQ process index in timer's user_data for identification in callback
        // ctx->harq_tx_processes[i].retransmission_timer.user_data = (void*)((uintptr_t)i);
        ctx->harq_tx_processes[i].retransmission_timer.user_data = (void *)ctx;
        ctx->harq_tx_processes[i].is_active = false;
        ctx->harq_tx_processes[i].needs_retransmission = false;
        ctx->harq_tx_processes[i].sdu = NULL;
        ctx->harq_tx_processes[i].tx_attempts = 0;
        ctx->harq_tx_processes[i].redundancy_version = 0;
        ctx->harq_tx_processes[i].original_hpc = 0;
        ctx->harq_tx_processes[i].original_psn = 0;
        ctx->harq_tx_processes[i].peer_short_id_for_ft_dl = 0;
        ctx->harq_tx_processes[i].scheduled_carrier = 0;
        ctx->harq_tx_processes[i].scheduled_tx_start_time = 0;

    }
    LOG_INF("MAC Data Path Initialized (HARQ Timers and Processes set up).");
}

static int find_free_harq_tx_process(dect_mac_context_t* ctx) {
    if (!ctx) return -1;
    k_spinlock_key_t key = k_spin_lock(&ctx->harq_lock);
    for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
        if (!ctx->harq_tx_processes[i].is_active) {
            k_spin_unlock(&ctx->harq_lock, key);
            return i;
        }
    }
    k_spin_unlock(&ctx->harq_lock, key);
    LOG_DBG("HARQ_ALLOC: No free HARQ TX processes available.");
    return -1; // No free process
}


void dect_mac_data_path_harq_timer_expired(struct k_timer *timer_id)
{
	if (!timer_id) {
		return;
	}

	/* Retrieve the context stored in the timer's user_data */
	dect_mac_context_t *ctx = (dect_mac_context_t *)timer_id->user_data;
	if (!ctx) {
		LOG_ERR("HARQ_TIMER: Could not retrieve context from timer!");
		return;
	}

	/* Get a pointer to the parent HARQ process struct to find the index */
	dect_harq_tx_process_t *harq_p =
		CONTAINER_OF(timer_id, dect_harq_tx_process_t, retransmission_timer);

	int harq_idx = harq_p - &ctx->harq_tx_processes[0];

	if (harq_idx < 0 || harq_idx >= MAX_HARQ_PROCESSES) {
		LOG_ERR("HARQ_TIMER: Could not determine HARQ process index for timeout.");
		return;
	}

	LOG_WRN("HARQ_TIMER: Timeout for HARQ process %d.", harq_idx);

	struct dect_mac_event_msg msg = {.type = MAC_EVENT_TIMER_EXPIRED_HARQ,
					 .ctx = ctx, /* Use the retrieved context */
					 .data.timer_data.id = harq_idx};

	if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
		LOG_ERR("HARQ_TIMER: Failed to queue HARQ expiry for proc %d, msgq full.", harq_idx);
	}
}




void dect_mac_data_path_handle_harq_ack_action(int harq_process_idx) {
    printk("[HARQ_FREE_DBG] Entering dect_mac_data_path_handle_harq_ack_action....\n");

    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (!ctx || harq_process_idx < 0 || harq_process_idx >= MAX_HARQ_PROCESSES) {
        LOG_ERR("HARQ_ACK: Invalid context or process_idx %d", harq_process_idx);
        return;
    }
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_process_idx];

    if (harq_p->is_active) {
        LOG_INF("HARQ_ACK: ACK received for HARQ process %d (PSN: %u, Attempts: %u).",
                harq_process_idx, harq_p->original_psn, harq_p->tx_attempts);
        k_timer_stop(&harq_p->retransmission_timer);

        k_spinlock_key_t key = k_spin_lock(&ctx->harq_lock);
        if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
            LOG_DBG("HARQ_ACK: Reporting success to DLC for SN %u.", harq_p->sdu->dlc_sn_for_status);
            g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, true);
        }        
        if (harq_p->sdu) {
            printk("[HARQ_FREE_DBG] Freeing SDU buffer at %p for HARQ process %d.\n",
			       (void *)harq_p->sdu, harq_process_idx);

            dect_mac_buffer_free(harq_p->sdu); // Free the SDU buffer
            harq_p->sdu = NULL;
        }
        // Reset process for reuse
        harq_p->is_active = false;
        harq_p->needs_retransmission = false;
        harq_p->tx_attempts = 0;
        harq_p->redundancy_version = 0;
        harq_p->original_hpc = 0;
        harq_p->original_psn = 0;
        harq_p->peer_short_id_for_ft_dl = 0;
        harq_p->scheduled_carrier = 0;
        harq_p->scheduled_tx_start_time = 0;
        k_spin_unlock(&ctx->harq_lock, key);
    } else {
        LOG_WRN("HARQ_ACK: Received for already inactive HARQ process %d.", harq_process_idx);
    }
}

void dect_mac_data_path_handle_harq_nack_action(int harq_process_idx) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();
     if (!ctx || harq_process_idx < 0 || harq_process_idx >= MAX_HARQ_PROCESSES) {
        LOG_ERR("HARQ_NACK: Invalid context or process_idx %d", harq_process_idx);
        return;
    }
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_process_idx];

    if (harq_p->is_active) {
        k_timer_stop(&harq_p->retransmission_timer); // Stop current ACK timeout timer

        k_spinlock_key_t key = k_spin_lock(&ctx->harq_lock);
        /* For best-effort, any NACK/timeout is a permanent failure. No retransmissions. */
        if (harq_p->flow_id == MAC_FLOW_BEST_EFFORT) {
            LOG_INF("HARQ_NACK: Best-effort SDU (HARQ %d, PSN %u) failed. Discarding.",
                    harq_process_idx, harq_p->original_psn);
            if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
                g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, false);
            }
            if (harq_p->sdu) {
                dect_mac_buffer_free(harq_p->sdu);
                harq_p->sdu = NULL;
            }
            harq_p->is_active = false;
            k_spin_unlock(&ctx->harq_lock, key);
            return; /* Exit before retransmission logic */
        }        

        if (harq_p->tx_attempts >= MAX_HARQ_RETRIES) {
            LOG_ERR("HARQ_NACK: Max retries (%u) reached for HARQ process %d (PSN: %u). Discarding SDU.",
                    MAX_HARQ_RETRIES, harq_process_idx, harq_p->original_psn);
            if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
                LOG_DBG("HARQ_NACK: Reporting permanent failure to DLC for SN %u.", harq_p->sdu->dlc_sn_for_status);
                g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, false);
            }                    
            if (harq_p->sdu) {
                dect_mac_buffer_free(harq_p->sdu);
                harq_p->sdu = NULL;
            }
            // Reset process for reuse
            harq_p->is_active = false;
            harq_p->needs_retransmission = false;
            harq_p->tx_attempts = 0;
            harq_p->redundancy_version = 0;
        } else {
            // tx_attempts is incremented in send_data_mac_sdu_via_phy_internal before the actual reTX
            LOG_WRN("HARQ_NACK: NACK or Timeout for HARQ process %d (PSN: %u, Current Attempts: %u). Scheduling re-TX.",
                    harq_process_idx, harq_p->original_psn, harq_p->tx_attempts);
            harq_p->needs_retransmission = true;

            // ETSI TS 103 636-4, Section 5.5.1: RV sequence {0, 2, 3, 1, 0, ...}
            // Current harq_p->redundancy_version holds the RV of the *last failed attempt*.
            // We set the RV for the *next* attempt here.
            switch (harq_p->redundancy_version) {
                case 0: harq_p->redundancy_version = 2; break;
                case 2: harq_p->redundancy_version = 3; break;
                case 3: harq_p->redundancy_version = 1; break;
                case 1: harq_p->redundancy_version = 0; // Cycle back to 0, or could be to 2 for shorter cycle if preferred
                        // If cycling back to 0 after RV1, it implies SDU is effectively "new" again for combiner
                        // Or, some implementations might stop after one full cycle {0,2,3,1}.
                        // For now, simple cycle.
                        break;
                default: // Should not happen if initialized to 0
                    LOG_ERR("HARQ_NACK: Invalid current RV %u for proc %d. Resetting to RV0 for next attempt.",
                            harq_p->redundancy_version, harq_process_idx);
                    harq_p->redundancy_version = 0;
                    break;
            }
            LOG_DBG("HARQ_NACK: Next RV for HARQ %d will be %u (after %u attempts).",
                    harq_process_idx, harq_p->redundancy_version, harq_p->tx_attempts);
            // The dect_mac_data_path_service_tx function will see needs_retransmission=true and pick it up.
        }
        k_spin_unlock(&ctx->harq_lock, key);
    } else {
        LOG_WRN("HARQ_NACK: Received for already inactive HARQ process %d.", harq_process_idx);
    }
}

void dect_mac_data_path_process_harq_feedback(const union nrf_modem_dect_phy_feedback *feedback,
                                              uint16_t peer_short_rd_id) {
    if (!feedback) {
        LOG_ERR("HARQ_FB_PROC: NULL feedback pointer.");
        return;
    }

    // The format code is in the same position for format1, format2, format3, format5, format6
    uint8_t format_code = (feedback->format1.format & 0x0F);

    LOG_DBG("HARQ_FB_PROC: Rcvd from Peer 0x%04X, PHY Feedback Format Code: %u", peer_short_rd_id, format_code);

    switch (format_code) {
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1: // Single HARQ process feedback
        {
            int harq_idx = feedback->format1.harq_process_number0;
            if (harq_idx >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT1: Invalid HARQ idx %d", harq_idx); return; }
            if (feedback->format1.transmission_feedback0 == 1) { // ACK
                dect_mac_data_path_handle_harq_ack_action(harq_idx);
            } else { // NACK
                dect_mac_data_path_handle_harq_nack_action(harq_idx);
            }
            break;
        }
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_3: // Dual HARQ process feedback
        {
            int harq_idx0 = feedback->format3.harq_process_number0;
            if (harq_idx0 >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT3: Invalid HARQ idx0 %d", harq_idx0); /* continue to idx1? */ }
            else {
                if (feedback->format3.transmission_feedback0 == 1) {
                    dect_mac_data_path_handle_harq_ack_action(harq_idx0);
                } else {
                    dect_mac_data_path_handle_harq_nack_action(harq_idx0);
                }
            }

            int harq_idx1 = feedback->format3.harq_process_number1;
            if (harq_idx1 >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT3: Invalid HARQ idx1 %d", harq_idx1); return; }
            else {
                if (feedback->format3.transmission_feedback1 == 1) {
                    dect_mac_data_path_handle_harq_ack_action(harq_idx1);
                } else {
                    dect_mac_data_path_handle_harq_nack_action(harq_idx1);
                }
            }
            break;
        }
		case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_6: /* DF Redundancy Version reset requested */
		{
			int harq_idx = feedback->format6.harq_process_number;

			if (harq_idx >= MAX_HARQ_PROCESSES) {
				LOG_ERR("HARQ_FB_FMT6: Invalid HARQ idx %d", harq_idx);
				return;
			}

			LOG_INF("HARQ_FB_FMT6: Peer 0x%04X requested RV0 reset for HARQ process %d.",
				peer_short_rd_id, harq_idx);
			/* This is effectively a NACK for the current state of the process. */
			dect_mac_data_path_handle_harq_nack_action(harq_idx);
			break;
		}
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_NONE: // Explicitly no feedback
            LOG_DBG("HARQ_FB_PROC: Received 'No Feedback' (Format 0).");
            break;
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_2: // MIMO / Codebook
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_4: // HARQ Bitmap
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_5: // MIMO / Codebook extended
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_7: // CQI select
            LOG_WRN("HARQ_FB_PROC: Feedback format %u currently unhandled/not expected for basic HARQ ACK/NACK.", format_code);
            break;
        default: // Reserved or unknown formats
            LOG_ERR("HARQ_FB_PROC: Unknown feedback format code %u from 0x%04X", format_code, peer_short_rd_id);
            break;
    }
}

/* Helper function to get peer context */
static dect_mac_peer_info_t *get_peer_context(dect_mac_context_t* ctx, int ft_target_peer_slot_idx, uint32_t receiver_long_id)
{
    if (ctx->role == MAC_ROLE_PT) {
        if (ctx->role_ctx.pt.associated_ft.is_valid && 
            ctx->role_ctx.pt.associated_ft.long_rd_id == receiver_long_id) {
            return &ctx->role_ctx.pt.associated_ft;
        }
    } else if (ctx->role == MAC_ROLE_FT && ft_target_peer_slot_idx != -1) {
        if (ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_valid &&
            ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].long_rd_id == receiver_long_id) {
            return &ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx];
        }
    }
    return NULL;
}

/* Helper function to update security context after transmission */
static void update_security_context_after_tx(dect_mac_context_t* ctx, int ft_target_peer_slot_idx,
                                           uint32_t receiver_long_id, bool send_hpc_resync_initiate,
                                           bool send_own_hpc_as_provided)
{
    dect_mac_peer_info_t *peer_ctx = get_peer_context(ctx, ft_target_peer_slot_idx, receiver_long_id);
    
    if (!peer_ctx) {
        return;
    }

    if (send_hpc_resync_initiate) {
        LOG_DBG("DATA_TX_INT: Cleared self_needs_to_request_hpc_from_peer for 0x%04X.",
                peer_ctx->short_rd_id);
        peer_ctx->self_needs_to_request_hpc_from_peer = false;
    }
    
    if (send_own_hpc_as_provided && peer_ctx->peer_requested_hpc_resync) {
        LOG_DBG("DATA_TX_INT: Cleared peer_requested_hpc_resync for 0x%04X after sending PROVIDED.",
                peer_ctx->short_rd_id);
        peer_ctx->peer_requested_hpc_resync = false;
    }
    
    if (ctx->send_mac_sec_info_ie_on_next_tx) {
        LOG_DBG("DATA_TX_INT: Cleared global send_mac_sec_info_ie_on_next_tx flag.");
        ctx->send_mac_sec_info_ie_on_next_tx = false;
    }
}
static int send_data_mac_sdu_via_phy_internal(dect_mac_context_t* ctx,
                                     mac_sdu_t *mac_sdu_dlc_pdu, /* Contains DLC PDU */
                                     int harq_proc_idx, bool is_retransmission,
                                     uint16_t tx_carrier_from_schedule,
                                     int ft_target_peer_slot_idx,
                                     uint64_t phy_op_target_start_time,
                                     mac_flow_id_t flow_id,
                                     const union nrf_modem_dect_phy_feedback *feedback)
{
    // Constants for better maintainability
    #define INVALID_SHORT_ID 0xFFFF
    #define MIC_SIZE 5
    #define SDU_BUFFER_PADDING 10
    
    int ret = 0;
    mac_sdu_t *pdu_sdu = NULL;
    uint8_t *full_mac_pdu_for_phy = NULL;
    bool pdu_sdu_allocated = false;
    bool mac_sdu_dlc_pdu_owned = false;

    // Debug prints
    printk("[HEADER_TYPE_DBG] MAC_COMMON_HEADER_TYPE_UNICAST = 0x%02X\n", MAC_COMMON_HEADER_TYPE_UNICAST);
    printk("[HEADER_TYPE_DBG] MAC_COMMON_HEADER_TYPE_DATA_PDU = 0x%02X\n", MAC_COMMON_HEADER_TYPE_DATA_PDU);
    printk("[HEADER_TYPE_DBG] MAC_COMMON_HEADER_TYPE_BEACON = 0x%02X\n", MAC_COMMON_HEADER_TYPE_BEACON);

    /* Use the public API to allocate the buffer */
    pdu_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
    if (!pdu_sdu) {
        LOG_ERR("DATA_TX_INT: Failed to alloc full MAC PDU TX buffer");
        ret = -ENOMEM;
        goto cleanup;
    }
    pdu_sdu_allocated = true;
    full_mac_pdu_for_phy = pdu_sdu->data;

    /* Validate input parameters */
    if (harq_proc_idx < 0 || harq_proc_idx >= MAX_HARQ_PROCESSES) {
        LOG_ERR("DATA_TX_INT: Invalid HARQ process index %d", harq_proc_idx);
        ret = -EINVAL;
        goto cleanup;
    }

    if (!mac_sdu_dlc_pdu || mac_sdu_dlc_pdu->len == 0) {
        LOG_ERR("DATA_TX_INT: Invalid MAC SDU DLC PDU");
        ret = -EINVAL;
        goto cleanup;
    }

    bool security_active_for_this_pdu = false;
    bool include_mac_sec_info_ie = false;
    const uint8_t *session_integrity_key = NULL;
    const uint8_t *session_cipher_key = NULL;

    uint32_t receiver_long_id = 0;
    uint16_t receiver_short_id = INVALID_SHORT_ID;

    /* Determine receiver IDs and security context based on role */
    if (ctx->role == MAC_ROLE_PT) {
        if (ctx->role_ctx.pt.associated_ft.is_valid) {
            receiver_long_id = ctx->role_ctx.pt.associated_ft.long_rd_id;
            receiver_short_id = ctx->role_ctx.pt.associated_ft.short_rd_id;
            if (ctx->role_ctx.pt.associated_ft.is_secure && ctx->keys_provisioned) {
                security_active_for_this_pdu = true;
                session_integrity_key = ctx->integrity_key;
                session_cipher_key = ctx->cipher_key;
            }
        } else {
            LOG_ERR("DATA_TX_INT: PT not associated, cannot determine target for data TX.");
            ret = -ENOTCONN;
            goto cleanup;
        }
    } else if (ctx->role == MAC_ROLE_FT) {
        if (ft_target_peer_slot_idx < 0 || ft_target_peer_slot_idx >= MAX_PEERS_PER_FT ||
            !ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_valid) {
            LOG_ERR("DATA_TX_INT: FT role, but invalid or inactive peer_slot_idx %d for TX.", 
                    ft_target_peer_slot_idx);
            ret = -EINVAL;
            goto cleanup;
        }
        receiver_long_id = ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].long_rd_id;
        receiver_short_id = ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].short_rd_id;
        if (ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_secure &&
            ctx->role_ctx.ft.keys_provisioned_for_peer[ft_target_peer_slot_idx]) {
            security_active_for_this_pdu = true;
            session_integrity_key = ctx->role_ctx.ft.peer_integrity_keys[ft_target_peer_slot_idx];
            session_cipher_key = ctx->role_ctx.ft.peer_cipher_keys[ft_target_peer_slot_idx];
        }
    } else {
        LOG_ERR("DATA_TX_INT: Invalid MAC role: %d", ctx->role);
        ret = -EINVAL;
        goto cleanup;
    }

    if (receiver_long_id == 0 || receiver_short_id == INVALID_SHORT_ID) {
        LOG_ERR("DATA_TX_INT: Invalid receiver ID for data TX (L:0x%08X, S:0x%04X).", 
                receiver_long_id, receiver_short_id);
        ret = -EINVAL;
        goto cleanup;
    }

    /* Determine PSN and HPC for this PDU */
    uint16_t psn_for_this_pdu;
    uint32_t hpc_for_tx_iv_build;

    if (!is_retransmission) {
        increment_psn_and_hpc(ctx);
        psn_for_this_pdu = ctx->psn;
        hpc_for_tx_iv_build = ctx->hpc;
    } else {
        if (!ctx->harq_tx_processes[harq_proc_idx].is_active) {
            LOG_ERR("DATA_TX_INT: Inactive HARQ process %d for retransmission.", harq_proc_idx);
            ret = -EINVAL; 
            goto cleanup;
        }
        psn_for_this_pdu = ctx->harq_tx_processes[harq_proc_idx].original_psn;
        hpc_for_tx_iv_build = ctx->harq_tx_processes[harq_proc_idx].original_hpc;
    }

    /* Determine security information element requirements */
    uint8_t sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
    bool send_hpc_resync_initiate_to_peer = false;
    bool send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = false;

    if (security_active_for_this_pdu && !is_retransmission) {
        dect_mac_peer_info_t *peer_context_for_tx_flags_check = get_peer_context(ctx, ft_target_peer_slot_idx, receiver_long_id);
        
        if (peer_context_for_tx_flags_check) {
            if (peer_context_for_tx_flags_check->self_needs_to_request_hpc_from_peer) {
                send_hpc_resync_initiate_to_peer = true;
            } else if (peer_context_for_tx_flags_check->peer_requested_hpc_resync) {
                send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = true;
            }
        }
        
        if (ctx->send_mac_sec_info_ie_on_next_tx && !send_hpc_resync_initiate_to_peer && 
            !send_own_hpc_as_provided_due_to_peer_req_or_self_wrap) {
            send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = true;
        }

        if (send_hpc_resync_initiate_to_peer) {
            include_mac_sec_info_ie = true;
            sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE;
        } else if (send_own_hpc_as_provided_due_to_peer_req_or_self_wrap) {
            include_mac_sec_info_ie = true;
            sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
        }
    }
    
    if (is_retransmission) {
        include_mac_sec_info_ie = false;
    }

    /* Build MAC header type octet */
    uint8_t mac_hdr_type_octet_byte = 0;
    // mac_hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F);
    mac_hdr_type_octet_byte |= (MAC_COMMON_HEADER_TYPE_DATA_PDU & 0x0F);

    if (security_active_for_this_pdu) {
        mac_hdr_type_octet_byte |= ((include_mac_sec_info_ie ? MAC_SECURITY_USED_WITH_IE : MAC_SECURITY_USED_NO_IE) & 0x03) << 4;
    } else {
        mac_hdr_type_octet_byte |= (MAC_SECURITY_NONE & 0x03) << 4;
    }

    printk("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    printk("[TX_DATA_DBG] Assembling data PDU. Using MAC Header Type: 0x%02X\n", mac_hdr_type_octet_byte);

    /* Build common header */
    dect_mac_data_pdu_header_t common_hdr;
    common_hdr.sequence_num_high = (psn_for_this_pdu >> 8) & 0x0F;
    common_hdr.reset_bit = !is_retransmission;
    common_hdr.reserved = 0;
    common_hdr.sequence_num_low = psn_for_this_pdu & 0xFF;

    /* Build SDU area with information elements */
    uint8_t sdu_area_buf[CONFIG_DECT_MAC_SDU_MAX_SIZE + SDU_BUFFER_PADDING];
    size_t current_sdu_area_len = 0;
    size_t len_of_muxed_sec_ie_for_crypto_calc = 0;

    /* Build security information element if needed */
    if (security_active_for_this_pdu && include_mac_sec_info_ie) {
        int ie_len = build_mac_security_info_ie_muxed(
            sdu_area_buf, sizeof(sdu_area_buf),
            0, ctx->current_key_index,
            sec_iv_type_for_current_tx_ie,
            ctx->hpc);
        if (ie_len < 0) { 
            ret = ie_len; 
            LOG_ERR("DATA_TX_INT: Build SecInfoIE failed: %d", ret); 
            goto cleanup; 
        }
        current_sdu_area_len += ie_len;
        len_of_muxed_sec_ie_for_crypto_calc = ie_len;
    }

    /* Build user data information element */
    int ie_len = build_user_data_ie_muxed(sdu_area_buf + current_sdu_area_len,
                                      sizeof(sdu_area_buf) - current_sdu_area_len,
                                      mac_sdu_dlc_pdu->data, mac_sdu_dlc_pdu->len,
                                      IE_TYPE_USER_DATA_FLOW_1);
    if (ie_len < 0) {
        ret = ie_len;
        LOG_ERR("DATA_TX_INT: Build User Data IE failed: %d", ret);
        goto cleanup;
    }
    current_sdu_area_len += ie_len;

    /* Assemble final PDU */
    uint16_t assembled_pdu_len_pre_mic;
    ret = dect_mac_phy_ctrl_assemble_final_pdu(
              full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE,
              mac_hdr_type_octet_byte,
              &common_hdr, sizeof(common_hdr),
              sdu_area_buf, current_sdu_area_len,
              &assembled_pdu_len_pre_mic);

    printk("[PDU_ASSEMBLY_DBG] Called assemble_final_pdu with mac_hdr_type: 0x%02X\n", mac_hdr_type_octet_byte);

    if (ret != 0) { 
        LOG_ERR("DATA_TX_INT: Assemble final PDU failed: %d", ret); 
        goto cleanup; 
    }

    uint16_t final_tx_pdu_len_for_phy_ctrl = assembled_pdu_len_pre_mic;

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
    /* Apply security (MIC calculation and encryption) if enabled */
    if (security_active_for_this_pdu) {
        uint8_t iv[16];
        security_build_iv(iv, ctx->own_long_rd_id, receiver_long_id, hpc_for_tx_iv_build, psn_for_this_pdu);

        uint8_t *mic_calculation_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t);
        size_t mic_calculation_length = sizeof(common_hdr) + current_sdu_area_len;

        /* Check if there's enough space for MIC */
        if ((assembled_pdu_len_pre_mic + MIC_SIZE) > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
            LOG_ERR("DATA_TX_INT: Not enough space in PDU buffer for MIC.");
            ret = -ENOMEM;
            goto cleanup;
        }
        
        uint8_t *mic_location_ptr = full_mac_pdu_for_phy + assembled_pdu_len_pre_mic;
        ret = security_calculate_mic(mic_calculation_start_ptr, mic_calculation_length,
                                   session_integrity_key, mic_location_ptr);
        if (ret != 0) {
            LOG_ERR("DATA_TX_INT: MIC calculation failed: %d", ret);
            goto cleanup;
        }
        final_tx_pdu_len_for_phy_ctrl = assembled_pdu_len_pre_mic + MIC_SIZE;

        /* Determine encryption range */
        uint8_t *encryption_start_ptr;
        size_t encryption_length;

        if (((mac_hdr_type_octet_byte >> 4) & 0x03) == MAC_SECURITY_USED_WITH_IE) {
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) +
                                   sizeof(common_hdr) + len_of_muxed_sec_ie_for_crypto_calc;
            encryption_length = (current_sdu_area_len - len_of_muxed_sec_ie_for_crypto_calc) + MIC_SIZE;
        } else {
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) + sizeof(common_hdr);
            encryption_length = current_sdu_area_len + MIC_SIZE;
        }

        /* Validate encryption range and perform encryption */
        if (encryption_length > 0) {
            uint8_t* pdu_buffer_end_with_mic = full_mac_pdu_for_phy + final_tx_pdu_len_for_phy_ctrl;
            if ((encryption_start_ptr + encryption_length) > pdu_buffer_end_with_mic) {
                LOG_ERR("DATA_TX_INT: Encryption range exceeds PDU buffer.");
                ret = -EINVAL; 
                goto cleanup;
            }
            
            ret = security_crypt_payload(encryption_start_ptr, encryption_length, 
                                       session_cipher_key, iv, true);
            if (ret != 0) {
                LOG_ERR("DATA_TX_INT: Encryption failed: %d", ret);
                goto cleanup;
            }
        }
        
        LOG_DBG("DATA_TX_INT: Secured PDU. Final len %u. Mode: %s, MUXSecIELen: %zu",
            final_tx_pdu_len_for_phy_ctrl,
            (((mac_hdr_type_octet_byte >> 4) & 0x03) == MAC_SECURITY_USED_WITH_IE) ? "WITH_SEC_IE" : "NO_SEC_IE",
            len_of_muxed_sec_ie_for_crypto_calc);
    }
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

    /* Generate PHY operation handle - just call sys_rand_get without checking return value */
    uint32_t phy_op_handle = 0;
    sys_rand_get(&phy_op_handle, sizeof(phy_op_handle));
    /* If sys_rand_get fails, phy_op_handle remains 0, which might be acceptable
       or we can use a fallback */
    if (phy_op_handle == 0) {
        // Use cycle counter as fallback
        phy_op_handle = (uint32_t)k_cycle_get_32();
    }

    /* Determine operation type and start PHY transmission */
    pending_op_type_t op_type_for_phy = (ctx->role == MAC_ROLE_PT) ? PENDING_OP_PT_DATA_TX_HARQ0 : PENDING_OP_FT_DATA_TX_HARQ0;
    op_type_for_phy = (pending_op_type_t)((int)op_type_for_phy + harq_proc_idx);

    ret = dect_mac_phy_ctrl_start_tx_assembled(
        tx_carrier_from_schedule,
        full_mac_pdu_for_phy,
        final_tx_pdu_len_for_phy_ctrl,
        receiver_short_id,
        false,
        phy_op_handle,
        op_type_for_phy,
        true,
        phy_op_target_start_time,
        ctx->own_phy_params.mu,
        feedback);

    if (ret != 0) {
        LOG_ERR("DATA_TX_INT: PHY TX schedule failed for HARQ %d (err %d).", harq_proc_idx, ret);
        goto cleanup;
    }

    /* Update HARQ process on success */
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_proc_idx];
    if (!is_retransmission) {
        harq_p->sdu = mac_sdu_dlc_pdu;
        harq_p->is_active = true;
        harq_p->flow_id = flow_id;
        harq_p->original_psn = psn_for_this_pdu;
        harq_p->original_hpc = hpc_for_tx_iv_build;
        harq_p->tx_attempts = 1;
        harq_p->redundancy_version = 0;
        harq_p->scheduled_carrier = tx_carrier_from_schedule;
        harq_p->scheduled_tx_start_time = phy_op_target_start_time;
        harq_p->peer_short_id_for_ft_dl = receiver_short_id;
        mac_sdu_dlc_pdu_owned = true; // HARQ now owns the SDU
    } else {
        harq_p->tx_attempts++;
    }
    harq_p->needs_retransmission = false;
    k_timer_start(&harq_p->retransmission_timer, K_MSEC(HARQ_ACK_TIMEOUT_MS), K_NO_WAIT);

    /* Update security context if security information element was included */
    if (include_mac_sec_info_ie && security_active_for_this_pdu && !is_retransmission) {
        update_security_context_after_tx(ctx, ft_target_peer_slot_idx, receiver_long_id,
                                       send_hpc_resync_initiate_to_peer, 
                                       send_own_hpc_as_provided_due_to_peer_req_or_self_wrap);
    }

    /* On success, pdu_sdu is now owned by PHY layer - don't free it */
    pdu_sdu_allocated = false;
    ret = 0;

cleanup:
    /* Cleanup resources based on ownership flags */
    if (pdu_sdu_allocated && pdu_sdu) {
        dect_mac_buffer_free(pdu_sdu);
    }
    
    if (ret != 0 && !mac_sdu_dlc_pdu_owned) {
        if (!is_retransmission) {
            LOG_ERR("DATA_TX_INT: Freeing SDU (len %u) due to failure for HARQ %d.",
                    mac_sdu_dlc_pdu->len, harq_proc_idx);
            dect_mac_buffer_free(mac_sdu_dlc_pdu);
        } else {
            ctx->harq_tx_processes[harq_proc_idx].needs_retransmission = true;
        }
    }

    #undef INVALID_SHORT_ID
    #undef MIC_SIZE
    #undef SDU_BUFFER_PADDING
    
    return ret;
}




void dect_mac_data_path_service_tx(void)
{
    printk("\n--- STARTED: dect_mac_data_path_service_tx ---\n");
	dect_mac_context_t *ctx = dect_mac_get_active_context();

if (ctx->state > 2 && ctx->state != 17 ){
	printk("\n--- DEBUG: dect_mac_data_path_service_tx ---\n");
	printk("  - Current State: %s(%d), Pending Op: %s(%d)\n", dect_mac_state_to_str(ctx->state), ctx->state,
	       dect_pending_op_to_str(ctx->pending_op_type), ctx->pending_op_type);
}

	bool can_service_tx = false;

	if (ctx->role == MAC_ROLE_PT) {
		/* A PT can send data when associated, or send its final release message */
		if (ctx->state == MAC_STATE_ASSOCIATED || ctx->state == MAC_STATE_PT_RELEASING) {
			can_service_tx = true;
		}
	} else { /* MAC_ROLE_FT */
		/* An FT can send data to its PTs when it is in its main operational state */
		if (ctx->state == MAC_STATE_FT_BEACONING) {
			can_service_tx = true;
		}
	}

	if (!can_service_tx) {
		return;
	}


/* --- 1. Prioritize HARQ Retransmissions --- */
printk("  - Checking for HARQ retransmissions...\n");
LOG_DBG("DP_SVC_TX: Starting HARQ processing for %d processes", MAX_HARQ_PROCESSES);

// Add locking if not already present
k_spinlock_key_t key = k_spin_lock(&ctx->harq_lock);

bool handled_retransmission = false;
int retransmission_attempts = 0;
int cleanup_count = 0;

for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
    
    LOG_DBG("DP_SVC_TX: Checking HARQ process %d: active=%d, needs_retx=%d, sdu=%p", 
            i, harq_p->is_active, harq_p->needs_retransmission, harq_p->sdu);

    // Check for retransmissions first
    if (harq_p->is_active && harq_p->needs_retransmission) {
        retransmission_attempts++;
        LOG_INF("DP_SVC_TX: Attempting re-TX for HARQ proc %d (attempt %d)", i, retransmission_attempts);
        
        int peer_idx = (ctx->role == MAC_ROLE_FT)
                       ? dect_mac_core_get_peer_slot_idx(harq_p->peer_short_id_for_ft_dl)
                       : 0;
        
        LOG_DBG("DP_SVC_TX: HARQ proc %d - role=%d, peer_idx=%d", i, ctx->role, peer_idx);
        
        if (peer_idx == -1 && ctx->role == MAC_ROLE_FT) {
            LOG_WRN("DP_SVC_TX: HARQ proc %d - Invalid peer index, skipping retransmission", i);
            continue;
        }

        uint64_t start_time;
        uint16_t carrier;
        dect_mac_schedule_t schedule;
        bool opportunity_found =
            (ctx->role == MAC_ROLE_FT)
                ? ft_get_next_tx_opportunity(peer_idx, &start_time, &carrier, &schedule)
                : pt_get_next_tx_opportunity(&start_time, &carrier, &schedule);

        LOG_DBG("DP_SVC_TX: HARQ proc %d - TX opportunity found=%d, carrier=%d, start_time=%llu", 
                i, opportunity_found, carrier, start_time);

        if (opportunity_found) {
            LOG_INF("DP_SVC_TX: Sending retransmission for HARQ proc %d on carrier %d", i, carrier);
            
            int result = send_data_mac_sdu_via_phy_internal(
                ctx, harq_p->sdu, i, true, carrier, peer_idx, start_time,
                harq_p->flow_id, NULL);
            
            LOG_DBG("DP_SVC_TX: send_data_mac_sdu_via_phy_internal returned %d for HARQ proc %d", result, i);
            
            handled_retransmission = true;
            break; // Use break instead of return to allow cleanup
        } else {
            LOG_WRN("DP_SVC_TX: No TX opportunity found for HARQ proc %d retransmission", i);
        }
    }
}

LOG_DBG("DP_SVC_TX: Retransmission phase complete - handled=%d, attempts=%d", 
        handled_retransmission, retransmission_attempts);

// Separate cleanup phase with detailed logging
for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
    
    /* Check for completed HARQ processes that need cleanup */
    if (harq_p->is_active && !harq_p->needs_retransmission && harq_p->sdu) {
        cleanup_count++;
        LOG_INF("DP_SVC_TX: Cleaning up completed HARQ process %d (cleanup %d/%d)", 
                i, cleanup_count, MAX_HARQ_PROCESSES);
        
        /* This process is done, free its buffer */
        LOG_DBG("DP_SVC_TX: Freeing SDU buffer %p for HARQ proc %d", harq_p->sdu, i);
        
        dect_mac_buffer_free(harq_p->sdu);
        harq_p->sdu = NULL;
        harq_p->is_active = false;
        
        LOG_DBG("DP_SVC_TX: HARQ proc %d cleanup complete - sdu=%p, active=%d", 
                i, harq_p->sdu, harq_p->is_active);
    } else if (harq_p->is_active) {
        // Log state of active but not-cleaned-up processes for debugging
        LOG_DBG("DP_SVC_TX: HARQ proc %d still active - needs_retx=%d, sdu=%p", 
                i, harq_p->needs_retransmission, harq_p->sdu);
    }
}

LOG_DBG("DP_SVC_TX: Cleanup phase complete - cleaned %d processes", cleanup_count);

k_spin_unlock(&ctx->harq_lock, key);

if (handled_retransmission) {
    LOG_DBG("DP_SVC_TX: Returning after handling retransmission and cleanup");
    return;
} else {
    LOG_DBG("DP_SVC_TX: No retransmissions handled, continuing with normal TX processing");
}

// Additional debug: Log summary of HARQ process states
// #ifdef CONFIG_DECT_MAC_DEBUG_HARQ
LOG_DBG("DP_SVC_TX: HARQ process state summary:");
for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
    if (harq_p->is_active) {
        LOG_DBG("  Proc %d: active=1, needs_retx=%d, sdu=%p, flow_id=%d", 
                i, harq_p->needs_retransmission, harq_p->sdu, harq_p->flow_id);
    }
}
// #endif



	/* --- 2. Service New SDUs based on Role --- */
    // printk("/* --- 2. Service New SDUs based on Role --- */ \n");
	if (ctx->role == MAC_ROLE_FT) {
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (!ctx->role_ctx.ft.connected_pts[i].is_valid) {
				continue;
			}
			dect_mac_peer_tx_dlist_set_t *dlists =
				&ctx->role_ctx.ft.peer_tx_data_dlists[i];
			sys_dlist_t *dlist_array[] = { &dlists->high_priority_dlist,
						       &dlists->reliable_data_dlist,
						       &dlists->best_effort_dlist };

			for (int j = 0; j < MAC_FLOW_COUNT; j++) {
				if (!sys_dlist_is_empty(dlist_array[j])) {
					uint64_t start_time;
					uint16_t carrier;
					dect_mac_schedule_t schedule;
					// if (ft_get_next_tx_opportunity(i, &start_time, &carrier, &schedule)) {
                    bool opportunity_found = ft_get_next_tx_opportunity(i, &start_time, &carrier, &schedule);

        printk("[DIAGNOSTIC_TRACE] Step 2: FT Scheduler ran. Opportunity found: %s\n",
		       opportunity_found ? "YES" : "NO");
		printk("  - pt_get_next_tx_opportunity returned: %s\n", opportunity_found ? "true" : "false");
        printk(" &start_time:%lluus &carrier:%d &schedule:%lluu \n", start_time, carrier, schedule.schedule_init_modem_time);

                    if (opportunity_found) {
						sys_dnode_t *node = sys_dlist_get(dlist_array[j]);
						mac_sdu_t *sdu = CONTAINER_OF(node, mac_sdu_t, node);
						// if (does_sdu_fit_schedule(ctx, sdu, &schedule, i)) {
                        bool fits = does_sdu_fit_schedule(ctx, sdu, &schedule, i);
                        printk("[SCHED_FIT_DBG] FT Checking if SDU (len %u) fits schedule. Result: %s\n",
                            sdu->len, fits ? "YES" : "NO");

					if (fits) {
							int ret = dect_mac_data_path_send_new_sdu(
								sdu, (mac_flow_id_t)j, i, carrier,
								start_time, NULL);
							if (ret == -EBUSY) {
								sys_dlist_prepend(dlist_array[j],
										  &sdu->node);
							}
							return;
						} else {
							sys_dlist_prepend(dlist_array[j],
									  &sdu->node);
						}
					}
				}
                else{
                    // printk("[SCHED_FIT_DBG] FT (!sys_dlist_is_empty(dlist_array[%d])) \n", j);
                }
			}
		}
	} else { /* PT Role */
		uint64_t start_time;
		uint16_t carrier;
		dect_mac_schedule_t schedule;
		printk("  - PT Role: Checking for TX opportunity...\n");
		bool opportunity_found = pt_get_next_tx_opportunity(&start_time, &carrier, &schedule);
        printk("[DIAGNOSTIC_TRACE] Step 2: Scheduler ran. Opportunity found: %s\n",
		       opportunity_found ? "YES" : "NO");
		// printk("  - pt_get_next_tx_opportunity returned: %s\n", opportunity_found ? "true" : "false");
        printk(" &start_time:%lluus &carrier:%d &schedule:%lluu \n", start_time, carrier, schedule.schedule_init_modem_time);


		if (opportunity_found) {
			for (int j = 0; j < MAC_FLOW_COUNT; j++) {
				if (!sys_dlist_is_empty(mac_tx_dlists[j])) {
                    printk("  - PT Role: sys_dlist_is NOT empty(mac_tx_dlists[%d])...\n",j);
					sys_dnode_t *node = sys_dlist_get(mac_tx_dlists[j]);
					mac_sdu_t *sdu = CONTAINER_OF(node, mac_sdu_t, node);
					// if (does_sdu_fit_schedule(ctx, sdu, &schedule, -1)) {
                    bool fits = does_sdu_fit_schedule(ctx, sdu, &schedule, -1);
					printk("[SCHED_FIT_DBG] PT Checking if SDU (len %u) fits schedule. Result: %s\n",
					       sdu->len, fits ? "YES" : "NO");

					if (fits) {
						int ret = dect_mac_data_path_send_new_sdu(
							sdu, (mac_flow_id_t)j, -1, carrier,
							start_time, NULL);
						if (ret == -EBUSY) {
							sys_dlist_prepend(mac_tx_dlists[j],
									  &sdu->node);
						}
						return;
					} else {
						LOG_WRN("FT_SCHED_TX: SDU (len %u) for PT %d does not fit schedule. Re-queueing.",
							sdu->len, -1);
						sys_dlist_prepend(mac_tx_dlists[j],
								  &sdu->node);
					}
				} else {
                    // printk("  - PT Role: sys_dlist_is_empty(mac_tx_dlists[%d])...\n",j);
                }
			}
		} 
	}
}


int dect_mac_data_path_send_new_sdu(mac_sdu_t *sdu, mac_flow_id_t flow_id, int peer_slot_idx,
                                    uint16_t carrier, uint64_t start_time,
                                    const union nrf_modem_dect_phy_feedback *feedback)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    int free_harq_idx = find_free_harq_tx_process(ctx);

    if (free_harq_idx == -1) {
        return -EBUSY; // No free HARQ process
    }

    return send_data_mac_sdu_via_phy_internal(ctx, sdu, free_harq_idx, false,
                                              carrier, peer_slot_idx, start_time,
                                              flow_id, feedback);
}


void dect_mac_data_path_handle_rx_sdu(const uint8_t *mac_sdu_area_data,
				      size_t mac_sdu_area_len,
				      uint32_t transmitter_long_rd_id)
{
	if (mac_sdu_area_data == NULL || mac_sdu_area_len == 0) {
        printk("RX_SDU_HANDLER: Received empty or NULL MAC SDU Area. \n");
		// LOG_DBG("RX_SDU_HANDLER: Received empty or NULL MAC SDU Area.");
		return;
	}

	printk("RX_SDU_HANDLER: Processing MAC SDU Area from 0x%08X, len %zu \n",
		transmitter_long_rd_id, mac_sdu_area_len);    
	// LOG_DBG("RX_SDU_HANDLER: Processing MAC SDU Area from 0x%08X, len %zu",
	// 	transmitter_long_rd_id, mac_sdu_area_len);

	const uint8_t *current_ie_ptr = mac_sdu_area_data;
	size_t remaining_sdu_area_len = mac_sdu_area_len;

    printk("RX_SDU_HANDLER: remaining_sdu_area_len:%zu > 0  \t", remaining_sdu_area_len);
	while (remaining_sdu_area_len > 0) {
        printk("%zu, ", remaining_sdu_area_len);

		uint8_t ie_type_from_mux;
		uint16_t dlc_pdu_len_from_mux;
		const uint8_t *dlc_pdu_ptr_from_mux;
		int parsed_mux_header_len;

		/* The parse_mac_mux_header function now correctly handles all length calculations internally. */
		parsed_mux_header_len =
			parse_mac_mux_header(current_ie_ptr, remaining_sdu_area_len,
					     &ie_type_from_mux, &dlc_pdu_len_from_mux,
					     &dlc_pdu_ptr_from_mux);

		if (parsed_mux_header_len < 0) {
			LOG_ERR("RX_SDU_HANDLER: Failed to parse MAC MUX header in SDU Area: %d. Dropping rest of SDU Area.",
				parsed_mux_header_len);
			break;
		}

        /* This check is still valid and important */
		if (remaining_sdu_area_len < (size_t)parsed_mux_header_len + dlc_pdu_len_from_mux) {
			printk("RX_SDU_HANDLER: MUX IE (type 0x%X) declared payload len %u exceeds actual remaining SDU area %zu. Corrupted PDU? \n",
				ie_type_from_mux, dlc_pdu_len_from_mux,
				remaining_sdu_area_len - parsed_mux_header_len);
			break;
		}

        printk("RX_SDU_HANDLER: IE type: 0x%X (%d), payload_len: %u\n", 
               ie_type_from_mux, ie_type_from_mux, dlc_pdu_len_from_mux);

        /* Signaling flows 1-2, User data flows 3-6 */
		if (ie_type_from_mux >= 1 && ie_type_from_mux <= 6) {        
            printk("RX_SDU_HANDLER: Higher layer flow detected (ID %d), checking DLC queue...\n", ie_type_from_mux);
            
            if (g_dlc_rx_sdu_dlist_ptr != NULL) {
                printk("RX_SDU_HANDLER: Allocating MAC SDU buffer...\n");
                mac_sdu_t *sdu_for_dlc = dect_mac_buffer_alloc(K_NO_WAIT);
                
                if (sdu_for_dlc) {
                    printk("RX_SDU_HANDLER: SDU buffer allocated successfully\n");
                    
                    if (dlc_pdu_len_from_mux <= CONFIG_DECT_MAC_SDU_MAX_SIZE) {
                        printk("RX_SDU_HANDLER: Copying %u bytes to SDU buffer...\n", dlc_pdu_len_from_mux);
                        memcpy(sdu_for_dlc->data, dlc_pdu_ptr_from_mux,
                               dlc_pdu_len_from_mux);
                        sdu_for_dlc->len = dlc_pdu_len_from_mux;
                        
                        printk("RX_SDU_HANDLER: Appending SDU to g_dlc_rx_sdu_dlist_ptr - IE type: 0x%X, length: %u, data ptr: %p, node ptr: %p\n",
                               ie_type_from_mux, sdu_for_dlc->len, sdu_for_dlc->data, &sdu_for_dlc->node);
                        
                        sys_dlist_append(g_dlc_rx_sdu_dlist_ptr, &sdu_for_dlc->node);
                        
                        printk("[DLIST_DBG] After append, g_dlc_rx_sdu_dlist_ptr is %s.\n",
				                sys_dlist_is_empty(g_dlc_rx_sdu_dlist_ptr) ? "EMPTY" : "NOT EMPTY");
                        printk("RX_SDU_HANDLER: Successfully added SDU to dlist. Dlist ptr: %p\n", 
                                g_dlc_rx_sdu_dlist_ptr);
                        
                        // Log first few bytes for verification
                        if (dlc_pdu_len_from_mux > 0) {
                            printk("RX_SDU_HANDLER: First 4 bytes of SDU data: %02X %02X %02X %02X\n",
                                   sdu_for_dlc->data[0], sdu_for_dlc->data[1],
                                   sdu_for_dlc->data[2], sdu_for_dlc->data[3]);
                        }
                    } else {
                        LOG_ERR("RX_SDU_HANDLER: Extracted DLC PDU too large (%u > %d). Dropped.",
                            dlc_pdu_len_from_mux,
                            CONFIG_DECT_MAC_SDU_MAX_SIZE);
                        printk("RX_SDU_HANDLER: Freeing SDU buffer due to size constraint\n");
                        dect_mac_buffer_free(sdu_for_dlc);
                    }
                } else {
                    LOG_ERR("RX_SDU_HANDLER: Failed to alloc SDU buffer for DLC RX. DLC PDU (len %u) dropped.",
                        dlc_pdu_len_from_mux);
                    printk("RX_SDU_HANDLER: Buffer allocation failed for IE type 0x%X\n", ie_type_from_mux);
                }
            } else {
                LOG_ERR("RX_SDU_HANDLER: DLC RX queue is NULL! DLC PDU dropped.");
                printk("RX_SDU_HANDLER: g_dlc_rx_sdu_dlist_ptr is NULL, cannot append SDU\n");
            }
		} else {
			LOG_DBG("RX_SDU_HANDLER: Skipping non-UserData MUX IE type 0x%X.",
				ie_type_from_mux);
            printk("RX_SDU_HANDLER: Non-user data IE type 0x%X skipped\n", ie_type_from_mux);
		}

        // Declare and calculate the total number of bytes consumed by the current IE.
        size_t consumed = (size_t)parsed_mux_header_len + dlc_pdu_len_from_mux;

        // Check for a zero-length IE to prevent an infinite loop.
        // This can happen with padding IEs or if the logic above results in zero.
        if (consumed == 0) {
            LOG_WRN("RX_SDU_HANDLER: Consumed 0 bytes for IE type 0x%X. Breaking loop to prevent stall.", ie_type_from_mux);
            break;
        }

        printk("RX_SDU_HANDLER: Consumed %zu bytes (header: %d, payload: %u) for IE type 0x%X\n",
               consumed, parsed_mux_header_len, dlc_pdu_len_from_mux, ie_type_from_mux);

		current_ie_ptr += consumed;
		remaining_sdu_area_len -= consumed;
        
        printk("RX_SDU_HANDLER: Remaining SDU area length: %zu\n", remaining_sdu_area_len);
        printk("RX_SDU_HANDLER: *******************************************   END ITERATION   *******************************************\n");
	}
    
    printk("RX_SDU_HANDLER: Finished processing MAC SDU Area. Total processed: %zu bytes\n", 
           mac_sdu_area_len - remaining_sdu_area_len);
}



void dect_mac_data_path_handle_op_complete(pending_op_type_t completed_type,
					   const struct nrf_modem_dect_phy_op_complete_event *event)
{
	int harq_idx = -1;
	if (completed_type >= PENDING_OP_PT_DATA_TX_HARQ0 && completed_type <= PENDING_OP_PT_DATA_TX_HARQ_MAX) {
		harq_idx = completed_type - PENDING_OP_PT_DATA_TX_HARQ0;
	} else if (completed_type >= PENDING_OP_FT_DATA_TX_HARQ0 && completed_type <= PENDING_OP_FT_DATA_TX_HARQ_MAX) {
		harq_idx = completed_type - PENDING_OP_FT_DATA_TX_HARQ0;
	}

	if (harq_idx != -1) {
		if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
			/* A successful TX operation means we now wait for the ACK in the feedback */
			/* The HARQ timer is already running. If no ACK is received, it will time out. */
			LOG_DBG("DATA_PATH: HARQ TX for proc %d completed successfully at PHY. Awaiting feedback.", harq_idx);
		} else {
			/* The PHY operation itself failed (e.g., LBT timeout). Treat as a NACK. */
			LOG_WRN("DATA_PATH: HARQ TX for proc %d failed at PHY with err %d. Triggering NACK.", harq_idx, event->err);
			dect_mac_data_path_handle_harq_nack_action(harq_idx);
		}
	}
}




#if IS_ENABLED(CONFIG_ZTEST)
dlc_tx_status_cb_t dect_mac_test_get_dlc_status_callback(void)
{
	return g_dlc_status_callback;
}
#endif /* IS_ENABLED(CONFIG_ZTEST) */