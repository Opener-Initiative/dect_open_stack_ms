/* dect_mac/dect_mac_main_dispatcher.c */
#include <zephyr/logging/log.h>
#include <string.h> // For memcpy if needed, snprintk
#include <stdio.h>  // For snprintk

#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_core.h>      // For get_mac_context()
#include <mac/dect_mac_context.h>   // For dect_mac_context_t and its members
#include <mac/dect_mac_sm.h>        // For event types, state enums
#include <mac/dect_mac_sm_pt.h>     // For dect_mac_sm_pt_handle_event()
#include <mac/dect_mac_sm_ft.h>     // For dect_mac_sm_ft_handle_event()
#include <mac/dect_mac_phy_ctrl.h>  // For dect_mac_phy_ctrl_handle_op_complete (though SMs call this now)
#include <mac/dect_mac_data_path.h>

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

LOG_MODULE_REGISTER(dect_mac_dispatcher, CONFIG_DECT_MAC_DISPATCHER_LOG_LEVEL);


/**
 * @brief Message queue for safely passing events from the PHY callback (modem library context)
 *        to the main MAC processing thread.
 *
 * This queue is consumed by the MAC thread in dect_mac_main.c.
 * Size should be sufficient for the expected event rate.
 */
#if IS_ENABLED(CONFIG_ZTEST) && IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)
K_MSGQ_DEFINE(mac_event_msgq, sizeof(struct dect_mac_event_msg), 32, 4); // 32 messages, 4-byte aligned
#else
K_MSGQ_DEFINE(mac_event_msgq, sizeof(struct dect_mac_event_msg), 16, 4); // 16 messages, 4-byte aligned
#endif

// Declare the callback pointer as external.
extern dect_mac_state_change_cb_t g_state_change_cb;


static dect_mac_public_state_t dect_mac_state_t_to_public(dect_mac_state_t internal_state)
{
	switch (internal_state) {
	case MAC_STATE_DEACTIVATED:
		return MAC_STATE_PUB_DEACTIVATED;

	case MAC_STATE_IDLE:
	case MAC_STATE_PT_SCANNING:
	case MAC_STATE_PT_BEACON_PDC_WAIT:
	case MAC_STATE_PT_ASSOCIATING:
	case MAC_STATE_PT_RACH_BACKOFF:
	case MAC_STATE_PT_WAIT_ASSOC_RESP:
	case MAC_STATE_PT_AUTHENTICATING:
	case MAC_STATE_PT_WAIT_AUTH_CHALLENGE:
	case MAC_STATE_PT_WAIT_AUTH_SUCCESS:
	case MAC_STATE_PT_HANDOVER_ASSOCIATING:
	case MAC_STATE_FT_SCANNING:
		return MAC_STATE_PUB_IDLE;

	case MAC_STATE_ASSOCIATED:
	case MAC_STATE_PT_PAGING:
	case MAC_STATE_FT_BEACONING:
	case MAC_STATE_FT_RACH_LISTEN:
	case MAC_STATE_FT_WAIT_AUTH_INIT:
	case MAC_STATE_FT_WAIT_AUTH_RESP:
		return MAC_STATE_PUB_ASSOCIATED;

	case MAC_STATE_ERROR:
	default:
		return MAC_STATE_PUB_ERROR;
	}
}



void dect_mac_change_state(dect_mac_state_t new_state) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    printk("[DISPATCH] dect_mac_change_state ctx->state:%d != new_state:%d \n", ctx->state, new_state);

	if (ctx->state != new_state) {
		dect_mac_public_state_t old_public_state = dect_mac_state_t_to_public(ctx->state);
		dect_mac_public_state_t new_public_state = dect_mac_state_t_to_public(new_state);

		printk("[DISPATCH] Changing MAC State: %s -> %s (Role: %s)\n", dect_mac_state_to_str(ctx->state),
			dect_mac_state_to_str(new_state), (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
		// LOG_INF("MAC State: %s -> %s (Role: %s)", dect_mac_state_to_str(ctx->state),
		// 	dect_mac_state_to_str(new_state), (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));

		/* "On Exit" logic for the old state */
		if (ctx->role == MAC_ROLE_PT && ctx->state == MAC_STATE_ASSOCIATED) {
			printk("Exiting ASSOCIATED state, stopping link timers.");
            // LOG_DBG("Exiting ASSOCIATED state, stopping link timers.");
			k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
			k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);
		}
		ctx->state = new_state;

        printk("old_public_state:%d != %d:new_public_state \n", old_public_state , new_public_state);
        printk("g_state_change_cb?%s \n", g_state_change_cb ? "Yes":"No!!!");
		/* If the abstract public state has changed, notify the upper layer */
		if (g_state_change_cb && (old_public_state != new_public_state)) {
			printk("Public MAC State Change: %d -> %d \n", old_public_state,
				new_public_state);
			// LOG_INF("Public MAC State Change: %d -> %d", old_public_state,
			// 	new_public_state);                
			printk("[DISPATCHER_EVIDENCE] State change callback is registered. About to invoke.\n");
            g_state_change_cb(new_public_state);
            printk("[DISPATCHER_EVIDENCE] State change callback was (temporarily) bypassed.\n");
		}
	}
}

void dect_mac_enter_error_state(const char *reason)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	LOG_ERR("Entering MAC_STATE_ERROR. Reason: %s. Halting MAC operations.", reason);

	/* Attempt to gracefully shut down the PHY */
	if (ctx->state >= MAC_STATE_FT_BEACONING) { /* Check if active */
		dect_mac_phy_ctrl_deactivate();
	}

	dect_mac_change_state(MAC_STATE_ERROR);
}

// --- Common PHY Event Pre-Processing ---
// Returns true if event should be dispatched to SM, false otherwise.
static bool common_pre_handle_phy_pcc_event(const struct nrf_modem_dect_phy_pcc_event *pcc_event_data,
                                            const dect_mac_context_t *ctx) {
    if (!pcc_event_data || !ctx) return false;

    printk("PCC_DISPATCH: Rcvd. H:%u, Status:%d, PHYType:%d, RSSI2:%f, SNR:%f, Carr:%u, TID:%u \n",
            pcc_event_data->handle, pcc_event_data->header_status, pcc_event_data->phy_type,
            (double)pcc_event_data->rssi_2 / 2,
            (double)pcc_event_data->snr / 4,
            ctx->current_rx_op_carrier,
            pcc_event_data->transaction_id);    
    // LOG_DBG("PCC_DISPATCH: Rcvd. H:%u, Status:%d, PHYType:%d, RSSI2:%.1f, SNR:%.2f, Carr:%u, TID:%u",
    //         pcc_event_data->handle, pcc_event_data->header_status, pcc_event_data->phy_type,
    //         // Cast to double to match the type expected by the %f format specifier
    //         (double)pcc_event_data->rssi_2 / 2.0,
    //         (double)pcc_event_data->snr / 4.0,
    //         ctx->current_rx_op_carrier,
    //         pcc_event_data->transaction_id);

    if (pcc_event_data->header_status != NRF_MODEM_DECT_PHY_HDR_STATUS_VALID) {
        printk("PCC_DISPATCH: Invalid PCC status (%d). Not dispatching to SM.\n", pcc_event_data->header_status);
        // LOG_WRN("PCC_DISPATCH: Invalid PCC status (%d). Not dispatching to SM.", pcc_event_data->header_status);
        return false; // Do not dispatch invalid PCCs further usually, SM might handle specific error if needed
    }

    uint8_t received_short_net_id_from_pcc;
    if (pcc_event_data->phy_type == 0) { // nRF PHY Type 0 (ETSI PCC Type 1)
        received_short_net_id_from_pcc = pcc_event_data->hdr.hdr_type_1.short_network_id;
    } else if (pcc_event_data->phy_type == 1) { // nRF PHY Type 1 (ETSI PCC Type 2)
        received_short_net_id_from_pcc = pcc_event_data->hdr.hdr_type_2.short_network_id;
    } else {
        printk("PCC_DISPATCH: Unknown nRF PHY header type in PCC: %d. Not dispatching.\n", pcc_event_data->phy_type);
        // LOG_ERR("PCC_DISPATCH: Unknown nRF PHY header type in PCC: %d. Not dispatching.", pcc_event_data->phy_type);
        return false;
    }


printk("PCC_DISPATCH: common_pre_handle_phy_pcc_event - ctx->role:%d && ctx->state:%d\n", ctx->role, ctx->state);


    // ETSI TS 103 636-4, 4.2.3.1: Last 8 LSB of Network ID are in PHY control field (PCC Short Network ID)
    // Only process PCCs from our network (or from any if NetID is 0 - e.g. during initial scan before NetID known)
    uint8_t own_net_id_lsb = (uint8_t)(ctx->network_id_32bit & 0xFF);
    if (own_net_id_lsb != 0 && received_short_net_id_from_pcc != own_net_id_lsb) {
        // Exception: PT scanning might listen for any network ID before it knows its target.
        // The SM should handle this. For now, this dispatcher assumes if own_net_id_lsb is set, it must match.
        // if (!(ctx->role == MAC_ROLE_PT && (ctx->state == MAC_STATE_PT_SCANNING || ctx->state == MAC_STATE_PT_BEACON_PDC_WAIT))) {
        if (!(ctx->role == MAC_ROLE_PT && (ctx->state == MAC_STATE_PT_SCANNING 
            || ctx->state == MAC_STATE_PT_BEACON_PDC_WAIT 
            || ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP
            || ctx->state == MAC_STATE_PT_AUTHENTICATING
        ))) {
            printk("PCC_DISPATCH: From different network_id_lsb (0x%02X vs our 0x%02X). Ignoring.\n",
                    received_short_net_id_from_pcc, own_net_id_lsb);
            // LOG_DBG("PCC_DISPATCH: From different network_id_lsb (0x%02X vs our 0x%02X). Ignoring.",
            //         received_short_net_id_from_pcc, own_net_id_lsb);                    
            return false;
        } else {
            LOG_DBG("PCC_DISPATCH: PT in scan/wait_pdc received from NetID LSB 0x%02X (own: 0x%02X). Allowing for now.",
                    received_short_net_id_from_pcc, own_net_id_lsb);
        }
    }
    return true; // Event is valid enough for SM to look at
}

static bool common_pre_handle_phy_pdc_event(const struct nrf_modem_dect_phy_pdc_event *pdc_event_data) {
    if (!pdc_event_data) return false;

    LOG_DBG("PDC_DISPATCH: Rcvd. H:%u, Len:%zu, RSSI2:%.1f, SNR:%.2f, TID:%u",
            pdc_event_data->handle, pdc_event_data->len,
            (double)pdc_event_data->rssi_2 / 2.0, (double)pdc_event_data->snr / 4.0,
            pdc_event_data->transaction_id);
            

    if (pdc_event_data->data == NULL && pdc_event_data->len > 0) {
        LOG_ERR("PDC_DISPATCH: Data NULL but len %zu > 0. Not dispatching.", pdc_event_data->len);
        return false;
    }
    // Allow zero length PDC (MAC PDU might be empty if only PCC was used for signaling ACK/NACK)
    if (pdc_event_data->len > CONFIG_DECT_MAC_PDU_MAX_SIZE) { // Max size for the SDU part of MAC PDU
        LOG_ERR("PDC_DISPATCH: Data too large (%zu > %d). Not dispatching.",
                pdc_event_data->len, CONFIG_DECT_MAC_PDU_MAX_SIZE);
        return false;
    }
    LOG_ERR("PDC_DISPATCH: common_pre_handle_phy_pdc_event TRUE.");
    return true; // Event seems valid for SM processing
}

/* Overview: Refactors the main event dispatcher to be context-aware. 
It now uses the context pointer from the event message itself for state checks and routing, 
completely eliminating the dependency on the flawed global context pointer.*/
// --- Main Event Dispatcher ---
void dect_mac_event_dispatch(const struct dect_mac_event_msg *msg)
{
    printk("[DISPATCH_DBG] Entering dect_mac_event_dispatch for event %s.\n", dect_mac_event_to_str(msg->type));

	if (!msg) {
		LOG_ERR("DISPATCH: Received NULL message pointer.");
		return;
	}

	dect_mac_context_t *ctx = msg->ctx;
	// printk("[DISPATCH_CTX_DBG] Event for context: %p, Globally active context: %p\n",
	//        (void *)ctx, (void *)dect_mac_get_active_context());

	if (!ctx) { /* Should not happen after core_init */
		LOG_ERR("DISPATCH: MAC Context not available! Cannot dispatch event %s.", dect_mac_event_to_str(msg->type));
		return;
	}

	/* Set the active context for the duration of this dispatch. */
	dect_mac_test_set_active_context(ctx);

    // printk("[DISPATCH_DBG] Event %s received. Current state is %s (%d).\n",
    //         dect_mac_event_to_str(msg->type), dect_mac_state_to_str(ctx->state), ctx->state);
	// printk("[DISPATCHER] Processing event %s. Active context is for role: %s (at %0X)\n",
	//        dect_mac_event_to_str(msg->type), (ctx->role == MAC_ROLE_PT ? "PT" : "FT"), ctx->own_short_rd_id);
           

	if (ctx->state == MAC_STATE_ERROR) {
		LOG_WRN("DISPATCH: In MAC_STATE_ERROR. Event %s discarded.", dect_mac_event_to_str(msg->type));
		return;
	}

    printk("[DISPATCHER] Dispatching event %s to role %s (0x%0X)\n",
	       dect_mac_event_to_str(msg->type), (ctx->role == MAC_ROLE_PT ? "PT" : "FT"), msg->ctx->own_short_rd_id);   
    // LOG_INF("[DISPATCHER] Dispatching event %s to role %s\n", dect_mac_event_to_str(msg->type), (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));

    // Update last known modem time from the event itself
    ctx->last_known_modem_time = msg->modem_time_of_event;

    printk("MAC Evt Dispatch: Evt %s (Time %llu) in State %s (Role %s, PendOp: %s, Hdl: %u) \n",
            dect_mac_event_to_str(msg->type),
            msg->modem_time_of_event,
            dect_mac_state_to_str(ctx->state),
            (ctx->role == MAC_ROLE_PT ? "PT" : "FT"),
            dect_pending_op_to_str(ctx->pending_op_type),
            ctx->pending_op_handle);

    bool dispatch_to_sm = true;
// printk("[DISPATCHER] msg->type:%s(%d) ",dect_mac_event_to_str(msg->type), msg->type);
// printk(" dispatch_to_sm: %s\n",(dispatch_to_sm ? "True" : "False"));

    switch (msg->type) {
        case MAC_EVENT_PHY_OP_COMPLETE: {
            pending_op_type_t completed_type = dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete);

            if (completed_type != PENDING_OP_NONE) {
                /* Check if this is a data path operation (HARQ) */
                if (completed_type >= PENDING_OP_PT_DATA_TX_HARQ0 &&
                    completed_type <= PENDING_OP_FT_DATA_TX_HARQ_MAX) {
                    LOG_DBG("DISPATCH: OP_COMPLETE for data path op %s. Forwarding to data path handler.",
                        dect_pending_op_to_str(completed_type));
                    dect_mac_data_path_handle_op_complete(completed_type, &msg->data.op_complete);
                    dispatch_to_sm = false; /* Data path handles this, not the SM */
                } else {
                    LOG_INF("DISPATCH: OP_COMPLETE for control plane op %s. Forwarding to SM.",
                        dect_pending_op_to_str(completed_type));
                }
            }
            break;
        }    
        case MAC_EVENT_PHY_PCC:
            printk("DISPATCH: MAC_EVENT_PHY_PCC\n");
            // LOG_INF("DISPATCH: MAC_EVENT_PHY_PCC");
            dispatch_to_sm = common_pre_handle_phy_pcc_event(&msg->data.pcc, ctx);
            break;
        case MAC_EVENT_PHY_PDC:
            printk("DISPATCH: MAC_EVENT_PHY_PDC\n");
            // LOG_INF("DISPATCH: MAC_EVENT_PHY_PDC");
            dispatch_to_sm = common_pre_handle_phy_pdc_event(&msg->data.pdc);
            break;
        case MAC_EVENT_PHY_PCC_ERROR:
            LOG_WRN("DISPATCH: PCC Error Rcvd (H:%u, RSSI2:%.1f, SNR:%.1f, TID:%u). Forwarding to SM.",
                    msg->data.pcc_crc_err.handle,
                    (double)msg->data.pcc_crc_err.rssi_2 / 2.0, (double)msg->data.pcc_crc_err.snr / 4.0,
                    msg->data.pcc_crc_err.transaction_id);
            break;
        case MAC_EVENT_PHY_PDC_ERROR:
            LOG_WRN("DISPATCH: PDC Error Rcvd (H:%u, RSSI2:%.1f, SNR:%.1f, TID:%u). Forwarding to SM.",
                    msg->data.pdc_crc_err.handle,
                    (double)msg->data.pdc_crc_err.rssi_2 / 2.0, (double)msg->data.pdc_crc_err.snr / 4.0,
                    msg->data.pdc_crc_err.transaction_id);
            break;
        case MAC_EVENT_PHY_RSSI_RESULT:
            LOG_INF("DISPATCH: RSSI Report Rcvd (H:%u, Carr:%u, Count:%u). Forwarding to SM.",
                    msg->data.rssi.handle, msg->data.rssi.carrier, msg->data.rssi.meas_len);
            break;

        // Timer events and Command events are dispatched directly to SM without pre-handling here
        case MAC_EVENT_TIMER_EXPIRED_HARQ:
            LOG_WRN("DISPATCH: HARQ Timer expired for proc %d. Forwarding to data path.", msg->data.timer_data.id);
            dect_mac_data_path_handle_harq_nack_action(msg->data.timer_data.id);
            dispatch_to_sm = false;
            break;

        case MAC_EVENT_TIMER_EXPIRED_BEACON:
        case MAC_EVENT_TIMER_EXPIRED_KEEPALIVE:
        case MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF:
        case MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW:
            LOG_DBG("DISPATCH: Timer/Cmd event %s. Forwarding to SM.", dect_mac_event_to_str(msg->type));
            break;

        case MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN:
        case MAC_EVENT_TIMER_EXPIRED_PAGING_CYCLE:
        case MAC_EVENT_CMD_ENTER_PAGING_MODE:
        case MAC_EVENT_CMD_RELEASE_LINK:
            printk("DISPATCH: Timer/Cmd event %s. Forwarding to SM.\n", dect_mac_event_to_str(msg->type));
            LOG_DBG("DISPATCH: Timer/Cmd event %s. Forwarding to SM.", dect_mac_event_to_str(msg->type));
            break;    
        default:
            // TODO: fix the error log output
            // LOG_WRN("DISPATCH: Unknown event type %d. Not dispatching.", msg->type);
            printk("DISPATCH: Unknown event type. Not dispatching.\n");
            // LOG_WRN("DISPATCH: Unknown event type. Not dispatching.");
            dispatch_to_sm = false;
            break;
    }

//  printk("[DISPATCHER] dispatch_to_sm: %s\n",(dispatch_to_sm ? "True" : "False"));  
    if (dispatch_to_sm) {
        if (ctx->role == MAC_ROLE_PT) {
            printk("[DISPATCHER] Calling PT SM handler with context %p ctx->role %s\n", (void *)ctx, (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
            dect_mac_sm_pt_handle_event(msg);
        } else { // MAC_ROLE_FT
            printk("[DISPATCHER] Calling FT SM handler with context %p ctx->role %s\n", (void *)ctx, (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
            dect_mac_sm_ft_handle_event(msg);
        }
    }
    // printk("[DISPATCH_DBG] Exiting dect_mac_event_dispatch.\n");
}


/* Overview: Adds a new, internal, test-only function for event injection. 
This function is not part of the public API and can safely access internal structures 
like the event message queue and context, resolving the circular dependency issue.*/
#if IS_ENABLED(CONFIG_ZTEST)
/* This is a test-only function and is not part of the public API.
 * It is defined here because it needs access to the static mac_event_msgq.
 */
int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					dect_mac_event_type_t event_type, int timer_id)
{
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = event_type,
		.modem_time_of_event = 0, /* Not critical for this test's timer events */
		.data.timer_data.id = timer_id
	};

	return k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);
}
#endif /* IS_ENABLED(CONFIG_ZTEST) */


// --- String Conversion Utility Functions ---
const char* dect_mac_state_to_str(dect_mac_state_t state) {
    static const char* const names[] = {
        [MAC_STATE_DEACTIVATED] = "DEACTIVATED",
        [MAC_STATE_IDLE] = "IDLE",
        [MAC_STATE_PT_SCANNING] = "PT_SCANNING",
        [MAC_STATE_PT_BEACON_PDC_WAIT] = "PT_BEACON_PDC_WAIT",
        [MAC_STATE_PT_ASSOCIATING] = "PT_ASSOCIATING",
        [MAC_STATE_PT_RACH_BACKOFF] = "PT_RACH_BACKOFF",
        [MAC_STATE_PT_WAIT_ASSOC_RESP] = "PT_WAIT_ASSOC_RESP",
        [MAC_STATE_PT_AUTHENTICATING] = "PT_AUTHENTICATING",
        [MAC_STATE_FT_WAIT_AUTH_INIT] = "PT_WAIT_AUTH_INITIATION",
        [MAC_STATE_FT_WAIT_AUTH_RESP] = "PT_WAIT_AUTH_RESPONCE",        
        [MAC_STATE_PT_WAIT_AUTH_CHALLENGE] = "PT_WAIT_AUTH_CHALLENGE",
        [MAC_STATE_PT_WAIT_AUTH_SUCCESS] = "PT_WAIT_AUTH_SUCCESS",
        [MAC_STATE_PT_HANDOVER_ASSOCIATING] = "PT_HANDOVER_ASSOCIATING",
        [MAC_STATE_PT_PAGING] = "PT_PAGING",
        [MAC_STATE_PT_RELEASING] = "PT_RELEASING",
        [MAC_STATE_FT_SCANNING] = "FT_SCANNING",
        [MAC_STATE_FT_BEACONING] = "FT_BEACONING",
        [MAC_STATE_FT_RACH_LISTEN] = "FT_RACH_LISTEN",
        [MAC_STATE_ASSOCIATED] = "ASSOCIATED",
        [MAC_STATE_ERROR] = "ERROR",
        [MAC_STATE_PT_CANCELLING_SCAN] = "PT_CANCELLING_SCAN",
    };
    if (state < MAC_STATE_COUNT && state >= 0) return names[state];
    return "S_UNKNOWN";
}

const char* dect_mac_event_to_str(dect_mac_event_type_t event_type) {
    static const char* const names[] = {
        [MAC_EVENT_PHY_OP_COMPLETE]    = "PHY_OP_COMPLETE",
        [MAC_EVENT_PHY_PCC]            = "PHY_PCC",
        [MAC_EVENT_PHY_PDC]            = "PHY_PDC",
        [MAC_EVENT_PHY_PCC_ERROR]      = "PHY_PCC_ERROR",
        [MAC_EVENT_PHY_PDC_ERROR]      = "PHY_PDC_ERROR",
        [MAC_EVENT_PHY_RSSI_RESULT]    = "PHY_RSSI_RESULT",
        [MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF] = "TMR_RACH_BACKOFF",
        [MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW] = "TMR_RACH_RESP_WIN",
        [MAC_EVENT_TIMER_EXPIRED_KEEPALIVE]    = "TMR_KEEPALIVE",
        [MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN]= "TMR_MOBILITY_SCAN",
        [MAC_EVENT_TIMER_EXPIRED_PAGING_CYCLE] = "TMR_PAGING_CYCLE",
        [MAC_EVENT_TIMER_EXPIRED_BEACON]       = "TMR_BEACON",
        [MAC_EVENT_TIMER_EXPIRED_HARQ]         = "TMR_HARQ",
        [MAC_EVENT_CMD_ENTER_PAGING_MODE]      = "CMD_ENTER_PAGING",
        [MAC_EVENT_CMD_RELEASE_LINK]           = "CMD_RELEASE_LINK",        
    };
    if (event_type < MAC_EVENT_TYPE_COUNT && event_type >= 0) return names[event_type];
    return "E_UNKNOWN";
}

const char* dect_pending_op_to_str(pending_op_type_t op_type) {
    // This mapping needs to be robust and cover all enum values.
    // Using a static array for direct mapping is better if enum values are contiguous.
    switch(op_type) {
        case PENDING_OP_NONE: return "NONE";
        case PENDING_OP_PT_SCAN: return "PT_SCAN";
        case PENDING_OP_PT_RACH_ASSOC_REQ: return "PT_RACH_ASSOC_REQ";
        case PENDING_OP_PT_WAIT_BEACON_PDC: return "PT_WAIT_BEACON_PDC";
        case PENDING_OP_PT_WAIT_ASSOC_RESP: return "PT_WAIT_ASSOC_RESP";
        case PENDING_OP_PT_KEEP_ALIVE: return "PT_KEEP_ALIVE";
        case PENDING_OP_PT_MOBILITY_SCAN: return "PT_MOBILITY_SCAN";
        case PENDING_OP_PT_AUTH_MSG_TX: return "PT_AUTH_MSG_TX";
        case PENDING_OP_PT_AUTH_MSG_RX: return "PT_AUTH_MSG_RX";
        case PENDING_OP_PT_PAGING_LISTEN: return "PT_PAGING_LISTEN";
        case PENDING_OP_PT_DATA_RX: return "PT_DATA_LISTEN";
        case PENDING_OP_FT_INITIAL_SCAN: return "FT_INITIAL_SCAN";
        case PENDING_OP_FT_BEACON: return "FT_BEACON";
        case PENDING_OP_FT_RACH_RX_WINDOW: return "FT_RACH_RX_WINDOW";
        case PENDING_OP_FT_GROUP_RX: return "FT_GROUP_RX";
        case PENDING_OP_FT_ASSOC_RESP: return "FT_ASSOC_RESP";
        case PENDING_OP_FT_AUTH_MSG_TX: return "FT_AUTH_MSG_TX";
        case PENDING_OP_FT_AUTH_MSG_RX: return "FT_AUTH_MSG_RX";
        case PENDING_OP_FT_DATA_RX: return "FT_DATA_RX";
        /* Generic Operations */
        case PENDING_OP_GENERIC_UNICAST_TX: return "GENERIC_UNICAST_TX";        
        default: break;
    }
    // Handle HARQ ranges
    if (op_type >= PENDING_OP_PT_DATA_TX_HARQ0 && op_type <= PENDING_OP_PT_DATA_TX_HARQ_MAX) {
        static char pt_harq_str[20];
        snprintk(pt_harq_str, sizeof(pt_harq_str), "PT_DATA_TX_HARQ%d", op_type - PENDING_OP_PT_DATA_TX_HARQ0);
        return pt_harq_str;
    }
    if (op_type >= PENDING_OP_FT_DATA_TX_HARQ0 && op_type <= PENDING_OP_FT_DATA_TX_HARQ_MAX) {
        static char ft_harq_str[20];
        snprintk(ft_harq_str, sizeof(ft_harq_str), "FT_DATA_TX_HARQ%d", op_type - PENDING_OP_FT_DATA_TX_HARQ0);
        return ft_harq_str;
    }
    return "P_UNKNOWN";
}

const char* nrf_modem_dect_phy_err_to_str(enum nrf_modem_dect_phy_err err) {
    // This switch case covers all defined errors in nrf_modem_dect_phy.h up to a certain version.
    // Ensure it's kept up-to-date with the header file.
    switch (err) {
        case NRF_MODEM_DECT_PHY_SUCCESS: return "SUCCESS";
        case NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY: return "LBT_CHANNEL_BUSY";
        case NRF_MODEM_DECT_PHY_ERR_UNSUPPORTED_OP: return "UNSUPPORTED_OP";
        case NRF_MODEM_DECT_PHY_ERR_NOT_FOUND: return "NOT_FOUND";
        case NRF_MODEM_DECT_PHY_ERR_NO_MEMORY: return "NO_MEMORY";
        case NRF_MODEM_DECT_PHY_ERR_NOT_ALLOWED: return "NOT_ALLOWED";
        case NRF_MODEM_DECT_PHY_OK_WITH_HARQ_RESET: return "OK_WITH_HARQ_RESET";
        case NRF_MODEM_DECT_PHY_ERR_OP_START_TIME_LATE: return "OP_START_TIME_LATE";
        case NRF_MODEM_DECT_PHY_ERR_LBT_START_TIME_LATE: return "LBT_START_TIME_LATE";
        case NRF_MODEM_DECT_PHY_ERR_RF_START_TIME_LATE: return "RF_START_TIME_LATE";
        case NRF_MODEM_DECT_PHY_ERR_INVALID_START_TIME: return "INVALID_START_TIME";
        case NRF_MODEM_DECT_PHY_ERR_OP_SCHEDULING_CONFLICT: return "OP_SCHEDULING_CONFLICT";
        case NRF_MODEM_DECT_PHY_ERR_OP_TIMEOUT: return "OP_TIMEOUT";
        case NRF_MODEM_DECT_PHY_ERR_NO_ONGOING_HARQ_RX: return "NO_ONGOING_HARQ_RX";
        case NRF_MODEM_DECT_PHY_ERR_PARAMETER_UNAVAILABLE: return "PARAMETER_UNAVAILABLE";
        case NRF_MODEM_DECT_PHY_ERR_PAYLOAD_UNAVAILABLE: return "PAYLOAD_UNAVAILABLE";
        case NRF_MODEM_DECT_PHY_ERR_OP_CANCELED: return "OP_CANCELED";
        case NRF_MODEM_DECT_PHY_ERR_COMBINED_OP_FAILED: return "COMBINED_OP_FAILED";
        case NRF_MODEM_DECT_PHY_ERR_RADIO_MODE_CONFLICT: return "RADIO_MODE_CONFLICT";
        case NRF_MODEM_DECT_PHY_ERR_UNSUPPORTED_CARRIER: return "UNSUPPORTED_CARRIER";
        case NRF_MODEM_DECT_PHY_ERR_UNSUPPORTED_DATA_SIZE: return "UNSUPPORTED_DATA_SIZE";
        case NRF_MODEM_DECT_PHY_ERR_INVALID_NETWORK_ID: return "INVALID_NETWORK_ID";
        case NRF_MODEM_DECT_PHY_ERR_INVALID_PHY_HEADER: return "INVALID_PHY_HEADER";
        case NRF_MODEM_DECT_PHY_ERR_INVALID_DURATION: return "INVALID_DURATION";
        case NRF_MODEM_DECT_PHY_ERR_INVALID_PARAMETER: return "INVALID_PARAMETER";
        case NRF_MODEM_DECT_PHY_ERR_TX_POWER_OVER_MAX_LIMIT: return "TX_POWER_OVER_MAX_LIMIT";
        case NRF_MODEM_DECT_PHY_ERR_MODEM_ERROR: return "MODEM_ERROR";
        case NRF_MODEM_DECT_PHY_ERR_MODEM_ERROR_RF_STATE: return "MODEM_ERROR_RF_STATE";
        case NRF_MODEM_DECT_PHY_ERR_TEMP_HIGH: return "TEMP_HIGH";
        case NRF_MODEM_DECT_PHY_ERR_PROD_LOCK: return "PROD_LOCK";
        default:
            {
                // Using a static buffer for unknown codes has thread-safety implications if multiple
                // threads call this concurrently for different unknown errors before the string is used.
                // For typical Zephyr logging from a single MAC thread, this is usually acceptable.
                static char unknown_err_str[24]; // Enough for "PHY_ERR_0xDEADBEEF"
                snprintk(unknown_err_str, sizeof(unknown_err_str), "PHY_ERR_0x%X", err);
                return unknown_err_str;
            }
    }
}