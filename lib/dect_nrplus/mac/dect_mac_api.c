/* dect_mac/dect_mac_api.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/dlist.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
#include <mac/dect_mac_security.h>
#endif

LOG_MODULE_REGISTER(dect_mac_api, CONFIG_DECT_MAC_API_LOG_LEVEL);


// Define the memory slab for mac_sdu_t buffers
K_MEM_SLAB_DEFINE(g_mac_sdu_slab, sizeof(mac_sdu_t), MAX_MAC_SDU_BUFFERS_CONFIG, 4);

sys_dlist_t *g_dlc_rx_sdu_dlist_ptr = NULL;


// Define the list structures
sys_dlist_t g_mac_tx_dlist_high_priority;
sys_dlist_t g_mac_tx_dlist_reliable_data;
sys_dlist_t g_mac_tx_dlist_best_effort;

sys_dlist_t * const mac_tx_dlists[MAC_FLOW_COUNT] = {
    &g_mac_tx_dlist_high_priority,
    &g_mac_tx_dlist_reliable_data,
    &g_mac_tx_dlist_best_effort
};

// External message queue for sending commands to the MAC thread
// Defined in dect_mac_phy_if.c
extern struct k_msgq mac_event_msgq;


int dect_mac_api_init(sys_dlist_t *rx_dlist_from_dlc)
{
	if (!rx_dlist_from_dlc) {
		LOG_WRN("DLC RX dlist is NULL. MAC will not be able to deliver received data.");
	}

    g_dlc_rx_sdu_dlist_ptr = rx_dlist_from_dlc;

	printk("[INIT_DLIST_DBG] MAC API received dlist pointer. g_dlc_rx_sdu_dlist_ptr is now: %p\n",
	       (void *)g_dlc_rx_sdu_dlist_ptr);

		   
	/* Initialize the MAC's own TX dlists */
	sys_dlist_init(&g_mac_tx_dlist_high_priority);
	sys_dlist_init(&g_mac_tx_dlist_reliable_data);
	sys_dlist_init(&g_mac_tx_dlist_best_effort);
        
    LOG_INF("MAC API Initialized. DLC RX dlist registered.");
    return 0;
}

mac_sdu_t* dect_mac_api_buffer_alloc(k_timeout_t timeout)
{
    mac_sdu_t *sdu = NULL;
    int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void **)&sdu, timeout);
	printk("[MEM_SLAB_DBG] dect_mac_buffer_alloc called. Result: %p. Used blocks: %u\n",
           (void *)sdu, g_mac_sdu_slab.info.num_used);

    if (ret != 0) {                
        if (ret == -ENOMEM) {
            if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
                LOG_DBG("Failed to allocate MAC SDU buffer (no wait), slab empty.");
            } else {
                LOG_WRN("Failed to allocate MAC SDU buffer (timeout), slab empty.");
            }
        } else {
            LOG_ERR("Failed to allocate MAC SDU buffer, unexpected err: %d", ret);
        }
        return NULL;
    }
    // This memset correctly initializes the new boolean field to false and the SN to 0.
    // memset(sdu, 0, sizeof(mac_sdu_t));
    sdu->len = 0;
    sdu->target_peer_short_rd_id = 0;
    sdu->dlc_status_report_required = false;
    sdu->dlc_sn_for_status = 0;
    return sdu;
}

// void dect_mac_api_buffer_free(mac_sdu_t *sdu)
// {
//     if (sdu == NULL) {
//         LOG_WRN("Attempted to free a NULL SDU buffer.");
//         return;
//     }

// 	printk("[MEM_SLAB_DBG] dect_mac_buffer_free called for buffer: %p. Used blocks before free: %u\n",
//            (void *)sdu, g_mac_sdu_slab.info.num_used);
// 	LOG_DBG("Freeing SDU buffer: %p. Used blocks before free: %u",
//             (void *)sdu, g_mac_sdu_slab.info.num_used);
//     k_mem_slab_free(&g_mac_sdu_slab, (void **)&sdu);
//     // k_mem_slab_free(&g_mac_sdu_slab, (void *)sdu);
// }
// In your .c file, make the function static and rename it
void dect_mac_api_buffer_free_internal(mac_sdu_t *sdu, const char *caller_func)
{
    if (sdu == NULL) {
        LOG_WRN("(%s) Attempted to free a NULL SDU buffer.", caller_func);
        return;
    }

    LOG_DBG("(%s) Freeing SDU buffer: %p. Used blocks before free: %u",
            caller_func, (void *)sdu, g_mac_sdu_slab.info.num_used);

    // k_mem_slab_free(&g_mac_sdu_slab, (void **)&sdu);
	k_mem_slab_free(&g_mac_sdu_slab, (void *)sdu);
}


int dect_mac_api_send(mac_sdu_t *sdu, mac_flow_id_t flow)
{
	printk("[API_SEND_DBG] REAL dect_mac_api_send() function was called.\n");
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (ctx->role == MAC_ROLE_FT) {
		LOG_ERR("Generic dect_mac_api_send() called by FT. Use dect_mac_api_ft_send_to_pt() instead.");
		if (sdu) {
			dect_mac_api_buffer_free(sdu);
		}
		return -EPERM;
	}

	if (sdu == NULL) {
		return -EINVAL;
	}
	if (flow >= MAC_FLOW_COUNT) {
		dect_mac_api_buffer_free(sdu);
		return -EINVAL;
	}

	if (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING) {
		LOG_DBG("PT_SEND_API: Handover in progress. Buffering SDU (len %u) to holding queue.", sdu->len);
		sys_dlist_append(&ctx->role_ctx.pt.handover_tx_holding_dlist, &sdu->node);
		return 0;
	}

	printk("[SEND_API_DBG] Checking state in dect_mac_api_send. Current state is %s (%d).\n",
	       dect_mac_state_to_str(ctx->state), ctx->state);

	if (ctx->state < MAC_STATE_ASSOCIATED) {
		LOG_WRN("PT_SEND_API: Cannot send, not associated. Dropping SDU.");
		dect_mac_api_buffer_free(sdu);
		return -ENETDOWN;
	}

	printk("PT_SEND_API: Queueing SDU (len %u) to generic MAC TX Flow %d (for associated FT)\n", sdu->len, flow);
    // Add SDU to the appropriate TX dlist based on flow
    sys_dlist_append(mac_tx_dlists[flow], &sdu->node);

    printk("[DIAGNOSTIC_TRACE] Step 1: SDU (len %u) for flow %d queued. Queue is now empty: %s\n",
	       sdu->len, flow, sys_dlist_is_empty(mac_tx_dlists[flow]) ? "yes" : "no");
           
	return 0;
}

int dect_mac_api_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    if (ctx->role != MAC_ROLE_FT) {
        LOG_ERR("FT_SEND_API: Not in FT role. Cannot send to specific PT.");
        if (sdu) dect_mac_api_buffer_free(sdu);
        return -EPERM;
    }
    if (sdu == NULL) {
        LOG_ERR("FT_SEND_API: Cannot send NULL SDU.");
        return -EINVAL;
    }
    if (flow >= MAC_FLOW_COUNT) {
        LOG_ERR("FT_SEND_API: Invalid MAC flow ID: %d", flow);
        dect_mac_api_buffer_free(sdu);
        return -EINVAL;
    }
    if (target_pt_short_rd_id == 0 || target_pt_short_rd_id == 0xFFFF) {
        LOG_ERR("FT_SEND_API: Invalid target PT ShortID: 0x%04X", target_pt_short_rd_id);
        dect_mac_api_buffer_free(sdu);
        return -EINVAL;
    }
    if (sdu->len == 0 || sdu->len > CONFIG_DECT_MAC_SDU_MAX_SIZE) {
        LOG_ERR("FT_SEND_API: Invalid SDU length for TX: %u", sdu->len);
        dect_mac_api_buffer_free(sdu);
        return -EMSGSIZE;
    }
    // sdu->target_peer_short_rd_id should already be set by caller or here.
    // Let's enforce it or set it if not already.
    sdu->target_peer_short_rd_id = target_pt_short_rd_id;

    int target_peer_slot_idx = -1;
    for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
        if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
            ctx->role_ctx.ft.connected_pts[i].short_rd_id == target_pt_short_rd_id) {
            target_peer_slot_idx = i;
            break;
        }
    }

    if (target_peer_slot_idx == -1) {
        LOG_ERR("FT_SEND_API: Target PT ShortID 0x%04X not found or not connected.", target_pt_short_rd_id);
        dect_mac_api_buffer_free(sdu);
        return -ENOTCONN;
    }

    LOG_DBG("FT_SEND_API: Queueing SDU (len %u) to PT Slot %d (ShortID 0x%04X), Flow %d",
            sdu->len, target_peer_slot_idx, target_pt_short_rd_id, flow);

    // // Use sys_dlist_append to add the SDU's node to the tail of the per-peer list.
    // sys_dlist_append(target_dlist_ptr, &sdu->node);


    if (!sdu || flow >= MAC_FLOW_COUNT || sdu->len > CONFIG_DECT_MAC_SDU_MAX_SIZE || !target_pt_short_rd_id) {
        return -EINVAL;
    }
    // Check if MAC is in FT role and PT is connected (implementation-specific)
    // Add SDU to TX dlist with target PT info
    sdu->target_peer_short_rd_id = target_pt_short_rd_id;
    // sys_dlist_append(mac_tx_dlists[flow], &sdu->node);
	sys_dlist_t *target_dlist_ptr = NULL;

	switch (flow) {
	case MAC_FLOW_HIGH_PRIORITY:
		target_dlist_ptr =
			&ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx]
				 .high_priority_dlist;
		break;
	case MAC_FLOW_RELIABLE_DATA:
		target_dlist_ptr =
			&ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx]
				 .reliable_data_dlist;
		break;
	case MAC_FLOW_BEST_EFFORT:
		target_dlist_ptr =
			&ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx]
				 .best_effort_dlist;
		break;
	default:
		/* This case is already handled by the check at the top */
		dect_mac_api_buffer_free(sdu);
		return -EINVAL;
	}

	LOG_DBG("FT_SEND_API: Queueing SDU (len %u) to PT Slot %d (ShortID 0x%04X), Flow %d",
		sdu->len, target_peer_slot_idx, target_pt_short_rd_id, flow);

	sys_dlist_append(target_dlist_ptr, &sdu->node);    

    return 0;    
}
dect_mac_role_t dect_mac_get_role(void)
{
	printk("[MAC_API] dect_mac_get_role() Active context is: %p\n",
	       (void *)dect_mac_get_active_context());
	return dect_mac_get_active_context()->role;
}

bool dect_mac_is_associated(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->role == MAC_ROLE_PT) {
		return ctx->role_ctx.pt.associated_ft.is_valid;
	}

	return (ctx->state == MAC_STATE_FT_BEACONING);
}

uint16_t dect_mac_get_short_id_for_long_id(uint32_t long_rd_id)
{
	return dect_mac_core_get_short_id_for_long_id(long_rd_id);
}

uint32_t dect_mac_get_hpc(void)
{
	return dect_mac_get_active_context()->hpc;
}

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
/* --- Public Security API Wrappers --- */
void dect_mac_security_build_iv(uint8_t *iv_out, uint32_t transmitter_long_rd_id,
				uint32_t receiver_long_rd_id, uint32_t hpc, uint16_t psn)
{
	security_build_iv(iv_out, transmitter_long_rd_id, receiver_long_rd_id, hpc, psn);
}

int dect_mac_security_calculate_mic(const uint8_t *pdu_data_for_mic, size_t pdu_data_len,
				    const uint8_t *integrity_key, uint8_t *mic_out_5_bytes)
{
	return security_calculate_mic(pdu_data_for_mic, pdu_data_len, integrity_key,
				      mic_out_5_bytes);
}

int dect_mac_security_crypt_payload(uint8_t *payload_in_out, size_t len,
				    const uint8_t *cipher_key, uint8_t *iv, bool encrypt)
{
	return security_crypt_payload(payload_in_out, len, cipher_key, iv, encrypt);
}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

int dect_mac_api_enter_paging_mode(void)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (ctx->role != MAC_ROLE_PT) {
        LOG_WRN("PAGING_CMD_API: Paging mode request ignored, not in PT role.");
        return -EPERM;
    }
    // Further state checks (e.g., must be associated) will be done by the PT state machine.

    struct dect_mac_event_msg msg = { .type = MAC_EVENT_CMD_ENTER_PAGING_MODE };
    // msg.modem_time_of_event = k_uptime_get(); // Or not strictly needed for CMD events
    int ret = k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("PAGING_CMD_API: Failed to queue CMD_ENTER_PAGING_MODE to MAC thread: %d", ret);
        return -EIO;
    }
    LOG_INF("PAGING_CMD_API: CMD_ENTER_PAGING_MODE queued to MAC thread.");
    return 0;
}

int dect_mac_api_ft_page_pt(uint32_t target_pt_long_rd_id)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->role != MAC_ROLE_FT) {
		LOG_ERR("PAGE_API: Not in FT role.");
		return -EPERM;
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
		    ctx->role_ctx.ft.connected_pts[i].long_rd_id == target_pt_long_rd_id) {
			LOG_INF("PAGE_API: Flagging PT 0x%08X (slot %d) for paging.",
				target_pt_long_rd_id, i);
			ctx->role_ctx.ft.connected_pts[i].paging_pending = true;
			return 0;
		}
	}

	LOG_WRN("PAGE_API: Could not find associated PT with Long ID 0x%08X to page.",
		target_pt_long_rd_id);
	return -ENOTCONN;
}

int dect_mac_release(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->role != MAC_ROLE_PT) {
		LOG_WRN("RELEASE_CMD_API: Release request ignored, not in PT role.");
		return -EPERM;
	}

	if (ctx->state < MAC_STATE_ASSOCIATED) {
		LOG_WRN("RELEASE_CMD_API: Release request ignored, not associated.");
		return -ENOTCONN;
	}

	struct dect_mac_event_msg msg = {.type = MAC_EVENT_CMD_RELEASE_LINK, .ctx = ctx };
	int ret = k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);

	if (ret != 0) {
        printk("RELEASE_CMD_API: Failed to queue CMD_RELEASE_LINK to MAC thread: %d \n",
			ret);
		LOG_ERR("RELEASE_CMD_API: Failed to queue CMD_RELEASE_LINK to MAC thread: %d",
			ret);
		return -EIO;
	}
    printk("[DIAGNOSTIC] CMD_RELEASE_LINK event successfully queued to MAC thread.\n");
	LOG_INF("RELEASE_CMD_API: CMD_RELEASE_LINK queued to MAC thread.");
	return 0;
}