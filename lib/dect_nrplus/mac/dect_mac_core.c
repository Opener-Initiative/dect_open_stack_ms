/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* dect_mac/dect_mac_core.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/sys/util.h> // For ARRAY_SIZE, MIN, MAX if needed
// #include <zephyr/autoconf.h>
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
#include <zephyr/crypto/crypto.h>
#include "psa/crypto.h"
#endif

#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_types.h>
#include <mac/dect_mac_random.h>
#include <mac/dect_mac_context.h>   // For dect_mac_context_t structure and sub-types
#include <mac/dect_mac_sm.h>        // For dect_mac_role_t, event types for timers
#include <mac/dect_mac_data_path.h> // For dect_mac_data_path_init and HARQ timer callback
#include <mac/dect_mac_sm_pt.h>     // For PT timer callback function extern declarations
#include <mac/dect_mac_sm_ft.h>     // For FT timer callback function extern declarations
#include <mac/dect_mac_main_dispatcher.h> // For mac_event_msgq (external)
#include <mac/dect_mac_nvs.h>


LOG_MODULE_REGISTER(dect_mac_core, CONFIG_DECT_MAC_CORE_LOG_LEVEL);

// static __thread dect_mac_context_t *g_active_mac_ctx;
static dect_mac_context_t g_mac_ctx;
static dect_mac_context_t *g_active_mac_ctx = &g_mac_ctx;
static struct k_spinlock g_active_mac_ctx_lock;

// External message queue (defined in dect_mac_phy_if.c, used by timer handlers here)
extern struct k_msgq mac_event_msgq;


/**
 * @brief Timer callback for expiring a PCC cache entry if its PDC doesn't arrive.
 */
static void pcc_cache_timeout_handler(struct k_timer *timer_id)
{
	/* The timer is a member of the struct, so we can get the parent struct's address */
	pcc_transaction_t *transaction =
		CONTAINER_OF(timer_id, pcc_transaction_t, timeout_timer);

	if (transaction->is_valid) {
		LOG_WRN("PCC_CACHE: Timeout for transaction_id %u. PDC never arrived. Invalidating entry.",
			transaction->transaction_id);
		transaction->is_valid = false;
	}
}


// Global MAC Context Definition
dect_mac_context_t *dect_mac_get_active_context(void)
{
	k_spinlock_key_t key = k_spin_lock(&g_active_mac_ctx_lock);
	dect_mac_context_t *ctx = g_active_mac_ctx;
	k_spin_unlock(&g_active_mac_ctx_lock, key);
	return ctx;
}

void dect_mac_test_set_active_context(dect_mac_context_t *ctx)
{
	k_spinlock_key_t key = k_spin_lock(&g_active_mac_ctx_lock);
	g_active_mac_ctx = ctx;
	k_spin_unlock(&g_active_mac_ctx_lock, key);
}

dect_mac_state_change_cb_t g_state_change_cb = NULL;

// void dect_mac_register_state_change_cb(dect_mac_state_change_cb_t cb)
// {
// 	g_state_change_cb = cb;
// }

void dect_mac_core_register_state_change_cb(dect_mac_state_change_cb_t cb)
{
	g_state_change_cb = cb;
}



void dect_mac_reset_context(dect_mac_context_t *ctx)
{
	if (!ctx) return;

	LOG_DBG("Resetting MAC context (Role: %s)", 
		(ctx->role == MAC_ROLE_PT ? "PT" : "FT"));

	k_spinlock_key_t key = k_spin_lock(&ctx->lock);

	/* 1. Stop common timers */
	k_timer_stop(&ctx->rach_context.rach_backoff_timer);
	k_timer_stop(&ctx->rach_context.rach_response_window_timer);

	/* 2. Stop role-specific timers and cleanup peer info */
	if (ctx->role == MAC_ROLE_PT) {
		k_timer_stop(&ctx->role_ctx.pt.beacon_listen_timer);
		k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
		k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);
		k_timer_stop(&ctx->role_ctx.pt.paging_cycle_timer);
		k_timer_stop(&ctx->role_ctx.pt.reject_timer);
		k_timer_stop(&ctx->role_ctx.pt.auth_timeout_timer);
		
		/* Reset PT contexts */
		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		memset(&ctx->role_ctx.pt.associated_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.rejected_ft_long_rd_id = 0;
		ctx->role_ctx.pt.current_assoc_retries = 0;

		/* Reset statistics */
		ctx->role_ctx.pt.beacon_rx_count = 0;
		ctx->role_ctx.pt.assoc_attempt_count = 0;
		ctx->role_ctx.pt.rach_tx_count = 0;
	} else { /* FT Role */
		k_timer_stop(&ctx->role_ctx.ft.beacon_timer);
		
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			k_timer_stop(&ctx->role_ctx.ft.connected_pts[i].link_supervision_timer);
			/* Hard reset of peer slots */
			memset(&ctx->role_ctx.ft.connected_pts[i], 0, sizeof(dect_mac_peer_info_t));
			ctx->role_ctx.ft.keys_provisioned_for_peer[i] = false;
		}

		/* Reset statistics */
		ctx->role_ctx.ft.beacon_tx_count = 0;
	}

	/* 3. Stop PCC cache timers */
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		k_timer_stop(&ctx->pcc_transaction_cache[i].timeout_timer);
		ctx->pcc_transaction_cache[i].is_valid = false;
	}

	/* 4. Stop HARQ retransmission timers */
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		k_timer_stop(&ctx->harq_tx_processes[i].retransmission_timer);
		ctx->harq_tx_processes[i].is_active = false;
	}

	/* 4. Reset sequence numbers and security flags */
	ctx->psn = 0;
	ctx->hpc = 1; 
	ctx->keys_provisioned = false;
	ctx->send_mac_sec_info_ie_on_next_tx = false;

	ctx->security_enabled = IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE);
	ctx->consecutive_mic_failures = 0;
	
	/* 5. Clear pending operations */
	ctx->pending_op_type = PENDING_OP_NONE;
	ctx->pending_op_handle = 0;
	ctx->last_known_modem_time = 0;
	ctx->last_phy_op_end_time = 0;

	k_spin_unlock(&ctx->lock, key);
}

void dect_mac_core_clear_pending_op(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (!ctx) return;
    
	k_spinlock_key_t key = k_spin_lock(&ctx->lock);
		// LOG_DBG("[CORE] Clearing pending_op: %d", ctx->pending_op_type);
		if (ctx->pending_op_type != PENDING_OP_NONE) {
			LOG_DBG("MAC_CORE: Clearing pending op (was Type: %s, Hdl: %u)",
				dect_pending_op_to_str(ctx->pending_op_type),
				ctx->pending_op_handle);
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
		k_spin_unlock(&ctx->lock, key);
}



// --- Timer Expiry Function Prototypes (actual handlers in SM or DataPath files) ---
// These are the functions that k_timer will call upon expiry.
// They typically just queue an event to the main MAC thread.

// Generic RACH Timer expiry functions (PT SM will react to the queued events)
static void rach_backoff_timer_expiry_fn(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id); // We use global context or pass ID via user_data if needed
    struct dect_mac_event_msg msg = {
	    .ctx = dect_mac_get_active_context(), /* RACH is modal, uses active context */
	    .type = MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF
    };
    // msg.modem_time_of_event = k_uptime_get(); // Example
    if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("RACH_BKO_TMR: Failed to queue expiry event.");
    }
}

static void rach_response_window_timer_expiry_fn(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id);
    struct dect_mac_event_msg msg = {
	    .ctx = dect_mac_get_active_context(), /* RACH is modal, uses active context */
	    .type = MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW
    };

    if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("RACH_RESP_TMR: Failed to queue expiry event.");
    }
}

// PT Specific Timer expiry functions
static void pt_keep_alive_timer_expiry_fn(struct k_timer *timer_id) {
	struct dect_mac_context *ctx = timer_id->user_data;
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = MAC_EVENT_TIMER_EXPIRED_KEEPALIVE
	};
    // struct dect_mac_event_msg msg = {
	//     .ctx = dect_mac_get_active_context(),
	//     .type = MAC_EVENT_TIMER_EXPIRED_KEEPALIVE
    // };

    if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("PT_KA_TMR: Failed to queue expiry event.");
    }
}

static void pt_mobility_scan_timer_expiry_fn(struct k_timer *timer_id) {
	struct dect_mac_context *ctx = timer_id->user_data;
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN
	};

    if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("PT_MOB_TMR: Failed to queue expiry event.");
    }
}

static void pt_beacon_listen_timer_expiry_fn(struct k_timer *timer_id)
{
	struct dect_mac_context *ctx = timer_id->user_data;
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = MAC_EVENT_TIMER_EXPIRED_BEACON_LISTEN
	};

	if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
		LOG_ERR("PT_BEACON_LSN_TMR: Failed to queue expiry event.");
	}
}

static void pt_auth_timeout_timer_expiry_fn(struct k_timer *timer_id)
{
	struct dect_mac_context *ctx = timer_id->user_data;
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = MAC_EVENT_TIMER_EXPIRED_AUTH_TIMEOUT
	};

	if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
		LOG_ERR("PT_AUTH_TMO_TMR: Failed to queue expiry event.");
	}
}

// FT Specific Timer expiry functions
static void ft_beacon_timer_expiry_fn(struct k_timer *timer_id)
{
	LOG_DBG("[FT_TIMER_CB] FT Beacon Timer Expired. Queueing event.");
	struct dect_mac_context *ctx = timer_id->user_data;
	struct dect_mac_event_msg msg = {
		.ctx = ctx,
		.type = MAC_EVENT_TIMER_EXPIRED_BEACON
	};

	// if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
    int err = k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);
    if (err){
		LOG_ERR("FT_BCN_TMR: Failed to queue expiry event(%d).", err);
	}
}

// --- PSN and HPC Management ---
void increment_psn_and_hpc(dect_mac_context_t *ctx)
{
    if (!ctx) return;
    ctx->psn = (ctx->psn + 1) & 0x0FFF; // 12-bit PSN wraps from 4095 to 0
    if (ctx->psn == 0) {
        ctx->hpc = (ctx->hpc + 1);
        if (ctx->hpc == 0) { // HPC wrapped around (32-bit)
            ctx->hpc = 1; // Re-initialize (ETSI IV must not be all zeros if derived from HPC=0, PSN=0)
            LOG_WRN("Own HPC wrapped around! Re-initialized to 1.");
        }
        
        dect_mac_nvs_save_hpc(ctx->hpc);

        LOG_INF("Own PSN wrapped, own HPC incremented to %u.", ctx->hpc);
        ctx->send_mac_sec_info_ie_on_next_tx = true;
    }
}


int dect_mac_core_get_peer_slot_idx(uint16_t peer_short_id)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	LOG_DBG("--- DEBUG: dect_mac_core_get_peer_slot_idx ---");
	LOG_DBG("Searching for peer with Short ID: 0x%04X", peer_short_id);

	if (ctx->role != MAC_ROLE_FT) {
		LOG_ERR("  - Role is not FT. Returning -1.");
		return -1;
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		LOG_DBG("  - Checking slot %d: is_valid=%d, short_rd_id=0x%04X", i,
		       ctx->role_ctx.ft.connected_pts[i].is_valid,
		       ctx->role_ctx.ft.connected_pts[i].short_rd_id);
		if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
		    ctx->role_ctx.ft.connected_pts[i].short_rd_id == peer_short_id) {
			LOG_DBG("  - Match found! Returning slot %d.", i);
			return i;
		}
	}

	LOG_DBG("  - No match found. Returning -1.");
	return -1;
}

// --- Core Initialization ---
int dect_mac_core_init(dect_mac_role_t role, uint32_t provisioned_long_rd_id)
{
    LOG_DBG("[CORE_INIT_DBG] Entered dect_mac_core_init.");
    dect_mac_context_t *ctx = dect_mac_get_active_context(); // Gets pointer to g_mac_ctx
    LOG_DBG("[CORE_INIT_DBG] 1. Context retrieved.");
    
    // Save PHY latency values populated asynchronously by the modem INIT event
    dect_phy_latency_values_t saved_latency = ctx->phy_latency;
    
    memset(ctx, 0, sizeof(dect_mac_context_t));
    LOG_DBG("[CORE_INIT_DBG] 2. Context zeroed.");
    
    // Restore PHY latency values
    ctx->phy_latency = saved_latency;

	/* Note: Spinlocks (ctx->lock, ctx->harq_lock) are already zeroed by memset above, 
	 * which is a valid initial state (unlocked, no owner) for Zephyr spinlocks.
	 */

    ctx->role = role;
    if (provisioned_long_rd_id != 0 && provisioned_long_rd_id != 0xFFFFFFFFU) {
        ctx->own_long_rd_id = provisioned_long_rd_id;
    } else {
		LOG_INF("No Long RD ID provisioned. Generating a random one.");
		// ctx->own_long_rd_id = sys_rand32_get();
        // if (ctx->own_long_rd_id == 0 || ctx->own_long_rd_id == 0xFFFFFFFFU) {
        //     ctx->own_long_rd_id = (sys_rand32_get() & 0xFFFFFFFEU) + 1;
        //     LOG_WRN("Derived/Random Long RD ID was reserved, re-randomized to 0x%08X", ctx->own_long_rd_id);
        // }
		do {
			dect_mac_rand_get((uint8_t *)&ctx->own_long_rd_id,
					  sizeof(ctx->own_long_rd_id));
		} while (ctx->own_long_rd_id == 0 || ctx->own_long_rd_id == 0xFFFFFFFFU);
    }

    // Generate Short RD ID (deterministic if long_id is provided, otherwise random)
    if (provisioned_long_rd_id != 0 && provisioned_long_rd_id != 0xFFFFFFFFU) {
        ctx->own_short_rd_id = (uint16_t)(provisioned_long_rd_id & 0xFFFF);
        if (ctx->own_short_rd_id == 0x0000 || ctx->own_short_rd_id == 0xFFFF) {
             ctx->own_short_rd_id = 0x3344; // Fallback to a safe non-zero value
        }
    } else {
        do {
            dect_mac_rand_get((uint8_t *)&ctx->own_short_rd_id,
                      sizeof(ctx->own_short_rd_id));
        } while (ctx->own_short_rd_id == 0x0000 || ctx->own_short_rd_id == 0xFFFF);
    }

    // Derive Network ID (ETSI TS 103 636-4, 4.2.3.1)
    if (role == MAC_ROLE_FT) {
        uint32_t net_id_ms24_part = CONFIG_DECT_MAC_NETWORK_ID_MS24_PREFIX;
        if (net_id_ms24_part == 0x000000) net_id_ms24_part = 0x000001;

        uint8_t net_id_ls8_part = (uint8_t)(ctx->own_long_rd_id & 0xFF);
        if (net_id_ls8_part == 0x00) net_id_ls8_part = 0x01;

        ctx->network_id_32bit = (net_id_ms24_part << 8) | net_id_ls8_part;
    } else {
        /* PT starts with no network ID (will learn from beacon or be provisioned later) */
        ctx->network_id_32bit = 0;
    }

    LOG_INF("MAC Core Init: Role %s, LongID 0x%08X, ShortID 0x%04X, NetID 0x%08X",
            (role == MAC_ROLE_PT) ? "PT" : "FT",
            ctx->own_long_rd_id, ctx->own_short_rd_id, ctx->network_id_32bit);

    // Initialize own primary PHY parameters
    ctx->own_phy_params.is_valid = true; // Mark as valid once set from config
    ctx->own_phy_params.mu = CONFIG_DECT_MAC_OWN_MU_CODE;
    ctx->own_phy_params.beta = CONFIG_DECT_MAC_OWN_BETA_CODE;
    LOG_INF("Own PHY Params -> mu_code: %u (val 2^%u), beta_code: %u (val %u)",
            ctx->own_phy_params.mu, ctx->own_phy_params.mu,
            ctx->own_phy_params.beta, ctx->own_phy_params.beta + 1);

    // Default MAC configurations
    ctx->config.rssi_threshold_min_dbm = CONFIG_DECT_MAC_RSSI_THR_MIN_DBM;
    ctx->config.rssi_threshold_max_dbm = CONFIG_DECT_MAC_RSSI_THR_MAX_DBM;
    ctx->config.rach_cw_min_idx = CONFIG_DECT_MAC_RACH_CW_MIN_IDX;
    ctx->config.rach_cw_max_idx = CONFIG_DECT_MAC_RACH_CW_MAX_IDX;
    ctx->config.rach_response_window_ms = CONFIG_DECT_MAC_RACH_RESP_WIN_MS;
    ctx->config.keep_alive_period_ms = CONFIG_DECT_MAC_PT_KEEP_ALIVE_MS;
#if IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)
    ctx->config.mobility_scan_interval_ms = CONFIG_DECT_MAC_PT_MOBILITY_SCAN_MS;
#else
    ctx->config.mobility_scan_interval_ms = 100000;
#endif

#if IS_ENABLED(CONFIG_DECT_MAC_ROLE_FT)
    ctx->config.ft_cluster_beacon_period_ms = CONFIG_DECT_MAC_FT_CLUSTER_BEACON_MS;
    ctx->config.ft_network_beacon_period_ms = CONFIG_DECT_MAC_FT_NETWORK_BEACON_MS;
    LOG_DBG("MAC_CORE_INIT (FT): Loaded ft_cluster_beacon_period_ms from Kconfig: %u",
            ctx->config.ft_cluster_beacon_period_ms);
#endif
    ctx->config.max_assoc_retries = MAX_RACH_ATTEMPTS_CONFIG;
    ctx->config.ft_policy_secure_on_assoc = IS_ENABLED(CONFIG_DECT_MAC_FT_SECURE_ON_ASSOC);
    ctx->config.default_tx_power_code = DEFAULT_TX_POWER_CODE;
    ctx->config.default_data_mcs_code = CONFIG_DECT_MAC_DEFAULT_DATA_MCS;


    // Placeholder PHY latencies (will be updated from PHY via dect_mac_phy_if.c)
    // memset(&ctx->phy_latency, 0, sizeof(dect_phy_latency_values_t));


    // Initialize common RACH context and timers
    k_timer_init(&ctx->rach_context.rach_response_window_timer, rach_response_window_timer_expiry_fn, NULL);
    k_timer_init(&ctx->rach_context.rach_backoff_timer, rach_backoff_timer_expiry_fn, NULL);
    ctx->rach_context.rach_cw_current_idx = ctx->config.rach_cw_min_idx;

    // Initialize HARQ processes (done by data_path_init)
    dect_mac_data_path_init();

	/* Initialize the PCC transaction cache and its timers */
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		ctx->pcc_transaction_cache[i].is_valid = false;
		k_timer_init(&ctx->pcc_transaction_cache[i].timeout_timer,
			     pcc_cache_timeout_handler, NULL);
	}    

    // Initialize role-specific contexts and timers
    if (role == MAC_ROLE_PT) {
        memset(&ctx->role_ctx.pt, 0, sizeof(pt_context_t));
        k_timer_init(&ctx->role_ctx.pt.beacon_listen_timer, pt_beacon_listen_timer_expiry_fn, NULL);
        ctx->role_ctx.pt.beacon_listen_timer.user_data = ctx;
        k_timer_init(&ctx->role_ctx.pt.keep_alive_timer, pt_keep_alive_timer_expiry_fn, NULL);
        ctx->role_ctx.pt.keep_alive_timer.user_data = ctx;
        k_timer_init(&ctx->role_ctx.pt.mobility_scan_timer, pt_mobility_scan_timer_expiry_fn, NULL);
        ctx->role_ctx.pt.mobility_scan_timer.user_data = ctx;
        
        k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer); /* Stop any keep alive timers */
        k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer); /* Stop any mobility scan timers */
        k_timer_init(&ctx->role_ctx.pt.reject_timer, NULL, NULL); /* Expiry is checked by status */
        k_timer_init(&ctx->role_ctx.pt.auth_timeout_timer, pt_auth_timeout_timer_expiry_fn, NULL);
        ctx->role_ctx.pt.auth_timeout_timer.user_data = ctx;

        // k_fifo_init(&ctx->role_ctx.pt.handover_tx_holding_fifo);
        k_queue_init(&ctx->role_ctx.pt.handover_tx_holding_queue);

        // Initialize other PT specific fields if needed
    } else { // MAC_ROLE_FT
        memset(&ctx->role_ctx.ft, 0, sizeof(ft_context_t));
        k_timer_init(&ctx->role_ctx.ft.beacon_timer, ft_beacon_timer_expiry_fn, NULL);
        ctx->role_ctx.ft.beacon_timer.user_data = ctx;
        ctx->role_ctx.ft.operating_carrier = CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CHANNEL; // Initial, until DCS selects one

        // Populate FT's advertised RACH parameters from its config for beacons
        // These use codes/indices directly as per IE definitions, not calculated ms/slot values
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.cwmin_sig_code = ctx->config.rach_cw_min_idx;
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.cwmax_sig_code = ctx->config.rach_cw_max_idx;
        uint32_t resp_win_subslots = (ctx->config.rach_response_window_ms * MAX_SUBSLOTS_IN_FRAME_NOMINAL) / FRAME_DURATION_MS_NOMINAL;
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.response_window_subslots_val_minus_1 = (resp_win_subslots > 0) ? (resp_win_subslots - 1) : 4; /* Default if 0 */
        // Other advertised_rach_params fields (start_subslot_index, num_subslots_or_slots, repetition_code, validity_frames etc.)
        // should be set by FT scheduler/DCS logic before first beacon. Example defaults:
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.start_subslot_index = 10; // Example
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.num_subslots_or_slots = 4;   // Example: 4 subslots for RACH
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units = 7; /* N-1 coded -> 8 units */
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.repetition_code = 0;       // Example: Repeat every frame (code 0 for 1)
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.validity_frames = 200;     // Example: Valid for 200 frames (~2s)		
        // ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.validity_frames = 0xFF;   // Unlimited: FT perpetually owns its RACH resource
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.sfn_validity_present = true;
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_field_present = true; // Advertise RACH channel
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_abs_num = ctx->role_ctx.ft.operating_carrier;



		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
            memset(&ctx->role_ctx.ft.connected_pts[i], 0,
			       sizeof(dect_mac_peer_info_t));
			k_queue_init(
				&ctx->role_ctx.ft.peer_tx_data_queues[i].high_priority_queue);
			k_queue_init(
				&ctx->role_ctx.ft.peer_tx_data_queues[i].reliable_data_queue);
			k_queue_init(
				&ctx->role_ctx.ft.peer_tx_data_queues[i].best_effort_queue);
			ctx->role_ctx.ft.keys_provisioned_for_peer[i] = false;
		}        
		// /* Register the FT's scheduler function with the data path service */
		// dect_mac_data_path_register_scheduler_hook(ft_service_schedules);        
    }

	/* Initialize the PCC transaction cache and its timers */
	for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
		ctx->pcc_transaction_cache[i].is_valid = false;
		k_timer_init(&ctx->pcc_transaction_cache[i].timeout_timer,
			     pcc_cache_timeout_handler, NULL);
	}    
    // Initialize general MAC state
    ctx->state = MAC_STATE_IDLE;
    ctx->pending_op_type = PENDING_OP_NONE;
    ctx->pending_op_handle = 0;
    ctx->last_known_modem_time = 0;
    ctx->ft_sfn0_modem_time_anchor = 0;
    ctx->current_sfn_at_anchor_update = 0;
    ctx->last_phy_op_end_time = 0;

    // Initialize security context
    dect_mac_nvs_init();
	LOG_INF("NVS initialized.");
	// psa_generate_random((uint8_t *)&ctx->psn, sizeof(ctx->psn));
    dect_mac_rand_get((uint8_t *)&ctx->psn, sizeof(ctx->psn));
	ctx->psn &= 0x0FFF; /* Ensure it's a 12-bit value */

    ctx->hpc = dect_mac_nvs_get_hpc();
    ctx->master_psk_provisioned = false; // Will be set if PSK loaded
    ctx->keys_provisioned = false;
    ctx->current_key_index = 0;
    ctx->send_mac_sec_info_ie_on_next_tx = false;

    LOG_DBG("[CORE_INIT_DBG] State is now set to: %s (%d)", dect_mac_state_to_str(ctx->state), ctx->state);
	/* The master_psk is loaded from Kconfig below. No hardcoded fallback should exist. */

    // Load PSK from Kconfig
    ctx->master_psk_provisioned = false;
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
LOG_INF("Loading PSK Security key");
	const char *psk_hex_str = CONFIG_DECT_MAC_MASTER_PSK_HEX;
	size_t psk_hex_len = strlen(psk_hex_str);

	if (psk_hex_len == 32) {
		size_t psk_bin_len;
		int ret = hex2bin(psk_hex_str, psk_hex_len, ctx->master_psk, sizeof(ctx->master_psk));
		if (ret < 0) {
			LOG_ERR("Failed to convert hex PSK to binary: %d", ret);
			return ret;
		}
		psk_bin_len = psk_hex_len / 2;

		if (psk_bin_len == 16) {
			ctx->master_psk_provisioned = true;
			LOG_INF("Master PSK loaded from Kconfig.");
			LOG_HEXDUMP_DBG(ctx->master_psk, sizeof(ctx->master_psk), "PSK Val:");
		} else {
			LOG_ERR("Failed to convert Kconfig PSK_HEX. Security will be impaired.");
		}
	} else if (psk_hex_len > 0) {
		LOG_ERR("Kconfig PSK_HEX has invalid length %zu (expected 32). Security will be impaired.", psk_hex_len);
	} else {
		LOG_WRN("Kconfig PSK_HEX is empty. No PSK provisioned.");
	}
#else
	LOG_INF("MAC Security is disabled. PSK not loaded.");
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

	LOG_DBG("**** network_id_32bit, own_long_rd_id (0x%X vs our 0x%X).",
						ctx->network_id_32bit, ctx->own_long_rd_id);
	LOG_INF("MAC Core Context Initialized. PSN=0x%03X, OwnHPC=%u", ctx->psn, ctx->hpc);
    return 0;
}




// void print_first_bytes_uint32(uint32_t value)
// {
//     const uint8_t *bytes = (const uint8_t *)&value;
//     LOG_DBG("First 4 bytes of uint32_t (0x%08x): ", value);
//     for (size_t i = 0; i < 4; i++) {
//         LOG_DBG("%02x ", bytes[i]);
//     }
//     LOG_DBG("\n");
// };

// void print_first_bytes_uint16(uint16_t value)
// {
//     const uint8_t *bytes = (const uint8_t *)&value;
//     LOG_DBG("First 4 bytes of uint16_t (0x%08x): ", value);
//     for (size_t i = 0; i < 4; i++) {
//         LOG_DBG("%02x ", bytes[i]);
//     }
//     LOG_DBG("\n");
// };



uint16_t dect_mac_core_get_short_id_for_long_id(uint32_t long_rd_id)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->role == MAC_ROLE_FT) {
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
			    ctx->role_ctx.ft.connected_pts[i].long_rd_id == long_rd_id) {
				return ctx->role_ctx.ft.connected_pts[i].short_rd_id;
			}
		}
	} else { /* PT Role */
		if (ctx->role_ctx.pt.associated_ft.is_valid &&
		    ctx->role_ctx.pt.associated_ft.long_rd_id == long_rd_id) {
			return ctx->role_ctx.pt.associated_ft.short_rd_id;
		}
	}

	return 0; /* Not found */
}