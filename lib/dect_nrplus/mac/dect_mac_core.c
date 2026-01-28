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
static dect_mac_context_t *g_active_mac_ctx;

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
		printk("[PCC_CACHE_TIMEOUT] Stale PCC with TID %u expired. Invalidating entry.\n",
		       transaction->transaction_id);        
		LOG_WRN("PCC_CACHE: Timeout for transaction_id %u. PDC never arrived. Invalidating entry.",
			transaction->transaction_id);
		transaction->is_valid = false;
	}
}


// Global MAC Context Definition
dect_mac_context_t *dect_mac_get_active_context(void)
{
	return g_active_mac_ctx;
}

void dect_mac_test_set_active_context(dect_mac_context_t *ctx)
{
	g_active_mac_ctx = ctx;
    // printk("[MAC_CORE] Setting active context to %p (Role: %s)\n", (void *)ctx,
    //     (ctx && ctx->role == MAC_ROLE_PT) ? "PT" : "FT");
    // printk("[MAC_CORE] dect_mac_test_set_active_context: Context-> State: %d \n", g_active_mac_ctx->state);
    // LOG_INF("Context-> State: %d ", g_active_mac_ctx->state);
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



void dect_mac_core_clear_pending_op(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
    printk("[CORE] Clearing pending_op: %d\n", ctx->pending_op_type);

	if (ctx) {
		if (ctx->pending_op_type != PENDING_OP_NONE) {
			LOG_DBG("MAC_CORE: Clearing pending op (was Type: %s, Hdl: %u)",
				dect_pending_op_to_str(ctx->pending_op_type),
				ctx->pending_op_handle);
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
	}
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

// FT Specific Timer expiry functions
static void ft_beacon_timer_expiry_fn(struct k_timer *timer_id)
{
	printk("[FT_TIMER_CB] FT Beacon Timer Expired. Queueing event.\n");
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
            LOG_WRN("MAC Core: Own HPC wrapped around! Re-initialized to 1.");
        }
        
        dect_mac_nvs_save_hpc(ctx->hpc);

        LOG_INF("MAC Core: Own PSN wrapped, own HPC incremented to %u.", ctx->hpc);
        ctx->send_mac_sec_info_ie_on_next_tx = true;
    }
}


int dect_mac_core_get_peer_slot_idx(uint16_t peer_short_id)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("\n--- DEBUG: dect_mac_core_get_peer_slot_idx ---\n");
	printk("Searching for peer with Short ID: 0x%04X\n", peer_short_id);

	if (ctx->role != MAC_ROLE_FT) {
		printk("  - Role is not FT. Returning -1.\n");
		return -1;
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		printk("  - Checking slot %d: is_valid=%d, short_rd_id=0x%04X\n", i,
		       ctx->role_ctx.ft.connected_pts[i].is_valid,
		       ctx->role_ctx.ft.connected_pts[i].short_rd_id);
		if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
		    ctx->role_ctx.ft.connected_pts[i].short_rd_id == peer_short_id) {
			printk("  - Match found! Returning slot %d.\n", i);
			return i;
		}
	}

	printk("  - No match found. Returning -1.\n");
	return -1;
}

// --- Core Initialization ---
int dect_mac_core_init(dect_mac_role_t role, uint32_t provisioned_long_rd_id)
{
    printk("[CORE_INIT_DBG] Entered dect_mac_core_init.\n");
    dect_mac_context_t *ctx = dect_mac_get_active_context(); // Gets pointer to g_mac_ctx
    printk("[CORE_INIT_DBG] 1. Context retrieved.\n");
    memset(ctx, 0, sizeof(dect_mac_context_t));
    printk("[CORE_INIT_DBG] 2. Context zeroed.\n");

    ctx->role = role;
    LOG_INF("MAC Core: No Long RD ID provisioned. Generating a random one. %d", provisioned_long_rd_id);
    if (provisioned_long_rd_id != 0 && provisioned_long_rd_id != 0xFFFFFFFFU) {
        ctx->own_long_rd_id = provisioned_long_rd_id;
    } else {
		LOG_INF("MAC Core: No Long RD ID provisioned. Generating a random one.");
		// ctx->own_long_rd_id = sys_rand32_get();
        // if (ctx->own_long_rd_id == 0 || ctx->own_long_rd_id == 0xFFFFFFFFU) {
        //     ctx->own_long_rd_id = (sys_rand32_get() & 0xFFFFFFFEU) + 1;
        //     LOG_WRN("MAC Core: Derived/Random Long RD ID was reserved, re-randomized to 0x%08X", ctx->own_long_rd_id);
        // }
		do {
			dect_mac_rand_get((uint8_t *)&ctx->own_long_rd_id,
					  sizeof(ctx->own_long_rd_id));
		} while (ctx->own_long_rd_id == 0 || ctx->own_long_rd_id == 0xFFFFFFFFU);
    }

    // Generate Short RD ID (random, not 0x0000 or 0xFFFF)
	do {
		dect_mac_rand_get((uint8_t *)&ctx->own_short_rd_id,
				  sizeof(ctx->own_short_rd_id));
	} while (ctx->own_short_rd_id == 0x0000 || ctx->own_short_rd_id == 0xFFFF);

    // Derive Network ID (ETSI TS 103 636-4, 4.2.3.1)
    // "first 24 MSB bits are used to identify a DECT-2020 network... The 8 LSB bits...are selected locally"
    // "neither the 8 LSB bits are 0x00 nor the 24 MSB bits are 0x000000."
    // Example: Use a fixed vendor part for MS24, LSB from Long RD ID.
    uint32_t net_id_ms24_part = CONFIG_DECT_MAC_NETWORK_ID_MS24_PREFIX; // From Kconfig
    if (net_id_ms24_part == 0x000000) net_id_ms24_part = 0x000001; // Ensure not all zeros

    uint8_t  net_id_ls8_part = (uint8_t)(ctx->own_long_rd_id & 0xFF);
    if (net_id_ls8_part == 0x00) net_id_ls8_part = 0x01; // Ensure not all zeros

    ctx->network_id_32bit = (net_id_ms24_part << 8) | net_id_ls8_part;

    LOG_INF("MAC Core Init: Role %s, LongID 0x%08X, ShortID 0x%04X, NetID 0x%08X",
            (role == MAC_ROLE_PT) ? "PT" : "FT",
            ctx->own_long_rd_id, ctx->own_short_rd_id, ctx->network_id_32bit);

    // Initialize own primary PHY parameters
    ctx->own_phy_params.is_valid = true; // Mark as valid once set from config
    ctx->own_phy_params.mu = CONFIG_DECT_MAC_OWN_MU_CODE;
    ctx->own_phy_params.beta = CONFIG_DECT_MAC_OWN_BETA_CODE;
    LOG_INF("MAC Core: Own PHY Params -> mu_code: %u (val 2^%u), beta_code: %u (val %u)",
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
    printk("MAC_CORE_INIT (FT): Loaded ft_cluster_beacon_period_ms from Kconfig: %u",
            ctx->config.ft_cluster_beacon_period_ms);
#endif
    ctx->config.max_assoc_retries = MAX_RACH_ATTEMPTS_CONFIG;
    ctx->config.ft_policy_secure_on_assoc = IS_ENABLED(CONFIG_DECT_MAC_FT_SECURE_ON_ASSOC);
    ctx->config.default_tx_power_code = DEFAULT_TX_POWER_CODE;
    ctx->config.default_data_mcs_code = CONFIG_DECT_MAC_DEFAULT_DATA_MCS;


    // Placeholder PHY latencies (will be updated from PHY via dect_mac_phy_if.c)
    memset(&ctx->phy_latency, 0, sizeof(dect_phy_latency_values_t));


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

        // k_fifo_init(&ctx->role_ctx.pt.handover_tx_holding_fifo);
        sys_dlist_init(&ctx->role_ctx.pt.handover_tx_holding_dlist);

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
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.sfn_validity_present = true;
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_field_present = true; // Advertise RACH channel
        ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_abs_num = ctx->role_ctx.ft.operating_carrier;



		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
            memset(&ctx->role_ctx.ft.connected_pts[i], 0,
			       sizeof(dect_mac_peer_info_t));
			sys_dlist_init(
				&ctx->role_ctx.ft.peer_tx_data_dlists[i].high_priority_dlist);
			sys_dlist_init(
				&ctx->role_ctx.ft.peer_tx_data_dlists[i].reliable_data_dlist);
			sys_dlist_init(
				&ctx->role_ctx.ft.peer_tx_data_dlists[i].best_effort_dlist);
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
    ctx->ft_sfn_zero_modem_time_anchor = 0;
    ctx->current_sfn_at_anchor_update = 0;
    ctx->last_phy_op_end_time = 0;

    // Initialize security context
    dect_mac_nvs_init();
	// psa_generate_random((uint8_t *)&ctx->psn, sizeof(ctx->psn));
    dect_mac_rand_get((uint8_t *)&ctx->psn, sizeof(ctx->psn));
	ctx->psn &= 0x0FFF; /* Ensure it's a 12-bit value */

    ctx->hpc = dect_mac_nvs_get_hpc();
    ctx->master_psk_provisioned = false; // Will be set if PSK loaded
    ctx->keys_provisioned = false;
    ctx->current_key_index = 0;
    ctx->send_mac_sec_info_ie_on_next_tx = false;

    printk("[CORE_INIT_DBG] State is now set to: %s (%d)\n", dect_mac_state_to_str(ctx->state), ctx->state);
	/* The master_psk is loaded from Kconfig below. No hardcoded fallback should exist. */

    // Load PSK from Kconfig
    ctx->master_psk_provisioned = false;
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
printk("Loading PSK Security key \n");
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
			LOG_INF("MAC Core: Master PSK loaded from Kconfig.");
			LOG_HEXDUMP_DBG(ctx->master_psk, sizeof(ctx->master_psk), "PSK Val:");
		} else {
			LOG_ERR("MAC Core: Failed to convert Kconfig PSK_HEX. Security will be impaired.");
		}
	} else if (psk_hex_len > 0) {
		LOG_ERR("MAC Core: Kconfig PSK_HEX has invalid length %zu (expected 32). Security will be impaired.", psk_hex_len);
	} else {
		LOG_WRN("MAC Core: Kconfig PSK_HEX is empty. No PSK provisioned.");
	}
#else
	LOG_INF("MAC Core: MAC Security is disabled. PSK not loaded.");
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

printk("MAC CORE: **** network_id_32bit, own_long_rd_id (0x%X vs our 0x%X).\n",
                    ctx->network_id_32bit, ctx->own_long_rd_id);

    printk("MAC Core Context Initialized. PSN=0x%03X, OwnHPC=%u \n", ctx->psn, ctx->hpc);
    LOG_INF("MAC Core Context Initialized. PSN=0x%03X, OwnHPC=%u", ctx->psn, ctx->hpc);
    return 0;
}




void print_first_bytes_uint32(uint32_t value)
{
    const uint8_t *bytes = (const uint8_t *)&value;
    printk("First 4 bytes of uint32_t (0x%08x): ", value);
    for (size_t i = 0; i < 4; i++) {
        printk("%02x ", bytes[i]);
    }
    printk("\n");
};

void print_first_bytes_uint16(uint16_t value)
{
    const uint8_t *bytes = (const uint8_t *)&value;
    printk("First 4 bytes of uint16_t (0x%08x): ", value);
    for (size_t i = 0; i < 4; i++) {
        printk("%02x ", bytes[i]);
    }
    printk("\n");
};



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