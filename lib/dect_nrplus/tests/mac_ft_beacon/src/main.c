/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* tests/mac_ft_beacon/src/main.c */
// Overview: A focused Ztest suite to validate the FT's initialization sequence, from DCS scan to beaconing.
// Uses the robust, multi-device simulation harness.

#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/kernel.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include "../../tests/utils/test_harness_helpers.h"

/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

static dect_mac_context_t g_mac_ctx_ft;
static mock_phy_context_t g_phy_ctx_ft;

/* Empty peer list for isolation */
static mock_phy_context_t *ft_peers[] = { NULL };

#if IS_ENABLED(CONFIG_ZTEST)
extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					       dect_mac_event_type_t event_type, int timer_id);
#endif

/* --- Simulation Harness --- */

static void process_all_mac_events(void)
{
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
		mock_phy_set_active_context(&g_phy_ctx_ft);
		dect_mac_test_set_active_context(msg.ctx);
		dect_mac_event_dispatch(&msg);
	}

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_mac_service();
}

static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
    uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
    mock_phy_context_t *all_phys[] = { &g_phy_ctx_ft };
    uint32_t iteration_count = 0;
    
    while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
        iteration_count++;
        if (iteration_count > 20000) break;
        
        if (break_cond_func && break_cond_func()) return true;
        
        uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 1);
        
        /* Check FT beacon timer */
        uint64_t next_timer_ticks = K_TICKS_FOREVER;
        uint64_t remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
        if (remaining > 0) {
            next_timer_ticks = remaining;
        }
        
        uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
            UINT64_MAX : current_time_us + k_ticks_to_us_floor64(next_timer_ticks);
        
        uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
        uint64_t time_to_advance_us;
        
        if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
            time_to_advance_us = MIN(1000, end_time_us - current_time_us);
        } else if (next_event_time > current_time_us) {
            time_to_advance_us = next_event_time - current_time_us;
        } else {
            time_to_advance_us = 1;
        }
        
        if (time_to_advance_us > 0) {
            k_sleep(K_USEC(time_to_advance_us));
        }
        
        current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
        process_all_mac_events();
    }
    
    return (break_cond_func && break_cond_func());
}

/* --- Test Conditions --- */
// static bool is_ft_scanning(void) { return g_mac_ctx_ft.state == MAC_STATE_FT_SCANNING; }
static bool is_ft_beaconing(void) { return g_mac_ctx_ft.state == MAC_STATE_FT_BEACONING; }
static bool beacon_captured(void) { return g_last_tx_pdu_len_capture > 0; }

/* --- Test Setup --- */

static void *dect_ft_beacon_setup(void)
{
	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	return NULL;
}

static void dect_ft_beacon_before(void *fixture)
{
    ARG_UNUSED(fixture);
    dect_mac_phy_if_reset_handle_map();
    memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
    mock_phy_complete_reset(&g_phy_ctx_ft);
    mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, 0);
    
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    zassert_ok(dect_mac_core_init(MAC_ROLE_FT, 0x11223344), "FT dect_mac_core_init failed");
    
    g_last_tx_pdu_len_capture = 0;
    memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
}

/* --- Test Cases --- */

ZTEST(dect_ft_beacon, test_ft_scans_and_enters_beaconing)
{
	printk("\n--- RUNNING TEST: %s ---\n", __func__);

	/* 1. Activate & Start FT. This should schedule the first RSSI scan. */
	printk("\n--> Step 1: Activating and starting FT...\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	dect_mac_start();
	process_all_mac_events();

	zassert_equal(g_mac_ctx_ft.state, MAC_STATE_FT_SCANNING, "FT is not in SCANNING state");

	/* 2. Run simulation until FT completes all DCS scans. */
    /* DCS scans take roughly DCS_NUM_CHANNELS * (Duration) */
	printk("\n--> Step 2: Waiting for DCS scans to complete...\n");
    zassert_true(run_simulation_until(5000000, is_ft_beaconing), "FT did not enter BEACONING state after scans");

	/* 3. Wait for the first beacon to be transmitted. */
	printk("\n--> Step 3: Waiting for beacon TX...\n");
    /* The first beacon should be scheduled shortly after entering BEACONING */
    zassert_true(run_simulation_until(500000, beacon_captured), "FT did not transmit a beacon");

	/* 4. Verify the beacon payload. */
	printk("\n--> Step 4: Verifying beacon payload...\n");
	uint8_t mac_hdr_type = (g_last_tx_pdu_capture[0] >> 0) & 0x0F;
	zassert_equal(mac_hdr_type, MAC_COMMON_HEADER_TYPE_BEACON, "Transmitted PDU is not a beacon");
}

ZTEST_SUITE(dect_ft_beacon, NULL, dect_ft_beacon_setup, dect_ft_beacon_before, NULL, NULL);