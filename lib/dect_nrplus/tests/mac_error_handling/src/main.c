/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* tests/mac_error_handling/src/main.c */
// Overview: Rewrite of mac_error_handling test using the robust mac_ass simulation harness.

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

static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft;
static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;

static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };

/* --- Simulation Harness --- */

static void process_all_mac_events(void)
{
    while (dect_mac_process_event_timeout(K_NO_WAIT) == 0) {
        // Just draining the queue
    }
}

static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
    uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
    mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };
    uint32_t iteration_count = 0;
    
    while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
        iteration_count++;
        if (iteration_count > 10000) break;
        
        if (break_cond_func && break_cond_func()) return true;
        
        uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
        uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        uint64_t next_event_time = next_phy_event_time;
        
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
        mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
        mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
        process_all_mac_events();
    }
    
    return (break_cond_func && break_cond_func());
}

/* --- Test Conditions --- */
static bool is_pt_scanning(void) { return g_mac_ctx_pt.state == MAC_STATE_PT_SCANNING; }
static bool is_pt_associating(void) { return g_mac_ctx_pt.state == MAC_STATE_PT_ASSOCIATING; }

/* --- Test Setup --- */

static void *mac_error_setup(void)
{
    zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
    return NULL;
}

static void mac_error_before(void *fixture)
{
    dect_mac_phy_if_reset_handle_map();
    memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
    memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
    
    mock_phy_complete_reset(&g_phy_ctx_pt);
    mock_phy_complete_reset(&g_phy_ctx_ft);
    
    mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, 1);
    mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, 1);
    
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    zassert_ok(dect_mac_core_init(MAC_ROLE_FT, 0x11223344), "FT dect_mac_core_init failed");
    
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    zassert_ok(dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD), "PT dect_mac_core_init failed");
}

/* --- Test Cases --- */

ZTEST(mac_error_tests, test_rach_max_retries)
{
    /* 1. Start FT (Beaconing) */
	printk("\n\n[TEST] 1. Start FT (Beaconing) \n");
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
    dect_mac_start();
    
    /* 2. Start PT and wait for it to start associating */
	printk("\n\n[TEST] 2. Start PT and wait for it to start associating \n");
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
	printk("\n\n[TEST] 2.1. g_mac_ctx_pt.config.max_assoc_retries = 2 \n");
    g_mac_ctx_pt.config.max_assoc_retries = 2;
	
    /* Speed up window for test */
	printk("\n\n[TEST] 2.2. g_mac_ctx_pt.config.rach_response_window_ms = 50 \n");
    g_mac_ctx_pt.config.rach_response_window_ms = 50; 
    
    nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
    dect_mac_start();
    
    /* Wait for PT to catch FT beacon and enter ASSOCIATING state */
	printk("\n\n[TEST] 3. Wait for PT to catch FT beacon and enter ASSOCIATING state \n");
    zassert_true(run_simulation_until(2000000, is_pt_associating), "PT failed to start association");
    
    /* 3. Drop all subsequent packets from FT to PT to simulate timeout */
	printk("\n\n[TEST] 4. Drop all subsequent packets from FT to PT to simulate timeout \n");
    mock_phy_test_config_error_injection(100);

	/* 5. PT should now attempt association twice and then fall back to scanning */
    /* Total time should be roughly 2 * response_window (~100ms) plus some backoff */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
	printk("\n\n[TEST] 6. PT should now attempt association twice and then fall back to scanning \n");
    zassert_true(run_simulation_until(2000000, is_pt_scanning), "PT did not fall back to scanning after max retries");
}

ZTEST_SUITE(mac_error_tests, NULL, mac_error_setup, mac_error_before, NULL, NULL);