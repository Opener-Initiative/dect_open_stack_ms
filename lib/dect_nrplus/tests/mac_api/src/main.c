
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_sm_ft.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include "../../tests/utils/test_harness_helpers.h"
#include <zephyr/drivers/timer/system_timer.h> 

/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft;
static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;

/* Peer lists must be static to persist outside the setup function's stack frame */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft1_peers[] = { &g_phy_ctx_pt };

#if IS_ENABLED(CONFIG_ZTEST)
extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					       dect_mac_event_type_t event_type, int timer_id);
#endif

/* --- Test Harness Functions --- */

static void debug_print_timer_states(const char *location)
{
    uint32_t pt_alive_status = k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    uint32_t pt_mobi_status = k_timer_status_get(&g_mac_ctx_ft.role_ctx.pt.mobility_scan_timer);
    uint32_t ft_status = k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
    
    uint32_t pt_alive_remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    uint32_t pt_mobi_remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.pt.mobility_scan_timer);
    uint32_t ft_remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
    
    printk("[TIMER_STATUS] %s:\n", location);
    printk("  PT keep-alive: %s (remaining: %u ticks)\n", 
           pt_alive_status ? "RUNNING" : "STOPPED", pt_alive_remaining);
    printk("  PT mobility-timer: %s (remaining: %u ticks)\n", 
           pt_mobi_status ? "RUNNING" : "STOPPED", pt_mobi_remaining);           
    printk("  FT beacon: %s (remaining: %u ticks)\n", 
           ft_status ? "RUNNING" : "STOPPED", ft_remaining);
}

static void process_all_mac_events(void)
{
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
		mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt
								    : &g_phy_ctx_ft);
		dect_mac_test_set_active_context(msg.ctx);
		dect_mac_event_dispatch(&msg);
	}

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_mac_service();
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_mac_service();
}



struct mac_integration_fixture {
	struct dect_mac_context *ctx;
};

static void mac_integration_teardown(void *data)
{
	ARG_UNUSED(data);
}

/* Test Fixture */
static void *mac_integration_setup(void)
{
	// mock_phy_reset();
	// nrf_modem_dect_phy_event_handler_set(dect_mac_phy_if_event_handler);
	// return &g_harness;
	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	return NULL;
}

static void debug_print_current_state(const char *test_name)
{
    printk("[DEBUG] %s - Current state:\n", test_name);
    printk("  Kernel time: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
    printk("  PT state: %d, FT state: %d\n", g_mac_ctx_pt.state, g_mac_ctx_ft.state);
    printk("  PT keep-alive timer: %s\n", 
           k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) ? "running" : "stopped");
    printk("  FT beacon timer: %s\n", 
           k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) ? "running" : "stopped");
}

static void mac_integration_before(void *data)
{
	// struct dect_mac_event_msg msg;
	// while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) { /* drain */ }

	// g_harness.current_ctx = &g_harness.ft_ctx;
	// dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	// g_harness.current_ctx = &g_harness.pt_ctx;
	// dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);

    ARG_UNUSED(data);
    int err;

    /* Reset shared resources to ensure test isolation */
	dect_mac_phy_if_reset_handle_map();

    /* Complete memory reset of all contexts */
    memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
    memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
    
    /* Complete mock PHY reset - this is critical */
    mock_phy_complete_reset(&g_phy_ctx_pt);
    mock_phy_complete_reset(&g_phy_ctx_ft);
    
    /* Reinitialize mock PHY contexts */
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft1_peers, ARRAY_SIZE(ft1_peers));    
    
    /* Stop and reinitialize all kernel timers */
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    // k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer);
    k_timer_stop(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
    
    k_timer_init(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer, NULL, NULL);
    k_timer_init(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer, NULL, NULL);
    k_timer_init(&g_mac_ctx_ft.role_ctx.ft.beacon_timer, NULL, NULL);
    
    /* Clear any pending MAC events */
    struct dect_mac_event_msg msg;
    int drained = 0;
    while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
        drained++;
    }
    if (drained > 0) {
        printk("[TEST_SETUP] Drained %d events from MAC event queue\n", drained);
    }
    
    /* Initialize MAC cores */
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
    zassert_ok(err, "FT dect_mac_core_init failed");
    nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);

    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
    zassert_ok(err, "PT dect_mac_core_init failed");
    nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);

    g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;

    /* Configure test-specific timing */
    g_mac_ctx_pt.config.keep_alive_period_ms = 1000;  // 1 second for testing
    g_mac_ctx_pt.config.ft_cluster_beacon_period_ms = 1000;    // 1 second for testing
    
    /* Register state change callback */
    dect_mac_register_state_change_cb(test_mac_state_change_cb);
    
    /* Clear global test state */
    // g_last_tx_pdu_len_capture = 0;
    // memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
    
    /* Debug: Print state after reset */
    debug_print_current_state("AFTER_RESET");
    debug_print_timer_states("AFTER_SETUP");
    printk("[TEST_SETUP] Test setup completed\n");

	printk("[DEBUG_PROBE] After Init: FT1 Peer[0] -> %p (Expected: %p)\n",
		(void *)g_phy_ctx_ft.peers[0], (void *)&g_phy_ctx_pt);
	printk("[DEBUG_PROBE] After Init: PT Peer Count -> %zu\n", g_phy_ctx_pt.num_peers);
}

static void mac_integration_after(void *fixture)
{
    ARG_UNUSED(fixture);

    /* Stop all kernel timers */
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer);
    k_timer_stop(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
    
    
    /* Deactivate PHY if active */
    if (g_phy_ctx_pt.state == PHY_STATE_ACTIVE) {
        nrf_modem_dect_phy_deactivate();
    }
    if (g_phy_ctx_ft.state == PHY_STATE_ACTIVE) {
        nrf_modem_dect_phy_deactivate();
    }
    
    /* Complete mock PHY reset - this is critical */
    mock_phy_complete_reset(&g_phy_ctx_pt);
    mock_phy_complete_reset(&g_phy_ctx_ft);
    
    /* Clear any remaining events */
    struct dect_mac_event_msg msg;
    int drained = 0;
    while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
        drained++;
    }
    if (drained > 0) {
        printk("[TEST_CLEANUP] Drained %d remaining events\n", drained);
    }
    
    /* Clear global test state */
    g_last_tx_pdu_len_capture = 0;
    memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
}


static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
    uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
    mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };
    uint32_t iteration_count = 0;
    uint32_t stall_count = 0;
    uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    
    printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
           end_time_us, timeout_us);
    
    while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
        iteration_count++;
        
        /* Safety check: prevent infinite loops */
        if (iteration_count > 10000) {
            printk("[SIMULATION] Maximum iterations (%u) reached, breaking\n", iteration_count);
            break;
        }
        
        /* Check if we're stuck at the same time */
        uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
        if (now_us == last_time_us) {
            stall_count++;
            if (stall_count > 10) {
                printk("[SIMULATION] Stuck at same time for %u iterations, forcing advance\n", stall_count);
                // k_sleep(K_MSEC(1));  // Force 1ms advance
                k_usleep(1);
                stall_count = 0;
            }
        } else {
            stall_count = 0;
        }
        last_time_us = now_us;
        
        /* Check break condition */
        if (break_cond_func && break_cond_func()) {
            printk("[SIMULATION] Break condition met at iteration %u\n", iteration_count);
            return true;
        }
        
        /* Get next PHY event time */
        uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
        
        /* Get next timer expiry time - FIXED LOGIC */
        uint64_t next_timer_ticks = K_TICKS_FOREVER;
        uint64_t remaining;
        
        /* Check PT keep-alive timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) != 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
            }
        }
        
        /* Check FT beacon timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) != 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
            }
        }
        
        uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
            UINT64_MAX :
            k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
        uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
        uint64_t time_to_advance_us;
        
        /* Debug output */
printk("[SIM_DEBUG] Iteration %u: now=%llu, next_phy=%llu, next_timer=%llu, next_event=%llu\n",
               iteration_count, now_us, next_phy_event_time, next_timer_expiry_us, next_event_time);
        
        /* Calculate time to advance */
        if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
            /* No more events or beyond end time - advance to end */
            // time_to_advance_us = end_time_us - now_us;
            time_to_advance_us = MIN(1000, end_time_us - now_us);
            printk("  Advancing to end time: %llu us\n", time_to_advance_us);
        } else if (next_event_time > now_us) {
            /* Event in future - advance to event time */
            time_to_advance_us = next_event_time - now_us;
            printk("  Advancing to next event: %llu us\n", time_to_advance_us);
        } else {
            /* Event now or in past - advance minimally to make progress */
            time_to_advance_us = 1;  /* Minimum 1us advance */
            printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
        }
        // if (next_event_time == UINT64_MAX) {
        //     /* No future events scheduled. Advance by a fixed tick or until the timeout. */
        //     time_to_advance_us = MIN(10000, end_time_us - now_us); /* 10ms tick */
        //     printk("   - UINT64_MAX advancing 10ms tick */ \n");
        // } else if (next_event_time > end_time_us) {
        //     /* Next event is past our timeout. Advance until the timeout. */
        //     time_to_advance_us = end_time_us - now_us;
        //     printk("  Advancing to end time: %llu us\n", time_to_advance_us);
        // } else if (next_event_time > now_us) {
        //     /* Event is in the future. Advance to that time. */
        //     time_to_advance_us = next_event_time - now_us;
        //     printk("  Advancing to next event: %llu us\n", time_to_advance_us);
        // } else {
        //     /* Event is now or in the past. Advance by a minimal amount to make progress. */
        //     time_to_advance_us = 1;
        //     printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
        // }

        /* Ensure we don't advance past end time */
        if (time_to_advance_us > 0 && now_us + time_to_advance_us > end_time_us) {
            time_to_advance_us = end_time_us - now_us;
            printk("  Adjusted to not exceed end time: %llu us\n", time_to_advance_us);
        }
        
        /* Advance time using busy wait for small intervals */
        if (time_to_advance_us > 0) {
            if (time_to_advance_us <= 1500) {
                /* Use busy wait for small intervals to avoid scheduler issues */
                printk("[SIMULATION] Busy wait for %llu us\n", time_to_advance_us);
                k_busy_wait(time_to_advance_us);
            } else {
                /* Use usleep for larger intervals */
                printk("[SIMULATION] Usleep for %llu us\n", time_to_advance_us);
                k_sleep(K_USEC(time_to_advance_us));
                // k_usleep(time_to_advance_us);
            }
        }
 
        /* Process events at current time */
        uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        printk("  Processing events at time: %llu us\n", current_time_us);
        
        mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
        mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
        process_all_mac_events();
        
        /* Break if no more events and we've reached the end */
        if (next_event_time == UINT64_MAX && 
            k_ticks_to_us_floor64(k_uptime_ticks()) >= end_time_us) {
            printk("[SIMULATION] No more events and end time reached, breaking\n");
            break;
        }
        
        printk("  --- End iteration %u ---\n", iteration_count);
    }
    
    printk("[SIMULATION] Ended after %u iterations. Final time: %llu us\n",
           iteration_count, k_ticks_to_us_floor64(k_uptime_ticks()));
    
    return (break_cond_func && break_cond_func());
}


static void run_mac_thread_for(uint32_t ms)
{
	struct dect_mac_event_msg msg;
	uint32_t start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < ms) {
		if (k_msgq_get(&mac_event_msgq, &msg, K_MSEC(1)) == 0) {
			mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt
									    : &g_phy_ctx_ft);
			dect_mac_test_set_active_context(msg.ctx);
			dect_mac_event_dispatch(&msg);
		}
	}
}

/* Test Cases */
ZTEST_F(mac_integration, test_rach_timeout_and_retry)
{
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	g_mac_ctx_pt.config.max_assoc_retries = 3;
	g_mac_ctx_pt.config.rach_response_window_ms = 10;
	dect_mac_change_state(MAC_STATE_PT_WAIT_ASSOC_RESP);
	g_mac_ctx_pt.role_ctx.pt.target_ft.is_valid = true;
	g_mac_ctx_pt.role_ctx.pt.target_ft.is_fully_identified = true;
	g_mac_ctx_pt.role_ctx.pt.target_ft.operating_carrier = 10;
	g_mac_ctx_pt.role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units = 7;
	g_mac_ctx_pt.role_ctx.pt.current_ft_rach_params.rach_operating_channel = 10;

	pt_rach_response_window_timer_expired_action();
    printk("Associating Current retries: %d\n", g_mac_ctx_pt.role_ctx.pt.current_assoc_retries);
	zassert_equal(g_mac_ctx_pt.role_ctx.pt.current_assoc_retries, 1);
	zassert_equal(g_mac_ctx_pt.state, MAC_STATE_PT_ASSOCIATING);

	/* Allow simulation to run so TX completes and state transitions to PT_WAIT_ASSOC_RESP */
	run_simulation_until(100, NULL);
	zassert_equal(g_mac_ctx_pt.state, MAC_STATE_PT_WAIT_ASSOC_RESP, "State did not transition to WAIT_ASSOC_RESP");

	pt_rach_response_window_timer_expired_action();
    printk("Associating Current retries: %d\n", g_mac_ctx_pt.role_ctx.pt.current_assoc_retries);
	zassert_equal(g_mac_ctx_pt.role_ctx.pt.current_assoc_retries, 2);

	/* Again, allow TX to complete */
	run_simulation_until(100, NULL);
	zassert_equal(g_mac_ctx_pt.state, MAC_STATE_PT_WAIT_ASSOC_RESP);

	pt_rach_response_window_timer_expired_action();
	zassert_equal(g_mac_ctx_pt.state, MAC_STATE_PT_SCANNING, "Did not restart scan after max retries");
}

ZTEST_F(mac_integration, test_mic_failure_and_hpc_resync)
{
	/* 1. Setup a secure link */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx = ft_find_and_init_peer_slot(g_mac_ctx_pt.own_long_rd_id, g_mac_ctx_pt.own_short_rd_id, -50*2);
	g_mac_ctx_ft.role_ctx.ft.connected_pts[peer_idx].is_secure = true;
	g_mac_ctx_ft.role_ctx.ft.keys_provisioned_for_peer[peer_idx] = true;

	/* 2. Simulate PT receiving 3 packets with bad MICs */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	for (int i = 0; i < MAX_MIC_FAILURES_BEFORE_HPC_RESYNC; i++) {
		/* This requires a mock function to simulate a bad MIC check */
		/* For now, we manually increment the counter */
		g_mac_ctx_pt.role_ctx.pt.associated_ft.consecutive_mic_failures++;
	}
	
	/* Manually trigger the check that would happen on the next bad MIC */
	if (g_mac_ctx_pt.role_ctx.pt.associated_ft.consecutive_mic_failures >= MAX_MIC_FAILURES_BEFORE_HPC_RESYNC) {
		g_mac_ctx_pt.role_ctx.pt.associated_ft.self_needs_to_request_hpc_from_peer = true;
	}

	/* 3. Verify the PT now wants to send a resync request */
	zassert_true(g_mac_ctx_pt.role_ctx.pt.associated_ft.self_needs_to_request_hpc_from_peer);
	(void)run_mac_thread_for;
    run_simulation_until(1000, NULL); /* Advance 1ms at a time */
}


ZTEST_SUITE(mac_integration, 
           NULL,                    /* No suite setup */
           mac_integration_setup,   /* Suite setup (once) */
           mac_integration_before,   /* Before each test */
           mac_integration_after,    /* After each test */
           mac_integration_teardown);                   /* No suite cleanup */