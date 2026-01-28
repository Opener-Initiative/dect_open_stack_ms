/* lib/dect_nrplus/tests/mac_mobility/src/main.c */
// Overview: This is the complete test suite for verifying the MAC layer's mobility (handover) procedure. It creates a three-node network (PT, FT1, FT2) and simulates the PT associating with FT1, then detecting a stronger signal from FT2 and successfully handing over to it.
// --- CREATE NEW FILE ---
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
#include <zephyr/drivers/timer/system_timer.h> 


/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

/* Three MAC and three PHY contexts are needed for this test */
static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft1;
static dect_mac_context_t g_mac_ctx_ft2;

static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft1;
static mock_phy_context_t g_phy_ctx_ft2;

/* Peer lists must be static to persist outside the setup function's stack frame */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft1, &g_phy_ctx_ft2 };
static mock_phy_context_t *ft1_peers[] = { &g_phy_ctx_pt };
static mock_phy_context_t *ft2_peers[] = { &g_phy_ctx_pt };


/* Forward declarations for mock PHY functions */
int mock_phy_queue_rx_packet(mock_phy_context_t *dest_ctx, const mock_rx_packet_t *packet);

#if IS_ENABLED(CONFIG_ZTEST)
extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					       dect_mac_event_type_t event_type, int timer_id);
#endif

/* --- Test Harness Functions --- */

static void process_all_mac_events(void)
{
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
		if (msg.ctx == &g_mac_ctx_pt) {
			mock_phy_set_active_context(&g_phy_ctx_pt);
            dect_mac_test_set_active_context(&g_mac_ctx_pt);
            printk("Active Context: g_phy_ctx_pt \n");
		} else if (msg.ctx == &g_mac_ctx_ft1) {
			mock_phy_set_active_context(&g_phy_ctx_ft1);
            dect_mac_test_set_active_context(&g_mac_ctx_ft1);
            printk("Active Context: g_phy_ctx_ft1 \n");
		} else if (msg.ctx == &g_mac_ctx_ft2) {
			mock_phy_set_active_context(&g_phy_ctx_ft2);
            dect_mac_test_set_active_context(&g_mac_ctx_ft2);
            printk("Active Context: g_phy_ctx_ft2 \n");
		} else {
            printk("ERROR: Message does not have a valid context !!!! \n");
            return;
        }
		// dect_mac_test_set_active_context(msg.ctx);
		dect_mac_event_dispatch(&msg);
	}

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_mac_service();
	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
	dect_mac_service();
	dect_mac_test_set_active_context(&g_mac_ctx_ft2);
	dect_mac_service();
}

static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
	uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
	mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft1, &g_phy_ctx_ft2 };
    uint32_t iteration_count = 0;
    uint32_t stall_count = 0;
    uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());

    printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
           end_time_us, timeout_us);

	while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
        iteration_count++;

        /* Safety check: prevent infinite loops */
        if (iteration_count > 1000) {
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
		uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 3);

		uint32_t next_timer_ticks = K_TICKS_FOREVER;
		uint32_t remaining;

        /* Check PT keep-alive timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) > 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
            }
        }        

        /* Check PT mobility scan timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer) > 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] PT mobility-scan timer running, expires in %u ticks\n", remaining);
            }
        }           

        /* Check FT beacon timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer) > 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] FT1 beacon timer running, expires in %u ticks\n", remaining);
            }
        }

        /* Check FT beacon timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer) > 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] FT2 beacon timer running, expires in %u ticks\n", remaining);
            }
        }

        uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
            UINT64_MAX :
            k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
        uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
        uint64_t time_to_advance_us;

/* Debug output */
// printk("[SIM_DEBUG] Iteration %u: now=%llu, next_phy=%llu, next_timer=%llu, next_event=%llu\n",
//                iteration_count, now_us, next_phy_event_time, next_timer_expiry_us, next_event_time);
        
        /* Calculate time to advance */
        if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
            /* No more events or beyond end time - advance to end */
            time_to_advance_us = end_time_us - now_us;
            // printk("  Advancing to end time: %llu us\n", time_to_advance_us);
        } else if (next_event_time > now_us) {
            /* Event in future - advance to event time */
            time_to_advance_us = next_event_time - now_us;
            // printk("  Advancing to next event: %llu us\n", time_to_advance_us);
        } else {
            /* Event now or in past - advance minimally to make progress */
            time_to_advance_us = 1;  /* Minimum 1us advance */
            // printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
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
            if (time_to_advance_us <= 1000) {
                /* Use busy wait for small intervals to avoid scheduler issues */
                // printk("[SIMULATION] Busy wait for %llu us\n", time_to_advance_us);
                k_busy_wait(time_to_advance_us);
            } else {
                /* Use usleep for larger intervals */
                // printk("[SIMULATION] Usleep for %llu us\n", time_to_advance_us);
                k_sleep(K_USEC(time_to_advance_us));
                // k_usleep(time_to_advance_us);
            }
        }
 
        /* Process events at current time */
        uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        printk("  Processing events at time: %llu us\n", current_time_us);

        // mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
        // mock_phy_process_events(&g_phy_ctx_ft1, current_time_us);
        // mock_phy_process_events(&g_phy_ctx_ft2, current_time_us);

        mock_phy_process_events(&g_phy_ctx_ft2, current_time_us);
        mock_phy_process_events(&g_phy_ctx_ft1, current_time_us);
        mock_phy_process_events(&g_phy_ctx_pt, current_time_us);

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

static bool is_ft1_beaconing(void) { return g_mac_ctx_ft1.state == MAC_STATE_FT_BEACONING; }
static bool is_ft2_beaconing(void) { 
    printk("[COND_CHECK_DBG] Checking is_ft2_beaconing. Current state: %s (%d)\n",
	       dect_mac_state_to_str(g_mac_ctx_ft2.state), g_mac_ctx_ft2.state);

    return g_mac_ctx_ft2.state == MAC_STATE_FT_BEACONING; 
}
// static bool is_pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }

static bool is_pt_associated_with_ft1(void)
{
    printk("\n\nis_pt_associated_with_ft1 \n");
    printk("   - pt:%s \n", dect_mac_state_to_str(g_mac_ctx_pt.state));
    printk("   - ft-1:%s \n", dect_mac_state_to_str(g_mac_ctx_ft1.state));
    printk("   - ft-2:%s \n", dect_mac_state_to_str(g_mac_ctx_ft2.state));
    printk("   - g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id[%0X] == g_mac_ctx_ft1.own_long_rd_id[%0X] \n", 
    g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id, g_mac_ctx_ft1.own_long_rd_id);

	return (g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED &&
		g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id == g_mac_ctx_ft1.own_long_rd_id);
}

static bool is_pt_associated_with_ft2(void)
{
    printk("\n\nis_pt_associated_with_ft2 \n");
    printk("   - pt:%s \n", dect_mac_state_to_str(g_mac_ctx_pt.state));
    printk("   - ft-1:%s \n", dect_mac_state_to_str(g_mac_ctx_ft1.state));
    printk("   - ft-2:%s \n", dect_mac_state_to_str(g_mac_ctx_ft2.state));
    printk("   - g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id[%0X] == g_mac_ctx_ft1.own_long_rd_id[%0X] \n", 
    g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id, g_mac_ctx_ft1.own_long_rd_id);

	return (g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED &&
		g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id == g_mac_ctx_ft2.own_long_rd_id);
}

static bool pt_sent_handover_request(void)
{
    printk("pt_sent_handover_request() Called... \n");
	uint16_t ie_len;

	return (g_last_tx_pdu_len_capture > 0 &&
		tx_capture_is_from_long_id(g_mac_ctx_pt.own_long_rd_id) &&
		get_ie_payload_from_pdu(g_last_tx_pdu_capture, g_last_tx_pdu_len_capture,
					IE_TYPE_ASSOC_REQ, &ie_len) != NULL);
}

static bool ft2_sent_handover_response(void)
{
    printk("ft2_sent_handover_response() Called... \n");
	uint16_t ie_len;

	return (g_last_tx_pdu_len_capture > 0 &&
		tx_capture_is_from_long_id(g_mac_ctx_ft2.own_long_rd_id) &&
		get_ie_payload_from_pdu(g_last_tx_pdu_capture, g_last_tx_pdu_len_capture,
					IE_TYPE_ASSOC_RESP, &ie_len) != NULL);
}



// static bool are_both_fts_beaconing(void)
// {
//     printk("ft-1:%s \n", dect_mac_state_to_str(g_mac_ctx_ft1.state));
//     printk("ft-2:%s \n", dect_mac_state_to_str(g_mac_ctx_ft2.state));

// 	return (g_mac_ctx_ft1.state == MAC_STATE_FT_BEACONING &&
// 		g_mac_ctx_ft2.state == MAC_STATE_FT_BEACONING);
// }


/* --- Test Setup --- */

static void *dect_mac_mobility_setup(void)
{
	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	return NULL;
}

static void dect_mac_mobility_before(void *fixture)
{
	ARG_UNUSED(fixture);
	int err;

    /* Complete memory reset of all contexts */
	// memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
	// memset(&g_mac_ctx_ft1, 0, sizeof(g_mac_ctx_ft1));
	// memset(&g_mac_ctx_ft2, 0, sizeof(g_mac_ctx_ft2));

    /* Complete mock PHY reset - this is critical */
    mock_phy_complete_reset(&g_phy_ctx_pt);
    mock_phy_complete_reset(&g_phy_ctx_ft1);
    mock_phy_complete_reset(&g_phy_ctx_ft2);

	// mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft1, &g_phy_ctx_ft2 };
	// mock_phy_context_t *ft1_peers[] = { &g_phy_ctx_pt };
	// mock_phy_context_t *ft2_peers[] = { &g_phy_ctx_pt };

	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft1, &g_mac_ctx_ft1, ft1_peers, ARRAY_SIZE(ft1_peers));
	mock_phy_init_context(&g_phy_ctx_ft2, &g_mac_ctx_ft2, ft2_peers, ARRAY_SIZE(ft2_peers));


    /* Stop and reinitialize all kernel timers */
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    // k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer);
    k_timer_stop(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
    k_timer_stop(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer);
    
    k_timer_init(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer, NULL, NULL);
    k_timer_init(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer, NULL, NULL);
    // k_timer_init(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer, NULL, NULL);
    // k_timer_init(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer, NULL, NULL);



	/* Init FT1 */
    printk("Init FT1 \n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
	mock_phy_set_active_context(&g_phy_ctx_ft1);
	err = dect_mac_core_init(MAC_ROLE_FT, 0x11111111);
	zassert_ok(err, "FT1 dect_mac_core_init failed");
    /* Set a specific beacon period for deterministic testing */
	g_mac_ctx_ft1.config.ft_cluster_beacon_period_ms = 2000;
    g_mac_ctx_ft1.config.ft_network_beacon_period_ms = 2000;

	/* Init FT2 */
    printk("Init FT2 \n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft2);
	mock_phy_set_active_context(&g_phy_ctx_ft2);
	err = dect_mac_core_init(MAC_ROLE_FT, 0x22222222);
	zassert_ok(err, "FT2 dect_mac_core_init failed");
    /* Set a specific beacon period for deterministic testing */
	g_mac_ctx_ft2.config.ft_cluster_beacon_period_ms = 2000;
    g_mac_ctx_ft2.config.ft_network_beacon_period_ms = 2000;

	/* Init PT */
    printk("Init PT \n");
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	zassert_ok(err, "PT dect_mac_core_init failed");

	/* Ensure all nodes are on the same network */
	g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft1.network_id_32bit;
	g_mac_ctx_ft2.network_id_32bit = g_mac_ctx_ft1.network_id_32bit;

	printk("[DEBUG_PROBE] FT1 Network ID: 0x%08X\n", g_mac_ctx_ft1.network_id_32bit);
	printk("[DEBUG_PROBE] FT2 Network ID: 0x%08X\n", g_mac_ctx_ft2.network_id_32bit);    

    /* Configure test-specific timing */
    // g_mac_ctx_pt.config.keep_alive_period_ms = 2000;  // 1 second for testing
    g_mac_ctx_pt.config.ft_cluster_beacon_period_ms = 5000;    // 1 second for testing
    g_mac_ctx_pt.config.ft_network_beacon_period_ms = 5000;

    // g_mac_ctx_pt.config.mobility_scan_interval_ms = 1000;    // 1 second for testing

    /* Register state change callback */
    dect_mac_register_state_change_cb(test_mac_state_change_cb);

    /* Clear global test state */
    // g_last_tx_pdu_len_capture = 0;
    // memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
    
    printk("[TEST_SETUP] Test setup completed\n");
}


static void dect_mac_mobility_after(void *fixture)
{
    ARG_UNUSED(fixture);

    /* Stop all kernel timers */
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
    k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.mobility_scan_timer);
    k_timer_stop(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
    k_timer_stop(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer);
    
    
    /* Deactivate PHY if active */
    if (g_phy_ctx_pt.state == PHY_STATE_ACTIVE) {
        nrf_modem_dect_phy_deactivate();
    }
    if (g_phy_ctx_ft1.state == PHY_STATE_ACTIVE) {
        nrf_modem_dect_phy_deactivate();
    }
    if (g_phy_ctx_ft2.state == PHY_STATE_ACTIVE) {
        nrf_modem_dect_phy_deactivate();
    }
    

    /* Complete mock PHY reset - this is critical */
    mock_phy_complete_reset(&g_phy_ctx_pt);
    mock_phy_complete_reset(&g_phy_ctx_ft1);
    mock_phy_complete_reset(&g_phy_ctx_ft2);
    
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

    // dump_mac_context_state("END of dect_mac_assoc_after", &g_mac_ctx_pt);
    // dump_mac_context_state("END of dect_mac_assoc_after", &g_mac_ctx_ft);    
}



/* --- Test Cases --- */

// ZTEST(dect_mac_mobility, test_pt_handover)
// {
//     printk("\n--- RUNNING TEST: %s ---\n", __func__);

//     /* Get current time at test start */
//     uint64_t test_start_time = k_ticks_to_us_floor64(k_uptime_ticks());
//     printk("[TEST] Test starting at time: %llu us\n", test_start_time);

//     /* --- 1. Start both FTs and let them start beaconing --- */
//     printk("\n\n[TEST] 1. Starting both FTs and waiting for beaconing\n");
//     dect_mac_test_set_active_context(&g_mac_ctx_ft1);
//     mock_phy_set_active_context(&g_phy_ctx_ft1);
//     g_mac_ctx_ft1.role_ctx.ft.operating_carrier = 2; /* Set distinct channels */
//     nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
//     dect_mac_start();
    
//     dect_mac_test_set_active_context(&g_mac_ctx_ft2);
//     mock_phy_set_active_context(&g_phy_ctx_ft2);
//     g_mac_ctx_ft2.role_ctx.ft.operating_carrier = 4;
//     nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
//     dect_mac_start();
    
//     process_all_mac_events();
    
//     zassert_true(run_simulation_until(2000000, are_both_fts_beaconing),
//                  "One or both FTs never started beaconing");
    
//     printk("TEST: Both FTs are beaconing\n");

// 	/* Trigger beacon (transmission is now handled automatically by mock TX) */
//     printk("\n\n Trigger beacon (transmission is now handled automatically by mock TX)\n");
// 	dect_mac_test_inject_event_internal(&g_mac_ctx_ft1, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
//     process_all_mac_events();
//     // dect_mac_test_inject_event_internal(&g_mac_ctx_ft2, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
// 	// process_all_mac_events();
    
//     /* --- 2. Start PT and let it associate with FT1 --- */
//     printk("\n\n[TEST] 2. Starting PT and waiting for association with FT1\n");
//     dect_mac_test_set_active_context(&g_mac_ctx_pt);
//     mock_phy_set_active_context(&g_phy_ctx_pt);
//     nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
//     dect_mac_start();
//     process_all_mac_events();
    
//     // zassert_true(run_simulation_until(5000000, is_pt_associated), "PT never became associated");
//     zassert_true(run_simulation_until(9000000, is_pt_associated_with_ft1),
//                  "PT never associated with FT1");
    
//     printk("TEST: PT is successfully associated with FT1\n");
    
//     /* --- 3. Simulate PT moving closer to FT2 by sending a strong beacon from FT2 --- */
//     printk("\n\n[TEST] 3. Simulating handover by sending strong beacon from FT2\n");
    
//     dect_mac_test_inject_event_internal(&g_mac_ctx_ft2, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
//     process_all_mac_events();
    
//     zassert_true(g_last_tx_pdu_len_capture > 0, "FT2 did not transmit a beacon");
    
//     /* Create a strong beacon packet from FT2 */
//     mock_rx_packet_t strong_beacon = {0};
//     strong_beacon.reception_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + 5000;
//     strong_beacon.carrier = g_mac_ctx_ft2.role_ctx.ft.operating_carrier;
//     strong_beacon.pcc_data.phy_type = 0;
//     strong_beacon.pcc_data.rssi_2 = -80; /* -40 dBm, much stronger than default */
    
//     /* Copy the beacon payload */
//     memcpy(strong_beacon.pdc_payload, g_last_tx_pdu_capture, g_last_tx_pdu_len_capture);
//     strong_beacon.pdc_len = g_last_tx_pdu_len_capture;
    
//     /* Queue the strong beacon to PT's PHY */
//     mock_phy_queue_rx_packet(&g_phy_ctx_pt, &strong_beacon);
    
//     printk("TEST: Strong beacon from FT2 queued for PT\n");
    
//     /* --- 4. Run simulation until PT completes handover to FT2 --- */
//     printk("\n\n[TEST] 4. Waiting for PT to complete handover to FT2\n");
    
//     zassert_true(run_simulation_until(3000000, is_pt_associated_with_ft2),
//                  "PT never completed handover to FT2");
    
//     printk("TEST: PT has successfully handed over to FT2\n");
    
//     /* --- 5. Final assertions --- */
//     printk("\n\n[TEST] 5. Verifying handover completion\n");
    
//     /* Check that PT is no longer associated with FT1 */
//     dect_mac_test_set_active_context(&g_mac_ctx_ft1);
//     int peer_idx_in_ft1 = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
//     zassert_true(peer_idx_in_ft1 < 0, "PT was not removed from FT1's peer list");
//     printk("TEST: PT successfully removed from FT1's peer list\n");
    
//     /* Check that PT is now associated with FT2 */
//     dect_mac_test_set_active_context(&g_mac_ctx_ft2);
//     int peer_idx_in_ft2 = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
//     zassert_true(peer_idx_in_ft2 >= 0, "PT was not added to FT2's peer list");
//     printk("TEST: PT successfully added to FT2's peer list\n");
    
//     /* Verify PT's association state */
//     zassert_true(g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED, 
//                  "PT is not in ASSOCIATED state after handover");
    
//     /* Verify PT's associated FT is FT2 */
//     zassert_true(g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id == g_mac_ctx_ft2.own_long_rd_id,
//                  "PT is not associated with FT2 after handover");
    
//     printk("TEST: Handover verification completed successfully\n");
// }
ZTEST(dect_mac_mobility, test_pt_handover)
{
	printk("\n--- RUNNING TEST: %s ---\n", __func__);

	/* --- 1. Start ONLY FT1 and wait for it to start beaconing --- */
    printk("\n\n[TEST] 1. Starting both FTs and waiting for beaconing\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
	mock_phy_set_active_context(&g_phy_ctx_ft1);
	g_mac_ctx_ft1.role_ctx.ft.operating_carrier = 2; /* Set distinct channels */
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	dect_mac_start();
	process_all_mac_events();
	zassert_true(run_simulation_until(5000000, is_ft1_beaconing), "FT never started beaconing");
	printk("TEST: FT1 is beaconing.\n");

	/* 1.5 Trigger beacon (transmission is now handled automatically by mock TX) */
    printk("\n\n[TEST] 1.5 Trigger beacon (transmission is now handled automatically by mock TX)\n");
	dect_mac_test_inject_event_internal(&g_mac_ctx_ft1, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
	process_all_mac_events();

	/* --- 2. Start PT and let it associate with FT1 --- */
    printk("\n\n\n\n[TEST] 2. Starting PT and waiting for association with FT1\n");
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	dect_mac_start();
	process_all_mac_events();
    // zassert_true(run_simulation_until(5000000, is_pt_associated), "PT never became associated");
	zassert_true(run_simulation_until(2000000, is_pt_associated_with_ft1), "PT never associated with FT1");
	printk("TEST: PT is successfully associated with FT1.\n");


    // /* Run simulation to allow messages to transmitted and received */
    // printk("\n\n    -  TAKING A BREAK   -\n\n\n");
	// run_simulation_until(1000000, NULL);      

	/* --- 3. NOW, start FT2 and wait for it to start beaconing --- */
    printk("\n\n\n\n[TEST] 3. NOW, start FT2 and wait for it to start beaconing...\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft2);
	mock_phy_set_active_context(&g_phy_ctx_ft2);
	g_mac_ctx_ft2.role_ctx.ft.operating_carrier = 3; /* Set distinct channels */
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	dect_mac_start();
	process_all_mac_events();
	zassert_true(run_simulation_until(5000000, is_ft2_beaconing), "FT never started beaconing");
	printk("TEST: FT2 is now also beaconing.\n");


	/* --- 4. Simulate PT moving closer to FT2 by sending a strong beacon from FT2 --- */
    printk("\n\n\n\n[TEST] 4. Simulate PT moving closer to FT2 by sending a strong beacon from FT2\n");
	// dect_mac_test_inject_event_internal(&g_mac_ctx_ft2, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
	// process_all_mac_events();
	// zassert_true(g_last_tx_pdu_len_capture > 0, "FT2 did not transmit a beacon");


    /* Run simulation to allow messages to transmitted and received */
    printk("\n\n    -  TAKING A BREAK   -\n\n\n");
	run_simulation_until(100000, NULL); 

	mock_rx_packet_t strong_beacon = {0};
	strong_beacon.reception_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + 1500;
	strong_beacon.carrier = g_mac_ctx_ft2.role_ctx.ft.operating_carrier;
	strong_beacon.pcc_data.phy_type = 0;
	strong_beacon.pcc_data.rssi_2 = -40; /* -40 dBm, much stronger than default */
    
	memcpy(strong_beacon.pdc_payload, g_last_tx_pdu_capture, g_last_tx_pdu_len_capture);
	strong_beacon.pdc_len = g_last_tx_pdu_len_capture;
	mock_phy_queue_rx_packet(&g_phy_ctx_pt, &strong_beacon);


	printk("[DEBUG_PROBE] FT1 Network ID: 0x%08X\n", g_mac_ctx_ft1.network_id_32bit);
	printk("[DEBUG_PROBE] FT2 Network ID: 0x%08X\n", g_mac_ctx_ft2.network_id_32bit);

	/* --- 5. Run simulation until PT completes handover to FT2 --- */
	printk("\n\n\n\n[TEST] 5. Waiting for PT to complete handover to FT2...\n");
	/* Verify the signaling sequence before checking the final state */
	printk("TEST: Verifying PT sends handover request to FT2...\n");
	g_last_tx_pdu_len_capture = 0;
	zassert_true(run_simulation_until(1000000, pt_sent_handover_request),
		     "PT did not send a handover request to FT2");

	printk("TEST: Verifying FT2 sends handover response to PT...\n");
	g_last_tx_pdu_len_capture = 0;
	zassert_true(run_simulation_until(1000000, ft2_sent_handover_response),
		     "PT did not receive a handover response from FT2");

	printk("TEST: Verifying PT finalizes association with FT2...\n");
	zassert_true(run_simulation_until(1000000, is_pt_associated_with_ft2),
		     "PT never completed handover to FT2");

	printk("TEST: PT has successfully handed over to FT2.\n");

	/* --- 6. Final assertions --- */
    printk("\n\n\n\n[TEST] 6. Final assertions\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
	int peer_idx_in_ft1 = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
	zassert_true(peer_idx_in_ft1 < 0, "PT was not removed from FT1's peer list");

	dect_mac_test_set_active_context(&g_mac_ctx_ft2);
	int peer_idx_in_ft2 = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
	zassert_true(peer_idx_in_ft2 >= 0, "PT was not added to FT2's peer list");    
}



ZTEST_SUITE(dect_mac_mobility, NULL, dect_mac_mobility_setup, dect_mac_mobility_before, dect_mac_mobility_after, NULL);


/**************************************************************************************************************************************** */
// /* lib/dect_nrplus/tests/mac_ass/src/main.c */
// // Overview: The complete, corrected test suite for MAC association. This version includes all functional tests (association, rejection, keep-alive, release), all of which are now compatible with the robust, kernel-time-based simulation harness.

// /* --- Test Harness Functions --- */

// // static void debug_print_timer_states(const char *location)
// // {
// //    uint32_t pt_status = k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
// //     uint32_t ft_status = k_timer_status_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
// //     uint32_t pt_remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
// //     uint32_t ft_remaining = k_timer_remaining_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
    
// //     printk("[TIMER_STATUS] %s:\n", location);
// //     printk("  PT keep-alive: %s (remaining: %u ticks)\n", 
// //            pt_status ? "RUNNING" : "STOPPED", pt_remaining);
// //     printk("  FT beacon: %s (remaining: %u ticks)\n", 
// //            ft_status ? "RUNNING" : "STOPPED", ft_remaining);
// // }

// static void process_all_mac_events(void)
// {
// 	struct dect_mac_event_msg msg;
// 	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
// 		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
// 		mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt
// 								    : &g_phy_ctx_ft1);
// 		dect_mac_test_set_active_context(msg.ctx);
// 		dect_mac_event_dispatch(&msg);
// 	}

// 	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
// 	dect_mac_service();
// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
// 	dect_mac_service();
// }


// // static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
// // {
// //     uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
// //     mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft1 };
// //     uint32_t iteration_count = 0;
// //     uint32_t stall_count = 0;
// //     uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    
// //     printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
// //            end_time_us, timeout_us);
    
// //     while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
// //         iteration_count++;
        
// //         /* Safety check: prevent infinite loops */
// //         if (iteration_count > 10000) {
// //             printk("[SIMULATION] Maximum iterations (%u) reached, breaking\n", iteration_count);
// //             break;
// //         }
        
// //         /* Check if we're stuck at the same time */
// //         uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
// //         if (now_us == last_time_us) {
// //             stall_count++;
// //             if (stall_count > 10) {
// //                 printk("[SIMULATION] Stuck at same time for %u iterations, forcing advance\n", stall_count);
// //                 // k_sleep(K_MSEC(1));  // Force 1ms advance
// //                 k_usleep(1);
// //                 stall_count = 0;
// //             }
// //         } else {
// //             stall_count = 0;
// //         }
// //         last_time_us = now_us;
        
// //         /* Check break condition */
// //         if (break_cond_func && break_cond_func()) {
// //             printk("[SIMULATION] Break condition met at iteration %u\n", iteration_count);
// //             return true;
// //         }
        
// //         /* Get next PHY event time */
// //         uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
        
// //         /* Get next timer expiry time - FIXED LOGIC */
// //         uint64_t next_timer_ticks = K_TICKS_FOREVER;
// //         uint64_t remaining;
        
// //         /* Check PT keep-alive timer only if it's running */
// //         if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) != 0) {
// //             remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
// //             if (remaining > 0) {
// //                 next_timer_ticks = MIN(next_timer_ticks, remaining);
// //                 printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
// //             }
// //         }
        
// //         /* Check FT beacon timer only if it's running */
// //         if (k_timer_status_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer) != 0) {
// //             remaining = k_timer_remaining_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
// //             if (remaining > 0) {
// //                 next_timer_ticks = MIN(next_timer_ticks, remaining);
// //                 printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
// //             }
// //         }
        
// //         uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
// //             UINT64_MAX :
// //             k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
// //         uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
// //         uint64_t time_to_advance_us;
        
// //         /* Calculate time to advance */
// //         if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
// //             /* No more events or beyond end time - advance to end */
// //             time_to_advance_us = end_time_us - now_us;
// //             printk("  Advancing to end time: %llu us\n", time_to_advance_us);
// //         } else if (next_event_time > now_us) {
// //             /* Event in future - advance to event time */
// //             time_to_advance_us = next_event_time - now_us;
// //             printk("  Advancing to next event: %llu us\n", time_to_advance_us);
// //         } else {
// //             /* Event now or in past - advance minimally to make progress */
// //             time_to_advance_us = 1;  /* Minimum 1us advance */
// //             printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
// //         }
        
// //         /* Ensure we don't advance past end time */
// //         if (time_to_advance_us > 0 && now_us + time_to_advance_us > end_time_us) {
// //             time_to_advance_us = end_time_us - now_us;
// //             printk("  Adjusted to not exceed end time: %llu us\n", time_to_advance_us);
// //         }
        
// //         /* Advance time */
// //         if (time_to_advance_us > 0) {
// //             printk("Time before advance: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
// //             printk("  Advancing by %llu us\n", time_to_advance_us);
// //             // k_sleep(K_USEC(time_to_advance_us));
// //             k_usleep(time_to_advance_us);
// //             printk("Time after advance: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
// //         }
 
// //         /* Process events at current time */
// //         uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
// //         printk("  Processing events at time: %llu us\n", current_time_us);
        
// //         mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
// //         mock_phy_process_events(&g_phy_ctx_ft1, current_time_us);
// //         process_all_mac_events();
        
// //         /* Break if no more events and we've reached the end */
// //         if (next_event_time == UINT64_MAX && 
// //             k_ticks_to_us_floor64(k_uptime_ticks()) >= end_time_us) {
// //             printk("[SIMULATION] No more events and end time reached, breaking\n");
// //             break;
// //         }
        
// //         printk("  --- End iteration %u ---\n", iteration_count);
// //     }
    
// //     printk("[SIMULATION] Ended after %u iterations. Final time: %llu us\n",
// //            iteration_count, k_ticks_to_us_floor64(k_uptime_ticks()));
    
// //     return (break_cond_func && break_cond_func());
// // }
// static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
// {
// 	uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
// 	mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft1, &g_phy_ctx_ft2 };
//     uint32_t iteration_count = 0;
//     uint32_t stall_count = 0;
//     uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());

//     printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
//            end_time_us, timeout_us);

// 	while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
//         iteration_count++;

//         /* Safety check: prevent infinite loops */
//         if (iteration_count > 1000) {
//             printk("[SIMULATION] Maximum iterations (%u) reached, breaking\n", iteration_count);
//             break;
//         }

//         /* Check if we're stuck at the same time */
//         uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
//         if (now_us == last_time_us) {
//             stall_count++;
//             if (stall_count > 10) {
//                 printk("[SIMULATION] Stuck at same time for %u iterations, forcing advance\n", stall_count);
//                 // k_sleep(K_MSEC(1));  // Force 1ms advance
//                 k_usleep(1);
//                 stall_count = 0;
//             }
//         } else {
//             stall_count = 0;
//         }
//         last_time_us = now_us;

//         /* Check break condition */
//         if (break_cond_func && break_cond_func()) {
//             printk("[SIMULATION] Break condition met at iteration %u\n", iteration_count);
//             return true;
//         }

//         /* Get next PHY event time */
// 		uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 3);

// 		uint32_t next_timer_ticks = K_TICKS_FOREVER;
// 		uint32_t remaining;

//         /* Check PT keep-alive timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
//             }
//         }        
        
//         /* Check FT beacon timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
//             }
//         }
// 		// remaining = k_timer_remaining_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
// 		// if (remaining > 0) { next_timer_ticks = MIN(next_timer_ticks, remaining); }

//         /* Check FT beacon timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
//             }
//         }
// 		// remaining = k_timer_remaining_get(&g_mac_ctx_ft2.role_ctx.ft.beacon_timer);
// 		// if (remaining > 0) { next_timer_ticks = MIN(next_timer_ticks, remaining); }


//         uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
//             UINT64_MAX :
//             k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
//         uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
//         uint64_t time_to_advance_us;

//         /* Calculate time to advance */
//         if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
//             /* No more events or beyond end time - advance to end */
//             time_to_advance_us = end_time_us - now_us;
//             printk("  Advancing to end time: %llu us\n", time_to_advance_us);
//         } else if (next_event_time > now_us) {
//             /* Event in future - advance to event time */
//             time_to_advance_us = next_event_time - now_us;
//             printk("  Advancing to next event: %llu us\n", time_to_advance_us);
//         } else {
//             /* Event now or in past - advance minimally to make progress */
//             time_to_advance_us = 1;  /* Minimum 1us advance */
//             printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
//         }
        
//         /* Ensure we don't advance past end time */
//         if (time_to_advance_us > 0 && now_us + time_to_advance_us > end_time_us) {
//             time_to_advance_us = end_time_us - now_us;
//             printk("  Adjusted to not exceed end time: %llu us\n", time_to_advance_us);
//         }
        
//         /* Advance time */
//         if (time_to_advance_us > 0) {
//             // printk("Time before advance: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
//             // printk("  Advancing by %llu us\n", time_to_advance_us);
//             k_sleep(K_USEC(time_to_advance_us));
//             // k_usleep(time_to_advance_us);
//             // printk("Time after advance: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
//         }
 
//         /* Process events at current time */
//         uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
//         printk("  Processing events at time: %llu us\n", current_time_us);

// 		// mock_phy_process_events(&g_phy_ctx_pt, k_ticks_to_us_floor64(k_uptime_ticks()));
// 		// mock_phy_process_events(&g_phy_ctx_ft1, k_ticks_to_us_floor64(k_uptime_ticks()));
// 		// mock_phy_process_events(&g_phy_ctx_ft2, k_ticks_to_us_floor64(k_uptime_ticks()));
//         mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
//         mock_phy_process_events(&g_phy_ctx_ft1, current_time_us);
//         mock_phy_process_events(&g_phy_ctx_ft2, current_time_us);

// 		process_all_mac_events();

//         /* Break if no more events and we've reached the end */
//         if (next_event_time == UINT64_MAX && 
//             k_ticks_to_us_floor64(k_uptime_ticks()) >= end_time_us) {
//             printk("[SIMULATION] No more events and end time reached, breaking\n");
//             break;
//         }

//         printk("  --- End iteration %u ---\n", iteration_count);
//     }
    
//     printk("[SIMULATION] Ended after %u iterations. Final time: %llu us\n",
//            iteration_count, k_ticks_to_us_floor64(k_uptime_ticks()));
// 	return (break_cond_func && break_cond_func());
// }



// static bool is_ft_beaconing(void) { return g_mac_ctx_ft1.state == MAC_STATE_FT_BEACONING; }
// static bool is_ft2_beaconing(void) { return g_mac_ctx_ft2.state == MAC_STATE_FT_BEACONING; }
// static bool is_pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }

// static void *dect_mac_assoc_setup(void)
// {
// 	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
// 	return NULL;
// }

// // /* --- Enhanced Test Reset Functions --- */

// // static void debug_print_current_state(const char *test_name)
// // {
// //     printk("[DEBUG] %s - Current state:\n", test_name);
// //     printk("  Kernel time: %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
// //     printk("  PT state: %d, FT state: %d\n", g_mac_ctx_pt.state, g_mac_ctx_ft1.state);
// //     printk("  PT keep-alive timer: %s\n", 
// //            k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) ? "running" : "stopped");
// //     printk("  FT beacon timer: %s\n", 
// //            k_timer_status_get(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer) ? "running" : "stopped");
// // }

// /* --- Enhanced Test Setup --- */

// // static void dect_mac_assoc_before(void *fixture)
// // {
// //     ARG_UNUSED(fixture);
// //     int err;
      
// //     /* Complete memory reset of all contexts */
// //     memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
// //     memset(&g_mac_ctx_ft1, 0, sizeof(g_mac_ctx_ft1));
    
// //     /* Complete mock PHY reset - this is critical */
// //     mock_phy_complete_reset(&g_phy_ctx_pt);
// //     mock_phy_complete_reset(&g_phy_ctx_ft1);
    
// //     /* Reinitialize mock PHY contexts */
// //     mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, &g_phy_ctx_ft1);
// //     mock_phy_init_context(&g_phy_ctx_ft1, &g_mac_ctx_ft1, &g_phy_ctx_pt);
    
// //     /* Stop and reinitialize all kernel timers */
// //     k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
// //     k_timer_stop(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
// //     k_timer_init(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer, NULL, NULL);
// //     k_timer_init(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer, NULL, NULL);
    
// //     /* Clear any pending MAC events */
// //     struct dect_mac_event_msg msg;
// //     int drained = 0;
// //     while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
// //         drained++;
// //     }
// //     if (drained > 0) {
// //         printk("[TEST_SETUP] Drained %d events from MAC event queue\n", drained);
// //     }
    
// //     /* Initialize MAC cores */
// //     dect_mac_test_set_active_context(&g_mac_ctx_ft1);
// //     mock_phy_set_active_context(&g_phy_ctx_ft1);
// //     err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
// //     zassert_ok(err, "FT dect_mac_core_init failed");

// //     dect_mac_test_set_active_context(&g_mac_ctx_pt);
// //     mock_phy_set_active_context(&g_phy_ctx_pt);
// //     err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
// //     zassert_ok(err, "PT dect_mac_core_init failed");

// //     g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft1.network_id_32bit;

// //     /* Configure test-specific timing */
// //     // g_mac_ctx_pt.config.keep_alive_period_ms = 1000;  // 1 second for testing
    
// //     /* Register state change callback */
// //     dect_mac_register_state_change_cb(test_mac_state_change_cb);
    
// //     /* Clear global test state */
// //     g_last_tx_pdu_len_capture = 0;
// //     memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
    
// //     /* Debug: Print state after reset */
// //     debug_print_current_state("AFTER_RESET");
// //     debug_print_timer_states("AFTER_SETUP");
// //     printk("[TEST_SETUP] Test setup completed\n");
// // }


// /* --- Enhanced Test Cleanup --- */

// static void dect_mac_assoc_after(void *fixture)
// {
//     ARG_UNUSED(fixture);
    
//     printk("[TEST_CLEANUP] Starting test cleanup\n");
    
//     /* Stop all kernel timers */
//     k_timer_stop(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
//     k_timer_stop(&g_mac_ctx_ft1.role_ctx.ft.beacon_timer);
    
//     /* Deactivate PHY if active */
//     if (g_phy_ctx_pt.state == PHY_STATE_ACTIVE) {
//         nrf_modem_dect_phy_deactivate();
//     }
//     if (g_phy_ctx_ft1.state == PHY_STATE_ACTIVE) {
//         nrf_modem_dect_phy_deactivate();
//     }
    
//     /* Complete mock PHY reset - this is critical */
//     mock_phy_complete_reset(&g_phy_ctx_pt);
//     mock_phy_complete_reset(&g_phy_ctx_ft1);
    
//     /* Clear any remaining events */
//     struct dect_mac_event_msg msg;
//     int drained = 0;
//     while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
//         drained++;
//     }
//     if (drained > 0) {
//         printk("[TEST_CLEANUP] Drained %d remaining events\n", drained);
//     }
    
//     /* Clear global test state */
//     g_last_tx_pdu_len_capture = 0;

//     printk("[TEST_CLEANUP] Test cleanup completed\n");
// }


// /* --- Test Cases --- */

// ZTEST(dect_mac_assoc, test_pt_ft_association)
// {
//     printk("\n--- RUNNING TEST: %s ---\n", __func__);

//     /* Get current time at test start */
//     uint64_t test_start_time = k_ticks_to_us_floor64(k_uptime_ticks());
//     printk("[TEST] Test starting at time: %llu us\n", test_start_time);

// 	/* 1. Start FT and run simulation until it's beaconing */
//     printk("\n\n[TEST] 1. Starting both FTs and waiting for beaconing\n");
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
// 	mock_phy_set_active_context(&g_phy_ctx_ft1);
//     g_mac_ctx_ft2.role_ctx.ft.operating_carrier = 2;
// 	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
// 	dect_mac_start();
// 	process_all_mac_events();
// 	zassert_true(run_simulation_until(5000000, is_ft_beaconing), "FT never started beaconing");

// 	/* 2. Trigger beacon (transmission is now handled automatically by mock TX) */
//     printk("\n\n[TEST] 2. Trigger beacon (transmission is now handled automatically by mock TX)\n");
// 	dect_mac_test_inject_event_internal(&g_mac_ctx_ft1, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
// 	process_all_mac_events();

// 	/* 3. Start PT and run simulation until it becomes associated */
//     printk("\n\n[TEST] 3. Starting PT and waiting for association with FT\n");
// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
// 	mock_phy_set_active_context(&g_phy_ctx_pt);
// 	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
// 	dect_mac_start();
// 	process_all_mac_events();
// 	zassert_true(run_simulation_until(5000000, is_pt_associated), "PT never became associated");

// 	/* --- 3. NOW, start FT2 and wait for it to start beaconing --- */
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft2);
// 	mock_phy_set_active_context(&g_phy_ctx_ft2);
// 	g_mac_ctx_ft2.role_ctx.ft.operating_carrier = 4;
// 	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
// 	dect_mac_start();
// 	process_all_mac_events();
// 	zassert_true(run_simulation_until(5000000, is_ft2_beaconing), "FT never started beaconing");
// 	printk("TEST: FT2 is now also beaconing.\n");

// 	/* --- 4. Simulate PT moving closer to FT2 by sending a strong beacon from FT2 --- */
// 	dect_mac_test_inject_event_internal(&g_mac_ctx_ft2, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
// 	process_all_mac_events();
// 	zassert_true(g_last_tx_pdu_len_capture > 0, "FT2 did not transmit a beacon");

// 	/* 4. Final assertions */
//     printk("\n\n[TEST] 4. Final assertions\n");
// 	zassert_true(g_mac_ctx_pt.role_ctx.pt.associated_ft.is_valid,
// 		     "PT's associated_ft is not valid");
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft1);
// 	int peer_idx = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
// 	zassert_true(peer_idx >= 0, "FT did not find the associated PT");
// }

// // ZTEST_SUITE(dect_mac_assoc, NULL, dect_mac_assoc_setup, dect_mac_assoc_before, NULL, NULL);
// ZTEST_SUITE(dect_mac_assoc, 
//            NULL,                    /* No suite setup */
//            dect_mac_assoc_setup,   /* Suite setup (once) */
//            dect_mac_mobility_before,   /* Before each test */
//            dect_mac_assoc_after,    /* After each test */
//            NULL);                   /* No suite cleanup */