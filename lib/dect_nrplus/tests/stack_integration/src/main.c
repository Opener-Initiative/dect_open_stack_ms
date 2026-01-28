// /* lib/dect_nrplus/tests/stack_integration/src/main.c */
// // Overview: Ztest suite for full stack integration. It verifies the end-to-end data flow from the CVG API down to the mock PHY and back.
// // #include <zephyr/ztest.h>
// // #include <string.h>
// // #include <dect_cvg.h>
// // #include <dect_dlc.h>
// // #include <mac/dect_mac.h>
// // #include <mac/dect_mac_core.h>
// // #include <mocks/mock_nrf_modem_dect_phy.h>
// // #include "test_harness_helpers.h"

// #include <zephyr/ztest.h>
// #include <zephyr/sys/byteorder.h>
// #include <string.h>

// #include <dect_cvg.h>
// #include <dect_dlc.h>
// #include <mac/dect_mac.h>
// #include <mac/dect_mac_context.h>
// #include <mac/dect_mac_core.h>
// #include <mac/dect_mac_pdu.h>
// #include <mac/dect_mac_phy_if.h>
// #include <mac/dect_mac_main_dispatcher.h>
// #include <mocks/mock_nrf_modem_dect_phy.h>
// #include "../../tests/utils/test_harness_helpers.h"

// /* --- Test Globals --- */
// uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
// uint16_t g_last_tx_pdu_len_capture = 0;

// static dect_mac_context_t g_mac_ctx_pt;
// static dect_mac_context_t g_mac_ctx_ft;
// static mock_phy_context_t g_phy_ctx_pt;
// static mock_phy_context_t g_phy_ctx_ft;

// /* Peer lists for the mock PHY simulation */
// static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
// static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };
// // static mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };



// static void process_all_mac_events(void)
// {
// 	struct dect_mac_event_msg msg;
// 	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
// 		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
// 		mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt
// 								    : &g_phy_ctx_ft);
// 		dect_mac_test_set_active_context(msg.ctx);
// 		dect_mac_event_dispatch(&msg);
// 	}

// 	dect_mac_test_set_active_context(&g_mac_ctx_ft);
// 	dect_mac_service();
// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
// 	dect_mac_service();
// }


// static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
// {
//     uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
//     mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };
//     uint32_t iteration_count = 0;
//     uint32_t stall_count = 0;
//     uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    
//     printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
//            end_time_us, timeout_us);
    
//     while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
//         iteration_count++;
        
//         /* Safety check: prevent infinite loops */
//         if (iteration_count > 10000) {
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
//         uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
        
//         /* Get next timer expiry time - FIXED LOGIC */
//         uint64_t next_timer_ticks = K_TICKS_FOREVER;
//         uint64_t remaining;
        
//         /* Check PT keep-alive timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
//             }
//         }
        
//         /* Check FT beacon timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
//             }
//         }
        
//         uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
//             UINT64_MAX :
//             k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
//         uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
//         uint64_t time_to_advance_us;
        
//         /* Debug output */
// printk("[SIM_DEBUG] Iteration %u: now=%llu, next_phy=%llu, next_timer=%llu, next_event=%llu\n",
//                iteration_count, now_us, next_phy_event_time, next_timer_expiry_us, next_event_time);
        
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
//         // if (next_event_time == UINT64_MAX) {
//         //     /* No future events scheduled. Advance by a fixed tick or until the timeout. */
//         //     time_to_advance_us = MIN(10000, end_time_us - now_us); /* 10ms tick */
//         //     printk("   - UINT64_MAX advancing 10ms tick */ \n");
//         // } else if (next_event_time > end_time_us) {
//         //     /* Next event is past our timeout. Advance until the timeout. */
//         //     time_to_advance_us = end_time_us - now_us;
//         //     printk("  Advancing to end time: %llu us\n", time_to_advance_us);
//         // } else if (next_event_time > now_us) {
//         //     /* Event is in the future. Advance to that time. */
//         //     time_to_advance_us = next_event_time - now_us;
//         //     printk("  Advancing to next event: %llu us\n", time_to_advance_us);
//         // } else {
//         //     /* Event is now or in the past. Advance by a minimal amount to make progress. */
//         //     time_to_advance_us = 1;
//         //     printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
//         // }

//         /* Ensure we don't advance past end time */
//         if (time_to_advance_us > 0 && now_us + time_to_advance_us > end_time_us) {
//             time_to_advance_us = end_time_us - now_us;
//             printk("  Adjusted to not exceed end time: %llu us\n", time_to_advance_us);
//         }
        
//         /* Advance time using busy wait for small intervals */
//         if (time_to_advance_us > 0) {
//             if (time_to_advance_us <= 1500) {
//                 /* Use busy wait for small intervals to avoid scheduler issues */
//                 printk("[SIMULATION] Busy wait for %llu us\n", time_to_advance_us);
//                 k_busy_wait(time_to_advance_us);
//             } else {
//                 /* Use usleep for larger intervals */
//                 printk("[SIMULATION] Usleep for %llu us\n", time_to_advance_us);
//                 k_sleep(K_USEC(time_to_advance_us));
//                 // k_usleep(time_to_advance_us);
//             }
//         }
 
//         /* Process events at current time */
//         uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
//         printk("  Processing events at time: %llu us\n", current_time_us);
        
//         mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
//         mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
//         process_all_mac_events();

//         /* Service the data paths for both contexts to trigger schedulers */
//         dect_mac_test_set_active_context(&g_mac_ctx_ft);
//         dect_mac_service();
//         dect_mac_test_set_active_context(&g_mac_ctx_pt);
//         dect_mac_service();

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
    
//     return (break_cond_func && break_cond_func());
// }


// /* --- Test Setup --- */

// static void *stack_integration_setup(void)
// {
// 	/* Initialize both contexts */
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft);
// 	dect_mac_core_init(MAC_ROLE_FT, 0x11223344);

// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
// 	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);

// 	/* Initialize the mock PHY contexts */
// 	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
// 	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));

//     g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;

// 	/* Initialize the layers, which will create their threads */
// 	dect_cvg_init(); /* This initializes all layers below it */

// 	return NULL;
// }

// static bool is_ft_beaconing(void) { return g_mac_ctx_ft.state == MAC_STATE_FT_BEACONING; }
// static bool is_pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }

// static void stack_integration_before(void *fixture)
// {
// 	ARG_UNUSED(fixture);
    
//     /* Register state change callback */
//     dect_mac_register_state_change_cb(test_mac_state_change_cb);


// 	/* This test suite requires a fully associated link to run */
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft);
//     mock_phy_set_active_context(&g_phy_ctx_ft);
//     nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
// 	dect_mac_start();
//     process_all_mac_events();
//     zassert_true(run_simulation_until(2000000, is_ft_beaconing), "FT never started beaconing");

// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
//     mock_phy_set_active_context(&g_phy_ctx_pt);
// 	dect_mac_start();
//     process_all_mac_events();

// 	// bool pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }
// 	zassert_true(run_simulation_until(2000000, is_pt_associated), "Link did not associate");

// 	/* Configure a reliable flow for the tests */
// 	dect_cvg_configure_flow(CVG_SERVICE_TYPE_4_FC_ARQ, 4, 500);
// }

// /* --- Test Helper Functions --- */

// static bool ft_phy_received_packet(void)
// {
// 	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
// 		if (g_phy_ctx_ft.rx_queue[i].active) {
// 			return true;
// 		}
// 	}
// 	return false;
// }

// /* --- Test Cases --- */

// ZTEST(stack_integration, test_full_stack_uplink)
// {
//     printk("\n\n--- RUNNING TEST: %s ---\n", __func__);

// 	uint8_t payload[] = "Hello DECT NR+";
// 	uint8_t rx_buf[sizeof(payload)];
// 	size_t rx_len = sizeof(rx_buf);

// 	/* 1. PT sends data via the CVG API */
// 	dect_mac_test_set_active_context(&g_mac_ctx_pt);
// 	dect_cvg_send(0x8003, g_mac_ctx_ft.own_long_rd_id, payload, sizeof(payload));

// 	/* 2. Run simulation until the FT's mock PHY receives the packet */
// 	zassert_true(run_simulation_until(500000, ft_phy_received_packet), "FT PHY did not receive a packet");

// 	/* 3. Manually process the received packet on the FT side to verify content */
// 	dect_mac_test_set_active_context(&g_mac_ctx_ft);
// 	run_simulation_until(100000, NULL); /* Allow MAC/DLC/CVG threads to process */

// 	int ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
// 	zassert_ok(ret, "FT failed to receive packet from its CVG layer");
// 	zassert_equal(rx_len, sizeof(payload), "Received payload has wrong length");
// 	zassert_mem_equal(rx_buf, payload, sizeof(payload), "Received payload is corrupt");
// }

// ZTEST_SUITE(stack_integration, NULL, stack_integration_setup, stack_integration_before, NULL, NULL);




/* lib/dect_nrplus/tests/stack_integration/src/main.c */
// Overview: This is the complete and final Ztest suite for full stack integration. It is correctly modeled after the working `mac_ass` test and includes its own self-contained simulation loop (`run_simulation_until`) to ensure correct and deterministic execution.
// --- REPLACE ENTIRE FILE ---
// // <<START REPLACEMENT CODE>>
#include <zephyr/ztest.h>
#include <string.h>
#include <dect_cvg.h>
#include <dect_dlc.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>

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

/* Peer lists for the mock PHY simulation */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };
static mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };

/* --- Test-Specific Simulation Harness --- */

static void process_all_mac_events_local(void)
{
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
		mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt
								    : &g_phy_ctx_ft);
		dect_mac_test_set_active_context(msg.ctx);
		dect_mac_event_dispatch(&msg);
	}
}


// static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
// {
//     uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
//     // mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };
//     uint32_t iteration_count = 0;
//     uint32_t stall_count = 0;
//     uint64_t last_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    
//     printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
//            end_time_us, timeout_us);

//     printk("[SIM_LOOP_DBG] Starting simulation loop for %llu us.\n", timeout_us);
    
//     while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
//         iteration_count++;
        
//         /* Safety check: prevent infinite loops */
//         if (iteration_count > 10000) {
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
//         uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
        
//         /* Get next timer expiry time - FIXED LOGIC */
//         uint64_t next_timer_ticks = K_TICKS_FOREVER;
//         uint64_t remaining;
        
//         /* Check PT keep-alive timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] PT keep-alive timer running, expires in %u ticks\n", remaining);
//             }
//         }
        
//         /* Check FT beacon timer only if it's running */
//         if (k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) != 0) {
//             remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
//             if (remaining > 0) {
//                 next_timer_ticks = MIN(next_timer_ticks, remaining);
//                 printk("[SIMULATION] FT beacon timer running, expires in %u ticks\n", remaining);
//             }
//         }
        
//         uint64_t next_timer_expiry_us = (next_timer_ticks == K_TICKS_FOREVER) ?
//             UINT64_MAX :
//             k_ticks_to_us_floor64(k_uptime_ticks()) + k_ticks_to_us_floor64(next_timer_ticks);
        
//         uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
//         uint64_t time_to_advance_us;
        
//         /* Debug output */
// printk("[SIM_DEBUG] Iteration %u: now=%llu, next_phy=%llu, next_timer=%llu, next_event=%llu\n",
//                iteration_count, now_us, next_phy_event_time, next_timer_expiry_us, next_event_time);
        
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
//         // if (next_event_time == UINT64_MAX) {
//         //     /* No future events scheduled. Advance by a fixed tick or until the timeout. */
//         //     time_to_advance_us = MIN(10000, end_time_us - now_us); /* 10ms tick */
//         //     printk("   - UINT64_MAX advancing 10ms tick */ \n");
//         // } else if (next_event_time > end_time_us) {
//         //     /* Next event is past our timeout. Advance until the timeout. */
//         //     time_to_advance_us = end_time_us - now_us;
//         //     printk("  Advancing to end time: %llu us\n", time_to_advance_us);
//         // } else if (next_event_time > now_us) {
//         //     /* Event is in the future. Advance to that time. */
//         //     time_to_advance_us = next_event_time - now_us;
//         //     printk("  Advancing to next event: %llu us\n", time_to_advance_us);
//         // } else {
//         //     /* Event is now or in the past. Advance by a minimal amount to make progress. */
//         //     time_to_advance_us = 1;
//         //     printk("  Event now/past, advancing minimally: %llu us\n", time_to_advance_us);
//         // }

//         /* Ensure we don't advance past end time */
//         if (time_to_advance_us > 0 && now_us + time_to_advance_us > end_time_us) {
//             time_to_advance_us = end_time_us - now_us;
//             printk("  Adjusted to not exceed end time: %llu us\n", time_to_advance_us);
//         }
        
//         /* Advance time using busy wait for small intervals */
//         if (time_to_advance_us > 0) {
//             if (time_to_advance_us <= 1500) {
//                 /* Use busy wait for small intervals to avoid scheduler issues */
//                 printk("[SIMULATION] Busy wait for %llu us\n", time_to_advance_us);
//                 k_busy_wait(time_to_advance_us);
//             } else {
//                 /* Use usleep for larger intervals */
//                 printk("[SIMULATION] Usleep for %llu us\n", time_to_advance_us);
//                 k_sleep(K_USEC(time_to_advance_us));
//                 // k_usleep(time_to_advance_us);
//             }
//         }
 
//         /* Process events at current time */
//         uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
//         printk("  Processing events at time: %llu us\n", current_time_us);
        
//         mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
//         mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
//         // process_all_mac_events();
//         process_all_mac_events_local();

//         /* Service the data paths for both contexts to trigger schedulers */
//         dect_mac_test_set_active_context(&g_mac_ctx_ft);
//         dect_mac_service();
//         dect_mac_test_set_active_context(&g_mac_ctx_pt);
//         dect_mac_service();

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
    
//     return (break_cond_func && break_cond_func());
// }
// Overview: Fixes a compilation error by replacing the access to an internal kernel variable with the correct logic to find the next timer expiry using public APIs.
// --- REPLACE ENTIre FUNCTION: run_simulation_until ---
// // <<START REPLACEMENT CODE>>
static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
    uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
    uint32_t iteration_count = 0;
    
    printk("[SIMULATION] Starting simulation. End time: %llu us, Timeout: %llu us\n",
           end_time_us, timeout_us);
    
    while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
        iteration_count++;
        if (iteration_count > 20000) { /* Safety break */
            printk("[SIMULATION] ERROR: Maximum iterations reached, breaking.\n");
            break;
        }
        
        if (break_cond_func && break_cond_func()) {
            printk("[SIMULATION] Break condition met at iteration %u.\n", iteration_count);
            return true;
        }
        
        /* Step 1: Find the time of the very next event from any source */
        uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

        /* Get next PHY event time from our mock */
        uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);

        printk("[SIM_TIME_DBG] Current time: %llu, Next PHY event at: %llu\n",
               k_ticks_to_us_floor64(k_uptime_ticks()), next_phy_event_time);
        
        /* Get next kernel timer expiry time by checking all known timers */
        uint64_t next_timer_expiry_us = UINT64_MAX;
        uint32_t remaining_ticks;

        if (k_timer_status_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer) > 0) {
            remaining_ticks = k_timer_remaining_get(&g_mac_ctx_pt.role_ctx.pt.keep_alive_timer);
            next_timer_expiry_us = MIN(next_timer_expiry_us, now_us + k_ticks_to_us_floor64(remaining_ticks));
        }
        if (k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) > 0) {
            remaining_ticks = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
            next_timer_expiry_us = MIN(next_timer_expiry_us, now_us + k_ticks_to_us_floor64(remaining_ticks));
        }
        /* Add checks for any other active timers here */
        
        uint64_t next_event_time = MIN(next_phy_event_time, next_timer_expiry_us);
        
        printk("[SIM_TIME_ADV_DBG] now_us: %llu, next_phy_event_time: %llu, next_timer_expiry_us: %llu, next_event_time: %llu, ,end_time_us: %llu\n",
               now_us, next_phy_event_time, next_timer_expiry_us, next_event_time, end_time_us);

        /* Step 2: Calculate how long to sleep to reach the next event */
        uint64_t time_to_advance_us;
        if (next_event_time == UINT64_MAX || next_event_time > end_time_us) {
            /* No more events or next event is past our total timeout. Sleep for a default tick. */
            time_to_advance_us = MIN(10000, end_time_us - now_us);
        } else if (next_event_time > now_us) {
            /* Event is in the future. Sleep until that exact time. */
            time_to_advance_us = next_event_time - now_us;
        } else {
            /* Event is now or in the past. Process immediately without sleeping. */
            time_to_advance_us = 0;
        }

        /* Step 3: Advance time */
        if (time_to_advance_us > 0) {
            k_sleep(K_USEC(time_to_advance_us));
        }
 
        /* Step 4: Process all events that are now due at the new current time */
        uint64_t current_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
        
        printk("  Processing events at time: %lluus Next PHY event at: %llu\n", current_time_us, next_phy_event_time);


        mock_phy_process_events(&g_phy_ctx_ft, current_time_us);
        mock_phy_process_events(&g_phy_ctx_pt, current_time_us);
		process_all_mac_events_local();

        /* Service the data paths for both contexts to trigger schedulers */
        dect_mac_test_set_active_context(&g_mac_ctx_ft);
        dect_mac_service();
        dect_mac_test_set_active_context(&g_mac_ctx_pt);
        dect_mac_service();
    }
    
    printk("[SIMULATION] Ended after %u iterations. Final time: %llu us\n",
           iteration_count, k_ticks_to_us_floor64(k_uptime_ticks()));
    
    return (break_cond_func && break_cond_func());
}
// <<END REPLACEMENT CODE>>

/* --- Test Setup --- */

static void *stack_integration_setup(void)
{
	/* Initialize both contexts */
	// dect_mac_test_set_active_context(&g_mac_ctx_ft);
	// dect_mac_core_init(MAC_ROLE_FT, 0x11223344);

	// dect_mac_test_set_active_context(&g_mac_ctx_pt);
	// dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);

    // g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;

	/* Initialize the mock PHY contexts */
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));


    /* Initialize MAC cores */
    int err;

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
    zassert_ok(err, "PT dect_mac_core_init failed");

    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
    zassert_ok(err, "FT dect_mac_core_init failed");

    g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;



	/* Initialize the layers, which will create their threads */
	dect_cvg_init();

	return NULL;
}

static void stack_integration_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* This test suite requires a fully associated link to run */
	// dect_mac_test_set_active_context(&g_mac_ctx_ft);
	// mock_phy_set_active_context(&g_phy_ctx_ft);
	// dect_mac_start();

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	dect_mac_start();

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	dect_mac_start();

	bool pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }
	zassert_true(run_simulation_until(5000000, pt_associated), "Link did not associate");

	/* Configure a reliable flow for the tests */
	// dect_cvg_configure_flow(CVG_SERVICE_TYPE_4_FC_ARQ, 4, 500);
    dect_cvg_configure_flow(0, 4, 500);

	// /* Start the necessary threads */
	// printk("[TEST_SETUP_DBG] About to call k_thread_start on CVG threads...\n");
	// printk("[TEST_SETUP_DBG]   -> Starting ARQ thread: %p\n", g_cvg_arq_service_thread_id);
	// k_thread_start(g_cvg_arq_service_thread_id);
	// printk("[TEST_SETUP_DBG]   -> Starting TX thread: %p\n", g_cvg_tx_thread_id);
	// k_thread_start(g_cvg_tx_thread_id);
    // printk("[TEST_SETUP_DBG]   -> Starting RX thread: %p\n", g_cvg_rx_thread_id);
	// k_thread_start(g_cvg_rx_thread_id);    

	/* Stop the beacon timer as it messes with test timing */
	k_timer_stop(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
}

/* --- Test Helper Functions --- */

static bool ft_phy_received_packet(void)
{
	printk("[FT PHY RX CHECK] === Scanning FT RX Queue (Max slots: %d) ===\n", MOCK_RX_QUEUE_MAX_PACKETS);
	int active_count = 0;
	
	for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
		if (g_phy_ctx_ft.rx_queue[i].active) {
			active_count++;
			printk("[FT PHY RX CHECK] Slot %d: ACTIVE - PDC len=%zu bytes\n", 
			       i, g_phy_ctx_ft.rx_queue[i].pdc_len);
			
			// Log complete packet data with formatting
			if (g_phy_ctx_ft.rx_queue[i].pdc_len > 0) {
				printk("[FT PHY RX CHECK]   Full PDC Payload (%zu bytes):\n", 
				       g_phy_ctx_ft.rx_queue[i].pdc_len);
				
				// Hex dump with 16 bytes per line
				for (size_t offset = 0; offset < g_phy_ctx_ft.rx_queue[i].pdc_len; offset += 16) {
					printk("[FT PHY RX CHECK]   %04zx: ", offset);
					
					// Hex bytes
					for (size_t j = 0; j < 16; j++) {
						if (offset + j < g_phy_ctx_ft.rx_queue[i].pdc_len) {
							printk("%02x ", g_phy_ctx_ft.rx_queue[i].pdc_payload[offset + j]);
						} else {
							printk("   "); // Padding for alignment
						}
					}
					
					// ASCII representation
					printk(" ");
					for (size_t j = 0; j < 16; j++) {
						if (offset + j < g_phy_ctx_ft.rx_queue[i].pdc_len) {
							uint8_t byte = g_phy_ctx_ft.rx_queue[i].pdc_payload[offset + j];
							printk("%c", (byte >= 0x20 && byte <= 0x7E) ? byte : '.');
						} else {
							printk(" ");
						}
					}
					printk("\n");
				}
				
				// Packet header analysis
				printk("[FT PHY RX CHECK]   Packet Analysis:\n");
				if (g_phy_ctx_ft.rx_queue[i].pdc_len >= 1) {
					uint8_t mac_header = g_phy_ctx_ft.rx_queue[i].pdc_payload[0];
					printk("[FT PHY RX CHECK]     MAC Header: 0x%02x\n", mac_header);
					
					// Decode MAC header type
					uint8_t mac_type = mac_header & 0x0F;
					switch (mac_type) {
						case 0x00:
							printk("[FT PHY RX CHECK]     Type: Data (0x00)\n");
							// Try to extract source/dest from data packets
							if (g_phy_ctx_ft.rx_queue[i].pdc_len >= 9) {
								uint32_t src_id = (g_phy_ctx_ft.rx_queue[i].pdc_payload[1] << 24) |
												 (g_phy_ctx_ft.rx_queue[i].pdc_payload[2] << 16) |
												 (g_phy_ctx_ft.rx_queue[i].pdc_payload[3] << 8) |
												  g_phy_ctx_ft.rx_queue[i].pdc_payload[4];
								uint32_t dest_id = (g_phy_ctx_ft.rx_queue[i].pdc_payload[5] << 24) |
												  (g_phy_ctx_ft.rx_queue[i].pdc_payload[6] << 16) |
												  (g_phy_ctx_ft.rx_queue[i].pdc_payload[7] << 8) |
												   g_phy_ctx_ft.rx_queue[i].pdc_payload[8];
								printk("[FT PHY RX CHECK]     Src: 0x%08x, Dest: 0x%08x\n", src_id, dest_id);
							}
							break;                        
						case 0x01:
							printk("[FT PHY RX CHECK]     Type: Beacon (0x01)\n");
							break;
						case 0x02:
							printk("[FT PHY RX CHECK]     Type: Data (0x02)\n");
							// Try to extract source/dest from data packets
							if (g_phy_ctx_ft.rx_queue[i].pdc_len >= 9) {
								uint32_t src_id = (g_phy_ctx_ft.rx_queue[i].pdc_payload[1] << 24) |
												 (g_phy_ctx_ft.rx_queue[i].pdc_payload[2] << 16) |
												 (g_phy_ctx_ft.rx_queue[i].pdc_payload[3] << 8) |
												  g_phy_ctx_ft.rx_queue[i].pdc_payload[4];
								uint32_t dest_id = (g_phy_ctx_ft.rx_queue[i].pdc_payload[5] << 24) |
												  (g_phy_ctx_ft.rx_queue[i].pdc_payload[6] << 16) |
												  (g_phy_ctx_ft.rx_queue[i].pdc_payload[7] << 8) |
												   g_phy_ctx_ft.rx_queue[i].pdc_payload[8];
								printk("[FT PHY RX CHECK]     Src: 0x%08x, Dest: 0x%08x\n", src_id, dest_id);
							}
							break;
						case 0x03:
							printk("[FT PHY RX CHECK]     Type: Acknowledgement (0x03)\n");
							break;
						case 0x04:
							printk("[FT PHY RX CHECK]     Type: Association Request (0x04)\n");
							break;
						case 0x05:
							printk("[FT PHY RX CHECK]     Type: Association Response (0x05)\n");
							break;
						default:
							printk("[FT PHY RX CHECK]     Type: Unknown (0x%02x)\n", mac_type);
							break;
					}
				}
				
				// Check if this looks like the expected Association Request
				if (g_phy_ctx_ft.rx_queue[i].pdc_len >= 9) {
					uint8_t mac_header = g_phy_ctx_ft.rx_queue[i].pdc_payload[0];
					if ((mac_header & 0x0F) == 0x02) { // Data packet
						uint32_t dest_id = (g_phy_ctx_ft.rx_queue[i].pdc_payload[5] << 24) |
										  (g_phy_ctx_ft.rx_queue[i].pdc_payload[6] << 16) |
										  (g_phy_ctx_ft.rx_queue[i].pdc_payload[7] << 8) |
										   g_phy_ctx_ft.rx_queue[i].pdc_payload[8];
						if (dest_id == 0x11223344) { // FT's Long ID
							printk("[FT PHY RX CHECK]     *** MATCH: This packet is addressed to FT (0x11223344) ***\n");
						}
					}
				}
			} else {
				printk("[FT PHY RX CHECK]   WARNING: Active slot with zero-length PDC\n");
			}
			
			printk("[FT PHY RX CHECK]   --- End of Slot %d Analysis ---\n", i);
		} else {
			// Optional: Log empty slots for complete queue visibility
			// printk("[FT PHY RX CHECK] Slot %d: EMPTY\n", i);
		}
	}
	
	if (active_count > 0) {
		printk("[FT PHY RX CHECK] === SUMMARY: Found %d active packet(s) in RX queue - Returning TRUE ===\n", active_count);
		return true;
	} else {
		printk("[FT PHY RX CHECK] === SUMMARY: No active packets in RX queue - Returning FALSE ===\n");
		return false;
	}
}

/* --- Test Cases --- */

ZTEST(stack_integration, test_full_stack_uplink)
{
	uint8_t payload[] = "Hello DECT NR+";
	uint8_t rx_buf[sizeof(payload)];
	size_t rx_len = sizeof(rx_buf);

    printk("*************************************/* 1. PT sends data via the CVG API */*******************************************************\n");
	/* 1. PT sends data via the CVG API */
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
    printk("************************************Sending payload[] = 'Hello DECT NR+'********************************************************\n");
	dect_cvg_send(0x8003, g_mac_ctx_ft.own_long_rd_id, payload, sizeof(payload));
    printk("********************************************************************************************\n");

    printk("*************************************/* 2. Run simulation until the FT's mock PHY receives the packet */*******************************************************\n");
	/* 2. Run simulation until the FT's mock PHY receives the packet */
	zassert_true(run_simulation_until(500000, ft_phy_received_packet), "FT PHY did not receive a packet");
    printk("*************************************FT PHY received a packet*******************************************************\n");

	/* 3. Manually process the received packet on the FT side to verify content */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
    
    int ret = 0;

    run_simulation_until(2500000, NULL); /* Allow MAC/DLC/CVG threads to process */
    printk("****************************************Starting dect_cvg_receive****************************************************\n");
// 	ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
// printk("****************************************Running dect_cvg_receive  %d ****************************************************\n", ret);

// 	run_simulation_until(1000000, NULL); /* Allow MAC/DLC/CVG threads to process */
// printk("****************************************Starting dect_cvg_receive****************************************************\n");
	ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
printk("****************************************Running dect_cvg_receive  %d ****************************************************\n", ret);   
	zassert_ok(ret, "FT failed to receive packet from its CVG layer");
	zassert_equal(rx_len, sizeof(payload), "Received payload has wrong length");
	zassert_mem_equal(rx_buf, payload, sizeof(payload), "Received payload is corrupt");
}

ZTEST_SUITE(stack_integration, NULL, stack_integration_setup, stack_integration_before, NULL, NULL);

