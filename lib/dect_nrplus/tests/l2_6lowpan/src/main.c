/* lib/dect_nrplus/tests/l2_6lowpan/src/main.c */
// Overview: The full stack integration test. It initializes two complete DECT stacks (PT and FT), associates them, sends a secure data packet from the PT's CVG layer, runs the simulation, and verifies the packet is correctly received and decrypted at the FT's CVG layer.
// --- CREATE NEW FILE ---
#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <dect_nrplus.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include <dect_cvg.h>
#include <dect_dlc.h>
#include <dect_cdd.h>
#include "../../utils/test_harness_helpers.h"

#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <zephyr/drivers/timer/system_timer.h> 

/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft;
static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;

static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };

static bool pt_associated_flag = false;
static void test_state_change_cb(dect_mac_public_state_t new_state)
{
	if (new_state == MAC_STATE_PUB_ASSOCIATED) {
		pt_associated_flag = true;
	}
}

static void dect_stack_test_before(void *fixture)
{
	ARG_UNUSED(fixture);
	int err;

	dect_mac_phy_if_reset_handle_map();

	mock_phy_complete_reset(&g_phy_ctx_pt);
	mock_phy_complete_reset(&g_phy_ctx_ft);
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));

	/* Initialize FT Stack */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	err = dect_nrplus_stack_init();
	zassert_ok(err, "FT stack init failed");

	/* Initialize PT Stack */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	err = dect_nrplus_stack_init();
	zassert_ok(err, "PT stack init failed");

	/* Set PT's network ID to match FT's for association */
	g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;

	/* Register state change callback to know when association is complete */
	dect_mac_register_state_change_cb(test_state_change_cb);
	pt_associated_flag = false;
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
                printk("[SIMULATION] PT keep-alive timer running, expires in %llu ticks\n", remaining);
            }
        }
        
        /* Check FT beacon timer only if it's running */
        if (k_timer_status_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer) != 0) {
            remaining = k_timer_remaining_get(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
            if (remaining > 0) {
                next_timer_ticks = MIN(next_timer_ticks, remaining);
                printk("[SIMULATION] FT beacon timer running, expires in %llu ticks\n", remaining);
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
            time_to_advance_us = end_time_us - now_us;
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



static bool is_pt_associated_cond(void)
{
	return pt_associated_flag;
}

ZTEST(dect_stack_integration, test_cvg_to_cvg_secure_transfer)
{
	printk("\n--- RUNNING TEST: %s ---\n", __func__);

	/* 1. Start both stacks */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	dect_nrplus_stack_start();

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	dect_nrplus_stack_start();

	/* 2. Run simulation until association is complete */
	printk("[TEST] Waiting for PT to associate with FT...\n");
	zassert_true(run_simulation_until(5000000, is_pt_associated_cond),
		     "Simulation timed out before PT became associated");
	printk("[TEST] PT is associated. Proceeding with data transfer.\n");

	/* 3. Send a test packet from the PT's CVG layer */
	const char *test_payload = "Hello DECT NR+";
	size_t payload_len = strlen(test_payload);

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	int ret = dect_cvg_send(CVG_EP_IPV6_PROFILE, g_mac_ctx_ft.own_long_rd_id,
				(const uint8_t *)test_payload, payload_len);
	zassert_ok(ret, "dect_cvg_send failed: %d", ret);
	printk("[TEST] PT CVG->send() successful.\n");

	/* 4. Run simulation to allow the packet to be transmitted and received */
	printk("[TEST] Running simulation to process TX/RX...\n");
	run_simulation_until(2000000, NULL);

	/* 5. Attempt to receive the packet at the FT's CVG layer */
	uint8_t rx_buf[128];
	size_t rx_len = sizeof(rx_buf);

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
	zassert_ok(ret, "dect_cvg_receive failed: %d", ret);

	/* 6. Verify the received packet content */
	zassert_equal(rx_len, payload_len, "Received packet length mismatch");
	zassert_mem_equal(rx_buf, test_payload, payload_len, "Received packet content mismatch");

	printk("[TEST] FT successfully received and verified the packet.\n");
}

ZTEST_SUITE(dect_stack_integration, NULL, NULL, dect_stack_test_before, NULL, NULL);