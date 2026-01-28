/* lib/dect_nrplus/tests/advanced_mac/src/main.c */
// Overview: Ztest suite for Advanced MAC procedures. Currently verifies Group Assignment scheduling.
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_random.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include "../../utils/test_harness_helpers.h"

#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_phy_ctrl.h>

#include "../../tests/utils/test_harness_helpers.h"
#include <zephyr/drivers/timer/system_timer.h> 


/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

static dect_mac_context_t g_mac_ctx_ft;
static dect_mac_context_t g_mac_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;
static mock_phy_context_t g_phy_ctx_pt;

/* Peer lists for Mock PHY */
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };

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


/* --- Setup / Teardown --- */

static void *advanced_mac_setup(void)
{
    /* One-time setup if needed */
    return NULL;
}

static void advanced_mac_before(void *fixture)
{
	ARG_UNUSED(fixture);

    /* 0. Initialize API Layer (Critical: Initializes TX dlists) */
    /* We pass NULL for the DLC RX list as this test focuses on MAC internal scheduling */
    dect_mac_api_init(NULL);

    /* 0b. Initialize PHY Interface (Registers event handler with Mock PHY) */
    dect_mac_phy_if_init();


	/* 1. Reset Mock PHYs */
	mock_phy_complete_reset(&g_phy_ctx_ft);
	mock_phy_complete_reset(&g_phy_ctx_pt);

	/* 2. Initialize Contexts & Activate PHYs */

	/* --- FT Setup --- */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_mac_core_init(MAC_ROLE_FT, 0x11111111);
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));

	/* Activate FT PHY so it accepts TX requests */
	mock_phy_set_active_context(&g_phy_ctx_ft);
	nrf_modem_dect_phy_init();
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);

	/* --- PT Setup --- */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_mac_core_init(MAC_ROLE_PT, 0x22222222);
	g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));

	/* Activate PT PHY */
	mock_phy_set_active_context(&g_phy_ctx_pt);
	nrf_modem_dect_phy_init();
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);

	/* Drain any events generated by init/activate so the queue is clean for the test logic */
	struct dect_mac_event_msg msg;
	extern struct k_msgq mac_event_msgq;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		/* Discard INIT/ACTIVATE events */
	}

	/* 3. Manually Establish Association State */

	// FT Side
	g_mac_ctx_ft.state = MAC_STATE_FT_BEACONING;
	g_mac_ctx_ft.role_ctx.ft.operating_carrier = 2;
	// Add PT to FT's connected list
	g_mac_ctx_ft.role_ctx.ft.connected_pts[0].is_valid = true;
	g_mac_ctx_ft.role_ctx.ft.connected_pts[0].long_rd_id = g_mac_ctx_pt.own_long_rd_id;
	g_mac_ctx_ft.role_ctx.ft.connected_pts[0].short_rd_id = g_mac_ctx_pt.own_short_rd_id;
	g_mac_ctx_ft.role_ctx.ft.connected_pts[0].group_id = 1;	    // Assign Group 1
	g_mac_ctx_ft.role_ctx.ft.connected_pts[0].resource_tag = 1; // Assign Tag 1

	// PT Side
	g_mac_ctx_pt.state = MAC_STATE_ASSOCIATED;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.is_valid = true;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id = g_mac_ctx_ft.own_long_rd_id;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.short_rd_id = g_mac_ctx_ft.own_short_rd_id;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier = 2;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.group_id = 1;	 // Matches FT
	g_mac_ctx_pt.role_ctx.pt.associated_ft.resource_tag = 1; // Matches FT

	// PT needs to know FT's PHY params to calculate subslots
	g_mac_ctx_pt.role_ctx.pt.associated_ft.peer_mu = g_mac_ctx_ft.own_phy_params.mu;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.peer_phy_params_known = true;

	/* 4. Setup FT's Group Schedule Template */
	g_mac_ctx_ft.role_ctx.ft.group_schedule.is_active = true;
	g_mac_ctx_ft.role_ctx.ft.group_schedule.alloc_type = RES_ALLOC_TYPE_UPLINK;
	g_mac_ctx_ft.role_ctx.ft.group_schedule.ul_start_subslot = 10;
	g_mac_ctx_ft.role_ctx.ft.group_schedule.ul_duration_subslots = 2;
	g_mac_ctx_ft.role_ctx.ft.group_schedule.repeat_type = RES_ALLOC_REPEAT_FRAMES_GROUP;
	g_mac_ctx_ft.role_ctx.ft.group_schedule.repetition_value = 10; // Every 10 frames
	g_mac_ctx_ft.role_ctx.ft.group_schedule.validity_value = 0xFF;
}

static void advanced_mac_after(void *fixture)
{
    ARG_UNUSED(fixture);
    /* Cleanup if necessary */
}

/* --- Helper Predicates --- */
static bool pt_has_active_ul_schedule(void) {
    printk("pt_has_active_ul_schedule Called... \n");
    return g_mac_ctx_pt.role_ctx.pt.ul_schedule.is_active;
}

static bool pt_is_idle(void) {
    /* Check if PT has cleared its pending operation */
    return (g_mac_ctx_pt.pending_op_type == PENDING_OP_NONE);
}

static bool ft_received_reconfig_req(void) {
    printk("[FT_RX_REC] ft_received_reconfig_req Starting...\n");
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
        if (g_phy_ctx_ft.rx_queue[i].active && g_phy_ctx_ft.rx_queue[i].pdc_len > 0) {
            printk("[FT_RX_REC] Loop rx_queue[%d].active:%d..pdc_len:%d\n", i,
                    g_phy_ctx_ft.rx_queue[i].active, g_phy_ctx_ft.rx_queue[i].pdc_len);
            /* Check for Reconfig Req IE (Type 13 = 0x0D) */
            /* PDU: Type(1) + UnicastHdr(9) + MuxHdr(1) + Payload */
            /* MuxHdr for Reconfig Req: Type 13 */
            uint8_t *p = g_phy_ctx_ft.rx_queue[i].pdc_payload;
            /* Skip MAC Header Type (1) + Unicast Header (10) = 11 bytes */
            printk("[FT_RX_REC] g_phy_ctx_ft.rx_queue[%d].pdc_len > 11(%d)\n",i ,g_phy_ctx_ft.rx_queue[i].pdc_len);
            if (g_phy_ctx_ft.rx_queue[i].pdc_len > 11) {
                
                uint8_t mux_hdr = p[11];
                uint8_t ie_type = mux_hdr & 0x3F;
                printk("[FT_RX_REC] type:%x:%x \n", ie_type, IE_TYPE_RECONFIG_REQ);
                if (ie_type == IE_TYPE_RECONFIG_REQ){
                    printk("[FT_RX_REC] ie_type == IE_TYPE_RECONFIG_REQ \n");
                    return true;

                } 
            }
        }
    }
    return false;
}

// static bool pt_received_reconfig_resp(void) {
//     printk("[PT_RX_REC] ft_received_reconfig_req Starting...\n");
//     for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
//         if (g_phy_ctx_pt.rx_queue[i].active && g_phy_ctx_pt.rx_queue[i].pdc_len > 0) {
//             /* Check for Reconfig Resp IE (Type 14 = 0x0E) */
//             uint8_t *p = g_phy_ctx_pt.rx_queue[i].pdc_payload;
//             if (g_phy_ctx_pt.rx_queue[i].pdc_len > 11) {
//                 uint8_t mux_hdr = p[11];
//                 uint8_t ie_type = mux_hdr & 0x3F;
//                 if (ie_type == IE_TYPE_RECONFIG_RESP) return true;
//             }
//         }
//     }
//     return false;
// }


static bool pt_received_reconfig_resp(void) {
    printk("[PT_RX_REC] pt_received_reconfig_resp Starting...\n");
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
        
        if (g_phy_ctx_pt.rx_queue[i].active && g_phy_ctx_pt.rx_queue[i].pdc_len > 0) {
            printk("[PT_RX_REC] Loop rx_queue[%d].active:%d..pdc_len:%d\n", i,
                    g_phy_ctx_pt.rx_queue[i].active, g_phy_ctx_pt.rx_queue[i].pdc_len);
            /* Check for Reconfig Resp IE (Type 14 = 0x0E) */
            uint8_t *p = g_phy_ctx_pt.rx_queue[i].pdc_payload;
            printk("[PT_RX_REC] g_phy_ctx_ft.rx_queue[%d].pdc_len > 11(%d)\n",i ,g_phy_ctx_ft.rx_queue[i].pdc_len);
            if (g_phy_ctx_pt.rx_queue[i].pdc_len > 11) {
                uint8_t mux_hdr = p[11];
                uint8_t ie_type = mux_hdr & 0x3F;
                printk("[PT_RX_REC] type:%x:%x \n", ie_type, IE_TYPE_RECONFIG_RESP);
                if (ie_type == IE_TYPE_RECONFIG_RESP){
                    printk("[PT_RX_REC] ie_type == IE_TYPE_RECONFIG_RESP \n");
                    return true;

                } 
            }
        }
    }
    return false;
}

/* --- Tests --- */

ZTEST(advanced_mac_tests, test_group_assignment_scheduling)
{
    printk("\n\n[TEST] Starting \n");
    /* 1. Configure FT to send Group Assignment */
    printk("\n\n[TEST] 1. Configure FT to send Group Assignment \n");
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    
    g_mac_ctx_ft.role_ctx.ft.group_assignment_pending = true;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.group_id = 1;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.is_single = false;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.is_direct = false;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.num_tags = 2;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.tags[0] = 0;
    g_mac_ctx_ft.role_ctx.ft.group_assignment_fields.tags[1] = 1;

    /* 2. Start PT Listening */
    printk("\n\n[TEST] 2. Start PT Listening  \n");
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    
    /* FORCE CARRIER TO 2 TO MATCH FT */
    g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier = 2; 

    uint32_t phy_rx_handle;
    dect_mac_rand_get((uint8_t *)&phy_rx_handle, sizeof(phy_rx_handle));
    
    printk("TEST: Starting PT RX to listen for beacon...");
    dect_mac_phy_ctrl_start_rx(
        g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier,
        0, /* Continuous */
        NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
        phy_rx_handle,
        0xFFFF, /* Listen for broadcast/beacons */
        PENDING_OP_PT_BEACON_LISTEN 
    );

    /* 3. Inject Beacon Timer Expiry to trigger FT TX */
    printk("\n\n[TEST] 3. Inject Beacon Timer Expiry to trigger FT TX  \n");
    dect_mac_test_inject_event_internal(&g_mac_ctx_ft, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
	process_all_mac_events();

    /* 4. Run Simulation */
    printk("\n\n[TEST] 4. Run Simulation \n");
    bool success = run_simulation_until(8000000, pt_has_active_ul_schedule);

    /* 5. Assertions */
    printk("\n\n[TEST] 5. Assertions \n");
    zassert_true(success, "PT did not activate UL schedule after receiving Group Assignment");

    dect_mac_schedule_t *pt_sched = &g_mac_ctx_pt.role_ctx.pt.ul_schedule;
    
    printk("PT Schedule Active: %d", pt_sched->is_active);
    printk("PT Schedule Start Subslot: %d", pt_sched->ul_start_subslot);
    printk("PT Schedule Repetition: %d", pt_sched->repetition_value);

    zassert_true(pt_sched->is_active, "PT UL schedule should be active");
    zassert_equal(pt_sched->alloc_type, RES_ALLOC_TYPE_UPLINK, "Schedule should be Uplink");
    zassert_equal(pt_sched->ul_start_subslot, 10, "Start subslot mismatch (should match FT template)");

    printk("\n\n[TEST] COMPLETE \n");
}



// ZTEST(advanced_mac_tests, test_reconfiguration_flow)
// {
//     /* 1. Construct Reconfig Request PDU (PT -> FT) */
//     dect_mac_test_set_active_context(&g_mac_ctx_pt);
    
//     uint8_t req_pdu[64];
//     uint8_t *p = req_pdu;
    
//     /* MAC Header Type: Unicast, No Security */
//     *p++ = (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F) | ((MAC_SECURITY_NONE & 0x03) << 4);
    
//     /* MAC Common Header: Unicast */
//     *p++ = 0; /* SN High/Reset */
//     *p++ = 1; /* SN Low */
//     /* Struct order is Receiver then Transmitter */
//     sys_put_be32(g_mac_ctx_ft.own_long_rd_id, p); p += 4; /* Rx: FT */
//     sys_put_be32(g_mac_ctx_pt.own_long_rd_id, p); p += 4; /* Tx: PT */
    
//     /* MAC SDU Area: Reconfig Req IE */
//     /* MUX Header: MAC_Ext=00 (No Len), Type=13 (Reconfig Req) */
//     *p++ = (0x00 << 6) | (IE_TYPE_RECONFIG_REQ & 0x3F);
//     /* Payload: 1 byte flags (0) */
//     *p++ = 0;
    
//     uint16_t req_len = p - req_pdu;
    
//     /* 2. Send Reconfig Request */
//     uint32_t tx_handle;
//     dect_mac_rand_get((uint8_t *)&tx_handle, sizeof(tx_handle));
    
//     dect_mac_phy_ctrl_start_tx_assembled(
//         g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier,
//         req_pdu, req_len,
//         g_mac_ctx_ft.own_short_rd_id, false,
//         tx_handle, PENDING_OP_GENERIC_UNICAST_TX,
//         true, 0, g_mac_ctx_pt.own_phy_params.mu, NULL
//     );
    
//     /* 3. Start FT RX to receive it */
//     dect_mac_test_set_active_context(&g_mac_ctx_ft);
//     mock_phy_set_active_context(&g_phy_ctx_ft);
//     uint32_t rx_handle;
//     dect_mac_rand_get((uint8_t *)&rx_handle, sizeof(rx_handle));
//     dect_mac_phy_ctrl_start_rx(
//         g_mac_ctx_ft.role_ctx.ft.operating_carrier, 0,
//         NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, rx_handle,
//         g_mac_ctx_ft.own_short_rd_id, PENDING_OP_FT_DATA_RX
//     );

//     /* 4. Run Simulation */
//     bool success = run_simulation_until(2000000, ft_received_reconfig_req);
//     zassert_true(success, "FT did not receive Reconfiguration Request");

    
//     /* 4b. Wait for PT to finish TX processing (clear pending op) */
//     /* We need to run the simulation a bit more to let the OP_COMPLETE event propagate */
//     int safety_counter = 0;
//     while (g_mac_ctx_pt.pending_op_type != PENDING_OP_NONE && safety_counter < 100) {
//         run_simulation_until(1000, NULL); /* Advance 1ms at a time */
//         safety_counter++;
//     }
//     zassert_equal(g_mac_ctx_pt.pending_op_type, PENDING_OP_NONE, "PT pending op did not clear after TX");

    

//     /* 5. Construct Reconfig Response PDU (FT -> PT) */
//     dect_mac_test_set_active_context(&g_mac_ctx_ft);
//     uint8_t resp_pdu[64];
//     p = resp_pdu;
    
//     /* MAC Header Type */
//     *p++ = (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F) | ((MAC_SECURITY_NONE & 0x03) << 4);
    
//     /* MAC Common Header */
//     *p++ = 0;
//     *p++ = 2;
//     /* Struct order is Receiver then Transmitter */
//     sys_put_be32(g_mac_ctx_pt.own_long_rd_id, p); p += 4; /* Rx: PT */
//     sys_put_be32(g_mac_ctx_ft.own_long_rd_id, p); p += 4; /* Tx: FT */
    
//     /* MAC SDU Area: Reconfig Resp IE */
//     /* MUX Header: MAC_Ext=00 (No Len), Type=14 (Reconfig Resp) */
//     *p++ = (0x00 << 6) | (IE_TYPE_RECONFIG_RESP & 0x3F);
//     /* Payload: 1 byte flags (0) */
//     *p++ = 0;
    
//     uint16_t resp_len = p - resp_pdu;
    
//     /* 6. Send Reconfig Response */
//     dect_mac_rand_get((uint8_t *)&tx_handle, sizeof(tx_handle));
//     dect_mac_phy_ctrl_start_tx_assembled(
//         g_mac_ctx_ft.role_ctx.ft.operating_carrier,
//         resp_pdu, resp_len,
//         g_mac_ctx_pt.own_short_rd_id, false,
//         tx_handle, PENDING_OP_GENERIC_UNICAST_TX,
//         true, 0, g_mac_ctx_ft.own_phy_params.mu, NULL
//     );
    
//     /* 7. Start PT RX to receive it */
//     dect_mac_test_set_active_context(&g_mac_ctx_pt);
//     mock_phy_set_active_context(&g_phy_ctx_pt);
//     dect_mac_rand_get((uint8_t *)&rx_handle, sizeof(rx_handle));
    
//     int ret = dect_mac_phy_ctrl_start_rx(
//         g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier, 0,
//         NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, rx_handle,
//         g_mac_ctx_pt.own_short_rd_id, PENDING_OP_PT_DATA_RX
//     );
//     /* Ensure the PT was actually able to start listening (i.e., previous TX op was cleared) */
//     zassert_ok(ret, "Failed to start PT RX for Reconfig Response (is pending op cleared?)");
    
//     /* 8. Run Simulation */
//     success = run_simulation_until(2000000, pt_received_reconfig_resp);
//     zassert_true(success, "PT did not receive Reconfiguration Response");
// }

ZTEST(advanced_mac_tests, test_reconfiguration_flow)
{
    /* 1. Construct Reconfig Request PDU (PT -> FT) */
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    
    uint8_t req_pdu[64];
    uint8_t *p = req_pdu;
    
    /* MAC Header Type: Unicast, No Security */
    *p++ = (MAC_COMMON_HEADER_TYPE_UNICAST & 0x0F) | ((MAC_SECURITY_NONE & 0x03) << 4);
    
    /* MAC Common Header: Unicast */
    *p++ = 0; /* SN High/Reset */
    *p++ = 1; /* SN Low */
    /* Struct order is Receiver then Transmitter */
    sys_put_be32(g_mac_ctx_ft.own_long_rd_id, p); p += 4; /* Rx: FT */
    sys_put_be32(g_mac_ctx_pt.own_long_rd_id, p); p += 4; /* Tx: PT */
    
    /* MAC SDU Area: Reconfig Req IE */
    /* MUX Header: MAC_Ext=00 (No Len), Type=13 (Reconfig Req) */
    *p++ = (0x00 << 6) | (IE_TYPE_RECONFIG_REQ & 0x3F);
    /* Payload: 1 byte flags (0) */
    *p++ = 0;
    
    uint16_t req_len = p - req_pdu;
    
    /* 2. Send Reconfig Request */
    uint32_t tx_handle;
    dect_mac_rand_get((uint8_t *)&tx_handle, sizeof(tx_handle));
    
    dect_mac_phy_ctrl_start_tx_assembled(
        g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier,
        req_pdu, req_len,
        g_mac_ctx_ft.own_short_rd_id, false,
        tx_handle, PENDING_OP_GENERIC_UNICAST_TX,
        true, 0, g_mac_ctx_pt.own_phy_params.mu, NULL
    );
    
    /* 3. Start FT RX to receive it */
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    uint32_t rx_handle;
    dect_mac_rand_get((uint8_t *)&rx_handle, sizeof(rx_handle));
    dect_mac_phy_ctrl_start_rx(
        g_mac_ctx_ft.role_ctx.ft.operating_carrier, 0,
        NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, rx_handle,
        g_mac_ctx_ft.own_short_rd_id, PENDING_OP_FT_DATA_RX
    );

    /* 4. Run Simulation: PT Sends -> FT Receives -> FT Sends Response */
    /* We wait until the FT has received the request. The FT SM will automatically
     * trigger the response TX inside the simulation loop. */
    bool success = run_simulation_until(2000000, ft_received_reconfig_req);
    zassert_true(success, "FT did not receive Reconfiguration Request");

    process_all_mac_events();
    /* 5. Wait for PT to finish its TX operation */
    printk("\n\n[TEST] 5. Wait for PT to finish its TX operation \n");
    /* This is critical: We cannot start the PT RX operation until the previous TX is fully complete and the pending op is cleared. */
    success = run_simulation_until(1000000, pt_is_idle);
    zassert_true(success, "PT did not complete TX operation (Pending Op not cleared)");

    /* 6. Start PT RX to receive the response */
    printk("\n\n[TEST] 6. Start PT RX to receive the response \n");
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    dect_mac_rand_get((uint8_t *)&rx_handle, sizeof(rx_handle));
    
    int ret = dect_mac_phy_ctrl_start_rx(
        g_mac_ctx_pt.role_ctx.pt.associated_ft.operating_carrier, 0,
        NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, rx_handle,
        g_mac_ctx_pt.own_short_rd_id, PENDING_OP_PT_DATA_RX
    );
    process_all_mac_events();
    zassert_ok(ret, "Failed to start PT RX for Reconfig Response");

    /* 7. Run Simulation until PT receives the response */
    printk("\n\n[TEST] 7. Run Simulation until PT receives the response \n");
    success = run_simulation_until(5000000, pt_received_reconfig_resp);
    zassert_true(success, "PT did not receive Reconfiguration Response");
    printk("\n\n[TEST] COMPLETE... \n");
}


ZTEST_SUITE(advanced_mac_tests, 
            NULL, 
            advanced_mac_setup, 
            advanced_mac_before, 
            advanced_mac_after, 
            NULL);