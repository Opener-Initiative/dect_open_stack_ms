/* lib/dect_nrplus/tests/stack_integration/src/main.c */

#include <zephyr/ztest.h>
#include <string.h>
#include <dect_cvg.h>
#include <dect_dlc.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>

#include <mac/dect_mac_context.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_main_dispatcher.h>
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

/* Peer lists for the mock PHY simulation */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };
static mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };

extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
							       dect_mac_event_type_t event_type, int timer_id);

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
            time_to_advance_us = MIN(6912, end_time_us - now_us);
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

/* --- Test Setup --- */
static void *stack_integration_setup(void)
{
	/* Initialize both contexts */

	// /* Initialize the mock PHY contexts */
	// mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	// mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));


    // /* Initialize MAC cores */
    // int err;

	// dect_mac_test_set_active_context(&g_mac_ctx_pt);
    // mock_phy_set_active_context(&g_phy_ctx_pt);
    // err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
    // zassert_ok(err, "PT dect_mac_core_init failed");

    // dect_mac_test_set_active_context(&g_mac_ctx_ft);
    // mock_phy_set_active_context(&g_phy_ctx_ft);
    // err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
    // zassert_ok(err, "FT dect_mac_core_init failed");

    // g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;



	/* Initialize the layers, which will create their threads */
	// dect_cvg_init();

	// return NULL;

	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	zassert_ok(dect_dlc_init(), "dect_dlc_init failed");
	zassert_ok(dect_cvg_init(), "dect_cvg_init failed");
	return NULL;
}

static void stack_integration_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* This test suite requires a fully associated link to run */
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

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	dect_mac_start();

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	dect_mac_start();

	bool pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }
	zassert_true(run_simulation_until(5000000, pt_associated), "Link did not associate");

	/* After association, inject an event to make FT open RX window */
	dect_mac_test_inject_event_internal(&g_mac_ctx_ft, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
	run_simulation_until(100000, NULL);

	/* Configure a reliable flow for the tests */
    dect_cvg_configure_flow(CVG_SERVICE_TYPE_0_TRANSPARENT, 4, 100, 5000);

	/* Stop the beacon timer as it messes with test timing */
	k_timer_stop(&g_mac_ctx_ft.role_ctx.ft.beacon_timer);
}

static void stack_integration_after(void *fixture)
{
    ARG_UNUSED(fixture);
    
    /* Stop all kernel timers via reset_context */
    dect_mac_reset_context(&g_mac_ctx_pt);
    dect_mac_reset_context(&g_mac_ctx_ft);
    
    /* Deactivate and reset PT PHY context */
    mock_phy_set_active_context(&g_phy_ctx_pt);
    nrf_modem_dect_phy_deactivate();
    mock_phy_complete_reset(&g_phy_ctx_pt);
    
    /* Deactivate and reset FT PHY context */
    mock_phy_set_active_context(&g_phy_ctx_ft);
    nrf_modem_dect_phy_deactivate();
    mock_phy_complete_reset(&g_phy_ctx_ft);
    
    /* Reset handle maps and other shared resources */
    dect_mac_phy_if_reset_handle_map();
    
    /* Complete memory reset of all contexts */
    memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
    memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
        
    // /* Complete mock PHY reset - this is critical */
    // mock_phy_complete_reset(&g_phy_ctx_pt);
    // mock_phy_complete_reset(&g_phy_ctx_ft);

    /* Clear MAC event queue */
    struct dect_mac_event_msg msg;
    int drained = 0;
    while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
        drained++;
    }
    if (drained > 0) {
        printk("[TEST_CLEANUP] Drained %d events from MAC event queue\n", drained);
    }
    
    /* Reset global test state */
    g_last_tx_pdu_len_capture = 0;
    memset(g_last_tx_pdu_capture, 0, sizeof(g_last_tx_pdu_capture));
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

/* After association completes, wait for FT to be ready */
bool ft_ready_for_data(void) {
    /* Check that FT has no pending ops and is in BEACONING state */
    return (g_mac_ctx_ft.state == MAC_STATE_FT_BEACONING && 
            g_mac_ctx_ft.pending_op_type == PENDING_OP_NONE);
}

/* After association, verify FT has active data RX */
bool ft_has_data_rx(void) {
    /* Check FT's pending op - should be FT_DATA_RX or similar */
    if (g_mac_ctx_ft.pending_op_type != PENDING_OP_FT_DATA_RX) {
        printk("FT pending_op is %s, not DATA_RX\n", dect_pending_op_to_str(g_mac_ctx_ft.pending_op_type));
        return false;
    }
    
    return true;
}


/* After FT sends data, run simulation until FT's TX completes */
bool ft_tx_completed(void) {
    /* Check if FT's pending TX op is done */
    return (g_mac_ctx_ft.pending_op_type == PENDING_OP_NONE);
}

__weak bool pt_phy_received_packet(void)
{
    printk("[PT PHY RX CHECK] Scanning PT RX Queue...\n");
    
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; i++) {
        if (g_phy_ctx_pt.rx_queue[i].active) {
            printk("[PT PHY RX CHECK] Found active packet in slot %d\n", i);
            return true;
        }
    }
    
    printk("[PT PHY RX CHECK] No active packets found\n");
    return false;
}

/* Add statistics to MAC context for verification */
struct dect_mac_stats {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t retransmissions;
    uint32_t window_stalls;
};

/* --- Test Cases --- */

// /* Test 4: Multiple Packets (Load Test) */
// ZTEST(stack_integration, test_4_multiple_packets)
// {
//     #define NUM_PACKETS 10
//     uint8_t tx_data[NUM_PACKETS][25];
//     uint8_t rx_data[NUM_PACKETS][25];
//     uint32_t seq_num[NUM_PACKETS];
//     size_t rx_len[NUM_PACKETS];
//     uint16_t flow_id = 0x8003; /* Use same flow ID as uplink test */
    
//     /* Prepare unique packets */
//     for (int i = 0; i < NUM_PACKETS; i++) {
//         snprintf(tx_data[i], sizeof(tx_data[i]), "Packet %d from PT", i);
//         seq_num[i] = i;
//     }
    
//     /* Send all packets from PT */
//     dect_mac_test_set_active_context(&g_mac_ctx_pt);
//     for (int i = 0; i < NUM_PACKETS; i++) {
//         dect_cvg_send(flow_id, g_mac_ctx_ft.own_long_rd_id, 
//                      tx_data[i], strlen(tx_data[i]) + 1);
//     }
    
//     /* Run simulation to allow all packets to be transmitted */
//     run_simulation_until(10000, NULL);
    
//     /* Receive and verify all packets at FT */
//     dect_mac_test_set_active_context(&g_mac_ctx_ft);
//     int received = 0;
//     while (received < NUM_PACKETS) {
//         if (dect_cvg_receive(rx_data[received], &rx_len[received], K_NO_WAIT) == 0) {
//             /* Find which packet this is by content */
//             for (int i = 0; i < NUM_PACKETS; i++) {
//                 if (strcmp(rx_data[received], tx_data[i]) == 0) {
//                     seq_num[i] = received; /* Mark as received */
//                     break;
//                 }
//             }
//             received++;
//         } else {
//             /* No more packets ready, advance simulation */
//             run_simulation_until(10000, NULL);
//         }
//     }
    
//     /* Verify all packets received */
//     for (int i = 0; i < NUM_PACKETS; i++) {
//         zassert_true(seq_num[i] < NUM_PACKETS, 
//                     "Packet %d was never received", i);
//     }
// }

/* Test 5: Flow Control / Window Full */
ZTEST(stack_integration, test_5_flow_control)
{
    int ret;
    int packets_sent = 0;
    int queue_full_count = 0;
	uint16_t flow_id = 0x8003; /* Use same flow ID as uplink test */
    
    printk("\n=== TEST: Flow Control ===\n");
    printk("Sending packets until queue is full...\n");
    
    /* Keep sending until we get a queue full error */
    for (int i = 0; i < 50; i++) {  /* Sufficient to fill flow-controlled queues */
        uint8_t data[20];
        snprintf(data, sizeof(data), "Packet %d", i);
        
        ret = dect_cvg_send(flow_id, g_mac_ctx_ft.own_long_rd_id, 
                           data, strlen(data) + 1);
        
        if (ret == 0) {
            packets_sent++;
            printk("Sent packet %d (total: %d)\n", i, packets_sent);
        } else if (ret == -ENOMEM || ret == -EAGAIN) {
            queue_full_count++;
            printk("Queue full at packet %d (sent %d, ret=%d)\n", 
                   i, packets_sent, ret);
            break;
        } else {
            zassert_true(false, "Unexpected error %d at packet %d", ret, i);
        }
    }
    
    zassert_true(queue_full_count > 0, 
                "Never hit queue full condition after %d packets", packets_sent);
    
    printk("Flow control test passed: queue full after %d packets\n", packets_sent);
}

/* Test 6: Retransmission on Packet Loss */
ZTEST(stack_integration, test_retransmission)
{
    /* Declare variables */
    int ret;
    uint8_t rx_buf[64];
    size_t rx_len = sizeof(rx_buf);
    uint16_t flow_id = 0x8003;  /* Same flow ID as uplink test */
    uint8_t data[] = "Important data";
    
    printk("\n=== TEST: Retransmission on Packet Loss ===\n");
    
    /* Configure mock PHY to drop 50% of packets */
    /* Using the ACTUAL function from your mock_phy */
    mock_phy_test_config_error_injection(50);  /* 50% packet loss */
    printk("Packet loss rate configured to 50%%\n");
    
    /* PT sends data to FT */
    printk("PT sending: '%s' to FT (with 50%% packet loss)\n", data);
    dect_mac_test_set_active_context(&g_mac_ctx_pt);
    mock_phy_set_active_context(&g_phy_ctx_pt);
    
    ret = dect_cvg_send(flow_id, g_mac_ctx_ft.own_long_rd_id, 
                        data, sizeof(data));
    zassert_ok(ret, "dect_cvg_send failed");
    
    /* Run simulation longer than normal to allow retransmissions */
    printk("Waiting for FT PHY to receive packet (allowing retransmissions)...\n");
    zassert_true(run_simulation_until(10000, ft_phy_received_packet), 
                "FT PHY did not receive packet after retransmissions");
    
    /* FT receives data */
    printk("FT attempting to receive...\n");
    dect_mac_test_set_active_context(&g_mac_ctx_ft);
    mock_phy_set_active_context(&g_phy_ctx_ft);
    
    /* May need multiple attempts as data moves up the stack */
    int attempts = 0;
    do {
        run_simulation_until(10000, NULL);
        ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
        attempts++;
        
        if (attempts > 50) {
            zassert_true(false, "Timeout waiting for CVG receive");
        }
    } while (ret == -EAGAIN);
    
    /* Verify received data */
    zassert_ok(ret, "dect_cvg_receive failed");
    printk("FT received: '%s' (len=%zu) after %d attempts\n", 
           rx_buf, rx_len, attempts);
    
    zassert_equal(rx_len, sizeof(data), 
                 "Received length %zu expected %zu", rx_len, sizeof(data));
    zassert_mem_equal(rx_buf, data, sizeof(data),
                     "Received data mismatch");
    
    /* Note: Since we don't have stats in the context, we can't verify 
       retransmission count directly. We rely on the fact that the packet 
       was received despite 50% loss rate, which implies retransmissions. */
    
    printk("=== TEST PASSED (packet received despite 50%% loss) ===\n");
    
    /* Reset error injection for subsequent tests */
    mock_phy_test_config_error_injection(0);
}

/* Test 7: Large Packet Segmentation */
ZTEST(stack_integration, test_7_large_packet)
{
    /* Use the MAX configured size, not an arbitrary large number */
    // uint8_t large_data[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE - 10];
    // TODO: create tests that use more than just mu=0
    uint8_t large_data[36]; /* 35 bytes is big for mu=0 */
    uint8_t rx_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
    size_t rx_len = sizeof(rx_buf);
	uint16_t flow_id = 0x8003; /* Use same flow ID as uplink test */
    int ret;
    
    printk("\n=== TEST: Large Packet (within limits) ===\n");
    
    /* Fill with pattern */
    for (int i = 0; i < sizeof(large_data); i++) {
        large_data[i] = i & 0xFF;
        // printk("Large Packet (within limits) %d\n", i);
    }
    
    printk("Sending %zu byte packet (max allowed)\n", sizeof(large_data));
    
    /* Send - this should work if within pool limits */
	printk("[TEST] 1. Send - this should work if within pool limits\n");
    ret = dect_cvg_send(flow_id, g_mac_ctx_ft.own_long_rd_id, 
                        large_data, sizeof(large_data));
    zassert_ok(ret, "dect_cvg_send failed for max-sized packet");
    
    /* Wait for transmission */
	printk("[TEST] 2. Wait for transmission\n");
    run_simulation_until(10000, ft_phy_received_packet);
    
    /* Receive and verify */
	printk("[TEST] 3. Receive and verify\n");
    ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
    
    int attempts = 0;
	int waiting = 10000;
    do {
        ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
        if (ret == -EAGAIN) {
            run_simulation_until(waiting, NULL);
            attempts++;
        }
        if (attempts > 50) {
            zassert_true(false, "Timeout waiting for PT to receive PONG");
        }
    } while (ret == -EAGAIN);

    zassert_ok(ret, "Failed to receive large packet");
    
    zassert_equal(rx_len, sizeof(large_data), 
                 "Length mismatch: got %zu expected %zu", 
                 rx_len, sizeof(large_data));
    
    /* Verify first few bytes to catch obvious corruption */
    zassert_equal(rx_buf[0], large_data[0], "First byte mismatch");
    zassert_equal(rx_buf[sizeof(large_data)-1], large_data[sizeof(large_data)-1], 
                 "Last byte mismatch");
    
    printk("=== TEST PASSED ===\n");
}

ZTEST(stack_integration, test_segmentation)
{
    /* This size should trigger segmentation */
    // uint8_t large_data[250];  /* Well over typical MTU */
    // TODO: create tests that use more than just mu=0
    uint8_t large_data[36]; /* 35 bytes is big for mu=0 */
    uint8_t rx_buf[100];
    size_t rx_len = sizeof(rx_buf);
	uint16_t flow_id = 0x8003; /* Use same flow ID as uplink test */
    int ret;
    
    printk("\n=== TEST: Segmentation ===\n");
	printk("CONFIG_DECT_DLC_NET_BUF_FRAG_SIZE: %d\n", CONFIG_DECT_DLC_NET_BUF_FRAG_SIZE );
	printk("Max SDU size: %d\n", CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE);
    printk("Sending %d byte packet (should be segmented)\n", sizeof(large_data));
    
    /* Fill with identifiable pattern */
    for (int i = 0; i < sizeof(large_data); i++) {
        large_data[i] = i % 256;
    }
    
    /* This should work if DLC segmentation is working */
    ret = dect_cvg_send(flow_id, g_mac_ctx_ft.own_long_rd_id, 
                        large_data, sizeof(large_data));
    zassert_ok(ret, "[TEST test_segmentation] dect_cvg_send failed - segmentation may not be supported");
    
    /* Wait longer for multiple segments */
    run_simulation_until(10000, ft_phy_received_packet);
    
    /* Receive reassembled packet */
    int attempts = 0;
	int waiting = 10000;
    do {
        ret = dect_cvg_receive(rx_buf, &rx_len, K_NO_WAIT);
        if (ret == -EAGAIN) {
            run_simulation_until(waiting, NULL);
            attempts++;
        }
        if (attempts > 50) {
            zassert_true(false, "Timeout waiting for PT to receive PONG");
        }
    } while (ret == -EAGAIN);
    zassert_equal(rx_len, sizeof(large_data), 
                 "[TEST test_segmentation] Reassembly failed: got %zu expected %zu", 
                 rx_len, sizeof(large_data));
    
    /* Verify a few key positions */
    zassert_equal(rx_buf[0], large_data[0], "First byte mismatch");
    zassert_equal(rx_buf[sizeof(large_data)-1], large_data[sizeof(large_data)-1], 
                 "Last byte mismatch");
    
    printk("=== TEST PASSED (segmentation works) ===\n");
}

ZTEST_SUITE(stack_integration, 
			NULL, 
			stack_integration_setup, 
            stack_integration_before, 
			stack_integration_after, 
			NULL);
            