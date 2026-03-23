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
    dect_cvg_configure_flow(CVG_SERVICE_TYPE_0_TRANSPARENT, 4, 100, 5000);

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

