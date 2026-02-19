/* lib/dect_nrplus/tests/mac_flow/src/main.c */
// Overview: Reworks the `mac_flow` test suite to use the new, robust test harness.
// --- CREATE NEW FILE ---
#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include "../../tests/utils/test_harness_helpers.h"

/* --- Test Globals --- */
uint8_t g_last_tx_pdu_capture[CONFIG_DECT_MAC_PDU_MAX_SIZE];
uint16_t g_last_tx_pdu_len_capture = 0;

static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft;
static dect_mac_context_t g_mac_test_ctx;
static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;

/* Peer lists must be static to persist outside the setup function's stack frame */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };

#if IS_ENABLED(CONFIG_ZTEST)
extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					       dect_mac_event_type_t event_type, int timer_id);
extern void ft_service_schedules(void);
#endif

/* --- Test Harness Functions --- */

static void process_all_mac_events(void)
{
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) {
		zassert_not_null(msg.ctx, "Event in queue has NULL context!");
		mock_phy_set_active_context(msg.ctx == &g_mac_ctx_pt ? &g_phy_ctx_pt : &g_phy_ctx_ft);
		dect_mac_test_set_active_context(msg.ctx);
		dect_mac_event_dispatch(&msg);
	}

	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	dect_mac_service();

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	dect_mac_service();
}

static void ft_service_schedules_if_due(uint64_t now_us)
{
	/* [SYNC FIX] Only trigger FT scheduling when we are close to a scheduled occurrence.
	 */
	uint64_t next_occurrence_us = UINT64_MAX;
	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (g_mac_ctx_ft.role_ctx.ft.connected_pts[i].is_valid &&
		    g_mac_ctx_ft.role_ctx.ft.peer_schedules[i].is_active) {
			/* Convert ticks to us for comparison */
			uint64_t occ_us = k_ticks_to_us_floor64(g_mac_ctx_ft.role_ctx.ft.peer_schedules[i].next_occurrence_modem_time);
			next_occurrence_us = MIN(next_occurrence_us, occ_us);
		}
	}

	/* Trigger if we are within 2ms of the next event */
	if (next_occurrence_us != UINT64_MAX && now_us + 2000 >= next_occurrence_us) {
		dect_mac_test_set_active_context(&g_mac_ctx_ft);
		mock_phy_set_active_context(&g_phy_ctx_ft);
		ft_service_schedules();
	}
}

static bool run_simulation_until(uint64_t timeout_us, bool (*break_cond_func)(void))
{
	uint64_t end_time_us = k_ticks_to_us_floor64(k_uptime_ticks()) + timeout_us;
	mock_phy_context_t *all_phys[] = { &g_phy_ctx_pt, &g_phy_ctx_ft };
	uint32_t iteration_count = 0;
	uint64_t last_now_us = k_ticks_to_us_floor64(k_uptime_ticks());

	printk("[SIM] Starting flow test simulation. Timeout: %llu us\n", timeout_us);

	while (k_ticks_to_us_floor64(k_uptime_ticks()) < end_time_us) {
		iteration_count++;
		if (iteration_count > 20000) {
			printk("[SIM_ERR] Maximum iterations reached!\n");
			break;
		}

		if (break_cond_func && break_cond_func()) {
			printk("[SIM] Break condition met at %llu us\n", k_ticks_to_us_floor64(k_uptime_ticks()));
			return true;
		}

		uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
		uint64_t next_phy_event_time = mock_phy_get_next_event_time(all_phys, 2);
		
		uint64_t next_event_time = MIN(next_phy_event_time, end_time_us);
		uint64_t time_to_advance_us;

		if (next_event_time > now_us) {
			time_to_advance_us = next_event_time - now_us;
		} else {
			time_to_advance_us = 1; /* Minimal step */
		}

		/* Limit step size for kernel tick granularity */
		time_to_advance_us = MIN(time_to_advance_us, 1000);

		if (time_to_advance_us > 0) {
			if (time_to_advance_us <= 1500) {
				k_busy_wait(time_to_advance_us);
			} else {
				k_sleep(K_USEC(time_to_advance_us));
			}
		}

		now_us = k_ticks_to_us_floor64(k_uptime_ticks());
		if (now_us > last_now_us) {
			mock_phy_process_events(&g_phy_ctx_pt, now_us);
			mock_phy_process_events(&g_phy_ctx_ft, now_us);
			
			/* Check if FT scheduler needs to run */
			ft_service_schedules_if_due(now_us);
			
			process_all_mac_events();
			last_now_us = now_us;
		}
	}

	return (break_cond_func && break_cond_func());
}

static bool ft_received_data(void)
{
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	int peer_idx = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
	if (peer_idx < 0) {
		return false;
	}
	bool received = (g_mac_ctx_ft.role_ctx.ft.connected_pts[peer_idx].last_rx_sdu_len >= 20);
	return received;
}

/* --- Test Setup --- */

static void *dect_mac_flow_setup(void)
{
	dect_mac_test_set_active_context(&g_mac_test_ctx);
	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	return NULL;
}

static void dect_mac_flow_before(void *fixture)
{
	ARG_UNUSED(fixture);
	int err;

	dect_mac_phy_if_reset_handle_map();

	memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
	memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
	
	mock_phy_complete_reset(&g_phy_ctx_pt);
	mock_phy_complete_reset(&g_phy_ctx_ft);
	
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));

	/* 1. Init FT */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	zassert_ok(err, "FT dect_mac_core_init failed");
	zassert_ok(nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY), "FT PHY activate failed");

	/* 2. Init PT */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	zassert_ok(err, "PT dect_mac_core_init failed");
	zassert_ok(nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY), "PT PHY activate failed");

	g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;
	printk("[TEST_DBG] FT and PT initialized and PHYs activated.\n");
}

/* --- Test Cases --- */

ZTEST(dect_mac_flow, test_data_transfer)
{
	/* 1. Setup: PT is associated with FT */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	dect_mac_change_state(MAC_STATE_FT_BEACONING);
	int peer_idx = ft_find_and_init_peer_slot(g_mac_ctx_pt.own_long_rd_id,
						  g_mac_ctx_pt.own_short_rd_id, -50 * 2);
	zassert_true(peer_idx >= 0, "FT could not allocate peer slot");

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	
	/* Correctly point PT back to FT */
	g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id = g_mac_ctx_ft.own_long_rd_id;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.short_rd_id = g_mac_ctx_ft.own_short_rd_id;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.is_valid = true;

	/* 1.1. Provide PT with an uplink schedule so it can transmit */
	dect_mac_schedule_t *ul_sched = &g_mac_ctx_pt.role_ctx.pt.ul_schedule;
	ul_sched->is_active = true;
	ul_sched->alloc_type = RES_ALLOC_TYPE_UPLINK;
	ul_sched->ul_start_subslot = 10;
	ul_sched->ul_duration_subslots = 10;
	ul_sched->repeat_type = RES_ALLOC_REPEAT_FRAMES;
	ul_sched->repetition_value = 1; /* Every frame */
	ul_sched->validity_value = 0xFF; /* Permanent */
	ul_sched->channel = 0;
	
	/* 100ms offset in ticks */
	uint64_t now_ticks = k_uptime_ticks();
	uint32_t offset_ticks = 100 * (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
	ul_sched->next_occurrence_modem_time = now_ticks + offset_ticks;
	ul_sched->schedule_init_modem_time = now_ticks;

	/* 1.2. FT must also know this schedule for the peer to listen */
	dect_mac_schedule_t *ft_peer_sched = &g_mac_ctx_ft.role_ctx.ft.peer_schedules[peer_idx];
	memcpy(ft_peer_sched, ul_sched, sizeof(dect_mac_schedule_t));
	ft_peer_sched->is_active = true; /* Ensure active on FT side too */

	printk("[TEST_DBG] PT and FT schedules synchronized. Sched start (ticks): %llu\n", 
	       ul_sched->next_occurrence_modem_time);


	/* 2. PT sends a data packet */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);

	mac_sdu_t *sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(sdu, "Failed to alloc SDU");
	sdu->len = 20;
	strcpy(sdu->data, "test_data_0123456789");
	zassert_ok(dect_mac_api_send(sdu, MAC_FLOW_RELIABLE_DATA), "dect_mac_api_send failed");

	/* 3. Run simulation to allow the packet to be sent and received */
	printk("[TEST_DBG] Starting simulation to wait for data reception...\n");
	
	zassert_true(run_simulation_until(1000000, ft_received_data),
		     "FT did not receive the data packet");
	printk("[TEST_DBG] Data transfer test PASSED.\n");
}

ZTEST_SUITE(dect_mac_flow, NULL, dect_mac_flow_setup, dect_mac_flow_before, NULL, NULL);
