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
static mock_phy_context_t g_phy_ctx_pt;
static mock_phy_context_t g_phy_ctx_ft;

/* Peer lists must be static to persist outside the setup function's stack frame */
static mock_phy_context_t *pt_peers[] = { &g_phy_ctx_ft };
static mock_phy_context_t *ft_peers[] = { &g_phy_ctx_pt };

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
		} else {
			mock_phy_set_active_context(&g_phy_ctx_ft);
		}
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
	uint64_t end_ticks = k_uptime_get() + k_us_to_ticks_ceil64(timeout_us);

	while (k_uptime_get() < end_ticks) {
		if (break_cond_func && break_cond_func()) {
			return true;
		}


		/* In native_sim, time advances automatically if we allow the kernel to idle,
		 * or we can use k_sleep to advance it. */
		k_sleep(K_TICKS(1));


		uint64_t now_us = k_ticks_to_us_floor64(k_uptime_get());
		mock_phy_process_events(&g_phy_ctx_pt, now_us);
		mock_phy_process_events(&g_phy_ctx_ft, now_us);
		process_all_mac_events();
	}

	return (break_cond_func && break_cond_func());
}

static bool is_ft_beaconing(void) { return g_mac_ctx_ft.state == MAC_STATE_FT_BEACONING; }
static bool is_pt_associated(void) { return g_mac_ctx_pt.state == MAC_STATE_ASSOCIATED; }

static bool ft_received_data(void)
{
	int peer_idx = dect_mac_core_get_peer_slot_idx(g_mac_ctx_pt.own_short_rd_id);
	if (peer_idx < 0) {
		return false;
	}
	return (g_mac_ctx_ft.role_ctx.ft.connected_pts[peer_idx].last_rx_sdu_len >= 20);
}

/* --- Test Setup --- */

static void *dect_mac_flow_setup(void)
{
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
	mock_phy_init_context(&g_phy_ctx_pt, &g_mac_ctx_pt, pt_peers, ARRAY_SIZE(pt_peers));
	mock_phy_init_context(&g_phy_ctx_ft, &g_mac_ctx_ft, ft_peers, ARRAY_SIZE(ft_peers));

	/* Init FT */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	mock_phy_set_active_context(&g_phy_ctx_ft);
	err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	zassert_ok(err, "FT dect_mac_core_init failed");

	/* Init PT */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	mock_phy_set_active_context(&g_phy_ctx_pt);
	err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	zassert_ok(err, "PT dect_mac_core_init failed");

	g_mac_ctx_pt.network_id_32bit = g_mac_ctx_ft.network_id_32bit;
}

/* --- Test Cases --- */

ZTEST(dect_mac_flow, test_data_transfer)
{
	/* 1. Setup: PT is associated with FT */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx = ft_find_and_init_peer_slot(g_mac_ctx_pt.own_long_rd_id,
						  g_mac_ctx_pt.own_short_rd_id, -50 * 2);
	zassert_true(peer_idx >= 0, "FT could not allocate peer slot");

	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	memcpy(&g_mac_ctx_pt.role_ctx.pt.associated_ft,
	       &g_mac_ctx_ft.role_ctx.ft.connected_pts[peer_idx],
	       sizeof(dect_mac_peer_info_t));


	/* 2. PT sends a data packet */
	mac_sdu_t *sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(sdu, "Failed to alloc SDU");
	sdu->len = 20;
	strcpy(sdu->data, "test_data");
	zassert_ok(dect_mac_api_send(sdu, MAC_FLOW_RELIABLE_DATA), "dect_mac_api_send failed");

	/* 3. Run simulation to allow the packet to be sent and received */
	zassert_true(run_simulation_until(1000000, ft_received_data),
		     "FT did not receive the data packet");
}

ZTEST_SUITE(dect_mac_flow, NULL, dect_mac_flow_setup, dect_mac_flow_before, NULL, NULL);