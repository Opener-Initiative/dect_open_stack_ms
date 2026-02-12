
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <nrf_modem_dect_phy.h>
#include "dect_mac_sm.h"
#include "dect_mac_phy_if.h"
#include "dect_mac_core.h"
#include "dect_mac_context.h"
#include "dect_mac_sm_pt.h"
#include "dect_mac_sm_ft.h"
#include <mac/dect_mac_main_dispatcher.h>
#include "mock_nrf_modem_dect_phy.h"

extern void mock_phy_reset(void);
extern void dect_mac_phy_if_event_handler(const struct nrf_modem_dect_phy_event *event);

struct mac_error_tests_fixture {
	dect_mac_context_t ft_ctx;
	dect_mac_context_t pt_ctx;
	dect_mac_context_t *current_ctx;
};

static struct mac_error_tests_fixture g_harness;

dect_mac_context_t *get_mac_context(void) { return g_harness.current_ctx; }
K_MSGQ_DEFINE(mac_event_msgq, sizeof(struct dect_mac_event_msg), 16, 4);

static void *setup(void)
{
	mock_phy_reset();
	nrf_modem_dect_phy_event_handler_set(dect_mac_phy_if_event_handler);
	return &g_harness;
}

static void before(void *data)
{
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
}

ZTEST_F(mac_error_tests, test_rach_max_retries)
{
	g_harness.current_ctx = &g_harness.pt_ctx;
	g_harness.pt_ctx.config.max_assoc_retries = 2;
	g_harness.pt_ctx.config.rach_response_window_ms = 10;

	dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);
	g_harness.pt_ctx.role_ctx.pt.target_ft.is_valid = true;
	g_harness.pt_ctx.role_ctx.pt.target_ft.is_fully_identified = true;

	/* Attempt 1 -> Timeout */
	pt_rach_response_window_timer_expired_action();
	zassert_equal(g_harness.pt_ctx.role_ctx.pt.current_assoc_retries, 1);
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_ASSOCIATING);

	/* Attempt 2 -> Timeout */
	pt_rach_response_window_timer_expired_action();
	zassert_equal(g_harness.pt_ctx.role_ctx.pt.current_assoc_retries, 2);
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_ASSOCIATING);

	/* Attempt 3 -> Max retries reached, should restart scan */
	pt_rach_response_window_timer_expired_action();
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_SCANNING, "Did not restart scan after max retries");
}

ZTEST_SUITE(mac_error_tests, NULL, setup, before, NULL, NULL);