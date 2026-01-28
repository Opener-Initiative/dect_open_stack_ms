/* tests/mac_ft_beacon/src/main.c */
// Overview: A focused Ztest suite to validate the FT's initialization sequence, from DCS scan to beaconing, in complete isolation.

#include <zephyr/ztest.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mocks/mock_nrf_modem_dect_phy.h>

/* The single, global context for the FT instance under test */
static dect_mac_context_t g_mac_ctx_ft;

/* Test-only function to inject timer events */
#if IS_ENABLED(CONFIG_ZTEST)
extern int dect_mac_test_inject_event_internal(struct dect_mac_context *ctx,
					       dect_mac_event_type_t event_type, int timer_id);
#endif

/**
 * @brief Helper to process all pending events for the active FT context.
 */
static void run_ft_event_loop(const char *step_name)
{
	printk("\n--- TEST: Processing events for step: %s ---\n", step_name);
	dect_mac_test_set_active_context(&g_mac_ctx_ft);

	while (dect_mac_process_event_timeout(K_NO_WAIT) == 0) {
		printk("  [EVENT_LOOP] Processed one event for FT.\n");
	}
	dect_mac_service();
	printk("--- TEST: Event loop finished for step: %s ---\n", step_name);
}

/**
 * @brief One-time setup for the entire test suite.
 */
static void *dect_ft_beacon_suite_setup(void)
{
	mock_phy_reset();
	/* Initialize shared MAC components ONCE */
	zassert_ok(dect_mac_init(NULL, NULL), "dect_mac_init failed");
	return NULL;
}

/**
 * @brief Per-test setup, runs before each test case.
 */
static void dect_ft_beacon_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Reset the FT context to a clean state */
	memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	zassert_ok(dect_mac_core_init(MAC_ROLE_FT, 0x11223344), "FT dect_mac_core_init failed");
}

ZTEST(dect_ft_beacon, test_ft_scans_and_enters_beaconing)
{
	printk("\n--- RUNNING TEST: %s ---\n", __func__);

	/* 1. Activate & Start FT. This should schedule the first RSSI scan. */
	printk("\n--> Step 1: Activating and starting FT...\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	dect_mac_start();
	run_ft_event_loop("FT Start");

	/* Verify that the first scan has been scheduled */
	mock_scheduled_operation_t *op = mock_phy_get_last_op_by_type(MOCK_OP_TYPE_RSSI);
	zassert_not_null(op, "FT did not schedule an initial RSSI scan");
	zassert_equal(g_mac_ctx_ft.state, MAC_STATE_FT_SCANNING, "FT is not in SCANNING state");

	/* 2. Loop through all DCS scans, advancing time and processing events for each one. */
	printk("\n--> Step 2: Simulating all %d FT DCS scans...\n", CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN);
	for (int i = 0; i < CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN; i++) {
		/* Advance time far enough to complete one scan */
		mock_phy_advance_time_us(200 * 1000);
		run_ft_event_loop("FT Scan Step");
	}

	/* 3. Verify the FT has completed scanning and is now in the BEACONING state. */
	printk("\n--> Step 3: Verifying FT has entered BEACONING state...\n");
	zassert_equal(g_mac_ctx_ft.state, MAC_STATE_FT_BEACONING, "FT did not enter BEACONING state after scans");

	/* 4. Manually inject the beacon timer event to force a beacon transmission. */
	printk("\n--> Step 4: Injecting beacon timer event...\n");
	dect_mac_test_inject_event_internal(&g_mac_ctx_ft, MAC_EVENT_TIMER_EXPIRED_BEACON, 0);
	run_ft_event_loop("After Beacon Timer");

	/* 5. Verify a beacon was actually transmitted. */
	printk("\n--> Step 5: Verifying beacon was scheduled...\n");
	op = mock_phy_get_last_op_by_type(MOCK_OP_TYPE_TX);
	zassert_not_null(op, "FT did not schedule a beacon TX after timer event");

	/* Check that the PDU looks like a beacon (MAC Header Type = 1) */
	zassert_true(op->pdu_len > 0, "Beacon PDU has zero length");
	uint8_t mac_hdr_type = (op->pdu_data[0] >> 0) & 0x0F;
	zassert_equal(mac_hdr_type, MAC_COMMON_HEADER_TYPE_BEACON, "Transmitted PDU is not a beacon");
}

ZTEST_SUITE(dect_ft_beacon, NULL, dect_ft_beacon_suite_setup, dect_ft_beacon_before, NULL, NULL);