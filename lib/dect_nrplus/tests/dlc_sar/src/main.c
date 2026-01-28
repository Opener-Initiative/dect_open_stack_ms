// // /* lib/dect_nrplus/tests/dlc_sar/src/main.c */
// // // Overview: Ztest suite for DLC Segmentation and Reassembly. It mocks the MAC layer to verify the DLC's behavior in isolation.

// // /* This dlist is what the DLC will put received data on for the "app" (our test) to consume */
// // static sys_dlist_t g_app_rx_dlist;

// // /* The DLC's RX thread will get data from this dlist, which our test controls */
// // extern sys_dlist_t g_dlc_internal_mac_rx_dlist;


// #include <zephyr/ztest.h>
// #include <string.h>
// #include <dect_dlc.h>
// #include <zephyr/sys/dlist.h>
// #include <mac/dect_mac.h>
// #include <mac/dect_mac_core.h>
// #include <mac/dect_mac_pdu.h>

// #include "../../tests/utils/test_harness_helpers.h"
// #include <zephyr/drivers/timer/system_timer.h> 

// /* --- Test Globals & Mocks --- */
// #define MAX_CAPTURED_PDUS 10
// static mac_sdu_t g_captured_pdus[MAX_CAPTURED_PDUS];
// static int g_capture_count;
// static dect_mac_context_t g_mac_ctx;

// /* This dlist is what the DLC will put received data on for the "app" (our test) to consume */
// static sys_dlist_t g_app_rx_dlist;

// /* The DLC's RX thread will get data from this dlist, which our test controls */
// extern sys_dlist_t g_dlc_internal_mac_rx_dlist;
// extern k_tid_t g_dlc_rx_thread_id;

// /* Test-specific spy function to intercept calls to dect_mac_send */
// static int test_send_spy_callback(mac_sdu_t *sdu, mac_flow_id_t flow)
// {
// 	if (g_capture_count < MAX_CAPTURED_PDUS) {
// 		/* Copy the SDU content for inspection. */
// 		memcpy(&g_captured_pdus[g_capture_count], sdu, sizeof(mac_sdu_t));
// 		g_capture_count++;
// 	}
// 	/* The spy must free the buffer, as the real MAC would eventually. */
// 	dect_mac_buffer_free(sdu);
// 	return 0;
// }

// static void dlc_sar_before(void *fixture)
// {
// 	ARG_UNUSED(fixture);
// 	g_capture_count = 0;
// 	memset(g_captured_pdus, 0, sizeof(g_captured_pdus));
// 	sys_dlist_init(&g_app_rx_dlist);

// 	/* Set the active context pointer BEFORE calling any core functions */
// 	dect_mac_test_set_active_context(&g_mac_ctx);

// 	/* Initialize MAC and DLC layers */
// 	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
// 	dect_dlc_init();

// 	/* Register our spy handler */
// 	dect_mac_test_set_send_spy(test_send_spy_callback);
// }

// static void dlc_sar_after(void *fixture)
// {
// 	ARG_UNUSED(fixture);
// 	/* Restore the real send handler to avoid affecting other test suites */
// 	dect_mac_test_set_send_spy(NULL);
// }

// ZTEST(dect_dlc_sar, test_tx_segmentation)
// {
// 	uint8_t large_sdu[500];
// 	for (int i = 0; i < sizeof(large_sdu); i++) {
// 		large_sdu[i] = (uint8_t)i;
// 	}

// 	dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x11223344, large_sdu, sizeof(large_sdu));

// 	zassert_true(g_capture_count > 1, "SDU was not segmented (capture_count=%d)", g_capture_count);

// 	/* Verify first segment */
// 	const dect_dlc_header_type123_basic_t *hdr0 = (const void *)g_captured_pdus[0].data;
// 	zassert_equal(dlc_hdr_t123_basic_get_si(hdr0), DLC_SI_FIRST_SEGMENT, "First segment SI is wrong");
// 	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr0);

// 	/* Verify middle and last segments */
// 	for (int i = 1; i < g_capture_count; i++) {
// 		const dect_dlc_header_type13_segmented_t *hdr = (const void *)g_captured_pdus[i].data;
// 		zassert_equal(dlc_hdr_t123_basic_get_sn((const void *)hdr), sn, "Segment %d has wrong SN", i);
// 		if (i < g_capture_count - 1) {
// 			zassert_equal(dlc_hdr_t123_basic_get_si((const void *)hdr), DLC_SI_MIDDLE_SEGMENT, "Middle segment %d SI is wrong", i);
// 		} else {
// 			zassert_equal(dlc_hdr_t123_basic_get_si((const void *)hdr), DLC_SI_LAST_SEGMENT, "Last segment SI is wrong");
// 		}
// 	}
// }


// ZTEST(dect_dlc_sar, test_rx_reassembly_in_order)
// {
// 	uint8_t large_sdu[500];
// 	uint8_t reassembled_sdu[500];
// 	size_t reassembled_len = sizeof(reassembled_sdu);
// 	dlc_service_type_t service;

// 	for (int i = 0; i < sizeof(large_sdu); i++) {
// 		large_sdu[i] = (uint8_t)i;
// 	}

// 	/* Manually create and inject segments in the correct order */
// 	dect_dlc_header_type123_basic_t hdr_first;
// 	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 123);
// 	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
// 	zassert_not_null(seg0, "Failed to allocate seg0");
// 	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
// 	memcpy(seg0->data + sizeof(hdr_first), large_sdu, 254);
// 	seg0->len = sizeof(hdr_first) + 254;
// 	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg0->node);

// 	dect_dlc_header_type13_segmented_t hdr_last;
// 	dlc_hdr_t13_segmented_set(&hdr_last, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_LAST_SEGMENT, 123, 254);
// 	mac_sdu_t *seg1 = dect_mac_buffer_alloc(K_NO_WAIT);
// 	zassert_not_null(seg1, "Failed to allocate seg1");
// 	memcpy(seg1->data, &hdr_last, sizeof(hdr_last));
// 	memcpy(seg1->data + sizeof(hdr_last), large_sdu + 254, 246);
// 	seg1->len = sizeof(hdr_last) + 246;
// 	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg1->node);

// 	/* A brief sleep allows the higher-priority DLC RX thread to run and process all queued items */
// 	k_sleep(K_MSEC(10));

// 	/* Attempt to receive the reassembled packet */
// 	int ret = dlc_receive_data(&service, reassembled_sdu, &reassembled_len, K_NO_WAIT);
// 	zassert_ok(ret, "dlc_receive_data failed: %d", ret);
// 	zassert_equal(reassembled_len, sizeof(large_sdu), "Reassembled length is wrong");
// 	zassert_mem_equal(reassembled_sdu, large_sdu, sizeof(large_sdu), "Reassembled data is corrupt");
// }

// ZTEST(dect_dlc_sar, test_rx_reassembly_out_of_order)
// {
// 	uint8_t large_sdu[500];
// 	uint8_t reassembled_sdu[500];
// 	size_t reassembled_len = sizeof(reassembled_sdu);
// 	dlc_service_type_t service;

// 	for (int i = 0; i < sizeof(large_sdu); i++) {
// 		large_sdu[i] = (uint8_t)i;
// 	}

// 	/* Manually create and inject segments in the WRONG order */
// 	dect_dlc_header_type13_segmented_t hdr_last;
// 	dlc_hdr_t13_segmented_set(&hdr_last, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_LAST_SEGMENT, 124, 254);
// 	mac_sdu_t *seg1 = dect_mac_buffer_alloc(K_NO_WAIT);
// 	zassert_not_null(seg1, "Failed to allocate seg1");
// 	memcpy(seg1->data, &hdr_last, sizeof(hdr_last));
// 	memcpy(seg1->data + sizeof(hdr_last), large_sdu + 254, 246);
// 	seg1->len = sizeof(hdr_last) + 246;
// 	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg1->node);

// 	dect_dlc_header_type123_basic_t hdr_first;
// 	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 124);
// 	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
// 	zassert_not_null(seg0, "Failed to allocate seg0");
// 	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
// 	memcpy(seg0->data + sizeof(hdr_first), large_sdu, 254);
// 	seg0->len = sizeof(hdr_first) + 254;
// 	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg0->node);

// 	/* A brief sleep allows the higher-priority DLC RX thread to run and process all queued items */
// 	k_sleep(K_MSEC(10));

// 	/* Attempt to receive the reassembled packet */
// 	int ret = dlc_receive_data(&service, reassembled_sdu, &reassembled_len, K_NO_WAIT);
// 	zassert_ok(ret, "dlc_receive_data failed: %d", ret);
// 	zassert_equal(reassembled_len, sizeof(large_sdu), "Reassembled length is wrong");
// 	zassert_mem_equal(reassembled_sdu, large_sdu, sizeof(large_sdu), "Reassembled data is corrupt");
// }

// ZTEST(dect_dlc_sar, test_rx_reassembly_timeout)
// {
// 	/* Inject only the first segment of an SDU */
// 	dect_dlc_header_type123_basic_t hdr_first;
// 	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 125);
// 	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
// 	zassert_not_null(seg0, "Failed to allocate seg0");
// 	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
// 	seg0->len = sizeof(hdr_first) + 100;
// 	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg0->node);

// 	/* A brief sleep allows the higher-priority DLC RX thread to run and process all queued items */
// 	k_sleep(K_MSEC(10));

// 	/* Advance time past the reassembly timeout */
// 	k_sleep(K_MSEC(5000 + 100));

// 	/* Attempt to receive data - should fail as the session was timed out */
// 	uint8_t dummy_buf[10];
// 	size_t len = sizeof(dummy_buf);
// 	dlc_service_type_t service;
// 	int ret = dlc_receive_data(&service, dummy_buf, &len, K_NO_WAIT);
// 	zassert_equal(ret, -EAGAIN, "dlc_receive_data should have returned -EAGAIN after timeout");
// }

// ZTEST_SUITE(dect_dlc_sar, NULL, NULL, dlc_sar_before, dlc_sar_after, NULL);


#include <zephyr/ztest.h>
#include <string.h>
#include <dect_dlc.h>
#include <zephyr/sys/dlist.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>

/* --- Test Globals & Mocks --- */
#define TEST_DLC_REASSEMBLY_TIMEOUT_MS 500
#define MAX_CAPTURED_PDUS 10
static mac_sdu_t g_captured_pdus[MAX_CAPTURED_PDUS];
static int g_capture_count;
static dect_mac_context_t g_mac_ctx;

/* This dlist is what the DLC will put received data on for the "app" (our test) to consume */
static sys_dlist_t g_app_rx_dlist;

/* The DLC's RX thread will get data from this dlist, which our test controls */
extern sys_dlist_t g_dlc_internal_mac_rx_dlist;

extern k_tid_t g_dlc_rx_thread_id;

/* Test-specific spy function to intercept calls to dect_mac_send */
static int test_send_spy_callback(mac_sdu_t *sdu, mac_flow_id_t flow)
{
	if (g_capture_count < MAX_CAPTURED_PDUS) {
		/* Copy the SDU content for inspection. */
		memcpy(&g_captured_pdus[g_capture_count], sdu, sizeof(mac_sdu_t));
		g_capture_count++;
	}
	/* The spy must free the buffer, as the real MAC would eventually. */
	dect_mac_buffer_free(sdu);
	return 0;
}

static void dlc_sar_before(void *fixture)
{
	ARG_UNUSED(fixture);
	g_capture_count = 0;
	memset(g_captured_pdus, 0, sizeof(g_captured_pdus));
	sys_dlist_init(&g_app_rx_dlist);

	/* Set the active context pointer BEFORE calling any core functions */
	dect_mac_test_set_active_context(&g_mac_ctx);

	/* Initialize MAC and DLC layers */
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	dect_dlc_init();

    /* Start the DLC RX thread so it can process packets we inject */
    printk("[TEST_SETUP_DBG] Calling k_thread_start on g_dlc_rx_thread_id (%p)...\n", g_dlc_rx_thread_id);
	k_thread_start(g_dlc_rx_thread_id);

	/* Register our spy handler */
	dect_mac_test_set_send_spy(test_send_spy_callback);
}

static void dlc_sar_after(void *fixture)
{
	ARG_UNUSED(fixture);

    /* Stop the thread to ensure a clean state for the next test */
	k_thread_abort(g_dlc_rx_thread_id);

	/* Restore the real send handler to avoid affecting other test suites */
	dect_mac_test_set_send_spy(NULL);
}

ZTEST(dect_dlc_sar, test_tx_segmentation)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t large_sdu[2500];
	for (int i = 0; i < sizeof(large_sdu); i++) {
		large_sdu[i] = (uint8_t)i;
	}

	dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x11223344, large_sdu, sizeof(large_sdu));
printk("xxxxxxxxxxxxxxxxxx\n");
	zassert_true(g_capture_count > 1, "SDU was not segmented (capture_count=%d)", g_capture_count);
printk("xxxxxxxxxxxxxxxxxx\n");
	/* Verify first segment */
	const dect_dlc_header_type123_basic_t *hdr0 = (const void *)g_captured_pdus[0].data;
	zassert_equal(dlc_hdr_t123_basic_get_si(hdr0), DLC_SI_FIRST_SEGMENT, "First segment SI is wrong");
	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr0);

	/* Verify middle and last segments */
	for (int i = 1; i < g_capture_count; i++) {
		const dect_dlc_header_type13_segmented_t *hdr = (const void *)g_captured_pdus[i].data;
		zassert_equal(dlc_hdr_t123_basic_get_sn((const void *)hdr), sn, "Segment %d has wrong SN", i);
		if (i < g_capture_count - 1) {
			zassert_equal(dlc_hdr_t123_basic_get_si((const void *)hdr), DLC_SI_MIDDLE_SEGMENT, "Middle segment %d SI is wrong", i);
		} else {
			zassert_equal(dlc_hdr_t123_basic_get_si((const void *)hdr), DLC_SI_LAST_SEGMENT, "Last segment SI is wrong");
		}
	}
}


ZTEST(dect_dlc_sar, test_rx_reassembly_timeout)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);
	printk("\n--- [TEST] Starting test_rx_reassembly_timeout ---\n");

	/* 1. Inject only the first segment of an SDU */
	printk("[TEST] 1. Queuing FIRST segment (SN=125) to trigger reassembly session...\n");
	dect_dlc_header_type123_basic_t hdr_first;
	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 125);
	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(seg0, "Failed to allocate seg0");
	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
	seg0->len = sizeof(hdr_first) + 100;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg0->node);

	/* 2. Give the DLC RX thread a slice of time to process the segment and start the timeout timer */
	printk("[TEST] 2. Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(10));

	/* 3. Verify that no complete packet is available yet */
    printk("[TEST] 3. Verify that no complete packet is available yet */\n");
	uint8_t dummy_buf[10];
	size_t len = sizeof(dummy_buf);
	dlc_service_type_t service;
	uint32_t source_addr;

	// int ret = dlc_receive_data(&service, dummy_buf, &len, K_NO_WAIT);
	int ret = dlc_receive_data(&service, &source_addr, dummy_buf, &len, K_NO_WAIT);
	zassert_equal(ret, -EAGAIN, "dlc_receive_data should not find a complete packet yet");

	/* 4. Advance time past the reassembly timeout to trigger the timer handler */
	printk("[TEST] Advancing time %dms to trigger reassembly timeout...\n", TEST_DLC_REASSEMBLY_TIMEOUT_MS + 100);
	k_sleep(K_MSEC(TEST_DLC_REASSEMBLY_TIMEOUT_MS + 100));

	/* 5. Attempt to receive data again - it should still fail because the session was discarded */
	printk("[TEST] Attempting to receive data after timeout...\n");
	// ret = dlc_receive_data(&service, dummy_buf, &len, K_NO_WAIT);
	ret = dlc_receive_data(&service, &source_addr, dummy_buf, &len, K_NO_WAIT);
	zassert_equal(ret, -EAGAIN, "dlc_receive_data should have returned -EAGAIN after timeout");
	printk("[TEST] --- Test PASSED ---\n");
}


ZTEST(dect_dlc_sar, test_rx_reassembly_out_of_order)
{
	uint8_t cvg_payload[500];
	uint8_t reassembled_sdu[512];
	size_t reassembled_len = sizeof(reassembled_sdu);
	dlc_service_type_t service;
	uint32_t source_addr;

	printk("\n--- [TEST] Starting test_rx_reassembly_out_of_order ---\n");

	for (int i = 0; i < sizeof(cvg_payload); i++) {
		cvg_payload[i] = (uint8_t)i;
		dlc_receive_data(&service, &source_addr, reassembled_sdu, &reassembled_len, K_NO_WAIT);
	}

	/* 1. Create the full SDU payload, including the routing header */
	uint8_t full_sdu_payload[sizeof(dect_dlc_routing_header_t) + sizeof(cvg_payload)];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
	rh.source_addr_be = 0x12345678;
	rh.dest_addr_be = g_mac_ctx.own_long_rd_id;
	rh.bitmap_be = sys_cpu_to_be16(bitmap);
	int rh_len = dlc_serialize_routing_header(full_sdu_payload, sizeof(full_sdu_payload), &rh);
	zassert_true(rh_len > 0, "Failed to serialize routing header");
	memcpy(full_sdu_payload + rh_len, cvg_payload, sizeof(cvg_payload));
	size_t full_sdu_len = rh_len + sizeof(cvg_payload);


	/* 2. Manually segment the full SDU and inject segments in the WRONG order */
	size_t first_seg_payload_len = 254;
	size_t last_seg_payload_len = full_sdu_len - first_seg_payload_len;

	printk("[TEST] Queuing LAST segment (SN=124, offset=%zu, len=%zu)...\n", first_seg_payload_len, last_seg_payload_len);
	dect_dlc_header_type13_segmented_t hdr_last;
	dlc_hdr_t13_segmented_set(&hdr_last, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_LAST_SEGMENT, 124, first_seg_payload_len);
	mac_sdu_t *seg1 = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(seg1, "Failed to allocate seg1");
	memcpy(seg1->data, &hdr_last, sizeof(hdr_last));
	memcpy(seg1->data + sizeof(hdr_last), full_sdu_payload + first_seg_payload_len, last_seg_payload_len);
	seg1->len = sizeof(hdr_last) + last_seg_payload_len;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg1->node);

	printk("[TEST] Queuing FIRST segment (SN=124, offset=0, len=%zu)...\n", first_seg_payload_len);
	dect_dlc_header_type123_basic_t hdr_first;
	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 124);
	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(seg0, "Failed to allocate seg0");
	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
	memcpy(seg0->data + sizeof(hdr_first), full_sdu_payload, first_seg_payload_len);
	seg0->len = sizeof(hdr_first) + first_seg_payload_len;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &seg0->node);

	/* 3. Give the DLC RX thread time to process both queued segments */
	printk("[TEST] Yielding to allow DLC thread to process both segments...\n");
	k_sleep(K_MSEC(50));

	/* 4. Attempt to receive the now reassembled packet */
	printk("[TEST] Attempting to receive reassembled packet...\n");
	int ret = dlc_receive_data(&service, &source_addr, reassembled_sdu, &reassembled_len, K_NO_WAIT);
	printk("[TEST] dlc_receive_data returned: %d\n", ret);

	zassert_ok(ret, "dlc_receive_data failed: %d", ret);
	zassert_equal(reassembled_len, sizeof(cvg_payload), "Reassembled length is wrong");
	zassert_mem_equal(reassembled_sdu, cvg_payload, sizeof(cvg_payload), "Reassembled data is corrupt");
	printk("[TEST] --- Test PASSED ---\n");
}




ZTEST_SUITE(dect_dlc_sar, NULL, NULL, dlc_sar_before, dlc_sar_after, NULL);