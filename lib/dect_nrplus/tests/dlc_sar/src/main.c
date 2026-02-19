#include <zephyr/ztest.h>
#include <string.h>
#include <dect_dlc.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>

/* --- Test Globals & Mocks --- */
#define TEST_DLC_REASSEMBLY_TIMEOUT_MS 500
#define MAX_CAPTURED_PDUS 100
static mac_sdu_t g_captured_pdus[MAX_CAPTURED_PDUS];
static int g_capture_count;
static dect_mac_context_t g_mac_ctx;


/* The DLC's RX thread will get data from this queue, which our test controls */
extern struct k_queue g_dlc_internal_mac_rx_queue;

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

	/* Register our spy handler */
	dect_mac_test_set_send_spy(test_send_spy_callback);

	/* Reset duplicate detection cache for test isolation */
	dect_dlc_test_reset_duplicate_cache();
}

static void dlc_sar_after(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Restore the real send handler to avoid affecting other test suites */
	dect_mac_test_set_send_spy(NULL);
}

static void *dlc_sar_suite_setup(void)
{
	/* Set the active context pointer BEFORE calling any core functions */
	dect_mac_test_set_active_context(&g_mac_ctx);

	/* Initialize MAC and DLC layers once for the whole suite */
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	dect_dlc_init();
	return NULL;
}

ZTEST(dect_dlc_sar, test_tx_segmentation)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t large_sdu[2500];
	for (int i = 0; i < sizeof(large_sdu); i++) {
		large_sdu[i] = (uint8_t)i;
	}

	dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x11223344, large_sdu, sizeof(large_sdu), MAC_FLOW_RELIABLE_DATA);

	zassert_true(g_capture_count > 1, "SDU was not segmented (capture_count=%d)", g_capture_count);

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
	k_queue_append(&g_dlc_internal_mac_rx_queue, seg0);

	/* 2. Give the DLC RX thread a slice of time to process the segment and start the timeout timer */
	printk("[TEST] 2. Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(10));

	/* 3. Verify that no complete packet is available yet */
	uint8_t dummy_buf[10];
	size_t len = sizeof(dummy_buf);
	dlc_service_type_t service;
	uint32_t source_addr;

	int ret = dlc_receive_data(&service, &source_addr, dummy_buf, &len, K_NO_WAIT);
	zassert_equal(ret, -EAGAIN, "dlc_receive_data should not find a complete packet yet");

	/* 4. Advance time past the reassembly timeout to trigger the timer handler */
	printk("[TEST] Advancing time %dms to trigger reassembly timeout...\n", TEST_DLC_REASSEMBLY_TIMEOUT_MS + 100);
	k_sleep(K_MSEC(TEST_DLC_REASSEMBLY_TIMEOUT_MS + 100));

	/* 5. Attempt to receive data again - it should still fail because the session was discarded */
	printk("[TEST] Attempting to receive data after timeout...\n");
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
	}

	/* 1. Create the full SDU payload, including the routing header */
	uint8_t full_sdu_payload[sizeof(dect_dlc_routing_header_t) + sizeof(cvg_payload)];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
	rh.source_addr = 0x12345678;
	rh.dest_addr = 0xAABBCCDD;
	rh.bitmap = bitmap;
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
	k_queue_append(&g_dlc_internal_mac_rx_queue, seg1);

	printk("[TEST] Queuing FIRST segment (SN=124, offset=0, len=%zu)...\n", first_seg_payload_len);
	dect_dlc_header_type123_basic_t hdr_first;
	dlc_hdr_t123_basic_set(&hdr_first, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_FIRST_SEGMENT, 124);
	mac_sdu_t *seg0 = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(seg0, "Failed to allocate seg0");
	memcpy(seg0->data, &hdr_first, sizeof(hdr_first));
	memcpy(seg0->data + sizeof(hdr_first), full_sdu_payload, first_seg_payload_len);
	seg0->len = sizeof(hdr_first) + first_seg_payload_len;
	k_queue_append(&g_dlc_internal_mac_rx_queue, seg0);

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

ZTEST(dect_dlc_sar, test_sar_min_sdu)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);
    uint8_t min_sdu = 0xA5;
    uint8_t rx_buf[16];
    size_t rx_len = sizeof(rx_buf);
    dlc_service_type_t service;
    uint32_t source_addr;

    /* Type 1 (Segmentation) must handle 1-byte SDUs correctly */
    int ret = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x44332211, &min_sdu, 1, MAC_FLOW_RELIABLE_DATA);
    zassert_ok(ret, "dlc_send_data (1B) failed");

    /* Should result in a single COMPLETE_SDU PDU */
    zassert_equal(g_capture_count, 1, "Expected exactly 1 captured PDU for 1B SDU");
    const dect_dlc_header_type123_basic_t *hdr = (const void *)g_captured_pdus[0].data;
    zassert_equal(dlc_hdr_t123_basic_get_si(hdr), DLC_SI_COMPLETE_SDU, "Expected SI=Complete SDU");

    /* Mock RX back to DLC */
    mac_sdu_t *mac_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
    zassert_not_null(mac_sdu, "Failed to allocate buffer for RX injection");
    memcpy(mac_sdu->data, g_captured_pdus[0].data, g_captured_pdus[0].len);
    mac_sdu->len = g_captured_pdus[0].len;
    k_queue_append(&g_dlc_internal_mac_rx_queue, mac_sdu);

    k_sleep(K_MSEC(10));
    ret = dlc_receive_data(&service, &source_addr, rx_buf, &rx_len, K_NO_WAIT);
    zassert_ok(ret, "dlc_receive_data (1B) failed");
    zassert_equal(rx_len, 1, "Expected 1 byte received");
    zassert_equal(rx_buf[0], 0xA5, "Wrong data received");
}

ZTEST(dect_dlc_sar, test_sar_max_mtu_sdu)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);
    static uint8_t mtu_sdu[1280];
    static uint8_t rx_buf[1300];
    size_t rx_len = sizeof(rx_buf);
    dlc_service_type_t service;
    uint32_t source_addr;

    for (int i=0; i<sizeof(mtu_sdu); i++) mtu_sdu[i] = (uint8_t)(i & 0xFF);

    int ret = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x44332211, mtu_sdu, sizeof(mtu_sdu), MAC_FLOW_RELIABLE_DATA);
    zassert_ok(ret, "dlc_send_data (1280B) failed");

    printk("[TEST] SDU 1280B resulted in %d segments\n", g_capture_count);
    zassert_true(g_capture_count >= 1, "SDU disappeared");

    /* Mock RX loop */
    for (int i=0; i<g_capture_count; i++) {
        mac_sdu_t *mac_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
        zassert_not_null(mac_sdu, "Failed to allocate buffer for segment %d", i);
        memcpy(mac_sdu->data, g_captured_pdus[i].data, g_captured_pdus[i].len);
        mac_sdu->len = g_captured_pdus[i].len;
        k_queue_append(&g_dlc_internal_mac_rx_queue, mac_sdu);
    }

    k_sleep(K_MSEC(50));
    ret = dlc_receive_data(&service, &source_addr, rx_buf, &rx_len, K_NO_WAIT);
    zassert_ok(ret, "dlc_receive_data (1280B) failed");
    zassert_equal(rx_len, sizeof(mtu_sdu), "Length mismatch");
    zassert_mem_equal(rx_buf, mtu_sdu, sizeof(mtu_sdu), "Data corruption");
}

ZTEST(dect_dlc_sar, test_sar_large_sdu_stress)
{
    printk("\n--- RUNNING TEST: %s ---\n", __func__);
#define STRESS_SIZE 4096
    static uint8_t stress_sdu[STRESS_SIZE];
    static uint8_t rx_buf[STRESS_SIZE + 100];
    size_t rx_len = sizeof(rx_buf);
    dlc_service_type_t service;
    uint32_t source_addr;

    for (int i=0; i<STRESS_SIZE; i++) stress_sdu[i] = (uint8_t)(i ^ 0x55);

    int ret = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x44332211, stress_sdu, STRESS_SIZE, MAC_FLOW_RELIABLE_DATA);
    zassert_ok(ret, "dlc_send_data (4KB) failed");

    printk("[TEST] Large SDU resulted in %d segments\n", g_capture_count);

    for (int i=0; i<g_capture_count; i++) {
        mac_sdu_t *mac_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
        zassert_not_null(mac_sdu, "Failed to allocate buffer for segment %d", i);
        memcpy(mac_sdu->data, g_captured_pdus[i].data, g_captured_pdus[i].len);
        mac_sdu->len = g_captured_pdus[i].len;
        printk("[TRACE] Injecting segment %d into RX queue (len %u, data ptr %p)...\n", i, mac_sdu->len, (void *)mac_sdu);
        k_queue_append(&g_dlc_internal_mac_rx_queue, mac_sdu);
        printk("[TRACE] Injection of segment %d complete.\n", i);
    }

    k_sleep(K_MSEC(100));
    ret = dlc_receive_data(&service, &source_addr, rx_buf, &rx_len, K_NO_WAIT);
    zassert_ok(ret, "dlc_receive_data (4KB) failed");
    zassert_equal(rx_len, STRESS_SIZE, "Length mismatch");
    zassert_mem_equal(rx_buf, stress_sdu, STRESS_SIZE, "Data corruption");
}

ZTEST_SUITE(dect_dlc_sar, NULL, dlc_sar_suite_setup, dlc_sar_before, dlc_sar_after, NULL);