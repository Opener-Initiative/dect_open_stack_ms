
#include <zephyr/ztest.h>
#include <string.h>
#include <dect_dlc.h>
#include <zephyr/sys/dlist.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_data_path.h> /* Needed for the test getter */

/* --- Test Globals & Mocks --- */
#define MAX_CAPTURED_PDUS 10
static mac_sdu_t g_captured_pdus[MAX_CAPTURED_PDUS];
static int g_capture_count;
static dect_mac_context_t g_mac_ctx;
static dlc_tx_status_cb_t g_dlc_status_cb;

extern k_tid_t g_dlc_tx_service_thread_id;
extern k_tid_t g_dlc_rx_thread_id;

/* Test-specific spy function to intercept calls to dect_mac_send */
static int test_send_spy_callback(mac_sdu_t *sdu, mac_flow_id_t flow)
{
	if (g_capture_count < MAX_CAPTURED_PDUS) {
		memcpy(&g_captured_pdus[g_capture_count], sdu, sizeof(mac_sdu_t));
		g_capture_count++;
	}
	printk("g_capture_count:%d \n", g_capture_count);
	dect_mac_buffer_free(sdu);
	return 0;
}

static void dlc_arq_before(void *fixture)
{
	ARG_UNUSED(fixture);
	g_capture_count = 0;
	g_dlc_status_cb = NULL;
	memset(g_captured_pdus, 0, sizeof(g_captured_pdus));

	dect_mac_test_set_active_context(&g_mac_ctx);

	/* Initialize MAC and DLC layers */
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	dect_dlc_init();

	/* Start the DLC threads so they can process events */
	k_thread_start(g_dlc_tx_service_thread_id);
	k_thread_start(g_dlc_rx_thread_id);

	/* Manually set the MAC state to ASSOCIATED for this test */
	dect_mac_get_active_context()->state = MAC_STATE_ASSOCIATED;
	dect_mac_get_active_context()->role_ctx.pt.associated_ft.is_valid = true;

	/* Register the send spy and retrieve the DLC status callback for the test */
	dect_mac_test_set_send_spy(test_send_spy_callback);
	g_dlc_status_cb = dect_mac_test_get_dlc_status_callback();
}

static void dlc_arq_after(void *fixture)
{
	ARG_UNUSED(fixture);
	k_thread_abort(g_dlc_tx_service_thread_id);
	k_thread_abort(g_dlc_rx_thread_id);
	dect_mac_test_set_send_spy(NULL);
}

ZTEST(dect_dlc_arq, test_arq_aa_success_on_first_try)
{
	printk("\n\n\n--- RUNNING TEST: %s ---\n", __func__);
	dect_mac_test_set_active_context(&g_mac_ctx);
	uint8_t sdu[] = {0xDE, 0xAD, 0xBE, 0xEF};
	dlc_send_data(DLC_SERVICE_TYPE_2_ARQ, 0x11223344, sdu, sizeof(sdu));

	zassert_equal(g_capture_count, 1, "Packet was not sent to MAC");
	const dect_dlc_header_type123_basic_t *hdr = (const void *)g_captured_pdus[0].data;
	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr);

	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(30));

	zassert_not_null(g_dlc_status_cb, "Failed to get DLC status callback");
	g_dlc_status_cb(sn, true);

	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(30));
	zassert_equal(g_capture_count, 1, "Packet was retransmitted after ACK");
}

ZTEST(dect_dlc_arq, test_arq_nack_and_retry_success)
{
	printk("\n\n\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t sdu[] = {0xFE, 0xED, 0xFA, 0xCE};
	dlc_send_data(DLC_SERVICE_TYPE_2_ARQ, 0x11223344, sdu, sizeof(sdu));
	zassert_equal(g_capture_count, 1, "Packet was not sent initially");
	const dect_dlc_header_type123_basic_t *hdr = (const void *)g_captured_pdus[0].data;
	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr);

	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(30));

	zassert_not_null(g_dlc_status_cb, "Failed to get DLC status callback");
	g_dlc_status_cb(sn, false);

	k_sleep(K_MSEC(50));
	zassert_equal(g_capture_count, 2, "Packet was not retransmitted after NACK");

	g_dlc_status_cb(sn, true);
	k_sleep(K_MSEC(30));
	zassert_equal(g_capture_count, 2, "Packet was retransmitted again after ACK");
}

ZTEST(dect_dlc_arq, test_arq_failure_after_max_retries)
{
	printk("\n\n\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t sdu[] = {0xC0, 0xDE};
	dlc_send_data(DLC_SERVICE_TYPE_2_ARQ, 0x11223344, sdu, sizeof(sdu));
	zassert_equal(g_capture_count, 1, "Packet was not sent initially");

	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(50));

	const dect_dlc_header_type123_basic_t *hdr = (const void *)g_captured_pdus[0].data;
	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr);

	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(50));

	zassert_not_null(g_dlc_status_cb, "Failed to get DLC status callback");
	#define MY_DLC_MAX_RETRIES 3
	for (int i = 0; i < (MY_DLC_MAX_RETRIES + 1); i++) {
		g_dlc_status_cb(sn, false);
		k_sleep(K_MSEC(75));
	}

	zassert_equal(g_capture_count, MY_DLC_MAX_RETRIES + 1, "Incorrect number of retransmissions");
}

ZTEST(dect_dlc_arq, test_arq_sdu_lifetime_expiry)
{
	printk("\n\n\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t sdu[] = {0xBA, 0xAD};
	dlc_send_data(DLC_SERVICE_TYPE_2_ARQ, 0x11223344, sdu, sizeof(sdu));
	zassert_equal(g_capture_count, 1, "Packet was not sent initially");
	const dect_dlc_header_type123_basic_t *hdr = (const void *)g_captured_pdus[0].data;
	uint16_t sn = dlc_hdr_t123_basic_get_sn(hdr);

	k_sleep(K_MSEC(CONFIG_DECT_DLC_DEFAULT_SDU_LIFETIME_MS + 50));

	zassert_not_null(g_dlc_status_cb, "Failed to get DLC status callback");
	g_dlc_status_cb(sn, false);
	k_sleep(K_MSEC(50));

	zassert_equal(g_capture_count, 1, "Packet was retransmitted after its lifetime expired");
}



ZTEST(dect_dlc_arq, test_routing_packet_forwarding)
{
	printk("\n\n\n--- RUNNING TEST: %s ---\n", __func__);

	uint8_t payload[] = {0xAA, 0xBB, 0xCC};
	uint32_t final_destination_id = 0xFEEDFACE;

	/* 1. Create a DLC SDU with a routing header destined for another node */
	uint8_t dlc_sdu_buf[64];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
	bitmap |= (1 << DLC_RH_HOP_COUNT_LIMIT_SHIFT);
	bitmap |= (DLC_RH_TYPE_LOCAL_FLOODING << DLC_RH_ROUTING_TYPE_SHIFT);
	// rh.source_addr_be = sys_cpu_to_be32(g_mac_ctx.own_long_rd_id);
	// rh.dest_addr_be = sys_cpu_to_be32(final_destination_id);
	rh.source_addr_be = (g_mac_ctx.own_long_rd_id);
	rh.dest_addr_be = (final_destination_id);
	rh.hop_count = 1;
	rh.hop_limit = 5;
	rh.bitmap_be = sys_cpu_to_be16(bitmap);
	int rh_len = dlc_serialize_routing_header(dlc_sdu_buf, sizeof(dlc_sdu_buf), &rh);
	zassert_true(rh_len > 0, "Failed to serialize routing header");
	printk("[TEST_DBG] Serialized Routing Header (len=%d):\n", rh_len);
	for (int i = 0; i < rh_len; i++) {
		printk("%02x ", dlc_sdu_buf[i]);
	}
	printk("\n");

	memcpy(dlc_sdu_buf + rh_len, payload, sizeof(payload));
	size_t dlc_sdu_len = rh_len + sizeof(payload);

	/* 2. Inject this as a "received" complete SDU */
	mac_sdu_t *rx_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	zassert_not_null(rx_sdu, "Failed to allocate rx_sdu");
	printk("Completed allocated rx_sdu... \n");
	dect_dlc_header_type123_basic_t dlc_hdr;
	dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_COMPLETE_SDU, 42);
	memcpy(rx_sdu->data, &dlc_hdr, sizeof(dlc_hdr));
	memcpy(rx_sdu->data + sizeof(dlc_hdr), dlc_sdu_buf, dlc_sdu_len);
	rx_sdu->len = sizeof(dlc_hdr) + dlc_sdu_len;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &rx_sdu->node);

	/* 3. Let the DLC thread run */
	printk("Yielding to allow DLC thread to start the reassembly timer...\n");
	k_sleep(K_MSEC(50));

	/* 4. Verify the packet was forwarded (sent back to the MAC) */
	zassert_equal(g_capture_count, 1, "Packet was not forwarded");

	/* 5. Verify the hop count was incremented */
	const dect_dlc_header_type123_basic_t *fwd_dlc_hdr = (const void *)g_captured_pdus[0].data;
	const uint8_t *fwd_sdu_payload = g_captured_pdus[0].data + sizeof(*fwd_dlc_hdr);
	size_t fwd_sdu_len = g_captured_pdus[0].len - sizeof(*fwd_dlc_hdr);
	dect_dlc_routing_header_t fwd_rh;
	int fwd_rh_len = dlc_parse_routing_header(fwd_sdu_payload, fwd_sdu_len, &fwd_rh);
	zassert_true(fwd_rh_len > 0, "Forwarded packet has no routing header");
	zassert_equal(fwd_rh.hop_count, 2, "Hop count was not incremented");
}

ZTEST(dect_dlc_arq, test_routing_duplicate_rejection)
{
	printk("\n--- [TEST] Starting test_routing_duplicate_rejection ---\n");
	uint8_t payload[] = {0xDD};
	uint32_t source_id = 0x12345678;
	uint8_t seq_num = 55;

	/* 1. Create a DLC SDU with a specific source and sequence number */
	uint8_t dlc_sdu_buf[64];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	bitmap |= (1 << DLC_RH_SEQ_NUM_SHIFT);
	bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
	bitmap |= (DLC_RH_TYPE_LOCAL_FLOODING << DLC_RH_ROUTING_TYPE_SHIFT);
	// rh.source_addr_be = sys_cpu_to_be32(source_id);
	rh.source_addr_be = (source_id);
	rh.sequence_number = seq_num;
	// rh.dest_addr_be = sys_cpu_to_be32(g_mac_ctx.own_long_rd_id);
	rh.dest_addr_be = g_mac_ctx.own_long_rd_id;
	rh.hop_count = 1;
	rh.hop_limit = 5;	
	rh.bitmap_be = sys_cpu_to_be16(bitmap);
	int rh_len = dlc_serialize_routing_header(dlc_sdu_buf, sizeof(dlc_sdu_buf), &rh);
	memcpy(dlc_sdu_buf + rh_len, payload, sizeof(payload));
	size_t dlc_sdu_len = rh_len + sizeof(payload);

	/* 2. Inject the packet twice */
	for (int i = 0; i < 2; i++) {
		mac_sdu_t *rx_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
		zassert_not_null(rx_sdu, "Failed to allocate rx_sdu");
		dect_dlc_header_type123_basic_t dlc_hdr;
		dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_COMPLETE_SDU, 100 + i);
		memcpy(rx_sdu->data, &dlc_hdr, sizeof(dlc_hdr));
		memcpy(rx_sdu->data + sizeof(dlc_hdr), dlc_sdu_buf, dlc_sdu_len);
		rx_sdu->len = sizeof(dlc_hdr) + dlc_sdu_len;
		sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &rx_sdu->node);
	}

	/* 3. Let the DLC thread run */
	k_sleep(K_MSEC(20));

	/* 4. Verify the packet was only processed once (and not forwarded as it's for us) */
	zassert_equal(g_capture_count, 0, "Packet was forwarded, but should have been for local delivery");
	/* We also need to check that it was delivered to the app only once */
	sys_dnode_t *node = sys_dlist_get(&g_dlc_to_app_rx_dlist);
	zassert_not_null(node, "Packet was not delivered to app");
	zassert_true(sys_dlist_is_empty(&g_dlc_to_app_rx_dlist), "Duplicate packet was delivered to app");
}

ZTEST(dect_dlc_arq, test_routing_hop_limit_expiry)
{
	printk("\n--- [TEST] Starting test_routing_hop_limit_expiry ---\n");
	uint8_t payload[] = {0xEE};
	uint32_t final_destination_id = 0xFEEDFACE;

	/* 1. Create a DLC SDU with hop_count == hop_limit */
	uint8_t dlc_sdu_buf[64];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
	bitmap |= (1 << DLC_RH_HOP_COUNT_LIMIT_SHIFT);
	bitmap |= (DLC_RH_TYPE_LOCAL_FLOODING << DLC_RH_ROUTING_TYPE_SHIFT);
	rh.source_addr_be = sys_cpu_to_be32(g_mac_ctx.own_long_rd_id);
	rh.dest_addr_be = sys_cpu_to_be32(final_destination_id);
	rh.hop_count = 5;
	rh.hop_limit = 5;
	rh.bitmap_be = sys_cpu_to_be16(bitmap);
	int rh_len = dlc_serialize_routing_header(dlc_sdu_buf, sizeof(dlc_sdu_buf), &rh);
	memcpy(dlc_sdu_buf + rh_len, payload, sizeof(payload));
	size_t dlc_sdu_len = rh_len + sizeof(payload);

	/* 2. Inject the packet */
	mac_sdu_t *rx_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	dect_dlc_header_type123_basic_t dlc_hdr;
	dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, DLC_SI_COMPLETE_SDU, 43);
	memcpy(rx_sdu->data, &dlc_hdr, sizeof(dlc_hdr));
	memcpy(rx_sdu->data + sizeof(dlc_hdr), dlc_sdu_buf, dlc_sdu_len);
	rx_sdu->len = sizeof(dlc_hdr) + dlc_sdu_len;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &rx_sdu->node);

	/* 3. Let the DLC thread run */
	k_sleep(K_MSEC(20));

	/* 4. Verify the packet was NOT forwarded */
	zassert_equal(g_capture_count, 0, "Packet was forwarded despite hop limit being reached");
}

ZTEST(dect_dlc_arq, test_control_route_error_ie)
{
	printk("\n--- [TEST] Starting test_control_route_error_ie ---\n");

	/* 1. Create a PDU containing a Route Error IE */
	uint8_t pdu_buf[64];
	dect_dlc_extension_header_t ext_hdr;
	dect_dlc_ie_route_error_t err_ie = {
		.error_reason = 1,
		.invalid_next_hop_addr_be = sys_cpu_to_be32(0xDEADBEEF)
	};
	dlc_ext_hdr_set(&ext_hdr, DLC_EXT_HDR_8BIT_LEN_FIELD, DLC_EXT_IE_ROUTE_ERROR);
	pdu_buf[0] = ext_hdr.ext_type_and_ie_type;
	pdu_buf[1] = sizeof(err_ie);
	memcpy(&pdu_buf[2], &err_ie, sizeof(err_ie));
	size_t sdu_payload_len = 2 + sizeof(err_ie);

	/* 2. Inject the packet */
	mac_sdu_t *rx_sdu = dect_mac_buffer_alloc(K_NO_WAIT);
	dect_dlc_header_type123_basic_t dlc_hdr;
	dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_EXT_HDR, DLC_SI_COMPLETE_SDU, 44);
	memcpy(rx_sdu->data, &dlc_hdr, sizeof(dlc_hdr));
	memcpy(rx_sdu->data + sizeof(dlc_hdr), pdu_buf, sdu_payload_len);
	rx_sdu->len = sizeof(dlc_hdr) + sdu_payload_len;
	sys_dlist_append(&g_dlc_internal_mac_rx_dlist, &rx_sdu->node);

	/* 3. Let the DLC thread run */
	k_sleep(K_MSEC(250));

	/* 4. Verify the packet was NOT delivered to the application */
	zassert_true(sys_dlist_is_empty(&g_dlc_to_app_rx_dlist), "Control IE was incorrectly delivered to app");
}




ZTEST_SUITE(dect_dlc_arq, NULL, NULL, dlc_arq_before, dlc_arq_after, NULL);