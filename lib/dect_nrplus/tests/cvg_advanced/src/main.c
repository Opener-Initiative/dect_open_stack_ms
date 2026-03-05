/* lib/dect_nrplus/tests/cvg_advanced/src/main.c */
/* This is a new file. It contains a Ztest suite for verifying advanced CVG features like Flow Control and ARQ. */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include "dect_cvg.h"
#include "dect_dlc.h"

#define TEST_ENDPOINT_ID  0x8001
#define TEST_DEST_LONG_ID 0x12345678
#define TEST_MAX_WINDOW   2

static struct k_sem g_dlc_send_sem;
static int g_dlc_send_count;
static uint16_t g_last_sn_received;

/* --- DLC Spies --- */

static int dlc_send_spy(dlc_service_type_t service, uint32_t dest_long_id,
			const uint8_t *payload, size_t len, uint8_t flow_id)
{
	ARG_UNUSED(service);
	ARG_UNUSED(dest_long_id);
	ARG_UNUSED(flow_id);

	const uint8_t *pdu_ptr = payload;
	size_t remaining_len = len;

	while (remaining_len > 0) {
		const cvg_header_t *hdr = (const cvg_header_t *)pdu_ptr;
		cvg_ie_type_t type = (cvg_ie_type_t)(hdr->ext_mt_f2c_or_type & 0x1F);
		size_t consumed = 0;

		if (type == CVG_IE_TYPE_DATA || type == CVG_IE_TYPE_DATA_EP) {
			const cvg_ie_data_base_t *db;
			if (type == CVG_IE_TYPE_DATA) {
				db = (const cvg_ie_data_base_t *)(pdu_ptr + sizeof(cvg_header_t));
			} else {
				db = (const cvg_ie_data_base_t *)(pdu_ptr + sizeof(cvg_ie_data_ep_base_t) - sizeof(cvg_ie_data_base_t));
			}
			g_last_sn_received = cvg_ie_data_base_get_sn(db);
			printk("[TEST SPY] Captured DLC send: SN=%u, len=%zu\n", g_last_sn_received, len);
			consumed = remaining_len; /* Stop here */
		} else if (type == CVG_IE_TYPE_EP_MUX) {
			consumed = sizeof(cvg_ie_ep_mux_t);
		} else if (type == CVG_IE_TYPE_ARQ_FEEDBACK) {
			consumed = sizeof(cvg_ie_arq_feedback_base_t);
		} else {
			consumed = remaining_len; /* Unknown IE, stop */
		}

		if (consumed == 0 || consumed > remaining_len) break;
		pdu_ptr += consumed;
		remaining_len -= consumed;
	}

	g_dlc_send_count++;
	k_sem_give(&g_dlc_send_sem);
	return 0;
}

static struct k_sem g_dlc_receive_sem;
static uint8_t g_inject_buf[256];
static size_t g_inject_len;

static int dlc_receive_spy(dlc_service_type_t *service, uint32_t *source_addr,
			   uint8_t *buf, size_t *len, k_timeout_t timeout)
{
	if (k_sem_take(&g_dlc_receive_sem, timeout) != 0) {
		return -EAGAIN;
	}

	*service = DLC_SERVICE_TYPE_0_TRANSPARENT;
	*source_addr = TEST_DEST_LONG_ID;
	size_t copy_len = MIN(*len, g_inject_len);
	memcpy(buf, g_inject_buf, copy_len);
	*len = copy_len;
	
	return 0;
}

/* --- Helpers --- */

static void inject_arq_feedback(bool is_ack, uint16_t sn)
{
	cvg_ie_arq_feedback_base_t fb;
	/* Format 1: [Ext(00) | MT(0) | Type(00110=6)] -> 0x06 */
	fb.header.ext_mt_f2c_or_type = CVG_IE_TYPE_ARQ_FEEDBACK;
	fb.an_fbinfo_sn_msb = (is_ack ? 0 : 0x80) | ((sn >> 8) & 0x0F);
	fb.sequence_number_lsb = (uint8_t)(sn & 0xFF);

	memcpy(g_inject_buf, &fb, sizeof(fb));
	g_inject_len = sizeof(fb);
	k_sem_give(&g_dlc_receive_sem);
}

static void inject_data_ie(uint16_t sn, const uint8_t *data, size_t data_len)
{
	uint8_t *ptr = g_inject_buf;
	memset(g_inject_buf, 0, sizeof(g_inject_buf));
	
	/* CVG Data IE */
	*ptr++ = CVG_IE_TYPE_DATA;
	cvg_ie_data_base_t db;
	cvg_ie_data_base_set(&db, CVG_SI_COMPLETE_SDU, false, sn);
	memcpy(ptr, &db, sizeof(db));
	ptr += sizeof(db);
	
	memcpy(ptr, data, data_len);
	ptr += data_len;
	
	g_inject_len = ptr - g_inject_buf;
	k_sem_give(&g_dlc_receive_sem);
}

static dect_mac_context_t g_mac_ctx;

static void *setup(void)
{
	dect_mac_test_set_active_context(&g_mac_ctx);
	dect_mac_core_init(MAC_ROLE_PT, 0x87654321);
	
	k_sem_init(&g_dlc_send_sem, 0, 100);
	k_sem_init(&g_dlc_receive_sem, 0, 1);
	
	dect_cvg_init();

	dlc_test_set_send_spy(dlc_send_spy);
	dlc_test_set_receive_spy(dlc_receive_spy);
	
	return NULL;
}

/* --- Test Cases --- */

ZTEST(cvg_advanced_tests, test_cvg_arq_and_flow_control)
{
	uint8_t payload[] = "hello";
	int ret;

	g_dlc_send_count = 0;

	/* 1. Configure a flow with window size 2 */
	ret = dect_cvg_configure_flow(CVG_SERVICE_TYPE_4_FC_ARQ, CVG_QOS_DATA_NORM, TEST_MAX_WINDOW, 5000);
	zassert_ok(ret, "configure flow failed");

	/* 2. Send 2 packets - should succeed and be transmitted */
	printk("--- Sending first 2 packets ---\n");
	ret = dect_cvg_send(TEST_ENDPOINT_ID, TEST_DEST_LONG_ID, payload, sizeof(payload));
	zassert_ok(ret, "send 1 failed");
	ret = dect_cvg_send(TEST_ENDPOINT_ID, TEST_DEST_LONG_ID, payload, sizeof(payload));
	zassert_ok(ret, "send 2 failed");

	/* Wait and verify 2 DLC sends */
	zassert_ok(k_sem_take(&g_dlc_send_sem, K_MSEC(100)), "timeout waiting for send 1");
	zassert_ok(k_sem_take(&g_dlc_send_sem, K_MSEC(100)), "timeout waiting for send 2");
	zassert_equal(g_dlc_send_count, 2, "Expected 2 sends to DLC");

	/* 3. Send 3rd packet - should be blocked in TX thread waiting for credit */
	printk("--- Sending 3rd packet (should wait for credit) ---\n");
	ret = dect_cvg_send(TEST_ENDPOINT_ID, TEST_DEST_LONG_ID, payload, sizeof(payload));
	zassert_ok(ret, "send 3 failed to queue");

	/* Verify NO DLC send for the 3rd packet yet */
	ret = k_sem_take(&g_dlc_send_sem, K_MSEC(200));
	zassert_not_equal(ret, 0, "Packet 3 was sent but window should be full");

	/* 4. Simulate an ACK for SN 0 */
	printk("--- Simulating ACK for SN 0 ---\n");
	inject_arq_feedback(true, 0);

	/* Allow CVG threads to run and process ACK */
	k_msleep(10);

	/* 5. Verify 3rd packet is now transmitted */
	zassert_ok(k_sem_take(&g_dlc_send_sem, K_MSEC(200)), "Packet 3 not sent after ACK");
	zassert_equal(g_last_sn_received, 2, "Expected packet 3 (SN 2) to be sent");

	/* 6. Simulate a NACK for SN 1 */
	printk("--- Simulating NACK for SN 1 ---\n");
	inject_arq_feedback(false, 1);

	/* Allow CVG threads to run and process NACK/re-TX */
	k_msleep(10);

	/* 7. Verify retransmission of SN 1 */
	zassert_ok(k_sem_take(&g_dlc_send_sem, K_MSEC(200)), "Retransmission of SN 1 not seen");
	zassert_equal(g_last_sn_received, 1, "Expected retransmission of SN 1");
}

ZTEST(cvg_advanced_tests, test_cvg_duplicate_removal)
{
	uint8_t rx_payload[] = "inbound";
	uint8_t out_buf[32];
	size_t out_len;
	int ret;

	/* Use SEQ_NUM service for duplicate removal */
	dect_cvg_configure_flow(CVG_SERVICE_TYPE_1_SEQ_NUM, CVG_QOS_DATA_NORM, 10, 5000);

	/* 1. Inject packet with SN 10 */
	printk("--- Injecting SN 10 ---\n");
	inject_data_ie(10, rx_payload, sizeof(rx_payload));

	/* 2. Verify it is received by app */
	out_len = sizeof(out_buf);
	ret = dect_cvg_receive(out_buf, &out_len, K_MSEC(200));
	zassert_ok(ret, "receive failed");
	zassert_mem_equal(out_buf, rx_payload, sizeof(rx_payload), "payload mismatch");

	/* 3. Inject same SN 10 again */
	printk("--- Injecting duplicate SN 10 ---\n");
	inject_data_ie(10, rx_payload, sizeof(rx_payload));

	/* 4. Verify it is DROPPED (no recieve) */
	out_len = sizeof(out_buf);
	ret = dect_cvg_receive(out_buf, &out_len, K_MSEC(300));
	zassert_not_equal(ret, 0, "Duplicate was not dropped");
}

ZTEST_SUITE(cvg_advanced_tests, NULL, setup, NULL, NULL, NULL);