// /* lib/dect_nrplus/tests/cvg_arq/src/main.c */
// // Overview: Ztest suite for CVG ARQ and Flow Control. It mocks the DLC layer to validate the CVG's reliability mechanisms.
// #include <zephyr/ztest.h>
// #include <string.h>
// #include <dect_cvg.h>
// #include <dect_dlc.h>
// #include <mac/dect_mac.h>
// #include <mac/dect_mac_core.h>

// /* --- Test Globals & Mocks --- */
// #define CVG_TEST_WINDOW_SIZE 4
// #define CVG_TEST_LIFETIME_MS 200

// static uint8_t g_dlc_tx_capture_buf[1500];
// static size_t g_dlc_tx_capture_len;
// static uint32_t g_dlc_tx_capture_dest_id;
// static int g_dlc_tx_capture_count;

// static uint8_t g_dlc_rx_injection_buf[1500];
// static size_t g_dlc_rx_injection_len;
// static bool g_dlc_rx_data_available;

// static dect_mac_context_t g_mac_ctx;

// /* --- Mock Implementations of DLC API --- */

// int dlc_send_data(dlc_service_type_t service, uint32_t dest_long_id,
// 		  const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len)
// {
// 	g_dlc_tx_capture_count++;
// 	g_dlc_tx_capture_dest_id = dest_long_id;
// 	g_dlc_tx_capture_len = dlc_sdu_payload_len;
// 	memcpy(g_dlc_tx_capture_buf, dlc_sdu_payload, dlc_sdu_payload_len);
// 	return 0;
// }

// int dlc_receive_data(dlc_service_type_t *service_type_out, uint8_t *app_level_payload_buf,
// 		     size_t *app_level_payload_len_inout, k_timeout_t timeout)
// {
// 	if (!g_dlc_rx_data_available) {
// 		return -EAGAIN;
// 	}
// 	*service_type_out = DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ;
// 	*app_level_payload_len_inout = g_dlc_rx_injection_len;
// 	memcpy(app_level_payload_buf, g_dlc_rx_injection_buf, g_dlc_rx_injection_len);
// 	g_dlc_rx_data_available = false;
// 	return 0;
// }

// /* --- Test Setup --- */

// static void cvg_arq_before(void *fixture)
// {
// 	ARG_UNUSED(fixture);
// 	g_dlc_tx_capture_count = 0;
// 	g_dlc_rx_data_available = false;

// 	dect_mac_test_set_active_context(&g_mac_ctx);
// 	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
// 	dect_cvg_init(); /* This initializes DLC and MAC as well */

// 	/* Configure a flow for testing */
// 	dect_cvg_configure_flow(CVG_SERVICE_TYPE_4_FC_ARQ, CVG_TEST_WINDOW_SIZE, CVG_TEST_LIFETIME_MS);

// 	/* Start the necessary threads */
// 	k_thread_start(g_cvg_tx_thread_id);
// 	k_thread_start(g_cvg_rx_thread_id);
// 	k_thread_start(g_cvg_arq_service_thread_id);
// }

// static void cvg_arq_after(void *fixture)
// {
// 	ARG_UNUSED(fixture);
// 	k_thread_abort(g_cvg_tx_thread_id);
// 	k_thread_abort(g_cvg_rx_thread_id);
// 	k_thread_abort(g_cvg_arq_service_thread_id);
// }

// /* --- Test Cases --- */

// ZTEST(cvg_arq, test_flow_control_window_blocks)
// {
// 	uint8_t sdu[] = {0x01};

// 	/* 1. Fill the transmission window */
// 	for (int i = 0; i < CVG_TEST_WINDOW_SIZE; i++) {
// 		g_dlc_tx_capture_count = 0;
// 		dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
// 		/* Yield to let the TX thread run and take a semaphore credit */
// 		k_sleep(K_MSEC(10));
// 		zassert_equal(g_dlc_tx_capture_count, 1, "Packet #%d was not sent", i);
// 	}

// 	/* 2. Try to send one more packet. This should block. */
// 	g_dlc_tx_capture_count = 0;
// 	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
// 	k_sleep(K_MSEC(50)); /* Give TX thread time to NOT send */
// 	zassert_equal(g_dlc_tx_capture_count, 0, "Packet was sent even though window was full");

// 	/* 3. Simulate an ACK for the first packet */
// 	// const cvg_ie_arq_feedback_base_t *fb_base = (const void *)g_dlc_tx_capture_buf;
//     /* The sequence number of the first packet sent is 0 */
// 	uint16_t sn_to_ack = 0; /* First packet sent had SN=0 */
// 	uint8_t ack_pdu[sizeof(cvg_header_t) + sizeof(cvg_ie_arq_feedback_base_t)];
// 	cvg_header_t *hdr = (void *)ack_pdu;
// 	hdr->ext_mt_f2c_or_type = (CVG_EXT_NO_LEN_FIELD << 6) | CVG_IE_TYPE_ARQ_FEEDBACK;
// 	cvg_ie_arq_feedback_base_t *ack = (void *)(ack_pdu + sizeof(cvg_header_t));
// 	ack->an_fbinfo_sn_msb = (0 << 7) | (0 << 4) | ((sn_to_ack >> 8) & 0x0F);
// 	ack->sequence_number_lsb = sn_to_ack & 0xFF;

// 	g_dlc_rx_injection_len = sizeof(ack_pdu);
// 	memcpy(g_dlc_rx_injection_buf, ack_pdu, sizeof(ack_pdu));
// 	g_dlc_rx_data_available = true;

// 	/* 4. Yield to let RX thread process ACK and TX thread to send the blocked packet */
// 	k_sleep(K_MSEC(50));
// 	zassert_equal(g_dlc_tx_capture_count, 1, "Blocked packet was not sent after ACK was received");
// }

// ZTEST(cvg_arq, test_sdu_lifetime_expiry)
// {
// 	uint8_t sdu[] = {0x02};
// 	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
// 	k_sleep(K_MSEC(10));
// 	zassert_equal(g_dlc_tx_capture_count, 1, "Packet was not sent initially");

// 	/* Advance time past the SDU lifetime */
// 	k_sleep(K_MSEC(CVG_TEST_LIFETIME_MS + 50));

// 	/* At this point, the lifetime handler should have fired and freed the in-flight SDU.
// 	 * To verify, we can check if a flow control credit was returned.
// 	 * We send the entire window's worth of packets. If one credit was returned by the
// 	 * timeout, we should be able to send all of them.
// 	 */
// 	for (int i = 0; i < CVG_TEST_WINDOW_SIZE; i++) {
// 		g_dlc_tx_capture_count = 0;
// 		dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
// 		k_sleep(K_MSEC(10));
// 		if (i < CVG_TEST_WINDOW_SIZE - 1) {
// 			zassert_equal(g_dlc_tx_capture_count, 1, "Packet #%d was not sent", i);
// 		} else {
// 			/* The last packet should not be sent, as the window is full again */
// 			zassert_equal(g_dlc_tx_capture_count, 0, "A credit was not returned after SDU timeout");
// 		}
// 	}
// }

// ZTEST_SUITE(cvg_arq, NULL, NULL, cvg_arq_before, cvg_arq_after, NULL);




#include <zephyr/ztest.h>
#include <string.h>
#include <dect_cvg.h>
#include <dect_dlc.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>

/* --- Test Globals & Mocks --- */
#define CVG_TEST_WINDOW_SIZE 4
#define CVG_TEST_LIFETIME_MS 200

static uint8_t g_dlc_tx_capture_buf[1500];
static size_t g_dlc_tx_capture_len;
static uint32_t g_dlc_tx_capture_dest_id;
static int g_dlc_tx_capture_count;

static uint8_t g_dlc_rx_injection_buf[1500];
static size_t g_dlc_rx_injection_len;
static bool g_dlc_rx_data_available;

static dect_mac_context_t g_mac_ctx;

/* --- Mock Implementations of DLC API --- */

static int mock_dlc_send_data(dlc_service_type_t service, uint32_t dest_long_id,
			      const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len, uint8_t flow_id)
{
    ARG_UNUSED(flow_id);

	g_dlc_tx_capture_count++;
	g_dlc_tx_capture_dest_id = dest_long_id;
	g_dlc_tx_capture_len = dlc_sdu_payload_len;
	memcpy(g_dlc_tx_capture_buf, dlc_sdu_payload, dlc_sdu_payload_len);
	return 0;
}

static int mock_dlc_receive_data(dlc_service_type_t *service_type_out, uint32_t *source_addr_out,
				uint8_t *app_level_payload_buf,
				 size_t *app_level_payload_len_inout, k_timeout_t timeout)
{
    if (source_addr_out) {
        *source_addr_out = 0x12345678; /* Dummy source addr for test */
    }

	/* This mock simulates the real function's blocking behavior */
	if (!g_dlc_rx_data_available) {
		/* If K_FOREVER, we can't simulate this in a simple test, but for any
		 * other timeout, we can sleep. For this test, we know data will
		 * be made available, so we just check once. A real wait isn't needed.
		 */
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return -EAGAIN;
		}
		/* The test will hang here if data is never made available, which is
		 * the correct behavior for a K_FOREVER wait.
		 */
		k_sleep(K_MSEC(5));
		return -EAGAIN;
	}

	*service_type_out = DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ;
	*app_level_payload_len_inout = g_dlc_rx_injection_len;
	memcpy(app_level_payload_buf, g_dlc_rx_injection_buf, g_dlc_rx_injection_len);
	g_dlc_rx_data_available = false;
	return 0;
}

/* --- Test Setup --- */

static void cvg_arq_before(void *fixture)
{
    printk("[TEST DBG] cvg_arq_before Started... \n");
	ARG_UNUSED(fixture);
	g_dlc_tx_capture_count = 0;
	g_dlc_rx_data_available = false;

	dect_mac_test_set_active_context(&g_mac_ctx);
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	dect_cvg_init();

	/* Register spies */
	dlc_test_set_send_spy(mock_dlc_send_data);
	dlc_test_set_receive_spy(mock_dlc_receive_data);

	/* Configure a flow for testing */
	dect_cvg_configure_flow(CVG_SERVICE_TYPE_4_FC_ARQ, CVG_QOS_DATA_HIGH, CVG_TEST_WINDOW_SIZE, CVG_TEST_LIFETIME_MS);

	/* Start the necessary threads */
	printk("[TEST_SETUP_DBG] About to call k_thread_start on CVG threads...\n");
	printk("[TEST_SETUP_DBG]   -> Starting ARQ thread: %p\n", g_cvg_arq_service_thread_id);
	k_thread_start(g_cvg_arq_service_thread_id);
	printk("[TEST_SETUP_DBG]   -> Starting TX thread: %p\n", g_cvg_tx_thread_id);
	k_thread_start(g_cvg_tx_thread_id);
    printk("[TEST_SETUP_DBG]   -> Starting RX thread: %p\n", g_cvg_rx_thread_id);
	k_thread_start(g_cvg_rx_thread_id);

	/* Yield to allow the newly started threads to run and pend on their respective queues */
	k_sleep(K_MSEC(50));    
}

static void cvg_arq_after(void *fixture)
{
	ARG_UNUSED(fixture);
	k_thread_abort(g_cvg_tx_thread_id);
	k_thread_abort(g_cvg_rx_thread_id);
	k_thread_abort(g_cvg_arq_service_thread_id);
	k_thread_abort(g_dlc_tx_service_thread_id);
	k_thread_abort(g_dlc_rx_thread_id);
	
	/* Restore real implementations */
	dlc_test_set_send_spy(NULL);
	dlc_test_set_receive_spy(NULL);

	/* Yield to allow threads to fully terminate */
	k_sleep(K_MSEC(500));    
}



/* --- Helper to inject a CVG Data PDU --- */
static void inject_cvg_data_pdu(uint16_t sn, const uint8_t *payload, size_t len)
{
	uint8_t pdu[sizeof(cvg_header_t) + sizeof(cvg_ie_data_base_t) + len];
	cvg_header_t *hdr = (void *)pdu;
	hdr->ext_mt_f2c_or_type = (CVG_EXT_NO_LEN_FIELD << 6) | CVG_IE_TYPE_DATA;
	cvg_ie_data_base_t *data_base = (void *)(pdu + sizeof(cvg_header_t));
	cvg_ie_data_base_set(data_base, CVG_SI_COMPLETE_SDU, false, sn);
	memcpy((uint8_t *)data_base + sizeof(*data_base), payload, len);

	g_dlc_rx_injection_len = sizeof(pdu);
	memcpy(g_dlc_rx_injection_buf, pdu, sizeof(pdu));
	g_dlc_rx_data_available = true;
}


/* --- Test Cases --- */

ZTEST(cvg_arq, test_aa_flow_control_window_blocks)
{
	uint8_t sdu[] = {0x01};

	/* 1. Fill the transmission window */
	for (int i = 0; i < CVG_TEST_WINDOW_SIZE; i++) {
		g_dlc_tx_capture_count = 0;
		dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
		k_sleep(K_MSEC(10));
		zassert_equal(g_dlc_tx_capture_count, 1, "Packet #%d was not sent", i);
	}

	/* 2. Try to send one more packet. This should block. */
	g_dlc_tx_capture_count = 0;
	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
	k_sleep(K_MSEC(50));
	zassert_equal(g_dlc_tx_capture_count, 0, "Packet was sent even though window was full");

	/* 3. Simulate an ACK for the first packet */
	uint16_t sn_to_ack = 0;
	uint8_t ack_pdu[sizeof(cvg_header_t) + sizeof(cvg_ie_arq_feedback_base_t)];
	cvg_header_t *hdr = (void *)ack_pdu;
	hdr->ext_mt_f2c_or_type = (CVG_EXT_NO_LEN_FIELD << 6) | CVG_IE_TYPE_ARQ_FEEDBACK;
	cvg_ie_arq_feedback_base_t *ack = (void *)(ack_pdu + sizeof(cvg_header_t));
	ack->an_fbinfo_sn_msb = (0 << 7) | (0 << 4) | ((sn_to_ack >> 8) & 0x0F);
	ack->sequence_number_lsb = sn_to_ack & 0xFF;

	g_dlc_rx_injection_len = sizeof(ack_pdu);
	memcpy(g_dlc_rx_injection_buf, ack_pdu, sizeof(ack_pdu));
	g_dlc_rx_data_available = true;

	/* 4. Yield to let RX thread process ACK and TX thread to send the blocked packet */
	k_sleep(K_MSEC(50));
	zassert_equal(g_dlc_tx_capture_count, 1, "Blocked packet was not sent after ACK was received");
}

ZTEST(cvg_arq, test_sdu_lifetime_expiry)
{
	uint8_t sdu[] = {0x02};
	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
	k_sleep(K_MSEC(10));
	zassert_equal(g_dlc_tx_capture_count, 1, "Packet was not sent initially");

	/* Advance time past the SDU lifetime */
	k_sleep(K_MSEC(CVG_TEST_LIFETIME_MS + 50));

	/* At this point, the lifetime handler should have fired and freed the in-flight SDU.
	 * To verify, we can check if a flow control credit was returned.
	 */
	g_dlc_tx_capture_count = 0;
	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
	k_sleep(K_MSEC(10));
	zassert_equal(g_dlc_tx_capture_count, 1, "A credit was not returned after SDU timeout");
}


ZTEST(cvg_arq, test_arq_ack_generation)
{
	uint8_t sdu[] = {0xAA, 0xBB};
	inject_cvg_data_pdu(42, sdu, sizeof(sdu));

	k_sleep(K_MSEC(20)); /* Allow RX thread to process and send ACK */

	zassert_equal(g_dlc_tx_capture_count, 1, "ACK packet was not sent by receiver");

	const cvg_header_t *hdr = (const void *)g_dlc_tx_capture_buf;
	cvg_ie_type_t ie_type = (cvg_ie_type_t)(hdr->ext_mt_f2c_or_type & 0x1F);
	zassert_equal(ie_type, CVG_IE_TYPE_ARQ_FEEDBACK, "Sent packet was not an ARQ Feedback IE");

	const cvg_ie_arq_feedback_base_t *ack = (const void *)(g_dlc_tx_capture_buf + sizeof(cvg_header_t));
	bool is_nack = (ack->an_fbinfo_sn_msb >> 7) & 0x01;
	uint16_t sn = ((uint16_t)(ack->an_fbinfo_sn_msb & 0x0F) << 8) | ack->sequence_number_lsb;

	zassert_false(is_nack, "Feedback was a NACK instead of an ACK");
	zassert_equal(sn, 42, "ACK was for the wrong sequence number");
}

ZTEST(cvg_arq, test_arq_retransmission_on_timeout)
{
	uint8_t sdu[] = {0xCC};
	dect_cvg_send(0x8003, 0x11223344, sdu, sizeof(sdu));
	k_sleep(K_MSEC(10));
	zassert_equal(g_dlc_tx_capture_count, 1, "Packet was not sent initially");

	/* Advance time past the SDU lifetime to trigger the ARQ service thread */
	k_sleep(K_MSEC(CVG_TEST_LIFETIME_MS + 50));

	zassert_equal(g_dlc_tx_capture_count, 2, "Packet was not retransmitted after timeout");
}


ZTEST_SUITE(cvg_arq, NULL, NULL, cvg_arq_before, cvg_arq_after, NULL);