

/* lib/dect_nrplus/tests/mac_flow/src/main.c */
/* This is a new file, replacing the old test_mac_association/src/main.c. It implements a comprehensive test harness that uses the mock PHY to simulate and verify the full association, data transfer, and error recovery flows between a PT and an FT. */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>

#include <mac/dect_mac_main.h>
#include <mac/dect_mac_core.h>      // For get_mac_context(), increment_psn_and_hpc()
#include <mac/dect_mac_context.h>   // For dect_mac_context_t access and constants
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_phy_ctrl.h>  // For dect_mac_phy_ctrl_start_tx_assembled, _assemble_final_pdu, calculate_pcc_params
#include <mac/dect_mac_pdu.h>       // For IE_TYPE_USER_DATA_FLOW_1, MAC Common Headers, MAC Hdr Type, parse_mac_mux_header
#include <mac/dect_mac_main_dispatcher.h> // For string utils for logging, mac_event_msgq
#include <mac/dect_mac_api.h>       // For dect_mac_api_buffer_free, mac_sdu_t, mac_tx_fifos (generic for PT), g_mac_sdu_slab
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_security.h>  // For security_build_iv, _calculate_mic, _crypt_payload
#include <mac/dect_mac_phy_tbs_tables.h> // For TBS lookup tables
#include <mac/dect_mac_timeline_utils.h>

#include "../../mocks/mock_nrf_modem_dect_phy.h"

/* The PHY interface needs to be visible for the event handler registration */
#include <mac/dect_mac_phy_if.h>

LOG_MODULE_REGISTER(test_mac_flow, LOG_LEVEL_DBG);

/* --- Test Harness State & Mocks --- */
struct mac_flow_fixture {
	dect_mac_context_t ft_ctx;
	dect_mac_context_t pt_ctx;
	dect_mac_context_t *current_ctx;
};

static struct mac_flow_fixture g_harness;

K_MSGQ_DEFINE(mac_event_msgq, sizeof(struct dect_mac_event_msg), 32, 4);

dect_mac_context_t *get_mac_context(void)
{
	return g_harness.current_ctx;
}

/* --- Test Fixture --- */
static void *setup(void)
{
	mock_phy_reset();
	// The event handler is now public via dect_mac_phy_if.h
	nrf_modem_dect_phy_event_handler_set(dect_mac_phy_if_event_handler);
	return &g_harness;
}

static void before(void *data)
{
	struct mac_flow_fixture *fixture = data;
	struct dect_mac_event_msg msg;
	while (k_msgq_get(&mac_event_msgq, &msg, K_NO_WAIT) == 0) { /* drain */
	}

	g_harness.current_ctx = &fixture->ft_ctx;
	dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	g_harness.current_ctx = &fixture->pt_ctx;
	dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);

	// Register scheduler hooks for the test
	dect_mac_data_path_register_scheduler_hook(dect_mac_data_path_service_tx);
}

static void teardown(void *data)
{
	ARG_UNUSED(data);
	/* This function is required by ZTEST_SUITE but can be empty if no cleanup is needed. */
}

/* --- Test Harness Helper --- */
static void run_mac_thread_for(uint32_t ms)
{
	struct dect_mac_event_msg msg;
	uint32_t start_time = k_uptime_get_32();

	while (k_uptime_get_32() - start_time < ms) {
		if (k_msgq_get(&mac_event_msgq, &msg, K_MSEC(1)) == 0) {
			/* Set context before dispatching */
			if (msg.type >= MAC_EVENT_PHY_OP_COMPLETE &&
			    msg.type <= MAC_EVENT_PHY_RSSI_RESULT) {
				uint32_t handle = 0;
				if (msg.type == MAC_EVENT_PHY_OP_COMPLETE) {
					handle = msg.data.op_complete.handle;
				} else if (msg.type == MAC_EVENT_PHY_PCC) {
					handle = msg.data.pcc.handle;
				} // Add other event types as needed
				if (g_harness.pt_ctx.pending_op_handle == handle) {
					g_harness.current_ctx = &g_harness.pt_ctx;
				} else {
					g_harness.current_ctx = &g_harness.ft_ctx;
				}
			}
			dect_mac_event_dispatch(&msg);
		}
	}
}

/* --- Test Cases --- */

ZTEST_SUITE(mac_flow_tests, NULL, setup, before, teardown, NULL);

ZTEST_F(mac_flow_fixture, test_full_association_flow)
{
	uint8_t pdu_capture_buf[256];
	uint16_t pdu_capture_len;

	TC_PRINT("Starting full association flow test...\n");

	/* 1. FT starts up, performs DCS, and enters beaconing state */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_sm_ft_start_operation();
	// Simulate successful DCS by injecting the final RSSI event
	struct nrf_modem_dect_phy_rssi_event rssi_event = { .handle = g_harness.ft_ctx.pending_op_handle };
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_RSSI, .rssi = rssi_event});
	struct nrf_modem_dect_phy_op_complete_event op_complete = { .handle = g_harness.ft_ctx.pending_op_handle, .err = NRF_MODEM_DECT_PHY_SUCCESS };
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .op_complete = op_complete});
	run_mac_thread_for(10);
	zassert_equal(g_harness.ft_ctx.state, MAC_STATE_FT_BEACONING, "FT did not start beaconing");

	/* 2. PT starts scanning */
	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_sm_pt_start_operation();
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_SCANNING, "PT did not start scanning");

	/* 3. FT sends a beacon. */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_sm_ft_beacon_timer_expired_action();
	run_mac_thread_for(10);
	mock_scheduled_operation_t *beacon_tx_op = mock_phy_get_last_scheduled_op();
	zassert_not_null(beacon_tx_op, "FT did not schedule a beacon TX");
	zassert_equal(beacon_tx_op->type, MOCK_OP_TYPE_TX, "Scheduled op was not a TX");
	mock_phy_capture_last_tx_pdu(pdu_capture_buf, &pdu_capture_len);
	zassert_true(pdu_capture_len > 0, "Captured beacon PDU is empty");

	/* 4. Simulate PT receiving this beacon */
	g_harness.current_ctx = &g_harness.pt_ctx;
	mock_rx_packet_t beacon_rx_pkt = {
		.reception_time_us = mock_phy_get_time_us() + 1000,
		.pcc_data.phy_type = 0,
		.pdc_len = pdu_capture_len,
	};
	memcpy(beacon_rx_pkt.pdc_payload, pdu_capture_buf, pdu_capture_len);
	mock_phy_queue_rx_packet(&beacon_rx_pkt);
	mock_phy_advance_time_us(2000);
	run_mac_thread_for(10);
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_ASSOCIATING,
		      "PT did not start associating after beacon");

	/* 5. PT now sends an Association Request. */
	run_mac_thread_for(10);
	mock_scheduled_operation_t *rach_tx_op = mock_phy_get_last_scheduled_op();
	zassert_not_null(rach_tx_op, "PT did not schedule RACH TX");
	mock_phy_capture_last_tx_pdu(pdu_capture_buf, &pdu_capture_len);
	zassert_true(pdu_capture_len > 0, "Captured Assoc Req PDU is empty");

	/* 6. Simulate FT receiving the Association Request */
	g_harness.current_ctx = &g_harness.ft_ctx;
	mock_rx_packet_t assoc_req_rx_pkt = {
		.reception_time_us = mock_phy_get_time_us() + 1000,
		.pcc_data.phy_type = 1,
		.pdc_len = pdu_capture_len,
	};
	memcpy(assoc_req_rx_pkt.pdc_payload, pdu_capture_buf, pdu_capture_len);
	mock_phy_queue_rx_packet(&assoc_req_rx_pkt);
	mock_phy_advance_time_us(2000);
	run_mac_thread_for(10);

	/* 7. Capture the FT's Association Response */
	run_mac_thread_for(10);
	mock_scheduled_operation_t *assoc_resp_tx_op = mock_phy_get_last_scheduled_op();
	zassert_not_null(assoc_resp_tx_op, "FT did not schedule Assoc Resp TX");
	mock_phy_capture_last_tx_pdu(pdu_capture_buf, &pdu_capture_len);
	zassert_true(pdu_capture_len > 0, "Captured Assoc Resp PDU is empty");

	/* 8. Simulate PT receiving the Association Response */
	g_harness.current_ctx = &g_harness.pt_ctx;
	mock_rx_packet_t assoc_resp_rx_pkt = {
		.reception_time_us = mock_phy_get_time_us() + 1000,
		.pcc_data.phy_type = 1,
		.pdc_len = pdu_capture_len,
	};
	memcpy(assoc_resp_rx_pkt.pdc_payload, pdu_capture_buf, pdu_capture_len);
	mock_phy_queue_rx_packet(&assoc_resp_rx_pkt);
	mock_phy_advance_time_us(2000);
	run_mac_thread_for(10);

	/* 9. Verify final states */
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_ASSOCIATED,
		      "PT did not reach ASSOCIATED state");
	zassert_true(g_harness.pt_ctx.role_ctx.pt.associated_ft.is_secure,
		     "PT link is not secure");
	zassert_true(g_harness.ft_ctx.role_ctx.ft.connected_pts[0].is_secure,
		     "FT link is not secure");
}

ZTEST_F(mac_flow_fixture, test_scheduled_access)
{
	TC_PRINT("Starting Scheduled Access and SFN Sync test...\n");

	/* 1. Setup: PT is associated with FT */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx = ft_find_and_init_peer_slot(g_harness.pt_ctx.own_long_rd_id,
						  g_harness.pt_ctx.own_short_rd_id, -50 * 2);
	zassert_true(peer_idx >= 0, "FT could not allocate peer slot");

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	memcpy(&g_harness.pt_ctx.role_ctx.pt.associated_ft,
	       &g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx],
	       sizeof(dect_mac_peer_info_t));

	/* 2. FT queues a packet for the PT */
	mac_sdu_t *sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	sdu->len = 10;
	sys_dlist_append(&g_harness.ft_ctx.role_ctx.ft.peer_tx_data_dlists[peer_idx].best_effort_dlist, &sdu->node);

	/* 3. FT scheduler runs and grants a schedule */
	g_harness.current_ctx = &g_harness.ft_ctx;
	ft_evaluate_and_update_pt_schedules();

	/* 4. Simulate time passing and service the TX path */
	uint64_t expected_dl_time =
		g_harness.ft_ctx.role_ctx.ft.peer_schedules[peer_idx].next_occurrence_modem_time;
    zassert_true(expected_dl_time > 0, "Scheduler did not set a valid next occurrence time");

	mock_phy_advance_time_to_us(expected_dl_time - 5000);
	g_harness.ft_ctx.last_known_modem_time = mock_phy_get_time_us();

	dect_mac_data_path_service_tx();

	/* 5. Verify the scheduled operation in the mock PHY */
	mock_scheduled_operation_t *op = mock_phy_get_last_scheduled_op();
	zassert_not_null(op, "FT scheduler did not schedule a TX operation");
	zassert_equal(op->type, MOCK_OP_TYPE_TX, "Scheduled op was not a TX");
	zassert_equal(op->start_time_us, expected_dl_time, "Scheduled TX has incorrect start time");
}

// ... (other test cases would be similarly rewritten to use public APIs and mock events) ...



ZTEST_F(mac_assoc_fixture, test_rach_timeout_and_retry)
{
	TC_PRINT("Starting RACH response timeout test...\n");

	/* Setup: PT is ready to send an association request */
	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);
	g_harness.pt_ctx.role_ctx.pt.target_ft.is_valid = true;
	g_harness.pt_ctx.role_ctx.pt.target_ft.is_fully_identified = true;
	g_harness.pt_ctx.role_ctx.pt.current_ft_rach_params.rach_operating_channel = 1881792;
	g_harness.pt_ctx.config.rach_response_window_ms = 50; /* Use a short timeout for the test */

	/* Action: Send the request, which starts the response timer */
	pt_send_association_request_action();
	run_mac_thread_for(10);
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_WAIT_ASSOC_RESP, "PT did not enter WAIT_ASSOC_RESP");
	zassert_equal(g_harness.pt_ctx.role_ctx.pt.current_assoc_retries, 0, "Initial retry count should be 0");

	/* Simulate time passing without a response */
	k_sleep(K_MSEC(100)); /* Wait longer than the timeout */
	run_mac_thread_for(10); /* Process the timer expiry event */

	/* Verification: Check that a retry occurred and state is back to associating */
	zassert_equal(g_harness.pt_ctx.role_ctx.pt.current_assoc_retries, 1, "Retry count did not increment");
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_ASSOCIATING, "PT did not re-enter ASSOCIATING state for retry");
}

ZTEST_F(mac_assoc_fixture, test_data_transfer_harq_ack_nack)
{
	struct dect_mac_event_msg msg;
	int ret;

	TC_PRINT("Starting HARQ Data Transfer test...\n");

	/* 1. Bring both state machines to a secure, associated state */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx = ft_find_and_init_peer_slot(g_harness.pt_ctx.own_long_rd_id, g_harness.pt_ctx.own_short_rd_id, -50*2);
	zassert_true(peer_idx >= 0, "FT could not allocate peer slot");
	g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx].is_secure = true;

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	memcpy(&g_harness.pt_ctx.role_ctx.pt.associated_ft, &g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx], sizeof(dect_mac_peer_info_t));
	g_harness.pt_ctx.role_ctx.pt.associated_ft.is_valid = true;
	g_harness.pt_ctx.role_ctx.pt.associated_ft.is_secure = true;


	/* 2. PT sends a data packet */
	TC_PRINT("PT sending data packet...\n");
	mac_sdu_t *sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	zassert_not_null(sdu, "Failed to alloc SDU");
	sdu->len = 20;
	strcpy(sdu->data, "test_data");
	ret = dect_mac_api_send(sdu, MAC_FLOW_RELIABLE_DATA);
	zassert_ok(ret, "dect_mac_api_send failed");

	/* 3. Service the PT's TX path to get the packet to the PHY mock */
	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_data_path_service_tx();
	run_mac_thread_for(10); /* Let PT process and schedule TX */

	/* Verify PT has an active HARQ process */
	int pt_harq_idx = -1;
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		if (g_harness.pt_ctx.harq_tx_processes[i].is_active) {
			pt_harq_idx = i;
			break;
		}
	}
	zassert_not_equal(pt_harq_idx, -1, "PT did not create an active HARQ process");

	/* 4. Simulate FT receiving the data and sending an ACK */
	TC_PRINT("Simulating FT received data, sending ACK...\n");
	g_harness.current_ctx = &g_harness.ft_ctx;
	g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx].num_pending_feedback_items = 1;
	g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx].pending_feedback_to_send[0].is_ack = true;
	g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx].pending_feedback_to_send[0].harq_process_num_for_peer = pt_harq_idx;

	/* 5. Simulate PT receiving the ACK */
	union nrf_modem_dect_phy_feedback feedback_from_ft;
	feedback_from_ft.format1.format = NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1;
	feedback_from_ft.format1.harq_process_number0 = pt_harq_idx;
	feedback_from_ft.format1.transmission_feedback0 = 1; /* ACK */

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_data_path_process_harq_feedback(&feedback_from_ft, g_harness.ft_ctx.own_short_rd_id);

	/* 6. Verify PT's HARQ process was cleared */
	zassert_false(g_harness.pt_ctx.harq_tx_processes[pt_harq_idx].is_active, "PT HARQ process was not cleared after ACK");

	/* --- NACK TEST --- */
	TC_PRINT("PT sending second data packet for NACK test...\n");
	sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	sdu->len = 20;
	ret = dect_mac_api_send(sdu, MAC_FLOW_RELIABLE_DATA);
	zassert_ok(ret);

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_data_path_service_tx();
	run_mac_thread_for(10);

	pt_harq_idx = -1;
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		if (g_harness.pt_ctx.harq_tx_processes[i].is_active) {
			pt_harq_idx = i;
			break;
		}
	}
	zassert_not_equal(pt_harq_idx, -1, "PT did not create a second HARQ process");

	TC_PRINT("Simulating FT received data, sending NACK...\n");
	feedback_from_ft.format1.transmission_feedback0 = 0; /* NACK */
	dect_mac_data_path_process_harq_feedback(&feedback_from_ft, g_harness.ft_ctx.own_short_rd_id);

	zassert_true(g_harness.pt_ctx.harq_tx_processes[pt_harq_idx].needs_retransmission, "PT HARQ process not marked for re-TX after NACK");
}

ZTEST_F(mac_assoc_fixture, test_ft_dcs_and_beaconing)
{
	struct dect_mac_event_msg msg;
	int ret;

	TC_PRINT("Starting FT DCS and Beaconing test...\n");

	/* 1. Configure mock PHY for a noisy channel and a clear channel */
	/* This requires a new mock PHY control function */
	/* For now, we will simulate the RSSI event directly */

	/* 2. FT starts up and enters scanning state */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_sm_ft_start_operation();
	zassert_equal(g_harness.ft_ctx.state, MAC_STATE_FT_SCANNING, "FT did not start scanning");
	run_mac_thread_for(10); /* Let it schedule the first scan */

	/* 3. Simulate completion of the first RSSI scan with a high noise level */
	int8_t noisy_scan_results[10] = { -60, -65, -62, -68, -61, -63, -65, -66, -64, -67 };
	struct nrf_modem_dect_phy_rssi_event noisy_rssi_event = {
		.handle = g_harness.ft_ctx.pending_op_handle,
		.carrier = g_harness.ft_ctx.role_ctx.ft.dcs_candidate_channels[0],
		.meas_len = ARRAY_SIZE(noisy_scan_results),
		.meas = noisy_scan_results,
	};
	struct nrf_modem_dect_phy_op_complete_event op_complete_event = {
		.handle = g_harness.ft_ctx.pending_op_handle,
		.err = NRF_MODEM_DECT_PHY_SUCCESS,
	};
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_RSSI, .data.rssi = noisy_rssi_event});
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .data.op_complete = op_complete_event});
	run_mac_thread_for(10); /* Process scan results and schedule next scan */

	/* 4. Simulate completion of the second RSSI scan with a clear channel */
	int8_t clear_scan_results[10] = { -110, -115, -112, -118, -111, -113, -115, -116, -114, -117 };
	struct nrf_modem_dect_phy_rssi_event clear_rssi_event = {
		.handle = g_harness.ft_ctx.pending_op_handle,
		.carrier = g_harness.ft_ctx.role_ctx.ft.dcs_candidate_channels[1],
		.meas_len = ARRAY_SIZE(clear_scan_results),
		.meas = clear_scan_results,
	};
	op_complete_event.handle = g_harness.ft_ctx.pending_op_handle;
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_RSSI, .data.rssi = clear_rssi_event});
	dect_mac_phy_if_event_handler(&(struct nrf_modem_dect_phy_event){.id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .data.op_complete = op_complete_event});
	run_mac_thread_for(10); /* Process scan results */

	/* Assume DCS is complete after 2 scans for this test */
	g_harness.ft_ctx.role_ctx.ft.dcs_scan_complete = true;
	ft_select_operating_carrier_and_start_beaconing(NULL);
	zassert_equal(g_harness.ft_ctx.state, MAC_STATE_FT_BEACONING, "FT did not enter beaconing state");
	zassert_equal(g_harness.ft_ctx.role_ctx.ft.operating_carrier, g_harness.ft_ctx.role_ctx.ft.dcs_candidate_channels[1], "FT did not select the clear channel");

	/* 5. PT starts scanning */
	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_sm_pt_start_operation();
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_SCANNING, "PT did not start scanning");

	/* 6. Manually inject a beacon PDC event to the PT's SM to simulate reception */
	/* This is a simplification because the mock PHY doesn't capture and replay TX data */
	dect_mac_cluster_beacon_ie_fields_t cb_fields;
	dect_mac_rach_info_ie_fields_t rach_fields;
	populate_cb_fields_from_ctx(&g_harness.ft_ctx, &cb_fields);
	rach_fields = g_harness.ft_ctx.role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields;
	
	pt_process_identified_beacon_and_attempt_assoc(&g_harness.pt_ctx, &cb_fields, &rach_fields,
		g_harness.ft_ctx.own_long_rd_id, g_harness.ft_ctx.own_short_rd_id, -60*2,
		g_harness.ft_ctx.role_ctx.ft.operating_carrier, 12345678ULL);

	/* 7. Verify PT has transitioned to associating state */
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_ASSOCIATING, "PT did not transition to associating state after beacon processing");
}

ZTEST_F(mac_assoc_fixture, test_pt_mobility_handover)
{
	TC_PRINT("Starting PT Mobility and Handover test...\n");

	/* 1. Setup: PT is associated with FT1 */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx_ft1 = ft_find_and_init_peer_slot(g_harness.pt_ctx.own_long_rd_id, g_harness.pt_ctx.own_short_rd_id, -70*2);
	zassert_true(peer_idx_ft1 >= 0, "FT1 could not allocate peer slot");

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	memcpy(&g_harness.pt_ctx.role_ctx.pt.associated_ft, &g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx_ft1], sizeof(dect_mac_peer_info_t));
	g_harness.pt_ctx.role_ctx.pt.associated_ft.is_valid = true;
	g_harness.pt_ctx.role_ctx.pt.initial_count_to_trigger = 3; /* Set a trigger count */

	/* 2. Simulate PT receiving a beacon from a much stronger FT2 */
	dect_mac_cluster_beacon_ie_fields_t cb_fields_ft2 = { .rel_quality_code = 2 /* 6dB */, .count_to_trigger_code = 3 };
	uint32_t ft2_long_id = 0x55667788;
	uint16_t ft2_short_id = 0x5566;
	int16_t ft2_rssi = -50 * 2; /* -50 dBm, much stronger than -70dBm */

	for (int i = 0; i < 3; i++) {
		TC_PRINT("Simulating PT receives strong beacon from FT2, attempt %d/3\n", i + 1);
		pt_evaluate_mobility_candidate(&g_harness.pt_ctx, &cb_fields_ft2, ft2_long_id, ft2_short_id, ft2_rssi, 1883520);
	}

	/* 3. Verify that the PT has triggered a handover and is now associating with FT2 */
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_HANDOVER_ASSOCIATING, "PT did not enter handover state");
	zassert_equal(g_harness.pt_ctx.role_ctx.pt.target_ft.long_rd_id, ft2_long_id, "PT is not targeting the correct new FT");
}

ZTEST_F(mac_assoc_fixture, test_pt_paging_cycle)
{
	TC_PRINT("Starting PT Paging Cycle test...\n");

	/* 1. Setup: PT is associated with FT, then enters paging mode */
	g_harness.current_ctx = &g_harness.ft_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	int peer_idx = ft_find_and_init_peer_slot(g_harness.pt_ctx.own_long_rd_id, g_harness.pt_ctx.own_short_rd_id, -60*2);
	zassert_true(peer_idx >= 0, "FT could not allocate peer slot");

	g_harness.current_ctx = &g_harness.pt_ctx;
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	memcpy(&g_harness.pt_ctx.role_ctx.pt.associated_ft, &g_harness.ft_ctx.role_ctx.ft.connected_pts[peer_idx], sizeof(dect_mac_peer_info_t));
	g_harness.pt_ctx.role_ctx.pt.associated_ft.is_valid = true;

	dect_mac_api_enter_paging_mode();
	run_mac_thread_for(10);
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_PT_PAGING, "PT did not enter paging state");

	/* 2. FT flags the PT for paging */
	g_harness.current_ctx = &g_harness.ft_ctx;
	zassert_ok(dect_mac_api_ft_page_pt(g_harness.pt_ctx.own_long_rd_id), "Failed to flag PT for paging");

	/* 3. Simulate PT receiving a beacon containing the page */
	g_harness.current_ctx = &g_harness.pt_ctx;
	pt_process_page_indication();
	run_mac_thread_for(10); /* Let PT process the page and send Keep-Alive */

	/* 4. Verify PT is back in associated state and sent a Keep-Alive */
	zassert_equal(g_harness.pt_ctx.state, MAC_STATE_ASSOCIATED, "PT did not return to associated state after page");
	zassert_equal(g_harness.pt_ctx.pending_op_type, PENDING_OP_PT_KEEP_ALIVE, "PT did not send a Keep-Alive in response to page");
}


ZTEST_SUITE(mac_flow_tests, NULL, setup, before, NULL, teardown);