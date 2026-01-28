/* tests/mac_pdu/src/main.c */
// Overview: Ztest suite for verifying the MAC PDU serialization and deserialization functions. This first test case validates the Association Request IE.
// --- CREATE NEW FILE ---
#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/sys_clock.h>  // Header for k_sys_tick_announce()


/* Include the internal PDU header and context header for IE structures */
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_context.h>

/* --- Test Setup --- */
static void *dect_mac_pdu_setup(void)
{
	// /* No setup needed for these stateless tests */
	// k_sys_tick_announce();
	return NULL;
}

/* --- Test Cases --- */

ZTEST(dect_mac_pdu, test_assoc_req_ie_serialization)
{
	uint8_t buffer[128];
	dect_mac_assoc_req_ie_t fields_to_serialize;
	dect_mac_assoc_req_ie_t fields_parsed;

	/* --- Test Case 1: Basic PT Association Request --- */
	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.setup_cause_val = ASSOC_CAUSE_INITIAL_ASSOCIATION;
	fields_to_serialize.power_const_active = false;
	fields_to_serialize.ft_mode_capable = false;
	fields_to_serialize.number_of_flows_val = 1;
	fields_to_serialize.flow_ids[0] = IE_TYPE_USER_DATA_FLOW_1;
	fields_to_serialize.harq_params_present = true;
	fields_to_serialize.harq_processes_tx_val = 2; /* 4 processes */
	fields_to_serialize.max_harq_re_tx_delay_code = 10;
	fields_to_serialize.harq_processes_rx_val = 2; /* 4 processes */
	fields_to_serialize.max_harq_re_rx_delay_code = 10;

	int len = serialize_assoc_req_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_true(len > 0, "Serialization failed, returned %d", len);

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	int ret = parse_assoc_req_ie_payload(buffer, len, &fields_parsed);
	zassert_ok(ret, "Parsing failed, returned %d", ret);

	zassert_equal(fields_to_serialize.setup_cause_val, fields_parsed.setup_cause_val,
			"Setup cause mismatch");
	zassert_equal(fields_to_serialize.number_of_flows_val, fields_parsed.number_of_flows_val,
			"Number of flows mismatch");
	zassert_equal(fields_to_serialize.flow_ids[0], fields_parsed.flow_ids[0],
			"Flow ID mismatch");
	zassert_equal(fields_to_serialize.harq_processes_tx_val,
			fields_parsed.harq_processes_tx_val, "HARQ TX procs mismatch");
	zassert_equal(fields_to_serialize.max_harq_re_tx_delay_code,
			fields_parsed.max_harq_re_tx_delay_code, "HARQ Re-TX delay mismatch");
}


ZTEST(dect_mac_pdu, test_assoc_resp_ie_serialization)
{
	uint8_t buffer[128];
	dect_mac_assoc_resp_ie_t fields_to_serialize;
	dect_mac_assoc_resp_ie_t fields_parsed;

	/* --- Test Case 1: Association ACCEPT with HARQ mods and Group Assign --- */
    printk("/* --- Test Case 1: Association ACCEPT with HARQ mods and Group Assign --- */ \n");
	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.ack_nack = true;
	fields_to_serialize.harq_mod_present = true;
	fields_to_serialize.number_of_flows_accepted = 7; /* All flows accepted */
	fields_to_serialize.group_assignment_active = true;
	fields_to_serialize.harq_processes_tx_val_ft = 1; /* 2 procs */
	fields_to_serialize.max_harq_re_tx_delay_code_ft = 5;
	fields_to_serialize.harq_processes_rx_val_ft = 1; /* 2 procs */
	fields_to_serialize.max_harq_re_rx_delay_code_ft = 5;
	fields_to_serialize.group_id_val = 42;
	fields_to_serialize.resource_tag_val = 12;

	int len = serialize_assoc_resp_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_true(len > 0, "Serialization for ACCEPT failed, returned %d", len);

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	int ret = parse_assoc_resp_ie_payload(buffer, len, &fields_parsed);
	zassert_ok(ret, "Parsing for ACCEPT failed, returned %d", ret);

	zassert_true(fields_parsed.ack_nack, "ACK/NACK mismatch");
	zassert_true(fields_parsed.harq_mod_present, "HARQ mod mismatch");
	zassert_true(fields_parsed.group_assignment_active, "Group assign mismatch");
	zassert_equal(fields_to_serialize.group_id_val, fields_parsed.group_id_val,
			"Group ID mismatch");
	zassert_equal(fields_to_serialize.resource_tag_val, fields_parsed.resource_tag_val,
			"Resource Tag mismatch");

	/* --- Test Case 2: Association REJECT --- */
    printk("/* --- Test Case 2: Association REJECT --- */ \n");
	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.ack_nack = false;
	fields_to_serialize.reject_cause = ASSOC_REJECT_CAUSE_NO_HW_CAP;
	fields_to_serialize.reject_timer_code = 8;

	len = serialize_assoc_resp_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_equal(len, 2, "Serialized REJECT length should be 2 bytes");

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	ret = parse_assoc_resp_ie_payload(buffer, len, &fields_parsed);
	zassert_ok(ret, "Parsing for REJECT failed, returned %d", ret);

	zassert_false(fields_parsed.ack_nack, "ACK/NACK mismatch for REJECT");
	zassert_equal(fields_to_serialize.reject_cause, fields_parsed.reject_cause,
			"Reject cause mismatch");
	zassert_equal(fields_to_serialize.reject_timer_code, fields_parsed.reject_timer_code,
			"Reject timer mismatch");
}


ZTEST(dect_mac_pdu, test_cluster_beacon_ie_serialization)
{
	uint8_t buffer[128];
	dect_mac_cluster_beacon_ie_fields_t fields_to_serialize;
	dect_mac_cluster_beacon_ie_fields_t fields_parsed;

	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.sfn = 0xAB;
	fields_to_serialize.tx_power_present = true;
	fields_to_serialize.clusters_max_tx_power_code = 0x0A; /* 7 dBm */
	fields_to_serialize.network_beacon_period_code = 3;   /* 1000 ms */
	fields_to_serialize.cluster_beacon_period_code = 8;   /* 8000 ms */
	fields_to_serialize.count_to_trigger_code = 4;
	fields_to_serialize.rel_quality_code = 2;
	fields_to_serialize.min_quality_code = 1;


    printk("/* --- Test Case 2: test_cluster_beacon_ie_serialization --- */ \n");

	int len = serialize_cluster_beacon_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_true(len > 0, "Serialization failed, returned %d", len);

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	int ret = parse_cluster_beacon_ie_payload(buffer, len, &fields_parsed);
	zassert_ok(ret, "Parsing failed, returned %d", ret);

	zassert_equal(fields_to_serialize.sfn, fields_parsed.sfn, "SFN mismatch");
	zassert_equal(fields_to_serialize.tx_power_present, fields_parsed.tx_power_present,
			"TX power present flag mismatch");
	zassert_equal(fields_to_serialize.clusters_max_tx_power_code,
			fields_parsed.clusters_max_tx_power_code, "Max TX power mismatch");
	zassert_equal(fields_to_serialize.cluster_beacon_period_code,
			fields_parsed.cluster_beacon_period_code, "Cluster period mismatch");
	zassert_equal(fields_to_serialize.count_to_trigger_code,
			fields_parsed.count_to_trigger_code, "Count to trigger mismatch");
	zassert_equal(fields_to_serialize.rel_quality_code, fields_parsed.rel_quality_code,
			"Rel quality mismatch");
	zassert_equal(fields_to_serialize.min_quality_code, fields_parsed.min_quality_code,
			"Min quality mismatch");
}



ZTEST(dect_mac_pdu, test_rach_info_ie_serialization)
{
	uint8_t buffer[128];
	dect_mac_rach_info_ie_fields_t fields_to_serialize;
	dect_mac_rach_info_ie_fields_t fields_parsed;
	uint8_t ft_mu_code = 1; /* mu=2, implies 8-bit start_subslot */

	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.sfn_validity_present = true;
	fields_to_serialize.channel_field_present = true;
	fields_to_serialize.start_subslot_index = 10;
	fields_to_serialize.num_subslots_or_slots = 4;
	fields_to_serialize.cwmin_sig_code = 2;
	fields_to_serialize.cwmax_sig_code = 5;
	fields_to_serialize.repetition_code = 1; /* Repeats every 2 frames */
	fields_to_serialize.sfn_value = 0x55;
	fields_to_serialize.validity_frames = 200;
	fields_to_serialize.channel_abs_num = 1152; /* Example valid 13-bit channel number */
	fields_to_serialize.mu_value_for_ft_beacon = ft_mu_code;

	int len = serialize_rach_info_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_true(len > 0, "Serialization failed, returned %d", len);

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	int ret = parse_rach_info_ie_payload(buffer, len, ft_mu_code, &fields_parsed);
	zassert_ok(ret, "Parsing failed, returned %d", ret);

	zassert_equal(fields_to_serialize.sfn_validity_present, fields_parsed.sfn_validity_present);
	zassert_equal(fields_to_serialize.channel_field_present,
			fields_parsed.channel_field_present);
	zassert_equal(fields_to_serialize.start_subslot_index, fields_parsed.start_subslot_index);
	zassert_equal(fields_to_serialize.num_subslots_or_slots,
			fields_parsed.num_subslots_or_slots);
	zassert_equal(fields_to_serialize.cwmin_sig_code, fields_parsed.cwmin_sig_code);
	zassert_equal(fields_to_serialize.cwmax_sig_code, fields_parsed.cwmax_sig_code);
	zassert_equal(fields_to_serialize.repetition_code, fields_parsed.repetition_code);
	zassert_equal(fields_to_serialize.sfn_value, fields_parsed.sfn_value);
	zassert_equal(fields_to_serialize.validity_frames, fields_parsed.validity_frames);
	zassert_equal(fields_to_serialize.channel_abs_num,
			fields_parsed.channel_abs_num);
}

ZTEST(dect_mac_pdu, test_rd_capability_ie_serialization)
{
	uint8_t buffer[128];
	dect_mac_rd_capability_ie_t fields_to_serialize;
	dect_mac_rd_capability_ie_t fields_parsed;

	memset(&fields_to_serialize, 0, sizeof(fields_to_serialize));
	fields_to_serialize.release_version = 1; /* Release 2 */
	fields_to_serialize.num_phy_capabilities = 1; /* One explicit set follows */
	fields_to_serialize.supports_mesh = true;
	fields_to_serialize.operating_modes_code = DECT_MAC_OP_MODE_BOTH;
	fields_to_serialize.mac_security_modes_code = DECT_MAC_SECURITY_SUPPORT_MODE1;

	dect_mac_phy_capability_set_t *phy_set = &fields_to_serialize.phy_variants[0];
	phy_set->mu_value = 1; /* mu=2 */
	phy_set->beta_value = 3; /* beta=4 */
	phy_set->max_nss_for_rx_code = 1; /* 2 streams */
	phy_set->max_mcs_code = 11;
	phy_set->harq_soft_buffer_size_code = 7;
	phy_set->num_harq_processes_code = 3; /* 8 processes */
	phy_set->harq_feedback_delay_code = 2;

	int len = serialize_rd_capability_ie_payload(buffer, sizeof(buffer), &fields_to_serialize);
	zassert_true(len > 0, "Serialization failed, returned %d", len);
	zassert_equal(len, 2 + 5, "Expected length for base + 1 PHY set is 7 bytes");

	memset(&fields_parsed, 0, sizeof(fields_parsed));
	int ret = parse_rd_capability_ie_payload(buffer, len, &fields_parsed);
	zassert_ok(ret, "Parsing failed, returned %d", ret);

	zassert_equal(fields_to_serialize.release_version, fields_parsed.release_version);
	zassert_equal(fields_to_serialize.num_phy_capabilities, fields_parsed.num_phy_capabilities);
	zassert_equal(fields_to_serialize.supports_mesh, fields_parsed.supports_mesh);
	zassert_equal(fields_to_serialize.operating_modes_code, fields_parsed.operating_modes_code);
	zassert_equal(fields_to_serialize.mac_security_modes_code,
			fields_parsed.mac_security_modes_code);
	zassert_equal(fields_parsed.actual_num_phy_variants_parsed, 1);
	zassert_mem_equal(&fields_to_serialize.phy_variants[0], &fields_parsed.phy_variants[0],
			  sizeof(dect_mac_phy_capability_set_t), "PHY capability set mismatch");
}


/* --- Test Suite Definition --- */
ZTEST_SUITE(dect_mac_pdu, NULL, dect_mac_pdu_setup, NULL, NULL, NULL);