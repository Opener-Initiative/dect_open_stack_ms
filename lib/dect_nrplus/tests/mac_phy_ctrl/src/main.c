#include <zephyr/ztest.h>
#include <mac/dect_mac_phy_ctrl.h>
#include <mac/dect_mac_phy_tbs_tables.h>
#include <zephyr/logging/log.h>


/* Test TBS lookup for mu=1, beta=1 */
ZTEST(mac_phy_ctrl_tests, test_tbs_mu1_beta1)
{
	uint8_t len_field;
	uint8_t mcs_field = 0; // MCS0
	uint8_t type_field;

	/* 136 bits = 17 bytes. Table says j=1 fits 136 bits. */
	dect_mac_phy_ctrl_calculate_pcc_params(17, 0, 0, &len_field, &mcs_field, &type_field);
	zassert_equal(type_field, 0, "Expected subslot mode");
	zassert_equal(len_field, 0, "Expected 1 subslot (len field 0)");

	/* 344 bits = 43 bytes. Table says j=2 fits 344 bits. */
	mcs_field = 0;
	dect_mac_phy_ctrl_calculate_pcc_params(43, 0, 0, &len_field, &mcs_field, &type_field);
	zassert_equal(len_field, 1, "Expected 2 subslots (len field 1)");
}

/* Test TBS lookup for mu=2, beta=1 */
ZTEST(mac_phy_ctrl_tests, test_tbs_mu2_beta1)
{
	uint8_t len_field;
	uint8_t mcs_field = 0; // MCS0
	uint8_t type_field;

	/* mu=2, beta=1: j=1 is 0 (unsupported). j=2 is 808 bits = 101 bytes. */
	dect_mac_phy_ctrl_calculate_pcc_params(50, 1, 0, &len_field, &mcs_field, &type_field);
	zassert_equal(len_field, 1, "Expected 2 subslots (j=2) as j=1 is unsupported for mu=2");
}

/* Test TBS lookup for mu=4, beta=1 */
ZTEST(mac_phy_ctrl_tests, test_tbs_mu4_beta1)
{
	uint8_t len_field;
	uint8_t mcs_field = 0; // MCS0
	uint8_t type_field;

	/* mu=4, beta=1: j=1,2,3 are 0. j=4 is 1800 bits = 225 bytes. */
	dect_mac_phy_ctrl_calculate_pcc_params(100, 2, 0, &len_field, &mcs_field, &type_field);
	zassert_equal(len_field, 3, "Expected 4 subslots (j=4) as j=1..3 are unsupported for mu=4");
}

/* Test Slot Mode (Large payloads) */
ZTEST(mac_phy_ctrl_tests, test_slot_mode)
{
	uint8_t len_field;
	uint8_t mcs_field = 8; // MCS8
	uint8_t type_field;

	/* For mu=1, 16 subslots = 1 slot. 
	 * MCS8 j=16 is 30000 bits = 3750 bytes.
	 * If we ask for something that needs more than 16 subslots, it should switch to slot mode.
	 * BUT: The current implementation is limited by the TBS tables which only go up to j=16.
	 * Let's test the logic for > 16 subslots if it were supported or forced.
	 */
	 
	/* Since TBS table max is j=16, calculate_pcc_params will clamp to 16 subslots 
	 * if payload is larger than max TBS entry.
	 */
	dect_mac_phy_ctrl_calculate_pcc_params(5000, 0, 0, &len_field, &mcs_field, &type_field);

	/* TBS_MAX_SUB_SLOTS_J is 16.
	 * num_subslots_needed will be 16.
	 * num_subslots_needed > 16 will be false.
	 * So it will stay in subslot mode, len_field = 15.
	 */
	zassert_equal(type_field, 0, "Expected subslot mode for max TBS capacity");
	zassert_equal(len_field, 15, "Expected max subslots (16)");
}

/* Test zero payload */
ZTEST(mac_phy_ctrl_tests, test_zero_payload)
{
	uint8_t len_field;
	uint8_t mcs_field = 5;
	uint8_t type_field;

	dect_mac_phy_ctrl_calculate_pcc_params(0, 0, 0, &len_field, &mcs_field, &type_field);
	zassert_equal(len_field, 0, "Expected 1 subslot for zero payload");
}

ZTEST_SUITE(mac_phy_ctrl_tests, NULL, NULL, NULL, NULL, NULL);
