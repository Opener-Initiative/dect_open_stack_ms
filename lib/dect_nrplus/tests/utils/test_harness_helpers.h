/* lib/dect_nrplus/tests/utils/test_harness_helpers.h */
// Overview: A new header for shared test utilities, including the safe MAC state change callback.
#ifndef TEST_HARNESS_HELPERS_H__
#define TEST_HARNESS_HELPERS_H__

#include <mac/dect_mac_types.h>

/**
 * @brief Searches a captured MAC PDU for a specific IE and returns its payload.
 *
 * @param pdu_data Pointer to the start of the captured MAC PDU.
 * @param pdu_len Total length of the PDU.
 * @param target_ie_type The IE type to search for.
 * @param out_ie_len Pointer to store the length of the found IE's payload.
 * @return Pointer to the start of the IE's payload, or NULL if not found.
 */
const uint8_t *get_ie_payload_from_pdu(const uint8_t *pdu_data, uint16_t pdu_len,
				       uint8_t target_ie_type, uint16_t *out_ie_len);

/**
 * @brief Checks if the last captured TX PDU was transmitted by a specific Long RD ID.
 *
 * @param long_id The Long RD ID to check against the transmitter field.
 * @return True if the transmitter ID matches, false otherwise.
 */
bool tx_capture_is_from_long_id(uint32_t long_id);


/**
 * @brief A safe, test-only MAC state change callback that only logs the new state.
 *
 * This function can be registered by any test suite to receive state change
 * notifications from the MAC layer without interacting with subsystems (like networking)
 * that may not be initialized in the test environment.
 *
 * @param new_state The new public state of the MAC layer.
 */
void test_mac_state_change_cb(dect_mac_public_state_t new_state);


bool tx_capture_is_from_long_id(uint32_t expected_tx_long_id);
const uint8_t *get_ie_payload_from_pdu(const uint8_t *pdu, uint16_t pdu_len,
					      uint8_t target_ie_type, uint16_t *payload_len);


#endif /* TEST_HARNESS_HELPERS_H__ */