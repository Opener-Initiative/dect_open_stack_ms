/* dect_mac/dect_mac_phy_ctrl.h */
#ifndef DECT_MAC_PHY_CTRL_H__
#define DECT_MAC_PHY_CTRL_H__

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>             // For size_t
#include "dect_mac_sm.h"        // For pending_op_type_t
#include "dect_mac_pdu.h"       // For dect_mac_header_type_octet_t

// Default TX Power code for PCC field (ETSI TS 103 636-4 Table 6.2.1-3a, e.g., 0dBm code 0111)
// This define is used if the context's config isn't directly available or for a true default.
#define DEFAULT_PCC_TX_POWER_CODE_PHY_CTRL 0b0111


/**
 * @brief Initializes the MAC PHY Control module.
 *        Currently, this function may be empty if all necessary state is managed
 *        within the global MAC context or if no specific module initialization is required.
 *        It could potentially initialize global buffers like g_phy_pcc_tx_constructor_buf.
 */
void dect_mac_phy_ctrl_init(void);

/**
 * @brief Starts a PHY RX operation.
 *
 * This function abstracts the call to `nrf_modem_dect_phy_rx`. It sets the
 * `pending_op_handle` and `pending_op_type` in the global MAC context to track
 * this operation.
 *
 * @param carrier The absolute channel frequency number for the reception.
 * @param duration_modem_units Duration of the reception in modem time units.
 *                             A value of 0 typically means continuous reception if the
 *                             `mode` supports it, until explicitly stopped or a
 *                             single-shot condition is met.
 * @param mode The RX mode (e.g., NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS).
 * @param phy_op_handle An application-defined handle for this specific PHY operation,
 *                      used to correlate completion events. Must be unique for active ops.
 * @param expected_receiver_id The Short RD ID that the received PCC's Receiver ID field
 *                             should match (or be broadcast 0xFFFF if applicable to PCC type).
 *                             For beacons (PCC Type 1), this is not used for filtering by ReceiverID.
 *                             For unicast (PCC Type 2), this should be own Short RD ID.
 * @param op_type The MAC-level operation type (pending_op_type_t) that this PHY RX
 *                operation corresponds to (e.g., PENDING_OP_PT_SCAN).
 * @return 0 on successful scheduling of the RX operation with the modem.
 * @retval -EBUSY If another MAC operation is already pending completion with a different handle.
 * @retval Other negative error codes from `nrf_modem_dect_phy_rx`.
 */
int dect_mac_phy_ctrl_start_rx(uint32_t carrier, uint32_t duration_modem_units,
                               enum nrf_modem_dect_phy_rx_mode mode,
                               uint32_t phy_op_handle, uint16_t expected_receiver_id,
                               pending_op_type_t op_type);

/**
 * @brief Starts a PHY TX operation using an already assembled MAC PDU, potentially at a scheduled time.
 *
 * This function abstracts the call to `nrf_modem_dect_phy_tx`. It constructs the
 * necessary Physical Control Channel (PCC) header based on the provided MAC PDU details
 * and then schedules the transmission with the modem. It also manages the
 * `pending_op_handle` and `pending_op_type` in the global MAC context.
 *
 * @param carrier The absolute channel frequency number for the transmission.
 * @param full_mac_pdu_to_send Pointer to the buffer containing the complete MAC PDU to be sent.
 *                             The content is: MAC Hdr Type (1B) | MAC Common Hdr | MAC SDU Area | [MIC].
 * @param full_mac_pdu_len Total length of the `full_mac_pdu_to_send` in bytes.
 * @param target_receiver_short_id Short RD ID of the intended receiver of this PDU. This is used
 *                                 to populate the Receiver ID field in the Type 2 PCC.
 *                                 For beacons (which use Type 1 PCC), this can be 0xFFFF.
 * @param is_beacon Boolean flag indicating if this transmission is a beacon.
 *                  True for beacons (uses nRF PHY Type 0 / ETSI PCC Type 1).
 *                  False for other PDUs (uses nRF PHY Type 1 / ETSI PCC Type 2).
 * @param phy_op_handle An application-defined handle for this specific PHY operation. Must be unique.
 * @param op_type The MAC-level operation type (pending_op_type_t) that this PHY TX
 *                operation corresponds to (e.g., PENDING_OP_FT_BEACON).
 * @param use_lbt True to perform Listen-Before-Talk before transmission, false otherwise.
 * @param phy_op_target_start_time The absolute modem time at which this operation should start.
 *                                 If 0, the operation is scheduled for immediate execution
 *                                 (subject to PHY latencies and radio mode).
 * @param mu_code mu_code needed for duration calculation
 * @return 0 on successful scheduling of the TX operation with the modem.
 * @retval -EBUSY If another MAC operation is already pending completion with a different handle.
 * @retval -EINVAL If parameters like PDU length are invalid.
 * @retval -ENOMEM If internal buffer copy fails.
 * @retval Other negative error codes from `nrf_modem_dect_phy_tx`.
 */
int dect_mac_phy_ctrl_start_tx_assembled(uint32_t carrier,
					 const uint8_t *full_mac_pdu_to_send,
					 uint16_t full_mac_pdu_len,
					 uint16_t target_receiver_short_id, bool is_beacon,
					 uint32_t phy_op_handle, pending_op_type_t op_type,
					 bool use_lbt, uint64_t phy_op_target_start_time,
					 uint8_t mu_code,
					 const union nrf_modem_dect_phy_feedback *feedback);

/**
 * @brief Starts a PHY RSSI scan operation.
 *
 * @param carrier The absolute channel frequency number to scan.
 * @param duration_modem_units Duration of the scan in modem time units.
 * @param reporting_interval Enum specifying how often RSSI measurements should be reported
 *                           via the NRF_MODEM_DECT_PHY_EVT_RSSI event.
 * @param phy_op_handle An application-defined handle for this PHY operation. Must be unique.
 * @param op_type The MAC-level operation type this scan corresponds to.
 * @return 0 on successful scheduling.
 * @retval -EBUSY If another MAC operation is pending with a different handle.
 * @retval Other negative error codes from `nrf_modem_dect_phy_rssi`.
 */
int dect_mac_phy_ctrl_start_rssi_scan(uint32_t carrier, uint32_t duration_modem_units,
                                      enum nrf_modem_dect_phy_rssi_interval reporting_interval,
                                      uint32_t phy_op_handle, pending_op_type_t op_type);

/**
 * @brief Cancels a pending or ongoing PHY operation identified by its handle.
 *
 * @param phy_op_handle Handle of the operation to cancel.
 *                      Use `NRF_MODEM_DECT_PHY_HANDLE_CANCEL_ALL` to cancel all scheduled/ongoing PHY ops.
 * @return 0 on success (cancel request sent to modem).
 * @retval Negative error codes from `nrf_modem_dect_phy_cancel`.
 */
int dect_mac_phy_ctrl_cancel_op(uint32_t phy_op_handle);

/**
 * @brief Handles the common logic for a NRF_MODEM_DECT_PHY_EVT_COMPLETED event.
 *
 * This function is called by the main MAC event dispatcher. It checks if the
 * `event->handle` matches the `pending_op_handle` stored in the MAC context.
 * If they match, it means the actively tracked MAC operation has just completed
 * at the PHY level. This function then clears the `pending_op_type` and
 * `pending_op_handle` in the MAC context to allow new operations to be scheduled.
 *
 * @param event Pointer to the const `nrf_modem_dect_phy_op_complete_event` structure.
 * @return The `pending_op_type_t` of the operation that just completed if it was
 *         the one actively tracked by the MAC.
 * @return `PENDING_OP_NONE` if the completed handle does not match the actively
 *         tracked one, or if no operation was actively pending with that handle.
 */
pending_op_type_t dect_mac_phy_ctrl_handle_op_complete(const struct nrf_modem_dect_phy_op_complete_event *event);

/**
 * @brief Assembles the final MAC PDU structure before security operations (MIC, encryption).
 *
 * This function takes the main components of a MAC PDU (MAC Header Type octet,
 * MAC Common Header, and the MAC SDU Area which contains MUXed IEs or a MUXed user data SDU)
 * and concatenates them into a target buffer.
 * The output buffer `target_final_mac_pdu_buf` will then be ready for security
 * processing by the calling SM or data path function.
 *
 * @param target_final_mac_pdu_buf Buffer to store the assembled MAC PDU.
 * @param target_buf_max_len Maximum size of `target_final_mac_pdu_buf`.
 * @param mac_hdr_type_octet Pointer to the `dect_mac_header_type_octet_t` structure.
 * @param common_hdr_data Pointer to the MAC Common Header data. Can be NULL if no common header.
 * @param common_hdr_len Length of the `common_hdr_data`. Must be 0 if `common_hdr_data` is NULL.
 * @param mac_sdu_area_data Pointer to the MAC SDU Area. Can be NULL if no SDU area.
 * @param mac_sdu_area_len Length of the `mac_sdu_area_data`. Must be 0 if `mac_sdu_area_data` is NULL.
 * @param out_assembled_pdu_len_cleartext Pointer where the total length of the
 *                                        assembled MAC PDU (before MIC) will be stored.
 * @return 0 on success.
 * @retval -ENOMEM If `target_final_mac_pdu_buf` is too small.
 * @retval -EINVAL If input parameters are invalid.
 */
int dect_mac_phy_ctrl_assemble_final_pdu(
    uint8_t *target_final_mac_pdu_buf,
    size_t target_buf_max_len,
    uint8_t mac_hdr_type_octet_byte,
    const void *common_hdr_data,
    size_t common_hdr_len,
    const uint8_t *mac_sdu_area_data,
    size_t mac_sdu_area_len,
    uint16_t *out_assembled_pdu_len_cleartext);


/**
 * @brief Calculates the Physical Control Channel (PCC) `packet_length`, `df_mcs`,
 *        and `packet_length_type` fields for a given PDC payload.
 *
 * CRITICAL TODO: Current implementation is simplified for MCS0, mu=1, beta=1 only.
 * Needs full implementation based on ETSI TS 103 636-3 tables/formulas for all
 * supported MCS, mu, and beta values.
 *
 * @param mac_pdc_payload_len_bytes Length of the MAC PDU's PDC part in bytes.
 *                                  (This is MAC Common Header + MAC SDU Area + MIC if secured).
 * @param out_packet_length_field Pointer to store calculated PCC `packet_length` field (N-1 coded).
 * @param in_out_selected_mcs_field Pointer to desired/selected MCS code. Function may adjust this
 *                                  if payload doesn't fit, and outputs the MCS used for calculation.
 * @param out_packet_length_type_field Pointer to store calculated `packet_length_type` (0=subslots, 1=slots).
 */
void dect_mac_phy_ctrl_calculate_pcc_params(size_t mac_pdc_payload_len_bytes,
                                           uint8_t mu, uint8_t beta, /* New parameters */
                                           uint8_t *out_packet_length_field,
                                           uint8_t *in_out_selected_mcs_field,
                                           uint8_t *out_packet_length_type_field);

/**
 * @brief Deactivates the DECT PHY software stack.
 *
 * This function is a wrapper around nrf_modem_dect_phy_deactivate().
 * It should be called when entering an error state or shutting down the MAC layer
 * to ensure the PHY is gracefully stopped. Deactivation implicitly cancels all
 * pending and ongoing PHY operations.
 *
 * @return 0 on success (request sent to modem), or a negative error code.
 */
int dect_mac_phy_ctrl_deactivate(void);


#endif /* DECT_MAC_PHY_CTRL_H__ */