/* dect_mac/dect_mac_data_path.h */
#ifndef DECT_MAC_DATA_PATH_H__
#define DECT_MAC_DATA_PATH_H__

#include <zephyr/kernel.h>      // For k_timer
#include <stdint.h>             // For uintxx_t types
#include <stddef.h>             // For size_t
#include "dect_dlc.h"           // For dlc_tx_status_cb_t

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

/**
 * @brief Initializes the MAC data path module.
 *
 * This function should be called once during MAC initialization (e.g., by `dect_mac_core_init`).
 * It's responsible for setting up any internal state for the data path,
 * primarily initializing the HARQ process structures and their associated timers.
 */
void dect_mac_data_path_init(void);

/**
 * @brief Registers the DLC's callback for TX status reports.
 *
 * The DLC layer calls this during its initialization to provide the MAC data
 * path with a function to call when a reportable SDU transmission is complete.
 *
 * @param cb The DLC's callback handler function.
 */
void dect_mac_data_path_register_dlc_callback(dlc_tx_status_cb_t cb);


/**
 * @brief Services the transmit (TX) queues, HARQ retransmissions, and scheduled TX opportunities.
 *
 * This function is intended to be called periodically by the main MAC processing thread's
 * loop or can be triggered by events such as a new SDU being added to a TX queue or
 * a HARQ timer expiring. It should only attempt to schedule one PHY operation per call.
 *
 * Its responsibilities include:
 *  1. Prioritizing and attempting HARQ retransmissions for active HARQ processes
 *     that are marked as needing retransmission, if a scheduled TX opportunity exists.
 *  2. If no retransmissions are pending/possible and a scheduled TX opportunity exists,
 *     and free HARQ processes are available, it attempts to dequeue new MAC SDUs (DLC PDUs)
 *     from the QoS-differentiated TX FIFOs (per-peer for FT DL, or generic for PT UL).
 *  3. For each SDU selected for transmission (new or retransmission), it prepares
 *     the full MAC PDU (including MAC headers, security if applicable) and requests
 *     the PHY control layer to schedule its transmission at the determined time and carrier.
 *  4. Manages HARQ process state (e.g., updating RV, tx_attempts, starting ACK timers).
 */
void dect_mac_data_path_service_tx(void);

/**
 * @brief Handles a received MAC SDU payload (typically a DLC PDU, encapsulated in MUXed IEs)
 *        from the PHY layer, after security processing (decryption/MIC) has been done by the SM.
 *
 * This function is called by the role-specific state machine's PDC handler
 * (e.g., `pt_handle_phy_pdc_internal` or `ft_handle_phy_pdc_internal`) after the
 * MAC Common Header has been parsed, security has been handled,
 * and the remaining payload is identified as a MAC SDU Area intended for data delivery
 * (e.g., containing one or more MUXed User Data IEs).
 *
 * Its responsibilities include:
 *  - Parsing the MAC Multiplexing header(s) from the SDU Area.
 *  - If a User Data IE is found, allocating a `mac_sdu_t` buffer.
 *  - Copying the User Data IE's payload (the DLC PDU) into this buffer.
 *  - Queueing the buffer to the upper layer (DLC's RX FIFO, `g_dlc_rx_sdu_fifo_ptr`).
 *
 * @param mac_sdu_area_data Pointer to the start of the (now cleartext) MAC SDU Area data.
 *                          This area starts *after* the MAC Common Header and potentially
 *                          after a MAC Security Info IE if that was sent cleartext.
 * @param mac_sdu_area_len Length of the `mac_sdu_area_data` (excluding any final MIC).
 * @param transmitter_long_rd_id Long RD ID of the device that sent this SDU. This information
 *                               is typically extracted from the MAC Common Header by the caller.
 */
void dect_mac_data_path_handle_rx_sdu(const uint8_t *mac_sdu_area_data,
                                      size_t mac_sdu_area_len,
                                      uint32_t transmitter_long_rd_id);

/**
 * @brief Processes HARQ feedback received in a Physical Control Channel (PCC).
 *
 * This function is called by the role-specific state machine's PCC handler
 * when HARQ feedback is present in the received PCC's Type 2 header.
 * It parses the `nrf_modem_dect_phy_feedback` information and updates the state
 * of the corresponding HARQ TX process(es) by calling
 * `dect_mac_data_path_handle_harq_ack_action` or
 * `dect_mac_data_path_handle_harq_nack_action`.
 *
 * @param feedback Pointer to the `nrf_modem_dect_phy_feedback` union received in the PCC.
 * @param peer_short_rd_id Short RD ID of the peer device that sent this HARQ feedback.
 */
void dect_mac_data_path_process_harq_feedback(const union nrf_modem_dect_phy_feedback *feedback,
                                             uint16_t peer_short_rd_id);

/**
 * @brief Callback function invoked when a HARQ retransmission timer expires.
 *
 * This function executes in the context of the system timer thread. It queues a
 * `MAC_EVENT_TIMER_EXPIRED_HARQ` message to the main MAC processing thread.
 * The `user_data` field of the `k_timer` instance should contain the HARQ process index.
 *
 * @param timer_id Pointer to the `k_timer` instance that expired.
 */
void dect_mac_data_path_harq_timer_expired(struct k_timer *timer_id);

/**
 * @brief Action function to handle a HARQ ACK for a specific TX process.
 *
 * This is called by the MAC state machine (or directly by `dect_mac_data_path_process_harq_feedback`)
 * in response to a parsed ACK in HARQ feedback or potentially other ACK-implying events.
 * It stops the retransmission timer, frees the SDU buffer associated with the
 * HARQ process, and marks the process as inactive/free.
 *
 * @param harq_process_idx The index of the HARQ TX process that was ACKed.
 */
void dect_mac_data_path_handle_harq_ack_action(int harq_process_idx);

/**
 * @brief Action function to handle a HARQ NACK (or timeout treated as NACK) for a specific TX process.
 *
 * This is called by the MAC state machine (e.g., in response to `MAC_EVENT_TIMER_EXPIRED_HARQ`)
 * or directly by `dect_mac_data_path_process_harq_feedback`.
 * It stops the current timer, marks the HARQ process as needing retransmission
 * (`needs_retransmission = true`), updates the redundancy version for the next attempt.
 * If max retries are reached, the SDU is discarded and the process is freed.
 * The `dect_mac_data_path_service_tx` function will pick it up for retransmission if applicable.
 *
 * @param harq_process_idx The index of the HARQ TX process that was NACKed or timed out.
 */
void dect_mac_data_path_handle_harq_nack_action(int harq_process_idx);


// /**
//  * @brief Services scheduled transmission opportunities for the FT role.
//  *
//  * This function is called by the main data path service loop when the device
//  * is an FT. It iterates through connected PTs and their schedules to find
//  * the next available TX or RX opportunity.
//  */
// void ft_service_schedules(void);

/**
 * @brief Checks if a given SDU will fit into a scheduled allocation.
 *
 * @param ctx Pointer to the MAC context.
 * @param sdu Pointer to the SDU to check.
 * @param schedule Pointer to the schedule for the allocation.
 * @param peer_slot_idx For FT role, the index of the target peer. -1 for PT role.
 * @return True if the SDU fits, false otherwise.
 */
bool does_sdu_fit_schedule(dect_mac_context_t *ctx, mac_sdu_t *sdu,
			   dect_mac_schedule_t *schedule, int peer_slot_idx);



/**
 * @brief Attempts to send a new data SDU from the specified peer and flow.
 *
 * This is the main entry point for the scheduler to send new data. This function
 * will find a free HARQ process and call the internal PHY sender.
 *
 * @return 0 on success, -EBUSY if no HARQ process is free, or other error codes.
 */
int dect_mac_data_path_send_new_sdu(mac_sdu_t *sdu, mac_flow_id_t flow_id, int peer_slot_idx,
                                    uint16_t carrier, uint64_t start_time,
                                    const union nrf_modem_dect_phy_feedback *feedback);

/**
 * @brief Handles a PHY_OP_COMPLETE event for a data path operation.
 *
 * @param completed_type The type of the completed operation.
 * @param event Pointer to the op_complete event data.
 */
void dect_mac_data_path_handle_op_complete(pending_op_type_t completed_type,
					   const struct nrf_modem_dect_phy_op_complete_event *event);






                       
                                    

#if IS_ENABLED(CONFIG_ZTEST)
/**
 * @brief [TEST ONLY] Gets the currently registered DLC TX status callback.
 *
 * @return The function pointer to the registered callback.
 */
dlc_tx_status_cb_t dect_mac_test_get_dlc_status_callback(void);
#endif /* IS_ENABLED(CONFIG_ZTEST) */




#endif /* DECT_MAC_DATA_PATH_H__ */