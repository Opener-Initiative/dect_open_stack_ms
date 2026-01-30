/* lib/dect_nrplus/mac/include/dect_mac.h */
/* Overview: New top-level public header for the entire MAC layer. This is the single point of entry for upper layers. */
#ifndef DECT_MAC_H__
#define DECT_MAC_H__

#include <zephyr/kernel.h>
#include <mac/dect_mac_types.h>
#include <mac/dect_mac_sm.h>


/**
 * @brief Necassary Fudge until we can get the DECT PHY working
 **/
enum nrf_modem_dect_phy_feedback_format {
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_NONE = 0,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1 = 1,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_2 = 2,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_3 = 3,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_4 = 4,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_5 = 5,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_6 = 6,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_7 = 7,
};

/* Dlist declares*/
extern sys_dlist_t g_mac_tx_dlist_high_priority;
extern sys_dlist_t g_mac_tx_dlist_reliable_data;
extern sys_dlist_t g_mac_tx_dlist_best_effort;
extern sys_dlist_t * const mac_tx_dlists[MAC_FLOW_COUNT];

/**
 * @brief Initializes the entire DECT NR+ MAC layer.
 *
 * This is the single entry point for an upper layer (like DLC) to initialize
 * the MAC. It orchestrates the initialization of all MAC sub-modules.
 *
 * @param rx_dlist_from_dlc Pointer to the dlist owned by the upper layer (DLC)
 *                          where the MAC will place received data packets.
 * @param status_cb         Pointer to a callback function in the upper layer (DLC)
 *                          that the MAC will use to report the final status of
 *                          transmissions that require end-to-end acknowledgement.
 * @return 0 on success, or a negative error code on failure.
 */
int dect_mac_init(sys_dlist_t *rx_dlist_from_dlc, dlc_tx_status_cb_t status_cb);

/**
 * @brief Starts the MAC layer's operation.
 *
 * This function starts the MAC state machine (e.g., scanning for a PT, or
 * beaconing for an FT). It must be called after a successful init.
 */
void dect_mac_start(void);

/**
 * @brief Allocates a buffer for a new MAC SDU (which will contain a DLC PDU).
 *
 * @param timeout The time to wait for a free buffer.
 * @return Pointer to a mac_sdu_t buffer, or NULL if none available.
 */
mac_sdu_t *dect_mac_buffer_alloc(k_timeout_t timeout);

/**
 * @brief Frees a MAC SDU buffer, returning it to the MAC's memory slab.
 *
 * @param sdu Pointer to the SDU buffer to free.
 */
void dect_mac_buffer_free(mac_sdu_t *sdu);

/**
 * @brief (PT ROLE) Sends a prepared MAC SDU to the associated FT.
 *
 * The MAC layer takes ownership of the buffer.
 *
 * @param sdu Pointer to the SDU buffer.
 * @param flow The Quality of Service flow to use.
 * @return 0 on success, or a negative error code.
 */
int dect_mac_send(mac_sdu_t *sdu, mac_flow_id_t flow);

/**
 * @brief (FT ROLE ONLY) Sends a prepared MAC SDU to a specific connected PT.
 *
 * The MAC layer takes ownership of the buffer.
 *
 * @param sdu Pointer to the SDU buffer.
 * @param flow The Quality of Service flow to use.
 * @param target_pt_short_rd_id The 16-bit Short RD ID of the target PT.
 * @return 0 on success, or a negative error code.
 */
int dect_mac_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id);

/**
 * @brief Registers a callback to be notified of MAC state changes.
 *
 * @param cb The callback function to register.
 */
void dect_mac_register_state_change_cb(dect_mac_state_change_cb_t cb);

/**
 * @brief Gets the device's own 32-bit Long RD ID.
 *
 * @return The Long RD ID.
 */
uint32_t dect_mac_get_own_long_id(void);

/**
 * @brief (PT ROLE) Gets the Long RD ID of the currently associated FT.
 *
 * @return The FT's Long RD ID, or 0 if not associated.
 */
uint32_t dect_mac_get_associated_ft_long_id(void);

/**
 * @brief (PT ROLE) Requests to release the current association with the FT.
 *
 * This function is non-blocking and queues a command to the MAC thread.
 * The MAC will attempt to send an Association Release PDU to the peer.
 * Regardless of TX success, the PT will transition to a non-associated state.
 *
 * @return 0 on success (command queued), or a negative error code.
 */
int dect_mac_release(void);


/**
 * @brief Gets the device's current operational role (PT or FT).
 *
 * @return The current dect_mac_role_t.
 */
dect_mac_role_t dect_mac_get_role(void);

/**
 * @brief (PT ROLE) Checks if the PT is currently associated with an FT.
 *
 * @return True if associated, false otherwise.
 */
bool dect_mac_is_associated(void);

/**
 * @brief Gets the short_rd_id for a known peer from its long_rd_id.
 *
 * @param long_rd_id The Long RD ID of the peer to find.
 * @return The corresponding 16-bit Short RD ID, or 0 if not found.
 */
uint16_t dect_mac_get_short_id_for_long_id(uint32_t long_rd_id);

/**
 * @brief Gets the device's current Hyper Packet Counter (HPC).
 *
 * @return The current HPC value.
 */
uint32_t dect_mac_get_hpc(void);

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
/**
 * @brief Builds the 16-byte Initialization Vector (IV) for AES-CTR encryption.
 *
 * @param iv_out 16-byte buffer to store the generated IV.
 * @param transmitter_long_rd_id Long RD ID of the transmitter.
 * @param receiver_long_rd_id Long RD ID of the receiver.
 * @param hpc The current 32-bit Hyper Packet Counter.
 * @param psn The current 12-bit Packet Sequence Number.
 */
void dect_mac_security_build_iv(uint8_t *iv_out, uint32_t transmitter_long_rd_id,
				uint32_t receiver_long_rd_id, uint32_t hpc, uint16_t psn);

/**
 * @brief Calculates the 5-byte Message Integrity Code (MIC) using AES-128-CMAC.
 *
 * @param pdu_data_for_mic Pointer to the data over which the MIC is calculated.
 * @param pdu_data_len Length of the data for MIC calculation.
 * @param integrity_key Pointer to the 16-byte session integrity key.
 * @param mic_out_5_bytes 5-byte buffer to store the truncated MIC.
 * @return 0 on success, or a negative error code.
 */
int dect_mac_security_calculate_mic(const uint8_t *pdu_data_for_mic, size_t pdu_data_len,
				    const uint8_t *integrity_key, uint8_t *mic_out_5_bytes);

/**
 * @brief Encrypts or decrypts a payload using AES-128 Counter (CTR) mode.
 *
 * @param payload_in_out Pointer to the payload to be encrypted/decrypted in-place.
 * @param len Length of the payload.
 * @param cipher_key Pointer to the 16-byte session cipher key.
 * @param iv Pointer to the 16-byte Initialization Vector.
 * @param encrypt True to encrypt, false to decrypt.
 * @return 0 on success, or a negative error code.
 */
int dect_mac_security_crypt_payload(uint8_t *payload_in_out, size_t len,
				    const uint8_t *cipher_key, uint8_t *iv, bool encrypt);
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */



/**
 * @brief Gets the device's current Hyper Packet Counter (HPC).
 *
 * @return The current HPC value.
 */
uint32_t dect_mac_get_hpc(void);


#if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
/**
 * @brief [TEST ONLY] Sets a spy callback for the dect_mac_send function.
 *
 * When a handler is set, all calls to dect_mac_send will be redirected to the
 * handler instead of the real implementation. This allows a test suite to
 * intercept and inspect data being sent to the MAC layer.
 *
 * @param handler The function pointer to the spy function, or NULL to restore the real implementation.
 */
void dect_mac_test_set_send_spy(int (*handler)(mac_sdu_t *sdu, mac_flow_id_t flow));
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API) */

// #if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
// /**
//  * @brief [TEST ONLY] Sets a spy callback for the dect_mac_init function.
//  *
//  * This allows a test to intercept the dlc_tx_status_cb_t that the DLC layer
//  * registers with the MAC, which is necessary for simulating MAC-level feedback
//  * in DLC ARQ tests.
//  *
//  * @param handler The function pointer to the spy function, or NULL to restore the real implementation.
//  */
// void dect_mac_test_set_init_spy(int (*handler)(sys_dlist_t *rx_dlist, dlc_tx_status_cb_t status_cb));
// #endif /* IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API) */

/**
 * @brief Services the MAC's periodic tasks, like checking TX queues.
 *
 * This function should be called periodically by the application thread.
 */
void dect_mac_service(void);

/**
 * @brief Initialize the MAC data plane API.
 *
 * This function must be called by the DLC layer once at startup. It provides
 * the MAC layer with the DLC's dlist for receiving MAC SDUs (DLC PDUs).
 *
 * @param rx_dlist_from_dlc A pointer to a sys_dlist_t initialized by the DLC.
 * @return 0 on success, negative error code on failure.
 */
int dect_mac_api_init(sys_dlist_t *rx_dlist_from_dlc);


/**
 * @brief Initialize the MAC data plane API.
 *
 * This function must be called by the DLC layer once at startup. It provides
 * the MAC layer with the DLC's dlist for receiving MAC SDUs (DLC PDUs).
 *
 * @param rx_dlist_from_dlc A pointer to a sys_dlist_t initialized by the DLC.
 * @return 0 on success, negative error code on failure.
 */
int dect_mac_api_init(sys_dlist_t *rx_dlist_from_dlc);

/**
 * @brief Allocate a buffer for a new MAC SDU (which will contain a DLC PDU).
 *
 * This function gets a free buffer from the MAC's internal memory slab. The DLC layer
 * should fill this buffer with a DLC PDU and then pass it to the appropriate send function.
 *
 * @param timeout The time to wait for a free buffer. Use K_NO_WAIT for non-blocking,
 *                K_FOREVER to wait indefinitely.
 * @return Pointer to a mac_sdu_t buffer, or NULL if no buffer is available within the timeout.
 */
mac_sdu_t* dect_mac_api_buffer_alloc(k_timeout_t timeout);

/**
 * @brief Free a MAC SDU buffer.
 *
 * This should be called by the DLC layer after it has finished processing a received
 * MAC SDU (DLC PDU) that it retrieved from its rx_fifo. This returns the buffer to the MAC's pool.
 * Also used by DLC to free buffers allocated for TX if sending fails before queueing to MAC,
 * or if an SDU is allocated but not used.
 *
 * @param sdu Pointer to the SDU buffer to free. Must have been allocated by `dect_mac_api_buffer_alloc`.
 */
// void dect_mac_api_buffer_free(mac_sdu_t *sdu);
void dect_mac_api_buffer_free_internal(mac_sdu_t *sdu, const char *caller_func);
// In your .h header file or at the top of the .c file
#define dect_mac_api_buffer_free(sdu) dect_mac_api_buffer_free_internal((sdu), __func__)
#define dect_mac_buffer_free(sdu) dect_mac_api_buffer_free_internal((sdu), __func__)
/**
 * @brief (PT ROLE or generic MAC internal) Send a prepared MAC SDU (containing a DLC PDU)
 *        to the MAC layer for transmission with a specific QoS flow.
 *
 * For a PT, the target is implicitly its associated FT.
 * The MAC layer takes ownership of the buffer. The caller (DLC/MAC internal) must not access it after
 * this call. The MAC will free the buffer back to the slab once transmission is complete
 * (or HARQ processing finishes/fails).
 *
 * @param sdu Pointer to the SDU buffer (obtained from dect_mac_api_buffer_alloc),
 *            where sdu->data contains the DLC PDU and sdu->len is its length.
 *            sdu->target_peer_short_rd_id can be 0 or the associated FT's ID for PT.
 * @param flow The Quality of Service flow to use for this SDU.
 * @return 0 on success.
 * @retval -ENOMEM If the specified MAC TX queue is full.
 * @retval -EINVAL For invalid parameters (NULL SDU, invalid flow, invalid SDU length).
 */
int dect_mac_api_send(mac_sdu_t *sdu, mac_flow_id_t flow);

/**
 * @brief (FT ROLE ONLY) Send a prepared MAC SDU (containing a DLC PDU) to a specific
 *        Portable Termination (PT) that is connected to this FT.
 *
 * The MAC layer takes ownership of the buffer.
 *
 * @param sdu Pointer to the SDU buffer (obtained from dect_mac_api_buffer_alloc).
 *            sdu->data contains the DLC PDU, sdu->len is its length.
 * @param flow The Quality of Service flow to use for this SDU.
 * @param target_pt_short_rd_id The 16-bit Short RD ID of the target PT.
 * @return 0 on success.
 * @retval -EPERM If called when MAC role is not FT.
 * @retval -EINVAL Invalid parameters (NULL SDU, invalid flow, invalid SDU length,
 *                   unknown or invalid target_pt_short_rd_id).
 * @retval -ENOMEM If the specified MAC TX queue for that PT/flow is full.
 * @retval -ENOTCONN If the target PT is not currently connected/known to the FT.
 */
int dect_mac_api_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id);


/**
 * @brief Processes a single event from the MAC's event queue, with a timeout.
 *
 * This function should be called by an external thread (application or test)
 * to drive the MAC state machine when CONFIG_DECT_MAC_OWN_THREAD is disabled.
 *
 * @param timeout The maximum time to wait for an event.
 * @return 0 if an event was processed, -EAGAIN if the timeout was reached.
 */
int dect_mac_process_event_timeout(k_timeout_t timeout);


#endif /* DECT_MAC_H__ */