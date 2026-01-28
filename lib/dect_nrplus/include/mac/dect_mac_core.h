/* dect_mac/dect_mac_core.h */
#ifndef DECT_MAC_CORE_H__
#define DECT_MAC_CORE_H__

#include <stdint.h>
#include <mac/dect_mac_types.h>
#include <mac/dect_mac_sm.h>        // For dect_mac_role_t
#include <mac/dect_mac_context.h>   // For dect_mac_context_t definition

/**
 * @brief Retrieves a pointer to the currently active MAC context for this thread.
 *
 * @return Pointer to the active dect_mac_context_t structure.
 */
dect_mac_context_t *dect_mac_get_active_context(void);

/**
 * @brief Sets the active MAC context for the current thread. FOR TEST USE ONLY.
 *
 * @param ctx Pointer to the MAC context to make active.
 */
void dect_mac_test_set_active_context(dect_mac_context_t *ctx);

/**
 * @brief Initializes the core MAC context and common MAC parameters.
 *
 * This function must be called once at startup by the entity responsible for
 * initializing the DECT stack (e.g., the DLC layer or main application setup)
 * before any other MAC operations or starting the MAC processing thread.
 *
 * It performs the following critical initializations:
 *  - Zeroes out the global MAC context structure.
 *  - Sets the device's operational role (PT or FT).
 *  - Sets or derives (if `long_rd_id` is 0) the device's Long RD ID.
 *  - Generates a random Short RD ID.
 *  - Derives the Network ID.
 *  - Loads default MAC configuration parameters (e.g., RSSI thresholds, RACH CW values, timer periods).
 *  - Initializes placeholder PHY latency values (should be updated from actual PHY capabilities).
 *  - Initializes common timers used by both roles (e.g., RACH-related timers).
 *  - Initializes HARQ process structures and their associated timers (via dect_mac_data_path_init).
 *  - Performs role-specific basic timer initializations (e.g., PT's keep-alive, FT's beacon timer).
 *  - Sets the initial MAC state to MAC_STATE_IDLE.
 *  - Initializes sequence numbers (PSN, HPC) and security context flags.
 *  - Initializes per-peer TX FIFOs if role is FT.
 *
 * Note: This function initializes the context but does *not* trigger the start of
 *       actual MAC operations like scanning or beaconing. Those are initiated by
 *       calling the respective role-specific start functions (e.g.,
 *       dect_mac_sm_pt_start_operation() or dect_mac_sm_ft_start_operation())
 *       after this core initialization is complete.
 *
 * @param role The desired operational role for this MAC instance (MAC_ROLE_PT or MAC_ROLE_FT).
 * @param long_rd_id The 32-bit Long Radio Device ID for this device. If 0, the function
 *                   will attempt to derive one (e.g., using hw_id_get()).
 * @return 0 on success, negative error code on failure (e.g., if HW ID cannot be read for derivation).
 */
int dect_mac_core_init(dect_mac_role_t role, uint32_t long_rd_id);

/**
 * @brief Increments the global MAC Packet Sequence Number (PSN) and handles
 *        Hyper Packet Counter (HPC) wrap-around and synchronization.
 *
 * This function should be called by any module that is about to construct a new
 * MAC PDU for transmission *before* it populates the PSN field in the MAC Common
 * Header and *before* it builds the IV for security (as the IV depends on HPC and PSN).
 *
 * If the PSN wraps around to 0, the HPC is incremented. If the HPC increments,
 * the `send_mac_sec_info_ie_on_next_tx` flag in the MAC context is set to true,
 * signaling that the next secured PDU transmitted should include a MAC Security Info IE
 * to inform the peer of the HPC update.
 *
 * @param ctx Pointer to the global MAC context.
 */
void increment_psn_and_hpc(dect_mac_context_t *ctx);

/**
 * @brief Gets the short_rd_id for a known peer from its long_rd_id.
 *
 * @param long_rd_id The Long RD ID of the peer to find.
 * @return The corresponding 16-bit Short RD ID, or 0 if not found.
 */
uint16_t dect_mac_core_get_short_id_for_long_id(uint32_t long_rd_id);


/**
 * @brief Gets the internal peer slot index for a given peer Short RD ID.
 *        (Only applicable when device role is FT).
 *
 * @param peer_short_id The Short RD ID of the peer to find.
 * @return The peer's index (0 to MAX_PEERS_PER_FT - 1), or -1 if not found.
 */
int dect_mac_core_get_peer_slot_idx(uint16_t peer_short_id);


/**
 * @brief Clears the MAC's pending PHY operation state.
 *
 * This should be called by the state machine once it has fully handled an
 * OP_COMPLETE event and is ready for a new PHY operation to be scheduled.
 */
void dect_mac_core_clear_pending_op(void);


/**
 * @brief (Internal) Registers a callback to be notified of MAC state changes.
 *
 * @param cb The callback function to register.
 */
void dect_mac_core_register_state_change_cb(dect_mac_state_change_cb_t cb);


void print_first_bytes_uint32(uint32_t value);

void print_first_bytes_uint16(uint16_t value);


#endif /* DECT_MAC_CORE_H__ */