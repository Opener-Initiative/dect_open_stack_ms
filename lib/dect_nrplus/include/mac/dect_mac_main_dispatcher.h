/* dect_mac/dect_mac_main_dispatcher.h */
#ifndef DECT_MAC_MAIN_DISPATCHER_H__
#define DECT_MAC_MAIN_DISPATCHER_H__

#include <mac/dect_mac_sm.h>    // For dect_mac_event_msg, dect_mac_state_t, pending_op_type_t, dect_mac_event_type_t

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

/* The central message queue for all events processed by the MAC layer. */
extern struct k_msgq mac_event_msgq;

/**
 * @brief Changes the global MAC operational state and logs the transition.
 *
 * This function should be used by state machine modules to update the MAC's
 * current high-level state. It ensures state changes are logged consistently.
 *
 * @param new_state The new dect_mac_state_t to transition to.
 */
void dect_mac_change_state(dect_mac_state_t new_state);


/**
 * @brief Enters a fatal error state for the MAC layer.
 *
 * This function should be called when an unrecoverable error occurs.
 * It logs the error, attempts to clean up by deactivating the PHY,
 * and moves the state machine to MAC_STATE_ERROR, where it will no longer
 * process events.
 *
 * @param reason A string describing the reason for entering the error state.
 */
void dect_mac_enter_error_state(const char *reason);

/**
 * @brief The main event dispatcher for the MAC layer.
 *
 * This function is called by the dedicated MAC processing thread whenever an event
 * is received from its message queue (`mac_event_msgq`).
 * It performs initial common pre-processing or validation of PHY events
 * (e.g., checking Network ID on PCC, basic PDU length checks for PDC)
 * and then dispatches the event to the appropriate role-specific state machine handler
 * (`dect_mac_sm_pt_handle_event` or `dect_mac_sm_ft_handle_event`)
 * based on the MAC's current role stored in the global context.
 *
 * @param msg Pointer to the dect_mac_event_msg structure containing the event
 *            type and its associated data payload. The modem_time_of_event field
 *            in the message is used to update the MAC's last_known_modem_time.
 */
void dect_mac_event_dispatch(const struct dect_mac_event_msg *msg);

/**
 * @brief Converts a MAC state enum to its string representation for logging.
 * @param state The MAC state.
 * @return Pointer to a const string representing the state. Returns "S_UNKNOWN" if invalid.
 */
const char* dect_mac_state_to_str(dect_mac_state_t state);

/**
 * @brief Converts a MAC event type enum to its string representation for logging.
 * @param event_type The MAC event type.
 * @return Pointer to a const string representing the event type. Returns "E_UNKNOWN" if invalid.
 */
const char* dect_mac_event_to_str(dect_mac_event_type_t event_type);

/**
 * @brief Converts a pending MAC operation type enum to its string representation for logging.
 * @param op_type The pending operation type.
 * @return Pointer to a const string representing the pending operation type. Returns "P_UNKNOWN" if invalid.
 */
const char* dect_pending_op_to_str(pending_op_type_t op_type);

/**
 * @brief Converts an nRF Modem DECT PHY error enum to its string representation for logging.
 * @param err The PHY error code from enum nrf_modem_dect_phy_err.
 * @return Pointer to a const string representing the PHY error. Returns "PHY_ERR_0xVAL" if unknown.
 */
const char* nrf_modem_dect_phy_err_to_str(enum nrf_modem_dect_phy_err err);


#endif /* DECT_MAC_MAIN_DISPATCHER_H__ */