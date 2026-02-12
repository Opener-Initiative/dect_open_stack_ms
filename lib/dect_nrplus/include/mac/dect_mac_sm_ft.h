/* dect_mac/dect_mac_sm_ft.h */
#ifndef DECT_MAC_SM_FT_H__
#define DECT_MAC_SM_FT_H__

#include <mac/dect_mac_sm.h> // For dect_mac_event_msg structure
#include <mac/dect_mac_context.h>   // For dect_mac_context_t access and constants

/**
 * @brief Queries the FT state machine for the next available TX opportunity for a specific peer.
 *
 * The data path calls this function when it has data to send to a peer. The SM checks
 * its internal schedules and returns the parameters for the next available slot if one
 * is found in the near future.
 *
 * @param peer_slot_idx The index of the peer to check for a schedule.
 * @param out_start_time Pointer to store the calculated modem start time for the TX op.
 * @param out_carrier Pointer to store the carrier frequency for the TX op.
 * @param out_schedule Pointer to store a copy of the schedule that provided the opportunity.
 * @return true if a valid, actionable TX opportunity was found.
 * @return false if no schedule is active or the next opportunity is too far in the future.
 */
bool ft_get_next_tx_opportunity(int peer_slot_idx, uint64_t *out_start_time,
				uint16_t *out_carrier, dect_mac_schedule_t *out_schedule);


/**
 * @brief Initializes and starts the Fixed Termination (FT) role operations.
 *
 * This function should be called once by the main MAC control logic (e.g., in dect_mac_main.c)
 * after dect_mac_core_init() has been successfully executed and if the device's
 * role is determined to be MAC_ROLE_FT.
 *
 * It sets the initial FT state and typically triggers the first action for an FT,
 * which is to perform an initial channel scan (Dynamic Channel Selection - DCS)
 * to find a suitable operating carrier.
 */
void dect_mac_sm_ft_start_operation(void);

/**
 * @brief Handles events for the Fixed Termination (FT) role state machine.
 *
 * This function is the main entry point for event processing when the MAC layer
 * is operating as an FT. It is called by the central MAC event dispatcher
 * (`dect_mac_event_dispatch`) with events received from the PHY layer or
 * internal MAC timers.
 *
 * The function processes the event based on the FT's current operational state
 * (e.g., SCANNING, BEACONING, ASSOCIATED) and the type of event received,
 * driving the FT through its defined procedures like beaconing, handling association
 * requests, and managing connected PTs.
 *
 * @param msg Pointer to the const dect_mac_event_msg structure containing the
 *            event type and its associated data payload.
 */
void dect_mac_sm_ft_handle_event(const struct dect_mac_event_msg *msg);

/**
 * @brief FT specific action function to be called by the dispatcher when a beacon timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_BEACON
 * is dispatched. It checks if a beacon can be sent (e.g., no other critical PHY operation
 * is pending) and then calls an internal action function to build and schedule the beacon PDU.
 * It also reschedules the beacon timer.
 */
void dect_mac_sm_ft_beacon_timer_expired_action(void);

/**
 * @brief FT handles an incoming authentication-related PDU from a PT.
 *
 * @param pt_short_id Short RD ID of the PT sending the PDU.
 * @param pt_long_id Long RD ID of the PT sending the PDU.
 * @param pdu_data Pointer to the authentication PDU payload.
 * @param pdu_len Length of the payload.
 */
// void dect_mac_sm_ft_handle_auth_pdu(uint16_t pt_short_id, const uint8_t *pdu_data, size_t pdu_len);
void dect_mac_sm_ft_handle_auth_pdu(uint16_t pt_short_id, uint32_t pt_long_id, const uint8_t *pdu_data, size_t pdu_len);

#if IS_ENABLED(CONFIG_ZTEST)
int ft_find_and_init_peer_slot(uint32_t pt_long_id, uint16_t pt_short_id, int16_t rssi);
#endif

#endif /* DECT_MAC_SM_FT_H__ */