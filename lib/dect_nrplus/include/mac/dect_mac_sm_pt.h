/* dect_mac/dect_mac_sm_pt.h */
#ifndef DECT_MAC_SM_PT_H__
#define DECT_MAC_SM_PT_H__

#include "dect_mac_sm.h" // For dect_mac_event_msg structure
#include <mac/dect_mac_context.h>

/**
 * @brief Queries the PT state machine for the next available UL TX opportunity.
 *
 * The data path calls this function when it has data to send to the associated FT.
 * The SM checks its UL schedule and returns the parameters for the next available slot.
 *
 * @param out_start_time Pointer to store the calculated modem start time for the TX op.
 * @param out_carrier Pointer to store the carrier frequency for the TX op.
 * @param out_schedule Pointer to store a copy of the schedule that provided the opportunity.
 * @return true if a valid, actionable TX opportunity was found.
 * @return false if no UL schedule is active or the next opportunity is too far away.
 */
bool pt_get_next_tx_opportunity(uint64_t *out_start_time, uint16_t *out_carrier,
				dect_mac_schedule_t *out_schedule);

/**
 * @brief Initializes and starts the Portable Termination (PT) role operations.
 *
 * This function should be called once by the main MAC control logic (e.g., in dect_mac_main.c)
 * after dect_mac_core_init() has been successfully executed and if the device's
 * role is determined to be MAC_ROLE_PT.
 *
 * It sets the initial PT state and typically triggers the first action for a PT,
 * which is to start scanning for Fixed Terminations (FTs).
 */
void dect_mac_sm_pt_start_operation(void);

/**
 * @brief Handles events for the Portable Termination (PT) role state machine.
 *
 * This function is the main entry point for event processing when the MAC layer
 * is operating as a PT. It is called by the central MAC event dispatcher
 * (`dect_mac_event_dispatch`) with events received from the PHY layer or
 * internal MAC timers.
 *
 * The function processes the event based on the PT's current operational state
 * (e.g., SCANNING, ASSOCIATING, ASSOCIATED) and the type of event received,
 * driving the PT through its defined procedures like scanning for FTs, associating,
 * maintaining the link (keep-alives), and handling mobility.
 *
 * @param msg Pointer to the const dect_mac_event_msg structure containing the
 *            event type and its associated data payload.
 */
void dect_mac_sm_pt_handle_event(const struct dect_mac_event_msg *msg);

/**
 * @brief PT specific action function to be called by the dispatcher when the keep-alive timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_KEEPALIVE
 * is dispatched. It checks if a Keep Alive PDU can be sent and then calls an internal
 * action function to build and schedule it. It also reschedules the keep-alive timer.
 */
void dect_mac_sm_pt_keep_alive_timer_expired_action(void);

/**
 * @brief PT specific action function to be called by the dispatcher when the mobility scan timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN
 * is dispatched. It checks if a mobility scan should be initiated (e.g., if associated and no
 * other critical operation is pending) and then calls an internal action function to start the scan.
 * It also reschedules the mobility scan timer.
 */
void dect_mac_sm_pt_mobility_scan_timer_expired_action(void);

/**
 * @brief PT specific action function called by the dispatcher when the RACH backoff timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF
 * is dispatched. It typically re-attempts sending the RACH PDU (e.g., Association Request) after
 * the backoff period.
 */
void pt_rach_backoff_timer_expired_action(void);

/**
 * @brief PT specific action function called by the dispatcher when the RACH response window timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW
 * is dispatched. This means the PT did not receive an expected response (e.g., Association Response)
 * in time. It handles retry logic or initiates a rescan.
 */
void pt_rach_response_window_timer_expired_action(void);

/**
 * @brief PT specific action function called by the dispatcher when the beacon listen timer event occurs.
 *
 * This function is invoked from the MAC main thread context when a MAC_EVENT_TIMER_EXPIRED_BEACON_LISTEN
 * is dispatched. It checks if a beacon listen operation can be scheduled and then calls an internal
 * action function to start the RX operation at the correct time.
 */
void dect_mac_sm_pt_beacon_listen_timer_expired_action(void);


/**
 * @brief PT initiates the (stubbed) authentication protocol with the associated FT.
 *
 * Called after a successful association response if security is desired.
 * For now, this directly triggers local PSK-based key derivation.
 * A real implementation would start exchanging authentication PDUs.
 */
void dect_mac_sm_pt_initiate_authentication_protocol(void);

/**
 * @brief PT handles an incoming authentication-related PDU from the FT.
 *
 * @param pdu_data Pointer to the authentication PDU payload.
 * @param pdu_len Length of the payload.
 */
void dect_mac_sm_pt_handle_auth_pdu(const uint8_t *pdu_data, size_t pdu_len);


 void pt_send_association_release_action(dect_mac_peer_info_t *old_ft_info);


#endif /* DECT_MAC_SM_PT_H__ */