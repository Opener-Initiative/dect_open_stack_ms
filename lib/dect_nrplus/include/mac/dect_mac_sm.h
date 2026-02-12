/* dect_mac/dect_mac_sm.h */
#ifndef DECT_MAC_SM_H__
#define DECT_MAC_SM_H__

#include "nrf_modem_dect_phy.h"// For nrf_modem_dect_phy event structs and enums
#include <stdint.h>             // For uintxx_t types
#include <mac/dect_mac_types.h>


/* Forward declare the context struct to break the circular include dependency.
 * The full definition is not needed here, only a pointer to it.
 */
struct dect_mac_context;

// Forward declaration for MAX_HARQ_PROCESSES if not defined by an earlier include (like dect_mac_context.h)
// It's better if dect_mac_context.h (which defines it) is included before this by files needing both.
// However, to make this header somewhat self-contained for its enum:
#ifndef MAX_HARQ_PROCESSES
#define MAX_HARQ_PROCESSES 8
#endif



/** @brief Defines the high-level operational states of the MAC layer. */
typedef enum {
    MAC_STATE_DEACTIVATED,      // PHY not initialized or inactive
    MAC_STATE_IDLE,             // Core initialized, role set, deciding next action
    /* Portable Termination (PT) Specific States */
    MAC_STATE_PT_SCANNING,          // PT actively scanning for beacons
    MAC_STATE_PT_BEACON_PDC_WAIT,   // PT saw a beacon PCC, waiting for its PDC to get FT details
    MAC_STATE_PT_CANCELLING_SCAN,   // PT found a beacon and is waiting for its scan op to cancel
    MAC_STATE_PT_ASSOCIATING,       // PT building/sending Association Request (RACH op in progress, or waiting for LBT slot)
    MAC_STATE_PT_RACH_BACKOFF,      // PT in RACH contention window backoff
    MAC_STATE_PT_WAIT_AUTH_CHALLENGE, // PT sent Auth Initiate, waiting for Challenge
    MAC_STATE_PT_WAIT_AUTH_SUCCESS,   // PT sent Auth Response, waiting for Success/Fail
    MAC_STATE_PT_WAIT_ASSOC_RESP,   // PT sent Assoc Req, PHY op done, RXing/waiting for Assoc Response PDU
    MAC_STATE_PT_AUTHENTICATING,    // PT performing security handshake after basic association
    MAC_STATE_PT_HANDOVER_ASSOCIATING, // PT associating with a new FT while still connected to the old one
    MAC_STATE_PT_PAGING,            // PT in low-power paging cycle (listening for pages)
    MAC_STATE_PT_RELEASING,           // PT has sent Assoc Release and is waiting for TX to complete
    /* Fixed Termination (FT) Specific States */
    MAC_STATE_FT_WAIT_AUTH_INIT,    // FT is beaconing, ready to receive Auth Initiate
    MAC_STATE_FT_WAIT_AUTH_RESP,    // FT sent Challenge, waiting for Response
    MAC_STATE_FT_SCANNING,          // FT performing initial channel scan (DCS)
    MAC_STATE_FT_BEACONING,         // FT is operational: transmitting beacons, ready for associations & data
    MAC_STATE_FT_RACH_LISTEN,       // FT specifically listening on RACH resources (can be a sub-state of BEACONING)

    /* Common Connected State (after association & authentication for PT; after PT connects for FT) */
    MAC_STATE_ASSOCIATED,           // Secure and/or operational link established, ready for data
    
    MAC_STATE_ERROR,                // An unrecoverable error occurred, stack is halted
    MAC_STATE_COUNT,                // Total number of states, for validation/arrays
} dect_mac_state_t;


/**
 * @brief Defines the types of MAC operations that might be pending a PHY op completion.
 * This helps the NRF_MODEM_DECT_PHY_EVT_COMPLETED handler understand what MAC procedure
 * just finished at the PHY level.
 */
typedef enum {
    PENDING_OP_NONE = 0,                // No PHY operation actively tracked by MAC SM

    /* PT Operations */
    PENDING_OP_PT_SCAN,                 // PT's continuous beacon scan RX operation
    PENDING_OP_PT_RACH_ASSOC_REQ,       // PT's TX of Association Request on RACH
    PENDING_OP_PT_WAIT_BEACON_PDC,      // PT's specific RX for a beacon's PDC (not generally used if scan is continuous)
    PENDING_OP_PT_WAIT_ASSOC_RESP,      // PT's RX for Association Response PDU
    PENDING_OP_PT_WAIT_AUTH_CHALLENGE,  // PT's RX for Authorisation Challange Resounse PDU
    PENDING_OP_PT_WAIT_AUTH_SUCCESS,    // PT's RX for Authorisation Success Response PDU
    PENDING_OP_PT_KEEP_ALIVE,           // PT's TX of Keep Alive message
    PENDING_OP_PT_MOBILITY_SCAN,        // PT's RSSI scan for mobility candidates (TODO)
    PENDING_OP_PT_AUTH_MSG_TX,          // PT sending an authentication message (TODO)
    PENDING_OP_PT_AUTH_MSG_RX,          // PT expecting an authentication message (TODO)
    PENDING_OP_PT_PAGING_LISTEN,        // PT listening for Paging Messages
    PENDING_OP_PT_BEACON_LISTEN,        // PT listening for an FT's beacon
    PENDING_OP_PT_DATA_RX,              // PT listening for data from the FT

    /* FT Operations */
    PENDING_OP_FT_INITIAL_SCAN,         // FT's initial RSSI scan (for DCS)
    PENDING_OP_FT_BEACON,               // FT's TX of a Beacon PDU
    PENDING_OP_FT_RACH_RX_WINDOW,       // FT is listening on its RACH resources
    PENDING_OP_FT_GROUP_RX,             // FT is listening for a scheduled group UL transmission
    PENDING_OP_FT_ASSOC_RESP,           // FT's TX of Association Response PDU
    PENDING_OP_FT_AUTH_MSG_TX,          // FT sending an authentication message (TODO)
    PENDING_OP_FT_AUTH_MSG_RX,          // FT expecting an authentication message (TODO)
    PENDING_OP_FT_DATA_RX,              // FT sending data ???

    /* Generic Operations */
    PENDING_OP_GENERIC_UNICAST_TX,      // A simple, non-HARQ unicast TX (e.g., Assoc Release)
    
    /* Data Path Operations - HARQ process index encoded */
    // Base value for PT data TX HARQ operations
    PENDING_OP_PT_DATA_TX_HARQ0 = 30, // Ensure distinct from other ops
    PENDING_OP_PT_DATA_TX_HARQ_MAX = PENDING_OP_PT_DATA_TX_HARQ0 + MAX_HARQ_PROCESSES - 1,

    // Base value for FT data TX HARQ operations
    PENDING_OP_FT_DATA_TX_HARQ0 = PENDING_OP_PT_DATA_TX_HARQ_MAX + 1, // Start after PT HARQ range
    PENDING_OP_FT_DATA_TX_HARQ_MAX = PENDING_OP_FT_DATA_TX_HARQ0 + MAX_HARQ_PROCESSES - 1,

    PENDING_OP_MAX_VALUE // For validation or array sizing if needed
} pending_op_type_t;


/**
 * @brief Defines the types of events that can be passed to the MAC thread's message queue.
 * These events originate from PHY layer callbacks (via nrf_modem_dect_phy) or internal MAC timers.
 */
typedef enum {
    /* PHY Layer Events - Mapped from nrf_modem_dect_phy_event_id */
    MAC_EVENT_PHY_OP_COMPLETE,          // NRF_MODEM_DECT_PHY_EVT_COMPLETED
    MAC_EVENT_PHY_PCC,                  // NRF_MODEM_DECT_PHY_EVT_PCC (PCC header received)
    MAC_EVENT_PHY_PDC,                  // NRF_MODEM_DECT_PHY_EVT_PDC (PDC data received)
    MAC_EVENT_PHY_PCC_ERROR,            // NRF_MODEM_DECT_PHY_EVT_PCC_ERROR (PCC CRC fail)
    MAC_EVENT_PHY_PDC_ERROR,            // NRF_MODEM_DECT_PHY_EVT_PDC_ERROR (PDC CRC fail)
    MAC_EVENT_PHY_RSSI_RESULT,          // NRF_MODEM_DECT_PHY_EVT_RSSI (RSSI measurements ready)
    // Other PHY events like _INIT_COMPLETE, _ACTIVATE_COMPLETE could be added
    // but are currently handled in dect_mac_phy_if.c without queueing to MAC SM directly.

    /* Timer Expiry Events - Generated by MAC internal timers */
    MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF,       // PT RACH backoff period ended
    MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW,   // PT timeout waiting for Association Response
    MAC_EVENT_TIMER_EXPIRED_KEEPALIVE,          // PT periodic Keep Alive timer
    MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN,      // PT periodic mobility scan timer (TODO)
    MAC_EVENT_TIMER_EXPIRED_PAGING_CYCLE,       // PT paging listen interval timer (TODO)
    MAC_EVENT_TIMER_EXPIRED_BEACON_LISTEN,      // PT periodic beacon listen timer
    MAC_EVENT_TIMER_EXPIRED_BEACON,             // FT periodic Beacon transmission timer
    MAC_EVENT_TIMER_EXPIRED_HARQ,               // Data Path HARQ retransmission/ACK timeout timer
    MAC_EVENT_TIMER_EXPIRED_LINK_SUPERVISION,   // FT Peer Link Supervision timer expired
    MAC_EVENT_TIMER_EXPIRED_AUTH_TIMEOUT,        // PT auth handshake timeout (no response from FT)

    /* Internal Command Events (e.g., from Application/DLC to MAC control plane) */
    MAC_EVENT_CMD_ENTER_PAGING_MODE,    // PT command from App/DLC (TODO)
    // MAC_EVENT_CMD_START_ASSOCIATION, // PT command from App/DLC (alternative to auto-scan)
    MAC_EVENT_CMD_RELEASE_LINK,      // PT or FT command from App/DLC or Command to release the current association

    MAC_EVENT_TYPE_COUNT                // Number of defined event types
} dect_mac_event_type_t;


/**
 * @brief The message structure passed in the mac_event_msgq from PHY event callbacks
 *        (via dect_mac_phy_if.c) and from MAC internal timer handlers to the
 *        main MAC processing thread.
 */
struct dect_mac_event_msg {
    struct dect_mac_context *ctx;// Pointer to the MAC instance this event is for 
    dect_mac_event_type_t type;
    uint64_t modem_time_of_event; // Copied from nrf_modem_dect_phy_event.time for PHY events
                                  // For timer events, this might be k_uptime_get() or similar if needed.
    union {
        /* PHY Event Payloads (these are direct copies from nrf_modem_dect_phy_event.data portion) */
        struct nrf_modem_dect_phy_op_complete_event op_complete;
        struct nrf_modem_dect_phy_pcc_event pcc;
        struct nrf_modem_dect_phy_pdc_event pdc; // Note: pdc.data points to modem-owned memory
        struct nrf_modem_dect_phy_pcc_crc_failure_event pcc_crc_err;
        struct nrf_modem_dect_phy_pdc_crc_failure_event pdc_crc_err;
        struct nrf_modem_dect_phy_rssi_event rssi; // Note: rssi.meas points to modem-owned memory

        /* Timer Event Payloads */
        struct {
            int id; // e.g., HARQ process index for MAC_EVENT_TIMER_EXPIRED_HARQ,
                    // or a specific timer identifier if multiple timers of same type exist.
        } timer_data;

        /* Command Event Payloads (if any specific data needed for commands) */
        // struct { uint16_t target_ft_short_id; } cmd_start_assoc_payload;

    } data;
};

/** @brief The global message queue for all MAC layer events. */
extern struct k_msgq mac_event_msgq;


// /**
//  * @brief Callback function prototype for MAC state changes.
//  */
// typedef void (*dect_mac_state_change_cb_t)(dect_mac_state_t new_state);

/**
 * @brief Registers a callback to be notified of MAC state changes.
 *
 * @param cb The callback function to register.
 */
void dect_mac_sm_register_state_change_cb(dect_mac_state_change_cb_t cb);



#endif /* DECT_MAC_SM_H__ */