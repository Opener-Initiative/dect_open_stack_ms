/* dect_mac/dect_mac_context.h */
#ifndef DECT_MAC_CONTEXT_H__
#define DECT_MAC_CONTEXT_H__

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include "dect_mac_sm.h"
// #include "../include/dect_cdd.h"

#ifndef MAX_MAC_SDU_BUFFERS_CONFIG
    #if IS_ENABLED(CONFIG_ZTEST) && IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)
    #define MAX_MAC_SDU_BUFFERS_CONFIG 64
    #else
    #define MAX_MAC_SDU_BUFFERS_CONFIG 32
    #endif
#endif

// For dect_mac_state_t, dect_mac_role_t, pending_op_type_t
// Note: dect_mac_api.h includes this for mac_sdu_t, creating a potential circular dependency
// if mac_sdu_t definition is *only* in dect_mac_api.h and dect_harq_tx_process_t needs it.
// To resolve, mac_sdu_t could be in a more basic types file, or use forward declaration if only pointers needed.
// For now, assuming dect_mac_api.h might have been included before by a top-level include,
// or we ensure mac_sdu_t is defined before dect_harq_tx_process_t.
// Let's move mac_sdu_t to a types file or ensure it's defined before its use in HARQ struct.
// For this output, I'll assume mac_sdu_t from "dect_mac_api.h" is somehow visible or we define it here.

// Forward declaration for mac_sdu_t if defined elsewhere and only pointer is needed
struct mac_sdu;

// --- Constants for array sizes and configurations ---
#ifndef MAX_HARQ_PROCESSES // Already in dect_mac_sm.h, ensure consistency
#define MAX_HARQ_PROCESSES 8
#endif
#define MAX_PEERS_PER_FT 8
#define MAX_MOBILITY_CANDIDATES 5
#define MAX_SUBSLOTS_IN_FRAME_NOMINAL 48 // For 10ms frame, 5 symbols/subslot, symbol dur ~41.67us => subslot ~208.3us
#define FRAME_DURATION_MS_NOMINAL 10
#define SUB_SLOTS_PER_ETSI_SLOT 24  // An ETSI slot is 24 subslots. A frame has 2 slots.

#define NOMINAL_SLOT_DURATION_MS 10 // Assuming 1 slot = 1 frame for some calculations
                                    // ETSI: 1 slot is 24 subslots = 120 symbols.
                                    // Duration = 120 * 41.66us = 5000us = 5ms if symbol duration is ~41.66us.
                                    // If frame is 10ms and has 2 slots, then this is consistent.
                                    // If NRF_MODEM_DECT_SYMBOL_DURATION = 2880 ticks,
                                    // 1 symbol = 2880 / 69120 kHz = 41.666... us
                                    // 1 subslot = 5 symbols = 208.333... us
                                    // 1 slot = 24 subslots = 5000 us = 5 ms.
                                    // 1 frame = 2 slots = 48 subslots = 10 ms.



#define SCAN_MEAS_DURATION_SLOTS_CONFIG 2 // e.g. 2 ETSI slots = 48 subslots = 10ms for background scan measurement
#define MAX_RACH_ATTEMPTS_CONFIG 5

#ifndef DEFAULT_DECT_CARRIER
#define DEFAULT_DECT_CARRIER 1924992 // kHz (Example: ETSI DECT EU Band: 1880-1900 MHz. This is outside.)
                                     // Let's use an ETSI example: GFSK carrier F_c = 1881.792 + k * 1.728 MHz
                                     // For k=0: 1881792 kHz. For k=1: 1883520 kHz
                                     // For US DECT 6.0: 1921.536 + k * 1.728 MHz. k=0..4
                                     // k=1 => 1923.264 MHz. k=2 => 1924.992 MHz (used as example)
#endif

#ifndef RSSI_HYSTERESIS_DB
#define RSSI_HYSTERESIS_DB 3
#endif

#ifndef DEFAULT_TX_POWER_CODE
#define DEFAULT_TX_POWER_CODE 0b0111 // Example: 0 dBm (ETSI Table 6.2.1-3a/3b code 0111 is 0dBm)
#endif

#ifndef MAX_MAC_PDU_SIZE_FOR_PCC_CALC // Should match CONFIG_DECT_MAC_PDU_MAX_SIZE - 1 (MAC Hdr Type octet)
#define MAX_MAC_PDU_SIZE_FOR_PCC_CALC 1637 // If CONFIG_DECT_MAC_PDU_MAX_SIZE is 1638 (true ETSI MAC PDU max)
#endif

#ifndef HARQ_ACK_TIMEOUT_MS
#define HARQ_ACK_TIMEOUT_MS 50
#endif

#ifndef MAX_HARQ_RETRIES
#define MAX_HARQ_RETRIES 3
#endif

#ifndef DEFAULT_DECT_CHANNEL
#define DEFAULT_DECT_CHANNEL 2 /* Example: ETSI DECT EU Ch 2 */
#endif


#if IS_ENABLED(CONFIG_ZTEST)
#define MAX_PENDING_PCC_TRANSACTIONS 16
#else
#define MAX_PENDING_PCC_TRANSACTIONS 4
#endif



/**
 * @brief Tracks a single, in-flight PCC transaction waiting for its PDC.
 */
typedef struct {
	bool is_valid;
	uint16_t transaction_id;
	struct nrf_modem_dect_phy_pcc_event pcc_data;
	uint64_t reception_time_us;
	struct k_timer timeout_timer; /* Timer to expire stale entries */
} pcc_transaction_t;


// --- Context-related Type Definitions ---

// Re-define mac_sdu_t here if not visible from dect_mac_api.h to break circular dependency for HARQ struct
// This definition must exactly match the one in dect_mac_api.h
#ifndef MAC_SDU_T_DEFINED_FOR_CONTEXT
#define MAC_SDU_T_DEFINED_FOR_CONTEXT
#endif
#ifndef CONFIG_DECT_MAC_SDU_MAX_SIZE
#define CONFIG_DECT_MAC_SDU_MAX_SIZE 1636
#endif



#ifndef HPC_RX_WINDOW_SIZE
#define HPC_RX_WINDOW_SIZE 64 // Example: Allow receiving HPCs up to 63 behind the newest known
#endif
#ifndef HPC_RX_FORWARD_WINDOW_MAX_ADVANCE
#define HPC_RX_FORWARD_WINDOW_MAX_ADVANCE 1024 // Max jump ahead for HPC to prevent huge skips
#endif
#ifndef MAX_MIC_FAILURES_BEFORE_HPC_RESYNC
#define MAX_MIC_FAILURES_BEFORE_HPC_RESYNC 3 // Example: after 3 MIC fails, request HPC resync
#endif

// ETSI TS 103 636-4, Table 6.4.3.3-1 Resource Allocation Bitmap related enums
typedef enum {
    RES_ALLOC_TYPE_RELEASE_ALL = 0b00,
    RES_ALLOC_TYPE_DOWNLINK    = 0b01,
    RES_ALLOC_TYPE_UPLINK      = 0b10,
    RES_ALLOC_TYPE_BIDIR       = 0b11,
} dect_alloc_type_t;

typedef enum {
    RES_ALLOC_REPEAT_SINGLE         = 0b000,
    RES_ALLOC_REPEAT_FRAMES         = 0b001,
    RES_ALLOC_REPEAT_SUBSLOTS       = 0b010,
    RES_ALLOC_REPEAT_FRAMES_GROUP   = 0b011,
    RES_ALLOC_REPEAT_SUBSLOTS_GROUP = 0b100,
} dect_repeat_type_t;

// ETSI TS 103 636-4, Table 6.4.2.3-1 (Cluster Beacon IE Fields)
typedef struct {
    uint8_t sfn;
    bool tx_power_present;
    bool power_constraints_active;
    bool frame_offset_present;
    bool next_channel_present;
    bool time_to_next_present;
    uint8_t network_beacon_period_code;
    uint8_t cluster_beacon_period_code;
    uint8_t count_to_trigger_code;
    uint8_t rel_quality_code;
    uint8_t min_quality_code;
    uint8_t clusters_max_tx_power_code; // Valid if tx_power_present
    bool    frame_offset_is_16bit;      // Helper: True if mu implies 16-bit FO field
    uint16_t frame_offset_value;        // Valid if frame_offset_present
    uint32_t next_cluster_channel_val;  // Valid if next_channel_present (as "Next Cluster Channel")
    uint32_t time_to_next_us;           // Valid if time_to_next_present
} dect_mac_cluster_beacon_ie_fields_t;

// ETSI TS 103 636-4, Table 6.4.3.4-1 (RACH Info IE Fields)
// This structure holds fields that are actually part of the beacon IE when serialized
typedef struct {
    bool repeat_type_is_subslots;
    bool sfn_validity_present;
    bool channel_field_present;
    bool channel2_field_present;
    bool max_len_type_is_slots;
    bool dect_delay_for_response;
    uint16_t start_subslot_index;    // 8 or 9 bits, based on mu
    bool length_type_is_slots;       // For the "Length" field that follows start_subslot
    uint8_t num_subslots_or_slots;   // 7 bits, actual count
    uint8_t max_rach_pdu_len_units;  // 7 bits, actual count (N-1 coded for PDU length in units)
    uint8_t cwmin_sig_code;          // 3 bits
    uint8_t cwmax_sig_code;          // 3 bits
    uint8_t repetition_code;         // 2 bits (ETSI: 00=1, 01=2, 10=4, 11=8 repetitions)
    uint8_t response_window_subslots_val_minus_1; // Value for 8-bit field (actual value+1)
    uint8_t sfn_value;               // If sfn_validity_present
    uint8_t validity_frames;         // If sfn_validity_present
	uint16_t channel_abs_num;        // If channel_field_present (13 bits)
	uint16_t channel2_abs_num;       // If channel2_field_present (13 bits)    
    uint8_t mu_value_for_ft_beacon;  // mu value of the FT advertising this RACH IE.
} dect_mac_rach_info_ie_fields_t;



// ETSI TS 103 636-4, Annex A.2. Details based on ETSI TS 103 636-3 Annex B.
typedef struct {
    // Octet 0 of the 5-octet set
    uint8_t dlc_service_type_support_code;  // 3 MSB: See Part 5, B.1.2 (e.g., 000=Type0, 001=Type1, ..., 101=Type0,1,2,3)
    uint8_t rx_for_tx_diversity_code;       // Next 3 bits: See Part 3, B.2 (TX Diversity Antennas: 0=1, 1=2, 2=4, 3=8)
    // 2 LSB Reserved

    // Octet 1 of the 5-octet set
    uint8_t mu_value;                       // 3 MSB: Subcarrier scaling factor μ (1-8, code 0-7 -> val mu=2^code)
    uint8_t beta_value;                     // Next 4 bits: Fourier transform scaling factor β (1-16, code 0-15 -> val beta=code+1)
    // 1 LSB Reserved

    // Octet 2 of the 5-octet set
    uint8_t max_nss_for_rx_code;            // 3 MSB: Max Spatial Streams (0=1, 1=2, 2=4, 3=8)
    uint8_t max_mcs_code;                   // Next 4 bits: Max MCS Index (0-11 for MCS0-MCS11)
    // 1 LSB Reserved

    // Octet 3 of the 5-octet set
    uint8_t harq_soft_buffer_size_code;     // 4 MSB: See Part 3, B.2 (codes for 16000 to 2048000 bytes)
    uint8_t num_harq_processes_code;        // Next 2 bits: (0=1, 1=2, 2=4, 3=8 processes)
    // 2 LSB Reserved

    // Octet 4 of the 5-octet set
    uint8_t harq_feedback_delay_code;       // 4 MSB: See Part 3, B.2 (codes for 0-6 subslots)
    bool supports_dect_delay;               // Bit 3: If DECT_Delay for RACH response is supported
    bool supports_half_duplex;              // Bit 2: If half-duplex operation (diff chan for RACH resp/DL sched) is supported
    // 2 LSB Reserved
} dect_mac_phy_capability_set_t;




#define MAX_FLOW_IDS_IN_ASSOC_REQ 6 // Max value for "Number of Flows" field coding for a list

#define MAX_PHY_CAPABILITY_VARIANTS_IN_IE 4 // Example: Allow up to 4 explicit 5-octet sets
                                            // num_phy_capabilities field (N-1) can be 0-7.
                                            // If N-1=7, then N=8 sets. Base + 7 explicit.
                                            // So array should be at least 7 if supporting max.
                                            // Let's use a smaller practical max for now.


/** @brief Information about a peer device. */
typedef struct {
    bool is_valid;                  // Is this peer slot/context entry currently in use and valid?
    bool is_secure;                 // Is the link with this peer currently secured?
    bool is_fully_identified;       // True if the Long RD ID of this peer is known.
    uint32_t long_rd_id;            // Peer's Long RD ID.
    uint16_t short_rd_id;           // Peer's Short RD ID.
    int16_t rssi_2;                 // Last measured RSSI-2 (Q7.1 format) from this peer.
    uint16_t operating_carrier;     // Primary operating carrier of this peer (if known, e.g., for an FT).
    uint32_t beacon_period_ms;      // Beacon period of this peer (if FT).

    // --- Mesh Routing Information from this Peer ---
    uint8_t route_cost;             // The route cost advertised by this peer. 255 is infinite/unknown.
    uint32_t sink_address_be;       // The Sink Long RD ID advertised by this peer.

    // --- HPC and Security Synchronization State with this Peer ---
    uint32_t hpc;                   // HPC value from the peer (from SecIE or inferred) used for the *current or last successfully processed PDU's IV* received from this peer.
    uint32_t highest_rx_peer_hpc;   // Highest HPC value ever validated and received *in a MAC Sec Info IE* from this peer.
    uint8_t peer_mu;                // Primary mu of the peer, from its RD Capability IE
    uint8_t peer_beta;              // Primary beta of the peer, from its RD Capability IE    
    bool peer_requested_hpc_resync; // True if this peer sent *us* a MAC Sec Info IE with SecIVType HPC_RESYNC_INITIATE.
    bool self_needs_to_request_hpc_from_peer; // True if *we* need to send RESYNC_INITIATE to *this peer*.
    uint8_t consecutive_mic_failures; // Count of consecutive MIC failures on PDUs received from this peer.
    uint8_t current_key_index_for_peer; // Key index the peer is using/we should use for TX to peer. (Context dependent)

    // --- Peer's PHY Parameters (learned from its RD Capability IE) ---
    uint8_t num_phy_variants;
    dect_mac_phy_capability_set_t phy_variants[MAX_PHY_CAPABILITY_VARIANTS_IN_IE];
    bool peer_phy_params_known;     // True if RD Capability IE has been successfully parsed for this peer.

    // --- Parameters Requested by PT in Association Request (stored by FT for this PT peer) ---
    bool pt_requested_harq_params_valid; // True if PT included HARQ params in its request
    uint8_t pt_req_harq_procs_tx;        // PT's requested number of TX HARQ processes (code)
    uint8_t pt_req_max_harq_retx_delay;  // PT's requested max re-TX delay (code)
    uint8_t pt_req_harq_procs_rx;        // PT's requested number of RX HARQ processes (code)
    uint8_t pt_req_max_harq_rerx_delay;  // PT's requested max re-RX delay (code)

    uint8_t pt_req_num_flows;            // Number of specific flows PT requested (0-6)
    uint8_t pt_req_flow_ids[MAX_FLOW_IDS_IN_ASSOC_REQ]; // Actual 6-bit flow IDs PT requested

    bool pt_is_ft_capable;              // From PT's AssocReq ft_mode_capable flag
    // TODO: Add fields for pt_req_ft_beacon_periods, pt_req_ft_next_channel etc. if FT needs to store them.

    // --- Dynamic Resource Allocation Tracking (for FT's view of PT) ---
    bool dl_data_pending_for_pt;      // True if FT has downlink data in its queues for this PT
    bool ul_resource_request_pending_from_pt; // True if PT has signaled need for UL resources
    uint32_t current_dl_throughput_bps; // Estimated or actual throughput to this PT (for advanced scheduler)
    uint16_t pt_requested_ul_duration_subslots; // If PT explicitly requested a duration
    uint16_t pt_granted_ul_duration_subslots;   // Currently granted UL duration by FT for this PT
    uint16_t pt_granted_dl_duration_subslots;   // Currently granted DL duration by FT for this PT
    bool paging_pending;                // True if FT needs to send a page to this PT
    uint8_t group_id;                   // Group ID assigned by FT (0-63)
    uint8_t resource_tag;               // Resource Tag assigned by FT (0-127)


    // --- Pending HARQ Feedback TO SEND to this peer (for PDUs *we* received from *them*) ---
    struct {
        bool valid;                 // True if this feedback slot is pending
        bool is_ack;                // True for ACK, false for NACK
        uint8_t harq_process_num_for_peer; // The HARQ process number *of the peer's transmission* that this feedback is for.
    } pending_feedback_to_send[2];  // Max 2 feedback items can be sent in one nRF PHY Type 2 PCC feedback field (using Format 3)
    uint8_t num_pending_feedback_items;

    // --- Authentication Handshake Context ---
    bool supports_secure_join; // True if this FT advertised a Joining Information IE
    uint32_t pt_nonce;         // Nonce generated by the PT
    uint32_t ft_nonce;         // Nonce generated by the FT

    struct k_timer link_supervision_timer; // Supervision timer for this peer link
    uint16_t last_rx_sdu_len;              // [DEBUG/TEST] Length of last successfully received SDU
} dect_mac_peer_info_t;



/** @brief Stores a parsed resource allocation schedule for a link. */
typedef struct {
    bool is_active;
    dect_alloc_type_t alloc_type; // For this entry (UL, DL, or if BIDIR, this struct might represent one direction)
    // For DL part (or if alloc_type is DL)
    uint16_t dl_start_subslot;
    uint8_t dl_duration_subslots; // Actual number of subslots
    bool dl_length_is_slots;
    // For UL part (or if alloc_type is UL)
    uint16_t ul_start_subslot;
    uint8_t ul_duration_subslots;
    bool ul_length_is_slots;

    dect_repeat_type_t repeat_type;
    uint8_t repetition_value;       // Actual value from IE (e.g., 1 means every frame/subslot)
    uint8_t validity_value;         // Actual value from IE (0xFF for permanent)
    uint32_t channel;
    uint64_t next_occurrence_modem_time;
    uint8_t sfn_of_initial_occurrence; // SFN from ResAlloc IE or SFN when non-SFN schedule activated
    uint64_t schedule_init_modem_time; // Modem time when schedule was parsed/activated
    // Flags to know if start_subslot fields are 8 or 9 bits (based on mu of the link)
    bool res1_is_9bit_subslot; // True if dl_start_subslot implies 9 bits
    bool res2_is_9bit_subslot; // True if ul_start_subslot implies 9 bits (for BIDIR context)
} dect_mac_schedule_t;

/** @brief Tracks a potential mobility handover candidate FT for a PT. */
typedef struct {
	bool is_valid;
	uint16_t short_rd_id;
	uint32_t long_rd_id;
	uint32_t operating_carrier;
	int16_t rssi_2;
	uint8_t trigger_count_remaining;
	dect_mac_rach_info_ie_fields_t rach_params_from_beacon;
	bool supports_secure_join;
} dect_mobility_candidate_t;

/** @brief Parameters for Random Access Channel (RACH) advertised by FT, stored by PT. */
typedef struct {
    dect_mac_rach_info_ie_fields_t advertised_beacon_ie_fields; // Parsed from FT's beacon
    // Derived/Operational values
    uint32_t rach_operating_channel; // Actual carrier to use for RACH TX
    uint16_t cw_min_val;             // Calculated 2^cwmin_sig_code
    uint16_t cw_max_val;             // Calculated 2^cwmax_sig_code
    uint32_t response_window_duration_us;
} dect_ft_rach_params_t; // Stored by PT for its target/associated FT

/** @brief Tracks the state of a single HARQ transmission process. */
typedef struct {
    bool is_active;
    bool needs_retransmission;
    uint8_t redundancy_version;
    uint8_t tx_attempts;
    mac_sdu_t *sdu; // Pointer to the MAC SDU (DLC PDU) being transmitted
    mac_flow_id_t flow_id;
    uint16_t original_psn;
    uint32_t original_hpc;
    struct k_timer retransmission_timer;
    uint16_t peer_short_id_for_ft_dl; // For FT: Target PT ShortID for this DL HARQ process
    uint16_t scheduled_carrier;       // For reTX: original carrier
    uint64_t scheduled_tx_start_time; // For reTX: original target start time
} dect_harq_tx_process_t;

/** @brief Stores PHY-specific latency values in microseconds. */
typedef struct {
    uint32_t scheduled_operation_startup_us;
    uint32_t scheduled_operation_transition_us;
    uint32_t idle_to_active_rx_us;
    uint32_t idle_to_active_tx_us;
    uint32_t active_to_idle_rx_us;
    uint32_t active_to_idle_tx_us;
    // Add other relevant latencies as needed
} dect_phy_latency_values_t;

/** @brief Stores own primary PHY operational parameters */
typedef struct {
    bool is_valid;      // True if these parameters have been set/determined
    uint8_t mu;         // Own operational mu (subcarrier scaling factor code, e.g., 0 for 1, 1 for 2, etc. as per RD Cap IE)
    uint8_t beta;       // Own operational beta (FFT scaling factor code, e.g., 0 for 1, 1 for 2, etc. as per RD Cap IE)
    // Other primary PHY params like default MCS could also go here if not in config
} dect_mac_own_phy_params_t;


/** @brief Stores MAC layer configuration parameters. */
typedef struct {
    int8_t rssi_threshold_min_dbm;  // RSSI-1 "free" threshold (ETSI RSSI_THRESHOLD_MIN)
    int8_t rssi_threshold_max_dbm;  // RSSI-1 "busy" threshold (ETSI RSSI_THRESHOLD_MAX)
    uint8_t rach_cw_min_idx;        // Code for Cwmin_sig (0-7) -> actual CW_MIN = 8 * 2^code
    uint8_t rach_cw_max_idx;        // Code for Cwmax_sig (0-7) -> actual CW_MAX = 8 * 2^code
    uint32_t rach_response_window_ms;
    uint32_t keep_alive_period_ms;
    uint32_t mobility_scan_interval_ms;
    uint32_t ft_cluster_beacon_period_ms;
    uint32_t ft_network_beacon_period_ms; // (Currently not sending separate Network Beacons)
    uint8_t max_assoc_retries;
    bool ft_policy_secure_on_assoc;
    uint8_t default_tx_power_code;  // For PCC transmit_power field (4 bits)
    uint8_t default_data_mcs_code;  // For PCC df_mcs field (3 or 4 bits depending on PCC type)
} dect_mac_config_params_t;

/** @brief Common timers and state for RACH procedure. */
typedef struct {
    struct k_timer rach_backoff_timer;
    struct k_timer rach_response_window_timer;
    uint8_t rach_cw_current_idx; // Current contention window code (0-7, maps to Cwmin_sig to Cwmax_sig)
} dect_mac_rach_context_t;

/** @brief Contains all state specific to the Portable Termination (PT) role. */
typedef struct {
    dect_mac_peer_info_t target_ft;
    dect_mac_peer_info_t associated_ft;
    
    dect_mobility_candidate_t mobility_candidates[MAX_MOBILITY_CANDIDATES];
    struct k_timer beacon_listen_timer;
    struct k_timer keep_alive_timer;
    struct k_timer mobility_scan_timer;
    struct k_timer paging_cycle_timer;
    struct k_timer auth_timeout_timer; /* Timeout for auth handshake (challenge/success) */
    dect_ft_rach_params_t current_ft_rach_params; // Parsed from associated/target FT's beacon
    uint8_t current_assoc_retries;
    dect_mac_schedule_t dl_schedule; // Schedule for downlink data from FT
    dect_mac_schedule_t ul_schedule; // Schedule for uplink data to FT
    dect_mac_schedule_t group_schedule; // Inactive schedule pattern received for group assignment
    uint8_t initial_count_to_trigger; // From current FT's beacon, used to reset candidate counters
    // struct k_fifo handover_tx_holding_fifo; // OLD FIFO logic
    sys_dlist_t handover_tx_holding_dlist;
	struct k_timer reject_timer;
	bool release_pending;    
} pt_context_t;


typedef struct { // Helper struct for FT's per-peer TX dlists
    sys_dlist_t high_priority_dlist;
    sys_dlist_t reliable_data_dlist;
    sys_dlist_t best_effort_dlist;
} dect_mac_peer_tx_dlist_set_t;


#ifdef CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN
#define DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN
#else
#define DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN 1
#endif

// --- Information Element Structures (Payloads AFTER MAC Mux Header) ---
typedef struct { // ETSI TS 103 636-4, Table 6.4.3.3-1 Resource Allocation IE fields
    // --- Bitmap fields ---
    dect_alloc_type_t alloc_type_val;
    bool add_allocation;
    bool id_present;
    dect_repeat_type_t repeat_val;
    bool sfn_present;
    bool channel_present;
    bool rlf_present; // dectScheduledResourceFailure timer code present

    // --- Resource 1 fields ---
    uint16_t start_subslot_val_res1; // Actual value to be serialized (8 or 9 bits based on flag)
    bool length_type_is_slots_res1;
    uint8_t length_val_res1;        // Value (0-127 representing 1-128 units)

    // --- Resource 2 fields (only if alloc_type_val == RES_ALLOC_TYPE_BIDIR) ---
    uint16_t start_subslot_val_res2;
    bool length_type_is_slots_res2;
    uint8_t length_val_res2;

    // --- Optional fields based on bitmap ---
    uint16_t short_rd_id_val;
    uint8_t repetition_value;       // Actual value (e.g. 1 means every frame/subslot)
    uint8_t validity_value;         // 0xFF for permanent
    uint8_t sfn_val;
    uint32_t channel_val;           // 13 MSB are channel, 3 LSB reserved (0)
    uint8_t dect_sched_res_fail_timer_code; // 4 MSB are code, 4 LSB reserved (0)

    // --- Helper flags for (de)serialization, set by caller/parser based on link's mu ---
    bool res1_is_9bit_subslot; // True if Start Subslot for Res1 should be 9 bits
    bool res2_is_9bit_subslot; // True if Start Subslot for Res2 should be 9 bits
} dect_mac_resource_alloc_ie_fields_t;



/* ETSI TS 103 636-4, Figure 6.4.3.9-1 */
#define MAX_RESOURCE_TAGS_PER_GROUP_IE 10 /* Example limit */
typedef struct {
	uint8_t single_direct_group_id; /* Single(1b)|Direct(1b)|GroupID(6b) */
	uint8_t resource_tags[MAX_RESOURCE_TAGS_PER_GROUP_IE]; /* Array of 7-bit tags */
} __attribute__((packed)) dect_mac_group_assignment_ie_t;


/**
 * @brief High-level representation of Group Assignment parameters for the FT state machine.
 */
typedef struct {
    bool is_single;
    bool is_direct;
    uint8_t group_id; // 0-63
    uint8_t tags[MAX_RESOURCE_TAGS_PER_GROUP_IE];
    uint8_t num_tags;
} dect_mac_group_assignment_fields_t; // Note the different name


/** @brief Contains all state specific to the Fixed Termination (FT) role. */
typedef struct {
    dect_mac_peer_info_t connected_pts[MAX_PEERS_PER_FT];
    dect_mac_schedule_t peer_schedules[MAX_PEERS_PER_FT];
    dect_mac_peer_tx_dlist_set_t peer_tx_data_dlists[MAX_PEERS_PER_FT];

    uint8_t sfn;
    uint8_t sfn_for_last_beacon_tx;
    uint32_t operating_carrier;
    struct k_timer beacon_timer;
    dect_ft_rach_params_t advertised_rach_params;
    uint16_t last_rach_pt_short_id;
    int16_t last_rach_rssi2;
    uint16_t last_assoc_resp_pt_short_id;

    bool keys_provisioned_for_peer[MAX_PEERS_PER_FT];
    uint8_t peer_integrity_keys[MAX_PEERS_PER_FT][16];
    uint8_t peer_cipher_keys[MAX_PEERS_PER_FT][16];

    // DCS (Dynamic Channel Selection) state
    uint8_t dcs_current_channel_scan_index;
    uint32_t dcs_candidate_channels[DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN];
    int16_t dcs_candidate_rssi_avg[DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN];
    uint8_t dcs_candidate_busy_percent[DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN];
    uint8_t dcs_num_valid_candidate_channels;
    bool dcs_scan_complete;

    /* Group Assignment state */
    /** @brief The high-level template for the group resource allocation. */
    dect_mac_schedule_t group_schedule;
    /** @brief The high-level parameters for building the Group Assignment IE. */
    dect_mac_group_assignment_fields_t group_assignment_fields;
    
    bool group_assignment_pending;
} ft_context_t;

#ifndef CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN
/* Provide a default value when FT role is not enabled, so this header can compile. */
#define CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN 1
#endif

/** @brief The single, global context structure for the entire MAC layer. */
typedef struct dect_mac_context {
	struct k_spinlock lock;       /* Global MAC state lock */
	struct k_spinlock harq_lock;  /* Dedicated lock for HARQ processes and queues */
    dect_mac_state_t state;
    dect_mac_role_t role;
    dect_mac_config_params_t config;

    // --- Mesh Routing State ---
    uint8_t own_route_cost;         // This device's calculated cost to the sink. 255 is infinite.
    uint32_t sink_long_rd_id;       // The Long RD ID of the current sink for this mesh segment.

    uint32_t own_long_rd_id;
    uint16_t own_short_rd_id;
    uint32_t network_id_32bit;

    dect_mac_own_phy_params_t own_phy_params; // <<-- NEW FIELD

    uint32_t pending_op_handle;
    pending_op_type_t pending_op_type;

    uint16_t current_rx_op_carrier; // Carrier of the currently active RX operation
    
    dect_mac_rach_context_t rach_context;

    dect_harq_tx_process_t harq_tx_processes[MAX_HARQ_PROCESSES];
    uint16_t psn;
    uint32_t hpc;

    dect_phy_latency_values_t phy_latency;

    uint8_t master_psk[16];
    bool master_psk_provisioned;

    uint8_t integrity_key[16]; // For PT: session key with FT. For FT: unused (per-peer keys).
    uint8_t cipher_key[16];    // For PT: session key with FT. For FT: unused.
    bool keys_provisioned;     // For PT: session keys derived. For FT: unused.
    uint8_t current_key_index; // Own current key index for TX.
    bool send_mac_sec_info_ie_on_next_tx; // Global flag, e.g. for PT to send its HPC if FT requested.

    uint64_t last_known_modem_time;
    uint64_t ft_sfn_zero_modem_time_anchor;
    uint8_t  current_sfn_at_anchor_update;
    uint64_t last_phy_op_end_time; /* Modem time when the last scheduled PHY op will end */    
    /* Cache for in-flight PCCs waiting for their corresponding PDCs. */
    pcc_transaction_t pcc_transaction_cache[MAX_PENDING_PCC_TRANSACTIONS];
    union {
        pt_context_t pt;
        ft_context_t ft;
    } role_ctx;

} dect_mac_context_t;

/**
 * @brief Stub function to get the current application sequence number from CDD.
 *
 * @return The current application sequence number.
 */
static inline uint8_t dect_cdd_get_app_seq_num(void)
{
	/* This is a stub. A real implementation would query the CDD module. */
	return 1;
}


#endif /* DECT_MAC_CONTEXT_H__ */