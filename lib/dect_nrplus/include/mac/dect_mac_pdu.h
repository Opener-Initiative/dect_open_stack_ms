/* dect_mac/dect_mac_pdu.h */
#ifndef DECT_MAC_PDU_H__
#define DECT_MAC_PDU_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> // For size_t

// Include context types if they define structures used here (like IE field structs)
// This will pull in dect_mac_cluster_beacon_ie_fields_t, etc.
// Assumes dect_mac_context.h includes or defines dect_mac_context_types.h which has these.
// If not, they need to be defined here or in a specific dect_mac_pdu_types.h.
// For now, assume they become visible via dect_mac_context.h or a similar include path.
#include "dect_mac_context.h" // This should bring in the IE field structures


// --- MAC Header Type (First Octet of MAC PDU - ETSI TS 103 636-4, 6.3.2) ---
typedef enum {
    MAC_SECURITY_NONE           = 0b00,
    MAC_SECURITY_USED_NO_IE     = 0b01,
    MAC_SECURITY_USED_WITH_IE   = 0b10,
    MAC_SECURITY_RESERVED       = 0b11,
} dect_mac_security_mode_t;

typedef enum { // ETSI Table 6.3.2-2
    MAC_COMMON_HEADER_TYPE_DATA_PDU     = 0b0000,
    MAC_COMMON_HEADER_TYPE_BEACON       = 0b0001,
    MAC_COMMON_HEADER_TYPE_UNICAST      = 0b0010,
    MAC_COMMON_HEADER_TYPE_RD_BROADCAST = 0b0011,
    MAC_COMMON_HEADER_TYPE_JOINING_BEACON = 0b0110,
    // 0b0100-0101, 0b0111-1110 are Reserved
    MAC_COMMON_HEADER_TYPE_ESCAPE       = 0b1111,
} dect_mac_common_header_type_val_t;

typedef enum { // ETSI TS 103 636-4, Table 6.4.3.5-1
	DECT_MAC_OP_MODE_PT_ONLY = 0b00,
	DECT_MAC_OP_MODE_FT_ONLY = 0b01,
	DECT_MAC_OP_MODE_BOTH = 0b10,
	DECT_MAC_OP_MODE_RESERVED = 0b11,
} dect_mac_operating_modes_code_t;

typedef enum { // ETSI TS 103 636-4, Table 6.4.3.5-1
	DECT_MAC_SECURITY_SUPPORT_NONE = 0b00,
	DECT_MAC_SECURITY_SUPPORT_MODE1 = 0b01,
	/* 0b10 and 0b11 are reserved */
} dect_mac_security_support_code_t;



typedef struct {
    dect_mac_common_header_type_val_t mac_header_type : 4;
    dect_mac_security_mode_t mac_security         : 2;
    uint8_t version                       : 2; // e.g., 00 for Release 2 of ETSI Part 4
} __attribute__((packed)) dect_mac_header_type_octet_t;


// --- MAC Common Headers (Follows MAC Header Type Octet - ETSI 6.3.3) ---
typedef struct { // ETSI 6.3.3.1 - DATA MAC PDU header
    uint8_t sequence_num_high : 4; // Bits 7-4 of first octet (PSN bits 11-8)
    bool reset_bit            : 1; // Bit 3
    uint8_t reserved          : 3; // Bits 2-0 (Set to 0)
    uint8_t sequence_num_low;      // Second octet (PSN bits 7-0)
} __attribute__((packed)) dect_mac_data_pdu_header_t;

typedef struct { // ETSI 6.3.3.2 - Beacon Header
    uint8_t network_id_ms24[3];          // Most significant 24 bits of Network ID (Big Endian on air)
    uint32_t transmitter_long_rd_id_be;  // Big Endian on air
} __attribute__((packed)) dect_mac_beacon_header_t;

typedef struct { // ETSI 6.3.3.3 - Unicast Header
    // First octet (PSN high bits + reset + reserved)
    uint8_t sequence_num_high_reset_rsv; // Combined for packing: SN_H(4b)|Reset(1b)|Rsv(3b)
    uint8_t sequence_num_low;            // PSN low 8 bits
    uint32_t receiver_long_rd_id_be;     // Big Endian on air
    uint32_t transmitter_long_rd_id_be;  // Big Endian on air
} __attribute__((packed)) dect_mac_unicast_header_t;

// Helper macro to set sequence_num_high_reset_rsv for Unicast/Data PDU headers
#define SET_SEQ_NUM_HIGH_RESET_RSV(psn_high_4b, reset_flag) \
    (((psn_high_4b) & 0x0F) << 4 | ((reset_flag) ? (1 << 3) : 0))

typedef struct {
	uint8_t network_id_ms24[3];
	/* Joining Information IE(s) follow in the SDU area */
} __attribute__((packed)) dect_mac_joining_beacon_header_t;

/* ETSI TS 103 636-4, Figure 6.4.3.2-1 */
typedef struct {
	uint32_t sink_address_be;
	uint8_t route_cost;
	uint8_t app_sequence_number;
} __attribute__((packed)) dect_mac_route_info_ie_t;

// /* ETSI TS 103 636-4, Figure 6.4.3.9-1 */
// #define MAX_RESOURCE_TAGS_PER_GROUP_IE 10 /* Example limit */
// typedef struct {
// 	uint8_t single_direct_group_id; /* Single(1b)|Direct(1b)|GroupID(6b) */
// 	uint8_t resource_tags[MAX_RESOURCE_TAGS_PER_GROUP_IE]; /* Array of 7-bit tags */
// } __attribute__((packed)) dect_mac_group_assignment_ie_t;


typedef struct {
	uint8_t flags; /* MaxAssoc(1b)|RD_PT_load(1b)|RACH_load(1b)|Channel_Load(1b)|Rsv(4b) */
	uint16_t max_associated_rds; /* 8 or 16 bits, based on flag */
	uint8_t currently_associated_rds_in_ft_mode_percentage;
	uint8_t currently_associated_rds_in_pt_mode_percentage;
	uint8_t rach_load_in_percentage;
	uint8_t percentage_of_subslots_detected_free;
	uint8_t percentage_of_subslots_detected_busy;
} __attribute__((packed)) dect_mac_load_info_ie_t;

/* ETSI TS 103 636-4, Figure 6.4.3.12-1 */
typedef struct {
	uint8_t flags; /* SNR(1b)|RSSI-2(1b)|RSSI-1(1b)|TX_count(1b)|RACH(1b)|Rsv(3b) */
	uint8_t snr_result;
	uint8_t rssi2_result;
	uint8_t rssi1_result;
	uint8_t tx_count_result;
} __attribute__((packed)) dect_mac_measurement_report_ie_t;

/* ETSI TS 103 636-4, Figure 6.4.3.6-1 */
typedef struct {
	uint8_t flags; /* RSV(2b)|LongID(1b)|ShortID(1b)|RSV(4b) */
	uint16_t short_rd_id;
	uint32_t long_rd_id;
} __attribute__((packed)) dect_mac_identity_update_ie_t;

/* ETSI TS 103 636-4, Figure 6.4.3.2-1 */
typedef struct {
	uint8_t flags; /* NetID(1b)|RSV(7b) */
	uint32_t network_id; /* 24 bits used if present */
} __attribute__((packed)) dect_mac_system_info_ie_t;



/* ETSI TS 103 636-4, Figure 6.4.3.17-1 */
#define MAX_JOINING_EPS 4
typedef struct {
	uint8_t num_eps; /* Number of EP fields included */
	uint16_t ep_values[MAX_JOINING_EPS];
} __attribute__((packed)) dect_mac_joining_info_ie_t;

/* --- Authentication Handshake IE Payloads --- */
typedef struct {
	uint32_t pt_nonce_be;
} __attribute__((packed)) dect_mac_auth_initiate_ie_t;

typedef struct {
	uint32_t pt_nonce_be;
	uint32_t ft_nonce_be;
} __attribute__((packed)) dect_mac_auth_challenge_ie_t;


#define DECT_MAC_AUTH_MAC_SIZE 8 /* Using a truncated 8-byte MAC for auth handshake */
typedef struct {
	uint8_t pt_mac[DECT_MAC_AUTH_MAC_SIZE];
} __attribute__((packed)) dect_mac_auth_response_ie_t;

typedef struct {
	uint8_t ft_mac[DECT_MAC_AUTH_MAC_SIZE];
} __attribute__((packed)) dect_mac_auth_success_ie_t;


#define DECT_MAC_COMMON_HEADER_MIN_SIZE sizeof(dect_mac_data_pdu_header_t) // Smallest common header (DATA PDU)

// --- Information Element Types (MAC SDU Area Content - ETSI 6.3.4 & Table 6.3.4-2/3) ---
// (These defines are for the 6-bit IE Type field used with MAC_Ext 00, 01, 10)
#define IE_TYPE_PADDING                     0b000000
#define IE_TYPE_HIGHER_LAYER_SIG_FLOW_1     0b000001 // Example, to be mapped to actual DLC/CVG flows
#define IE_TYPE_HIGHER_LAYER_SIG_FLOW_2     0b000010
#define IE_TYPE_USER_DATA_FLOW_1            0b000011 // Example for User Data (DLC PDU)
#define IE_TYPE_USER_DATA_FLOW_2            0b000100
#define IE_TYPE_USER_DATA_FLOW_3            0b000101
#define IE_TYPE_USER_DATA_FLOW_4            0b000110
#define IE_TYPE_NETWORK_BEACON              0b001000 // Note: This is a MAC Message, not just an IE in SDU area. Beacon PDU has specific common header.
#define IE_TYPE_CLUSTER_BEACON              0b001001 // This IE is part of Beacon PDU's SDU Area
#define IE_TYPE_ASSOC_REQ                   0b001010
#define IE_TYPE_ASSOC_RESP                  0b001011
#define IE_TYPE_ASSOC_RELEASE               0b001100
#define IE_TYPE_RECONFIG_REQ                0b001101
#define IE_TYPE_RECONFIG_RESP               0b001110
#define IE_TYPE_ADDITIONAL_MAC_MSG          0b001111
#define IE_TYPE_MAC_SECURITY_INFO           0b010000
#define IE_TYPE_ROUTE_INFO                  0b010001
#define IE_TYPE_RES_ALLOC                   0b010010 // Resource Allocation IE
#define IE_TYPE_RACH_INFO                   0b010011 // Random Access Resource IE
#define IE_TYPE_RD_CAPABILITY               0b010100 // Radio Device Capability IE
#define IE_TYPE_NEIGHBOURING_INFO           0b010101
#define IE_TYPE_BROADCAST_IND               0b010110
#define IE_TYPE_GROUP_ASSIGNMENT            0b010111
#define IE_TYPE_LOAD_INFO                   0b011000
#define IE_TYPE_MEASUREMENT_REPORT          0b011001
#define IE_TYPE_SOURCE_ROUTING              0b011010
#define IE_TYPE_JOINING_BEACON_MSG          0b011011 // This is a MAC Message type, not just an IE in SDU area
#define IE_TYPE_JOINING_INFORMATION         0b011100
#define IE_TYPE_AUTH_INITIATE               0b011101
#define IE_TYPE_AUTH_CHALLENGE              0b011110
#define IE_TYPE_AUTH_RESPONSE               0b011111
#define IE_TYPE_AUTH_SUCCESS                0b100000
#define IE_TYPE_SYSTEM_INFO                 0b100001
#define IE_TYPE_IDENTITY_UPDATE             0b100010
#define IE_TYPE_ESCAPE_TO_PROPRIETARY       0b111110 // 6-bit
#define IE_TYPE_EXTENSION                   0b111111 // 6-bit, indicates 1-byte extension for IE type

// Short IE Types (for MAC_Ext = 11, IE Type is 5 bits)
#define IE_TYPE_SHORT_PADDING               0b00000
#define IE_TYPE_SHORT_CONFIG_REQ            0b00001 // Payload 0 byte
#define IE_TYPE_SHORT_KEEP_ALIVE            0b00010 // Payload 0 byte
#define IE_TYPE_SHORT_MAC_SEC_INFO_HPC_REQ  0b10000 // Payload 0 byte

#define IE_TYPE_SHORT_RD_STATUS             0b00001 // Payload 1 byte
#define IE_TYPE_SHORT_RD_CAP_SHORT          0b00010 // Payload 1 byte
#define IE_TYPE_SHORT_ASSOC_CTRL            0b00011 // Payload 1 byte

typedef enum { // ETSI TS 103 636-4, Table 6.4.2.5-2 Association Reject Cause
    ASSOC_REJECT_CAUSE_NO_RADIO_CAP   = 0,
    ASSOC_REJECT_CAUSE_NO_HW_CAP      = 1,
    ASSOC_REJECT_CAUSE_CONFLICT_SHORTID = 2,
    ASSOC_REJECT_CAUSE_NON_SECURED_NOT_ACCEPTED = 3,
    ASSOC_REJECT_CAUSE_OTHER          = 4,
    // 5-7 are reserved
} dect_assoc_reject_cause_t;

typedef struct { // ETSI TS 103 636-4, Table 6.4.2.5-1 Association Response IE fields
    // --- Octet 0 ---
    bool ack_nack;                  // Bit 7: 1 for ACK (accepted), 0 for NACK (rejected).
    bool harq_mod_present;          // Bit 6: 1 if HARQ parameters (Octets 2-3) are present and different from request.
    uint8_t number_of_flows_accepted; // Bits 5-3: 0-6 for N flow_id fields; 7 means "all requested flows accepted".
    bool group_assignment_active;   // Bit 2: 1 if Group ID and Resource Tag (Octets N+M+1, N+M+2) are present.
    uint8_t reserved_3bits;         // Bits 1-0: Reserved, set to 0. (Note: ETSI table shows 3 bits for this in diagram, 2 in text)
                                    // For simplicity, we'll assume bits 1-0 are reserved as per text.

    // --- Conditional: Reject Cause & Timer (Octet 1, if ack_nack = 0 (NACK)) ---
    dect_assoc_reject_cause_t reject_cause; // 4 bits (Octet 1, bits 3-0)
    uint8_t reject_timer_code;              // 4 bits (Octet 1, bits 7-4)

    // --- Conditional: HARQ Parameters (Octets 2 & 3, if ack_nack = 1 AND harq_mod_present = 1) ---
    // These are the FT's *actual* HARQ parameters if they differ from what PT requested.
    uint8_t harq_processes_tx_val_ft;       // 3 bits (Octet 2, bits 7-5): FT's num HARQ TX processes for this PT.
    uint8_t max_harq_re_tx_delay_code_ft;   // 5 bits (Octet 2, bits 4-0): FT's max HARQ re-TX delay.
    uint8_t harq_processes_rx_val_ft;       // 3 bits (Octet 3, bits 7-5): FT's num HARQ RX processes for this PT.
    uint8_t max_harq_re_rx_delay_code_ft;   // 5 bits (Octet 3, bits 4-0): FT's max HARQ re-RX delay.

    // --- Conditional: Accepted Flow IDs (Variable length, if ack_nack = 1 AND number_of_flows_accepted is 1-6) ---
    // The number_of_flows_accepted field indicates how many entries in this array are valid.
    uint8_t accepted_flow_ids[MAX_FLOW_IDS_IN_ASSOC_REQ]; // Each entry is a 6-bit flow ID.

    // --- Conditional: Group ID & Resource Tag (2 octets, if ack_nack = 1 AND group_assignment_active = 1) ---
    uint8_t group_id_val;                   // 7 MSBs of first octet
    uint8_t resource_tag_val;               // 7 MSBs of second octet (1 LSB of each octet reserved)
                                            // Simplified: store as raw uint8_t, serializer handles reserved bits.
                                            // Or define more precise bitfields if needed.
                                            // ETSI Figure 6.4.3.9-1 Group Assignment IE shows GroupID(7)+Rsv(1), ResTag(7)+Rsv(1)
                                            // But Table 6.4.2.5-1 for AssocResp only says "Group ID" and "Resource Tag".
                                            // Let's assume they are raw 7-bit values that the (de)serializer packs correctly.
                                            // For simplicity, we'll use 7 bits for each value.
} dect_mac_assoc_resp_ie_t;


typedef enum {
	ASSOC_RELEASE_CAUSE_CONN_TERMINATION = 0,
	ASSOC_RELEASE_CAUSE_MOBILITY = 1,
	/* Other causes from ETSI Table 6.4.2.6-1 can be added here */
} dect_assoc_release_cause_t;

typedef struct {
	dect_assoc_release_cause_t cause;
} dect_mac_assoc_release_ie_t;


typedef struct { // ETSI TS 103 636-4, Table 6.4.3.5-1 RD Capability IE fields
    // --- Octet 0 ---
    uint8_t num_phy_capabilities;   // 3 MSB: (N-1) value. If 0, means one base set (no explicit 5-octet sets).
                                    // If 1, means one explicit 5-octet set follows.
    uint8_t release_version;        // 5 LSB: e.g., 1 for "Release 2" of DECT NR+ standard.

    // --- Octet 1 ---
    bool supports_group_assignment; // Bit 7
    bool supports_paging;           // Bit 6
    uint8_t operating_modes_code;   // Bits 5-4 (00=PT, 01=FT, 10=Both, 11=Rsvd)
    bool supports_mesh;             // Bit 3
    bool supports_sched_data;       // Bit 2
    uint8_t mac_security_modes_code;// Bits 1-0 (00=None, 01=Mode1)

    // Conditional: PHY Capability Sets (Octets 2 to (1 + (num_phy_capabilities_field_value) * 5))
    // num_phy_capabilities field stores (Actual_Num_Explicit_Sets - 1) if we follow N-1 for explicit sets.
    // Or, if num_phy_capabilities field stores X, there are X explicit sets.
    // ETSI: "Num PHY Capabilities (N-1)". If field is X, there are X explicit 5-octet sets.
    // So, if field is 0, 0 explicit sets. If 1, 1 explicit set. Max field value 7 -> 7 explicit sets.
    dect_mac_phy_capability_set_t phy_variants[MAX_PHY_CAPABILITY_VARIANTS_IN_IE];
    uint8_t actual_num_phy_variants_parsed; // Helper: How many were actually parsed/populated
} dect_mac_rd_capability_ie_t;


// #define MAX_FLOW_IDS_IN_ASSOC_REQ 6 // Max value for "Number of Flows" field coding for a list

typedef enum { // ETSI TS 103 636-4, Table 6.4.2.4-2 Association Setup Cause
    ASSOC_CAUSE_INITIAL_ASSOCIATION = 0,
    ASSOC_CAUSE_REQUEST_NEW_FLOWS   = 1,
    ASSOC_CAUSE_MOBILITY            = 2,
    ASSOC_CAUSE_REASSOC_AFTER_ERROR = 3,
    ASSOC_CAUSE_CHANGE_OWN_OP_CH    = 4,
    ASSOC_CAUSE_CHANGE_OP_MODE      = 5,
    ASSOC_CAUSE_PAGING_RESPONSE     = 6,
    ASSOC_CAUSE_ALL_PREV_CONFIGURED = 7, // Special meaning for Number of Flows = 7
    // ASSOC_CAUSE_RESERVED         = 7 // If not using "all previously configured" interpretation
} dect_assoc_setup_cause_t;

typedef struct { // ETSI TS 103 636-4, Table 6.4.2.4-1 Association Request IE fields
    // --- Octet 0 ---
    bool power_const_active;        // Bit 7: If PT has power constraints for this association.
    bool ft_mode_capable;           // Bit 6: If PT can also operate as an FT.
    uint8_t number_of_flows_val;    // Bits 5-3: 0-6 for N flow_id fields; 7 means "all previously configured".
    dect_assoc_setup_cause_t setup_cause_val; // Bits 2-0: Cause of association.

    // --- Octet 1 & 2: HARQ Parameters (Mandatory according to structure, not conditional flag in Octet 0) ---
    // Serializer will include these if harq_params_present (application helper flag) is true.
    // Parser will try to read these if enough bytes are present after Octet 0.
    bool harq_params_present;               // Application helper: true to include Octets 1 & 2.
    uint8_t harq_processes_tx_val;          // 3 bits (Octet 1, bits 7-5): Number of HARQ TX processes requested.
    uint8_t max_harq_re_tx_delay_code;      // 5 bits (Octet 1, bits 4-0): Max HARQ re-TX delay code.
    uint8_t harq_processes_rx_val;          // 3 bits (Octet 2, bits 7-5): Number of HARQ RX processes requested.
    uint8_t max_harq_re_rx_delay_code;      // 5 bits (Octet 2, bits 4-0): Max HARQ re-RX delay code.

    // --- Conditional: Flow IDs (Present if number_of_flows_val is 1-6) ---
    // Array to hold the actual 6-bit flow IDs.
    // The number_of_flows_val indicates how many entries in this array are valid.
    uint8_t flow_ids[MAX_FLOW_IDS_IN_ASSOC_REQ];

    // --- Conditional: FT Mode Parameters (Present if ft_mode_capable is true) ---
    // These fields effectively mirror parts of a Cluster Beacon IE, allowing the PT (acting as FT)
    // to signal its preferred operational parameters if it were to become an FT.

    // FT Beacon Periods (1 octet, if either period is to be signaled)
    bool ft_beacon_periods_octet_present;   // Helper: true if the beacon periods octet should be included.
    uint8_t ft_network_beacon_period_code;  // 4 bits (if present in octet)
    uint8_t ft_cluster_beacon_period_code;  // 4 bits (if present in octet)

    // FT Parameter Presence Flags (1 octet, if any of NextChan/TimeToNext/CurrentChan are signaled)
    bool ft_param_flags_octet_present;      // Helper: true if this flags octet should be included.
    bool ft_next_channel_present;           // Bit 7 of flags octet
    bool ft_time_to_next_present;           // Bit 6 of flags octet
    bool ft_current_channel_present;        // Bit 5 of flags octet (NOTE: ETSI Table 6.4.2.4-1 implies this is part of this flag octet)

    // FT Next Cluster Channel (2 octets, if ft_next_channel_present is true)
    uint32_t ft_next_cluster_channel_val;   // 13 bits data + 3 reserved

    // FT Time To Next (4 octets, if ft_time_to_next_present is true)
    uint32_t ft_time_to_next_us_val;

    // FT Current Cluster Channel (2 octets, if ft_current_channel_present is true)
    // Note: "Current Cluster Channel" is present only if "Next Cluster Channel" is also present AND indicates a different channel.
    // The ft_current_channel_present flag helps manage this.
    uint32_t ft_current_cluster_channel_val; // 13 bits data + 3 reserved

} dect_mac_assoc_req_ie_t;

// MAC Security Info IE Payload (ETSI 6.4.3.1) - This one is specific to PDU module
typedef struct {
    // Octet 0: Version (2b) | Key Index (3b) | Security IV Type (3b)
    uint8_t version_keyidx_secivtype;
    uint32_t hpc_be; // Hyper Packet Counter (Big Endian on air)
} __attribute__((packed)) dect_mac_security_info_ie_payload_t;

// Bitfield masks and shifts for dect_mac_security_info_ie_payload_t.version_keyidx_secivtype
#define MAC_SEC_IE_VERSION_SHIFT     6
#define MAC_SEC_IE_VERSION_MASK      (0x03 << MAC_SEC_IE_VERSION_SHIFT)
#define MAC_SEC_IE_KEYIDX_SHIFT      3
#define MAC_SEC_IE_KEYIDX_MASK       (0x07 << MAC_SEC_IE_KEYIDX_SHIFT)
#define MAC_SEC_IE_SECIVTYPE_SHIFT   0
#define MAC_SEC_IE_SECIVTYPE_MASK    (0x07 << MAC_SEC_IE_SECIVTYPE_SHIFT)

// Security IV Types for Mode 1 (ETSI Table 6.4.3.1-2)
#define SEC_IV_TYPE_MODE1_HPC_PROVIDED          0b000 // Current HPC is provided
#define SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE   0b001 // Request peer to send its HPC
#define SEC_IV_TYPE_MODE1_HPC_ONETIME_WITH_REQ  0b010 // One-time HPC with request (for stateless)


// --- PDU (De)Serialization Function Prototypes ---

int build_mac_mux_header(uint8_t *buf, size_t buf_len,
			 uint8_t ie_type_value, uint16_t ie_payload_len,
			 uint8_t mac_ext_format_override);

int build_load_info_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
			     const dect_mac_load_info_ie_t *load_info);

int parse_load_info_ie_muxed(const uint8_t *ie_payload_buf, uint16_t payload_len,
			     dect_mac_load_info_ie_t *out_load_info);

int build_measurement_report_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				      const dect_mac_measurement_report_ie_t *report);

int parse_measurement_report_ie_muxed(const uint8_t *ie_payload_buf, uint16_t payload_len,
				      dect_mac_measurement_report_ie_t *out_report);

int build_identity_update_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				   const dect_mac_identity_update_ie_t *id_update);

int parse_identity_update_ie_muxed(const uint8_t *ie_payload_buf, uint16_t payload_len,
				   dect_mac_identity_update_ie_t *out_id_update);

int build_system_info_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
			       const dect_mac_system_info_ie_t *sys_info);

int parse_system_info_ie_muxed(const uint8_t *ie_payload_buf, uint16_t payload_len,
			       dect_mac_system_info_ie_t *out_sys_info);

/**
 * @brief Parses a MAC Multiplexing header.
 *
 * @param buf Pointer to the start of the MUX header.
 * @param len Remaining length of the buffer from `buf`.
 * @param out_ie_type_value Pointer to store the parsed IE Type (5 or 6 bit value).
 * @param out_ie_payload_len Pointer to store the parsed IE Payload length.
 *                           If MAC_Ext=00, this will be 0, and actual length is type-defined or to end of PDU.
 * @param out_ie_payload_ptr Pointer to be set to the start of the IE's payload.
 * @return Length of the parsed MUX header in bytes (1, 2, or 3), or negative error code.
 */
int parse_mac_mux_header(const uint8_t *buf, size_t len,
                         uint8_t *out_ie_type_value, uint16_t *out_ie_payload_len,
                         const uint8_t **out_ie_payload_ptr);

// Functions to build the SDU Area (concatenation of MUXed IEs)
int build_assoc_req_ies_area(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
                             const dect_mac_assoc_req_ie_t *req_ie_fields,
                             const dect_mac_rd_capability_ie_t *cap_ie_fields);

int build_assoc_resp_sdu_area_content(uint8_t *target_sdu_area_buf, size_t target_sdu_area_max_len,
                                      int initial_offset, /* Offset to start writing from */
                                      const dect_mac_assoc_resp_ie_t *resp_fields,
                                      const dect_mac_rd_capability_ie_t *ft_cap_fields, // Can be NULL if NACK
                                      const dect_mac_resource_alloc_ie_fields_t *res_alloc_fields); // Can be NULL if NACK

int build_beacon_sdu_area_content(uint8_t *target_sdu_area_buf, size_t target_sdu_area_max_len,
                                  const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
                                  const dect_mac_rach_info_ie_fields_t *rach_beacon_ie_fields);

int build_broadcast_indication_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
                                        uint16_t paged_pt_short_id);
                                                                          
int build_keep_alive_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len);

int build_mac_security_info_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
                                     uint8_t version, uint8_t key_index,
                                     uint8_t sec_iv_type, uint32_t hpc_val);

int build_user_data_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
                             const uint8_t *dlc_pdu_data, uint16_t dlc_pdu_len,
                             uint8_t user_data_flow_ie_type);

int build_assoc_release_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				 const dect_mac_assoc_release_ie_t *release_fields);



/**
 * @brief Builds a complete MUXed Route Info IE.
 *
 * @param target_ie_area_buf Buffer to write the full MUXed IE into.
 * @param target_buf_max_len Max length of the buffer.
 * @param route_info_fields Pointer to the structure holding the Route Info fields.
 * @return Total length of the MUXed IE, or a negative error code.
 */
int build_route_info_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
			      const dect_mac_route_info_ie_t *route_info_fields);


int build_group_assignment_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				    const dect_mac_group_assignment_fields_t *fields);

int build_load_info_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
			     const dect_mac_load_info_ie_t *load_info);

int build_measurement_report_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
					const dect_mac_measurement_report_ie_t *report);


int build_joining_information_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				       const uint16_t *ep_values, uint8_t num_eps);

int build_auth_initiate_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				 uint32_t pt_nonce);

int build_auth_challenge_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				  uint32_t pt_nonce, uint32_t ft_nonce);

int build_auth_response_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				 const uint8_t *pt_mac);

int build_auth_success_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				const uint8_t *ft_mac);
                    

/**
 * @brief Serializes the payload of a Route Info IE.
 *
 * @param buf Buffer to write the serialized payload into.
 * @param buf_max_len Max length of the buffer.
 * @param route_info Pointer to the structure holding the Route Info fields.
 * @return Length of the serialized payload, or negative error code.
 */
int serialize_route_info_ie_payload(uint8_t *buf, size_t buf_max_len,
				    const dect_mac_route_info_ie_t *route_info);

/**
 * @brief Deserializes the payload of a Route Info IE.
 *
 * @param ie_payload Pointer to the IE payload.
 * @param ie_payload_len Length of the payload.
 * @param out_route_info Pointer to the structure to store the deserialized fields.
 * @return 0 on success, or a negative error code.
 */
int parse_route_info_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
				dect_mac_route_info_ie_t *out_route_info);


/**
 * @brief Builds a complete MUXed Resource Allocation IE.
 *
 * @param target_ie_area_buf Buffer to write the full MUXed IE into.
 * @param target_buf_max_len Max length of the buffer.
 * @param res_alloc_fields Pointer to the structure holding the ResAlloc fields.
 * @return Total length of the MUXed IE, or negative error code.
 */
int build_resource_alloc_ie_muxed(uint8_t *target_ie_area_buf, size_t target_buf_max_len,
				  const dect_mac_resource_alloc_ie_fields_t *res_alloc_fields);                


// Functions to parse specific IE payloads (after MUX header is stripped)
int parse_cluster_beacon_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
                                    dect_mac_cluster_beacon_ie_fields_t *out_cb_fields);
int parse_rach_info_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
			       uint8_t ft_mu_code, /* mu_code (0-3) of the FT that sent the beacon */
			       dect_mac_rach_info_ie_fields_t *out_rach_fields);
int parse_assoc_req_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
                               dect_mac_assoc_req_ie_t *out_req_fields);
int parse_assoc_resp_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
                                dect_mac_assoc_resp_ie_t *out_resp_fields);
int parse_assoc_release_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
				   dect_mac_assoc_release_ie_t *out_release_fields);
int parse_rd_capability_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
                                   dect_mac_rd_capability_ie_t *out_cap_fields);
int parse_resource_alloc_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
				    uint8_t link_mu_code, /* mu_code (0-3) of the link */
				    dect_mac_resource_alloc_ie_fields_t *out_ra_fields);
int parse_mac_security_info_ie_payload(const uint8_t *ie_payload, uint16_t ie_payload_len,
                                       uint8_t *out_version, uint8_t *out_key_index,
                                       uint8_t *out_sec_iv_type, uint32_t *out_hpc_val);

int serialize_assoc_req_ie_payload(uint8_t *buf, size_t buf_max_len,
                                   const dect_mac_assoc_req_ie_t *req_fields);

int serialize_assoc_resp_ie_payload(uint8_t *buf, size_t buf_max_len,
				    const dect_mac_assoc_resp_ie_t *resp_fields);

int serialize_cluster_beacon_ie_payload(uint8_t *buf, size_t buf_max_len,
					const dect_mac_cluster_beacon_ie_fields_t *cb_fields);
                    
int serialize_rach_info_ie_payload(uint8_t *buf, size_t buf_max_len,
                                   const dect_mac_rach_info_ie_fields_t *rach_fields);

int serialize_rd_capability_ie_payload(uint8_t *buf, size_t buf_max_len,
				       const dect_mac_rd_capability_ie_t *cap_fields);

                       

#endif /* DECT_MAC_PDU_H__ */