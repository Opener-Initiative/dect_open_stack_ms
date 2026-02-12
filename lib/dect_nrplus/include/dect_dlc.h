#ifndef DECT_DLC_H__
#define DECT_DLC_H__

#include <zephyr/kernel.h> // For k_timeout_t
#include <stdint.h>        // For uintxx_t types
#include <stddef.h>        // For size_t
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/dlist.h>

#include <mac/dect_mac_context.h>
// #include "mac/dect_mac_sm.h"   // For dect_mac_role_t (used by dect_stack_init)
#include <mac/dect_mac_sm.h>   // For dect_mac_role_t (used by dect_stack_init)
#include <mac/dect_mac_security.h>
#include <mac/dect_mac.h>
#include <net_platform_time.h> /**< For net_tick_t */

/* DList for data from DLC to the App/CVG layer. Made extern for test access. */
#if IS_ENABLED(CONFIG_ZTEST)
/* Expose thread IDs for test control */
// extern sys_dlist_t g_dlc_to_app_rx_dlist;;
extern k_tid_t g_dlc_tx_service_thread_id;
extern k_tid_t g_dlc_rx_thread_id;
#endif /* IS_ENABLED(CONFIG_ZTEST) */



/* --- DLC Routing Header Definitions (ETSI TS 103 636-5, Clause 5.3.4) --- */

/* Bitmap field definitions - FOR BIG-ENDIAN (WIRE FORMAT) */
#define DLC_RH_QOS_SHIFT	          12  /* Bits 12-15: MSB of first byte */
#define DLC_RH_DELAY_SHIFT	          11  /* Bit 11 */
#define DLC_RH_HOP_COUNT_LIMIT_SHIFT   9   /* Bit 9 */  
#define DLC_RH_DEST_ADDR_SHIFT	        6   /* Bit 6 */
#define DLC_RH_SRC_ADDR_SHIFT	        5   /* Bit 5 */
#define DLC_RH_SEQ_NUM_SHIFT	        4   /* Bit 4 */
#define DLC_RH_ROUTING_TYPE_SHIFT      0   /* Bits 0-3: LSB of second byte */

/* Routing Type field values (Table 5.3.4-2) */
#define DLC_RH_TYPE_UPLINK_TO_BACKEND 0b000
#define DLC_RH_TYPE_DOWNLINK_FLOODING 0b011
#define DLC_RH_TYPE_DOWNLINK_SSR	  0b100
#define DLC_RH_TYPE_LOCAL_FLOODING	  0b101

/* Dest_Add field values (Table 5.3.4-1) */
#define DLC_RH_DEST_ADD_PRESENT	   0b000
#define DLC_RH_DEST_ADD_BROADCAST  0b001
#define DLC_RH_DEST_ADD_BACKEND	   0b010
#define DLC_RH_SRC_ADD_IS_BACKEND  0b011

typedef struct {
	uint16_t bitmap_be; /* Routing bitmap field (2 octets, Big Endian) */
	/* Optional fields follow based on bitmap. This struct represents the maximum possible size. */
	uint32_t source_addr_be;
	uint32_t dest_addr_be;
	uint8_t hop_count;
	uint8_t hop_limit;
	uint32_t delay_be;
	uint8_t sequence_number;
} __attribute__((packed)) dect_dlc_routing_header_t;


/**
 * @brief Defines the DLC service types available to the application.
 * Based on ETSI TS 103 636-5, Clause 4.3.1.1.
 */
typedef enum {
    /** Transparent, best-effort data transfer. Relies on MAC HARQ. No DLC segmentation. */
    DLC_SERVICE_TYPE_0_TRANSPARENT,
    /** Unreliable data transfer with Segmentation and Reassembly by DLC. No DLC ARQ. */
    DLC_SERVICE_TYPE_1_SEGMENTATION,
    /** Acknowledged data transfer with end-to-end retransmissions (ARQ) by DLC. No DLC segmentation. */
    DLC_SERVICE_TYPE_2_ARQ,
    /** Acknowledged data transfer with Segmentation, Reassembly, and ARQ by DLC. */
    DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ,
    DLC_SERVICE_TYPE_COUNT
} dlc_service_type_t;

/**
 * @brief DLC PDU IE Type field values relevant to data transfer.
 * (ETSI TS 103 636-5 Table 5.3.1-1, first 4 bits of DLC PDU).
 * These define the structure of the DLC PDU header.
 */
typedef enum {
    DLC_IE_TYPE_DATA_TYPE_0_WITH_ROUTING    = 0b0000, // DLC Svc Type 0, DLC SDU contains Routing Hdr + CVG PDU
    DLC_IE_TYPE_DATA_TYPE_0_NO_ROUTING      = 0b0001, // DLC Svc Type 0, DLC SDU contains CVG PDU directly
    DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING  = 0b0010, // DLC Svc Types 1,2,3, DLC SDU contains Routing Hdr + CVG PDU
    DLC_IE_TYPE_DATA_TYPE_123_NO_ROUTING    = 0b0011, // DLC Svc Types 1,2,3, DLC SDU contains CVG PDU directly
    DLC_IE_TYPE_TIMERS_CONFIG_CTRL          = 0b0100, // DLC Timers configuration Control IE
    DLC_IE_TYPE_DATA_TYPE_0_EXT_HDR         = 0b0101, // Type 0 data followed by DLC Extension header
    DLC_IE_TYPE_DATA_TYPE_123_EXT_HDR       = 0b0110, // Type 1/2/3 data followed by DLC Extension header
    // 0b0111 to 0b1101 are Reserved
    DLC_IE_TYPE_ESCAPE                      = 0b1110,
    DLC_IE_TYPE_RESERVED_MAX                = 0b1111, // Also Reserved by ETSI for future use
} dlc_ie_type_val_t;

/**
 * @brief DLC Segmentation Indication (SI) field.
 * (ETSI TS 103 636-5 Table 5.3.3.1-1, bits 5-4 of first octet for Type 1/2/3 PDUs if IE Type uses 4 MSB).
 * Note: ETSI table uses bits 3-2. If IE Type is 4 bits, SI is bits 5-4.
 * Let's assume IE Type is indeed 4 bits, so SI is bits 5-4 (0-indexed from MSB: bit0=MSB).
 * Octet1: [IE Type (4b) | SI (2b) | SN_ms2b (2b)]
 */
typedef enum {
    DLC_SI_COMPLETE_SDU     = 0b00,
    DLC_SI_FIRST_SEGMENT    = 0b01,
    DLC_SI_LAST_SEGMENT     = 0b10,
    DLC_SI_MIDDLE_SEGMENT   = 0b11,
} dlc_segmentation_indication_t;

/**
 * @brief DLC Header for Type 0 (Transparent) PDUs.
 * Total 1 octet. (ETSI TS 103 636-5, Figure 5.3.2-1).
 * The DLC SDU (which is CVG PDU or CVG PDU + DLC Routing Header) immediately follows.
 */
typedef struct {
    uint8_t ie_type_val_reserved; // Bits 7-4: IE Type, Bits 3-0: Reserved (must be 0)
} __attribute__((packed)) dect_dlc_header_type0_t;

// Helper to set/get fields for dect_dlc_header_type0_t
static inline void dlc_hdr_type0_set(dect_dlc_header_type0_t *hdr, dlc_ie_type_val_t type) {
    hdr->ie_type_val_reserved = ((uint8_t)type & 0x0F) << 4; // Ensures LSB 4 bits are 0
}
static inline dlc_ie_type_val_t dlc_hdr_type0_get_type(const dect_dlc_header_type0_t *hdr) {
    return (dlc_ie_type_val_t)((hdr->ie_type_val_reserved >> 4) & 0x0F);
}



/**
 * @brief DLC Header for Type 1, 2, 3 PDUs (unsegmented: SI=00, or first segment: SI=01).
 * Total 2 octets. (ETSI TS 103 636-5, Figure 5.3.3.1-1).
 * The DLC SDU (or first segment of DLC SDU) immediately follows.
 * Octet 1: [IE Type (4b) | SI (2b) | SN_ms2b (2b)]
 * Octet 2: [SN_ls8b (8b)]
 */
typedef struct {
    uint8_t ie_type_si_sn_msb;   // Octet 1: IE Type(4b)|SI(2b)|SN_ms2b(2b)
    uint8_t sequence_number_lsb; // Octet 2: SN_ls8b(8b)
} __attribute__((packed)) dect_dlc_header_type123_basic_t;

// Helpers for dect_dlc_header_type123_basic_t
static inline void dlc_hdr_t123_basic_set(dect_dlc_header_type123_basic_t *hdr, dlc_ie_type_val_t type,
                                         dlc_segmentation_indication_t si, uint16_t sn_10bit) {
    hdr->ie_type_si_sn_msb = (((uint8_t)type & 0x0F) << 4) |
                             (((uint8_t)si & 0x03) << 2) |
                             ((uint8_t)((sn_10bit >> 8) & 0x03)); // SN ms2b
    hdr->sequence_number_lsb = (uint8_t)(sn_10bit & 0xFF);     // SN ls8b
}

static inline dlc_ie_type_val_t dlc_hdr_t123_basic_get_type(const dect_dlc_header_type123_basic_t *hdr) {
    return (dlc_ie_type_val_t)((hdr->ie_type_si_sn_msb >> 4) & 0x0F);
}
static inline dlc_segmentation_indication_t dlc_hdr_t123_basic_get_si(const dect_dlc_header_type123_basic_t *hdr) {
    return (dlc_segmentation_indication_t)((hdr->ie_type_si_sn_msb >> 2) & 0x03);
}
static inline uint16_t dlc_hdr_t123_basic_get_sn(const dect_dlc_header_type123_basic_t *hdr) {
    return (uint16_t)(((hdr->ie_type_si_sn_msb & 0x03) << 8) | hdr->sequence_number_lsb);
}


/**
 * @brief DLC Header for Type 1 or Type 3 PDUs (middle segment: SI=11, or last segment: SI=10).
 * Total 4 octets. (ETSI TS 103 636-5, Figure 5.3.3.1-2).
 * The DLC SDU segment immediately follows.
 * Octets 3-4: Segmentation Offset (Big Endian on air)
 */
typedef struct {
    uint8_t ie_type_si_sn_msb;        // Octet 1
    uint8_t sequence_number_lsb;      // Octet 2
    uint16_t segmentation_offset_be;  // Octets 3-4: Big Endian on air
} __attribute__((packed)) dect_dlc_header_type13_segmented_t;

// Helpers for dect_dlc_header_type13_segmented_t (setters similar to basic, plus offset)
static inline void dlc_hdr_t13_segmented_set(dect_dlc_header_type13_segmented_t *hdr, dlc_ie_type_val_t type,
                                            dlc_segmentation_indication_t si, uint16_t sn_10bit, uint16_t seg_offset) {
    hdr->ie_type_si_sn_msb = (((uint8_t)type & 0x0F) << 4) |
                             (((uint8_t)si & 0x03) << 2) |
                             ((uint8_t)((sn_10bit >> 8) & 0x03));
    hdr->sequence_number_lsb = (uint8_t)(sn_10bit & 0xFF);
    hdr->segmentation_offset_be = sys_cpu_to_be16(seg_offset);
}

// Getters for type, SI, SN are same as basic.
static inline uint16_t dlc_hdr_t13_segmented_get_offset(const dect_dlc_header_type13_segmented_t *hdr) {
    return sys_be16_to_cpu(hdr->segmentation_offset_be);
}

/**
 * @name DLC Control and Extension Headers
 * @{
 * Structures and helpers for DLC Control IEs and the Extension Header mechanism,
 * as per ETSI TS 103 636-5, Clauses 5.3.3.2 and 5.3.3.3.
 */

/**
 * @brief DLC Header for Timers Configuration Control IE.
 * Total 2 octets. (ETSI TS 103 636-5, Figure 5.3.3.2-1).
 * Used to configure SDU lifetime timers.
 */
typedef struct {
    uint8_t ie_type_val_reserved; // Bits 7-4: IE Type (0b0100), Bits 3-0: Reserved
    uint8_t lifetime_timer_val;   // The 8-bit lifetime value code from Table 5.3.3.2-2
} __attribute__((packed)) dect_dlc_header_timers_config_t;

/**
 * @brief DLC Extension Header Length Type (DLC Ext field).
 * (ETSI TS 103 636-5, Table 5.3.3.3-1, bits 7-6 of first octet).
 */
typedef enum {
    DLC_EXT_HDR_NO_LEN_FIELD    = 0b00,
    DLC_EXT_HDR_8BIT_LEN_FIELD  = 0b01,
    DLC_EXT_HDR_16BIT_LEN_FIELD = 0b10,
    DLC_EXT_HDR_RESERVED        = 0b11,
} dect_dlc_ext_len_type_t;

/**
 * @brief DLC Extension IE Type field values.
 * (ETSI TS 103 636-5, Table 5.3.3.3-2, bits 5-0 of first octet).
 */
typedef enum {
    DLC_EXT_IE_ROUTING_HEADER = 0b000000,
    DLC_EXT_IE_CVG_PDU        = 0b000001,
    DLC_EXT_IE_NEXT_HOP_ADDR  = 0b000010,
    DLC_EXT_IE_ROUTE_REGISTER = 0b000011,
    DLC_EXT_IE_ROUTE_ERROR    = 0b000100,
    // ... other reserved values
} dect_dlc_ext_ie_type_t;

/**
 * @brief DLC Extension Header base structure (first octet).
 * (ETSI TS 103 636-5, Figure 5.3.3.3-1).
 * This is a generic wrapper for other IEs like Routing Header, Next Hop Address, etc.
 * The full header may be 1, 2, or 3 bytes depending on the `dlc_ext_len_type`.
 */
typedef struct {
    uint8_t ext_type_and_ie_type; // Bits 7-6: DLC Ext, Bits 5-0: Extension IE Type
} __attribute__((packed)) dect_dlc_extension_header_t;

// Helpers for dect_dlc_extension_header_t
static inline void dlc_ext_hdr_set(dect_dlc_extension_header_t *hdr, dect_dlc_ext_len_type_t len_type,
                                   dect_dlc_ext_ie_type_t ie_type) {
    hdr->ext_type_and_ie_type = (((uint8_t)len_type & 0x03) << 6) | ((uint8_t)ie_type & 0x3F);
}
static inline dect_dlc_ext_len_type_t dlc_ext_hdr_get_len_type(const dect_dlc_extension_header_t *hdr) {
    return (dect_dlc_ext_len_type_t)((hdr->ext_type_and_ie_type >> 6) & 0x03);
}
static inline dect_dlc_ext_ie_type_t dlc_ext_hdr_get_ie_type(const dect_dlc_extension_header_t *hdr) {
    return (dect_dlc_ext_ie_type_t)(hdr->ext_type_and_ie_type & 0x3F);
}

/** @} */

/**
 * @name DLC Information Elements (Payloads for Extension Header)
 * @{
 * Structures for IEs that are wrapped by the DLC Extension Header.
 */

/**
 * @brief Next Hop Address IE payload.
 * (ETSI TS 103 636-5, Figure 5.3.3.4-1). 4 bytes total.
 */
typedef struct {
    uint32_t long_rd_id_be;
} __attribute__((packed)) dect_dlc_ie_next_hop_t;

/**
 * @brief Route Register IE payload.
 * (ETSI TS 103 636-5, Figure 5.3.3.5-1). 4 bytes total.
 */
typedef struct {
    uint32_t source_routing_id_be;
} __attribute__((packed)) dect_dlc_ie_route_register_t;

/**
 * @brief Route Error IE payload.
 * (ETSI TS 103 636-5, Figure 5.3.3.6-1). 5 bytes total.
 */
typedef struct {
    uint8_t error_reason;
    uint32_t invalid_next_hop_addr_be;
} __attribute__((packed)) dect_dlc_ie_route_error_t;


/**
 * @brief Item structure for the FIFO queue from DLC RX thread to the CVG/Application layer.
 *
 * Contains the received/reassembled DLC SDU (which is a CVG PDU) and the
 * DLC service type that was applicable to its reception/processing by the DLC.
 */
typedef struct {
	void *fifo_reserved; /* For k_queue internal use */
	/* mac_sdu_t *sdu_buf;  Buffer containing the CVG PDU (DLC SDU payload) */
	void *sdu_buf; /* Generic pointer to a slab-allocated buffer */
	bool is_app_sdu; /* Flag to indicate if sdu_buf is a dlc_app_sdu_t or mac_sdu_t */
	dlc_service_type_t dlc_service_type; /* The DLC service type of this SDU */

	/* Scheduler Metadata */
	net_tick_t entry_timestamp; /**< Time when packet entered the queue */
	uint8_t priority;           /**< Priority level */
	void *origin_queue;         /**< Pointer to the source k_queue */
} dlc_rx_delivery_item_t;


/**
 * @brief Initializes the DLC layer.
 *
 * Sets up the DLC receive thread and reassembly sessions. This function is
 * called by the CVG layer during stack initialization.
 *
 * @return 0 on success, or a negative error code.
 */
int dect_dlc_init(void);

/**
 * @brief Sends application data (which forms a CVG PDU payload from DLC's perspective)
 *        over the DECT link using a specific DLC service.
 *
 * This function will handle necessary DLC procedures, including prepending appropriate DLC headers
 * and potentially segmenting the data if the selected DLC service type involves
 * segmentation and the data is too large for a single MAC SDU (DLC PDU).
 * The data provided is considered the DLC SDU payload (e.g., a CVG PDU or an IP packet if CVG is bypassed).
 *
 * @param service The DLC service type to use for this data transmission.
 * @param dest_long_id, The Destination adress long ID.
 * @param dlc_sdu_payload Pointer to the application data payload (e.g., a CVG PDU).
 * @param dlc_sdu_payload_len Length of the data payload.
 * @param flow_id The ETSI Flow ID (0-7) for QoS.
 * @return 0 on success (data queued to MAC).
 * @retval -EINVAL If service type is invalid, data is NULL, or length is invalid.
 * @retval -ENOMEM If a MAC SDU buffer cannot be allocated for transmission.
 * @retval -EMSGSIZE If dlc_sdu_payload is too large for an unsegmented service type,
 *                   or if segmentation is required but fails (e.g., too many segments).
 */
int dlc_send_data(dlc_service_type_t service, uint32_t dest_long_id,
		  const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len,
		  uint8_t flow_id);


/**
 * @brief Sends a DLC Timers Configuration Control IE to a peer.
 *
 * This allows for over-the-air configuration of SDU lifetime values.
 *
 * @param dest_long_id The Long RD ID of the peer to configure.
 * @param lifetime_ms The desired SDU lifetime in milliseconds. The value will be
 *                    converted to the appropriate 8-bit code for transmission.
 * @return 0 on success, or a negative error code.
 */
int dect_dlc_send_timers_config(uint32_t dest_long_id, uint32_t lifetime_ms);


/**
 * @brief Receives application data (CVG PDU payload from DLC's perspective) from the DECT stack.
 *
 * This function is blocking and should typically be called from a dedicated application thread
 * that processes incoming data. It retrieves a complete, reassembled Service Data Unit payload
 * (e.g., a CVG PDU) from the DLC layer, providing the data and its original DLC service type.
 *
 * @param service_type_out Pointer to store the DLC service type of the received data.
 * @param app_level_payload_buf Buffer to store the incoming data payload.
 * @param app_level_payload_len_inout Pointer to a size_t variable.
 *                                    On input, it must contain the maximum size of `app_level_payload_buf`.
 *                                    On successful return, it will contain the actual length of the received payload.
 * @param timeout The maximum time to wait for incoming data. Use K_FOREVER to wait
 *                indefinitely, K_NO_WAIT for non-blocking.
 * @return 0 on success.
 * @retval -EINVAL If any pointer arguments are NULL.
 * @retval -EMSGSIZE If the provided `app_level_payload_buf` is too small to hold the received
 *                   data. `*app_level_payload_len_inout` will be updated with the required size.
 * @retval -EAGAIN If the operation timed out (K_NO_WAIT and no data, or `timeout` expired).
 * @retval Other negative error codes for internal issues.
 */
// int dlc_receive_data(dlc_service_type_t *service_type_out, uint8_t *app_level_payload_buf, size_t *app_level_payload_len_inout, k_timeout_t timeout);
int dlc_receive_data(dlc_service_type_t *service_type_out, uint32_t *source_addr_out,
		     uint8_t *app_level_payload_buf, size_t *app_level_payload_len_inout,
		     k_timeout_t timeout);

/**
 * @brief Forwards a complete DLC PDU to the MAC layer. (Internal use for routing).
 *
 * This function is used by the DLC routing service to forward a packet that
 * is not destined for this device. It takes an already-formed DLC PDU and
 * queues it for transmission towards the current parent FT.
 *
 * @param dlc_pdu Pointer to the full DLC PDU (including headers).
 * @param dlc_pdu_len Length of the full DLC PDU.
 * @return 0 on success, or a negative error code.
 */
int dlc_forward_pdu(const uint8_t *dlc_pdu, size_t dlc_pdu_len);

/**
 * @brief Callback function prototype for reporting final DLC SDU TX status.
 *
 * The MAC layer will invoke this callback to inform the DLC layer about the
 * final success or failure of a transmission for an SDU that required a status report.
 *
 * @param dlc_sn The 10-bit DLC sequence number of the SDU.
 * @param success True if the SDU was successfully acknowledged by the peer,
 *                false if it failed after all HARQ retries or timed out.
 */
typedef void (*dlc_tx_status_cb_t)(uint16_t dlc_sn, bool success);




#if IS_ENABLED(CONFIG_DECT_DLC_API_MOCK)
/**
 * @brief [TEST ONLY] Sets a spy callback for the dlc_send_data function.
 */
void dlc_test_set_send_spy(int (*handler)(dlc_service_type_t, uint32_t, const uint8_t *, size_t, uint8_t));

/**
 * @brief [TEST ONLY] Sets a spy callback for the dlc_receive_data function.
 */
void dlc_test_set_receive_spy(int (*handler)(dlc_service_type_t *, uint32_t *, uint8_t *, size_t *, k_timeout_t));
#endif /* IS_ENABLED(CONFIG_DECT_DLC_API_MOCK) */






/* DList for data from MAC to the DLC layer. Made extern for test access. */
extern sys_dlist_t g_dlc_internal_mac_rx_dlist;

/**
 * @brief [TEST ONLY] Serializes a DLC routing header structure into a byte buffer.
 *
 * @param target_buf Buffer to write the serialized header into.
 * @param target_buf_len Maximum length of the target buffer.
 * @param rh Pointer to the routing header structure.
 * @return The length of the serialized header, or a negative error code.
 */
int dlc_serialize_routing_header(uint8_t *target_buf, size_t target_buf_len,
				 const dect_dlc_routing_header_t *rh);

/**
 * @brief [TEST ONLY] Parses a serialized DLC routing header from a byte buffer.
 *
 * @param buf Pointer to the start of the serialized routing header.
 * @param len The maximum length of the buffer.
 * @param rh Pointer to the routing header structure to populate.
 * @return The number of bytes consumed, or a negative error code.
 */
int dlc_parse_routing_header(const uint8_t *buf, size_t len, dect_dlc_routing_header_t *rh);





#endif /* DECT_DLC_H__ */