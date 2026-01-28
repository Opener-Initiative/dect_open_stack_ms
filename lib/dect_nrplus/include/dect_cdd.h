/* lib/dect_nrplus/dect_cdd.h */
// Overview: New file defining the API and data structures for the Configuration Data Distribution (CDD) service.

#ifndef DECT_CDD_H__
#define DECT_CDD_H__

#include <stdint.h>
#include <stddef.h>
#include <zephyr/net/net_ip.h>


/**
 * @brief Maximum number of data items that can be stored in a single CDD PDU.
 * This controls the size of the static CDD context buffer.
 */
#define DECT_CDD_MAX_ITEMS 4

/**
 * @brief Maximum payload size for a single data item within the CDD PDU.
 * Must be large enough to hold the largest expected item (e.g., an IPv6 prefix element).
 */
#define DECT_CDD_MAX_ITEM_PAYLOAD 32


/* As per ETSI TS 103 636-5, a management endpoint is used for the CDD request/response exchange. */
#define CVG_EP_MANAGEMENT_CDD 0x8001 /* Placeholder for general CDD exchange */

/* As per ETSI TS 103 874-3, the IPv6 profile data item is identified by this specific EP. */
#define CVG_EP_IPV6_PROFILE 0x8003

/* ETSI TS 103 874-3, Table A.2-2: IPv6 Address Element Header */
typedef struct {
	uint8_t element_type : 2;
	uint8_t element_version : 2;
	uint8_t rfu : 2;
	uint8_t prefix_type : 1;
	uint8_t context_usage : 1;
	uint8_t context_id : 4;
	uint8_t service_id : 4;
} __attribute__((packed)) cdd_ipv6_addr_element_hdr_t;

/* ETSI TS 103 636-5, Figure C.3.2-2 */
typedef struct {
	uint8_t ep_address[2]; /* Endpoint address of the management entity */
	uint8_t length;	       /* Length of the payload */
	// Use a fixed-size array for the payload
	uint8_t payload[DECT_CDD_MAX_ITEM_PAYLOAD];   /* Flexible array member for variable-length payload */
} cdd_data_item_t;

/* ETSI TS 103 636-5, Figure C.3.2-1 */
typedef struct {
	uint8_t type; /* 0x00000 for Full Configuration Data Content PDU */
	uint32_t sink_addr_be;
	uint8_t app_seq_num;
	uint8_t num_data_items;
	// Use a fixed-size array for the items
	cdd_data_item_t items[DECT_CDD_MAX_ITEMS]; /* Flexible array member for variable number of items */
} cdd_content_pdu_t;

/* ETSI TS 103 636-5, Figure C.3.1-1 */
typedef struct {
	uint8_t reserved_type; /* Reserved (3b) | Type (5b) */
} cdd_request_pdu_t;

#define CDD_REQUEST_TYPE_COMPLETE_CONFIG 0x00


/**
 * @brief Callback function prototype for handling a new IPv6 prefix.
 *
 * An upper layer (like the L2 driver) implements a function with this signature
 * and registers it using dect_cdd_register_prefix_handler to be notified
 * when a new prefix is received and parsed from a CDD PDU.
 *
 * @param prefix Pointer to the in6_addr struct containing the /64 prefix.
 * @param len The length of the prefix in bits (typically 64).
 * @param context_id The 6LoWPAN context ID associated with this prefix.
 */
typedef void (*cdd_ipv6_prefix_handler_t)(const struct in6_addr *prefix, uint8_t len,
					  uint8_t context_id);


/* Callback to notify application/L2 of new/updated IPv6 prefix */
typedef void (*cdd_ipv6_prefix_handler_t)(const struct in6_addr *prefix, uint8_t len,
					  uint8_t context_id);

/* Initializes the CDD service module. */
void dect_cdd_init(void);

/**
 * @brief Registers a handler to be called when a new IPv6 prefix is received.
 *
 * @param handler The function to call when a prefix is parsed from CDD content.
 */
void dect_cdd_register_prefix_handler(cdd_ipv6_prefix_handler_t handler);

/**
 * @brief Builds the FT's own configuration data to be served via CDD.
 *
 * This function should be called by the FT when it initializes. It populates
 * the CDD content with network parameters, like the IPv6 prefix.
 *
 * @param ft_prefix Pointer to the in6_addr struct containing the /64 prefix for the network.
 * @return 0 on success, or a negative error code on failure.
 */
int dect_cdd_ft_build_own_config(const struct in6_addr *ft_prefix);

/* Called by the FT to load its configuration data to be served to PTs. */
int dect_cdd_ft_set_data(const cdd_content_pdu_t *data);

/* Called by the PT state machine when it receives beacon info. */
void dect_cdd_pt_process_beacon_info(uint32_t sink_addr, uint8_t app_seq_num);

/* Called by the CVG layer to process an incoming CDD PDU. */
void dect_cdd_handle_incoming_pdu(const uint8_t *data, size_t len, uint32_t source_rd_id);

#endif /* DECT_CDD_H__ */