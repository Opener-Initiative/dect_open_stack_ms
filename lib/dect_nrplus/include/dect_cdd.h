/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/dect_cdd.h */
// Overview: Header defining the spec-pure API for the Configuration Data Distribution (CDD) service.

#ifndef DECT_CDD_H__
#define DECT_CDD_H__

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Maximum number of data items that can be stored in a single CDD PDU.
 * This controls the size of the static CDD context buffer.
 */
#define DECT_CDD_MAX_ITEMS 4

/**
 * @brief Maximum payload size for a single data item within the CDD PDU.
 * Must be large enough to hold the largest expected item.
 */
#define DECT_CDD_MAX_ITEM_PAYLOAD 32


/* As per ETSI TS 103 636-5, a management endpoint is used for the CDD request/response exchange. */
#define CVG_EP_MANAGEMENT_CDD 0x8001

/* As per ETSI TS 103 874-3, the IPv6 profile data item is identified by this specific EP. */
#define CVG_EP_IPV6_PROFILE 0x8003

/* ETSI TS 103 636-5, Figure C.3.2-2 */
typedef struct {
	uint8_t ep_address[2]; /* Endpoint address of the management entity (Big Endian) */
	uint8_t length;	       /* Length of the payload */
	uint8_t payload[DECT_CDD_MAX_ITEM_PAYLOAD];
} cdd_data_item_t;

/* ETSI TS 103 636-5, Figure C.3.2-1 */
typedef struct {
	uint8_t type; /* 0x00 for Full Configuration Data Content PDU */
	uint8_t sink_addr_be[4];
	uint8_t app_seq_num;
	uint8_t num_data_items;
	cdd_data_item_t items[DECT_CDD_MAX_ITEMS];
} cdd_content_pdu_t;

/* ETSI TS 103 636-5, Figure C.3.1-1 */
typedef struct {
	uint8_t reserved_type; /* Reserved (3b) | Type (5b) */
} cdd_request_pdu_t;

#define CDD_REQUEST_TYPE_COMPLETE_CONFIG 0x00


/**
 * @brief Callback function prototype for handling a received CDD data item.
 *
 * An upper layer (like a Zephyr L2 driver) registers a handler to be notified
 * when a specific data item (identified by its management endpoint) is received.
 *
 * @param endpoint The management endpoint identifying the data item type.
 * @param data Pointer to the raw payload of the data item.
 * @param len The length of the payload.
 */
typedef void (*cdd_data_item_handler_t)(uint16_t endpoint, const uint8_t *data, size_t len);


/* Initializes the CDD service module. */
void dect_cdd_init(void);

/**
 * @brief Registers a handler for CDD data items.
 *
 * @param handler The function to call when a data item is parsed from CDD content.
 */
void dect_cdd_register_handler(cdd_data_item_handler_t handler);

/**
 * @brief FT: Adds a data item to the local configuration to be served via CDD.
 *
 * @param endpoint The management endpoint for this item.
 * @param data The raw data to include.
 * @param len Length of the data.
 * @return 0 on success, or a negative error code.
 */
int dect_cdd_ft_add_item(uint16_t endpoint, const uint8_t *data, size_t len);

/**
 * @brief FT: Sets the target sink address for the configuration content.
 */
void dect_cdd_ft_set_sink_addr(uint32_t sink_addr);

/* Called by the PT state machine when it receives beacon info. */
void dect_cdd_pt_process_beacon_info(uint32_t sink_addr, uint8_t app_seq_num);

/* Called by the CVG layer to process an incoming CDD PDU. */
void dect_cdd_handle_incoming_pdu(const uint8_t *data, size_t len, uint32_t source_rd_id);

#endif /* DECT_CDD_H__ */