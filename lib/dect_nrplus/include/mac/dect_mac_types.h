/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/mac/include/dect_mac_types.h */
// Overview: This header consolidates all public MAC data structures and enums required by upper layers.
#ifndef DECT_MAC_TYPES_H__
#define DECT_MAC_TYPES_H__

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>

#ifndef CONFIG_DECT_MAC_SDU_MAX_SIZE
#define CONFIG_DECT_MAC_SDU_MAX_SIZE 2048
#endif

/** @brief Defines the logical role of the MAC instance. */
typedef enum {
	MAC_ROLE_PT, /* Portable Termination */
	MAC_ROLE_FT, /* Fixed Termination */
} dect_mac_role_t;

/** @brief Defines the high-level operational states of the MAC layer, visible to upper layers. */
typedef enum {
	MAC_STATE_PUB_DEACTIVATED,
	MAC_STATE_PUB_IDLE,
	MAC_STATE_PUB_ASSOCIATED,
	MAC_STATE_PUB_ERROR,
} dect_mac_public_state_t;

/** @brief Callback function prototype for MAC state changes. */
typedef void (*dect_mac_state_change_cb_t)(dect_mac_public_state_t new_state);

/** @brief Callback function prototype for reporting final DLC SDU TX status. */
typedef void (*dlc_tx_status_cb_t)(uint16_t dlc_sn, bool success);

/** @brief Represents a MAC Service Data Unit (SDU). */
typedef struct mac_sdu {
	void *fifo_reserved;
	uint8_t data[CONFIG_DECT_MAC_SDU_MAX_SIZE];
	uint16_t len;
	uint16_t target_peer_short_rd_id;
	bool dlc_status_report_required;
	uint16_t dlc_sn_for_status;
	
	/* QoS Metadata */
	uint8_t flow_id;       /**< ETSI Flow ID (0-7) */
	bool flow_id_present;  /**< Indicates if flow_id is valid */

	void *ctx;             /**< MAC context for multi-node simulation support */
} mac_sdu_t;

/** @brief Defines the QoS-aware data flows available to the application. */
typedef enum {
	MAC_FLOW_HIGH_PRIORITY = 0,
	MAC_FLOW_RELIABLE_DATA,
	MAC_FLOW_BEST_EFFORT,
	MAC_FLOW_COUNT
} mac_flow_id_t;

/**
 * @brief DECT-specific metadata stored in net_buf->user_data.
 *
 * Fits within the 16-byte user_data field of a Zephyr net_buf.
 * Used to carry protocol metadata across DLC and CVG data paths
 * without requiring wrapper structs or separate allocations.
 *
 * IMPORTANT: sizeof(dect_buf_meta_t) MUST be <= CONFIG_NET_BUF_USER_DATA_SIZE.
 * Add a BUILD_ASSERT in any file that uses this.
 */
typedef struct {
	uint16_t dlc_sn;          /**< DLC/CVG Sequence Number */
	uint16_t endpoint_id;     /**< CVG Endpoint Mux ID (0 = not present) */
	uint32_t source_addr;     /**< Source Long RD ID */
	uint32_t dest_addr;       /**< Destination Long RD ID */
	uint8_t  flow_id;         /**< QoS Flow ID (mac_flow_id_t) */
	uint8_t  service_type;    /**< DLC service type (dlc_service_type_t) */
	uint8_t  flags;           /**< Bitmask: bit0=ARQ, bit1=forward, bit2=encrypted */
	uint8_t  _pad;            /**< Reserved for alignment */
} dect_buf_meta_t;

BUILD_ASSERT(sizeof(dect_buf_meta_t) <= 16,
	     "dect_buf_meta_t exceeds net_buf user_data size of 16 bytes");

/** @brief Helper: get DECT metadata from a net_buf */
static inline dect_buf_meta_t *dect_buf_meta(struct net_buf *buf)
{
	return (dect_buf_meta_t *)net_buf_user_data(buf);
}

/** Flag bits for dect_buf_meta_t.flags */
#define DECT_BUF_FLAG_ARQ_PENDING  BIT(0)
#define DECT_BUF_FLAG_FORWARD      BIT(1)
#define DECT_BUF_FLAG_ENCRYPTED    BIT(2)

#endif /* DECT_MAC_TYPES_H__ */