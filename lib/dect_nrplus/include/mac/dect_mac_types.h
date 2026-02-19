/* lib/dect_nrplus/mac/include/dect_mac_types.h */
// Overview: This header consolidates all public MAC data structures and enums required by upper layers.
#ifndef DECT_MAC_TYPES_H__
#define DECT_MAC_TYPES_H__

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#ifndef CONFIG_DECT_MAC_SDU_MAX_SIZE
#define CONFIG_DECT_MAC_SDU_MAX_SIZE 1636
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

#endif /* DECT_MAC_TYPES_H__ */