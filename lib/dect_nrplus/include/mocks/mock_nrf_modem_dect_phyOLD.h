/* include/mocks/mock_nrf_modem_dect_phy.h */
// Overview: Refactors the mock PHY header to define the new context-based structures and the time-aware API.
// --- REPLACE ENTIRE FILE ---
#ifndef MOCK_NRF_MODEM_DECT_PHY_H__
#define MOCK_NRF_MODEM_DECT_PHY_H__

#include <mac/nrf_modem_dect_phy.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/* --- Public flags for controlling mock behavior from tests --- */
extern bool g_force_phy_schedule_failure;
extern enum nrf_modem_dect_phy_err g_phy_schedule_failure_code;
extern bool g_strict_scheduling_mode;

#define MOCK_TIMELINE_MAX_EVENTS 64
#define MOCK_RX_QUEUE_MAX_PACKETS 32

/* Forward declare the main context struct for the self-referential peer pointer */
struct mock_phy_context_t;

/** @brief Defines the internal states of the mock PHY. */
typedef enum {
	PHY_STATE_DEINITIALIZED,
	PHY_STATE_INITIALIZED,
	PHY_STATE_ACTIVE,
} mock_phy_internal_state_t;

/** @brief Defines the type of a scheduled mock operation. */
typedef enum {
	MOCK_OP_TYPE_TX,
	MOCK_OP_TYPE_RX,
	MOCK_OP_TYPE_RSSI,
} mock_op_type_t;

/** @brief Represents a scheduled operation on a mock PHY's timeline. */
typedef struct {
	bool active;
	bool running;
	uint32_t handle;
	mock_op_type_t type;
	uint64_t start_time_us;
	uint64_t end_time_us;
	uint8_t pdu_data[CONFIG_DECT_MAC_PDU_MAX_SIZE];
	uint16_t pdu_len;
	uint16_t carrier;
	void *context;
} mock_scheduled_operation_t;

/** @brief Represents a packet to be "received" by a mock PHY. */
typedef struct {
	bool active;
	uint64_t reception_time_us;
	uint16_t carrier;
	struct nrf_modem_dect_phy_pcc_event pcc_data;
	uint8_t pdc_payload[CONFIG_DECT_MAC_PDU_MAX_SIZE];
	size_t pdc_len;
} mock_rx_packet_t;

/** @brief Represents the entire state of a single virtual PHY instance. */
typedef struct mock_phy_context_t {
	mock_phy_internal_state_t state;
	struct dect_mac_context *mac_ctx;
	// struct mock_phy_context_t *peer_ctx;
	struct mock_phy_context_t **peers;
	size_t num_peers;
	mock_scheduled_operation_t timeline[MOCK_TIMELINE_MAX_EVENTS];
	mock_rx_packet_t rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];
	uint16_t transaction_id_counter;
	mock_scheduled_operation_t *active_rx_ops[MOCK_TIMELINE_MAX_EVENTS];
	size_t num_active_rx_ops;	
} mock_phy_context_t;


/* --- Public Mock Control Functions --- */
void mock_phy_complete_reset(mock_phy_context_t *ctx);
// void mock_phy_init_context(mock_phy_context_t *ctx, struct dect_mac_context *mac_ctx, mock_phy_context_t *peer_ctx);
void mock_phy_init_context(mock_phy_context_t *ctx, struct dect_mac_context *mac_ctx, mock_phy_context_t **peers, size_t num_peers);
void mock_phy_set_active_context(mock_phy_context_t *ctx);
uint64_t mock_phy_get_next_event_time(mock_phy_context_t *const phy_contexts[], size_t num_contexts);
void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us);

#endif /* MOCK_NRF_MODEM_DECT_PHY_H__ */

