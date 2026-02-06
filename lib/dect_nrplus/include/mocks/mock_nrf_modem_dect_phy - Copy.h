/* include/mocks/mock_nrf_modem_dect_phy.h */
/**
 * @file mock_nrf_modem_dect_phy.h
 * @brief Enhanced Mock PHY for Nordic DECT NR+ with Microsecond Logic
 * This mock PHY provides a realistic simulation of the Nordic DECT NR+ modem
 * while using microseconds for timing to aid development and testing.
 * 
 * This mock provides a discrete-event simulation of the Nordic PHY.
 * Key Features:
 * - Microsecond-based internal timeline.
 * - Multi-device peer-to-peer simulation.
 * - Configurable RF environment (Noise floor per carrier).
 * - Error injection (Probabilistic packet loss).
 * - Collision detection (Overlapping transmissions on same carrier).
 */

#ifndef MOCK_NRF_MODEM_DECT_PHY_H__
#define MOCK_NRF_MODEM_DECT_PHY_H__

#include <mac/nrf_modem_dect_phy.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================
 * CONFIGURATION & CONSTANTS
 * ======================================================================== */

#define MOCK_TIMELINE_MAX_EVENTS 64
#define MOCK_RX_QUEUE_MAX_PACKETS 64
#define MOCK_MAX_CARRIERS 10

/* ========================================================================
 * GLOBAL FLAGS
 * ======================================================================== */

/**
 * @brief Force the next PHY scheduling operation to fail
 * 
 * When set to true, the next call to nrf_modem_dect_phy_tx(), _rx(), or _rssi()
 * will fail with the error code specified in g_phy_schedule_failure_code.
 * The flag is not automatically cleared after use.
 * 
 * @note Used for testing error handling paths
 */
extern bool g_force_phy_schedule_failure;

/**
 * @brief Error code to return when g_force_phy_schedule_failure is true
 */
extern enum nrf_modem_dect_phy_err g_phy_schedule_failure_code;

/**
 * @brief Enable strict scheduling conflict checking
 * 
 * When enabled, the mock PHY will check for overlapping operations and
 * return NRF_MODEM_DECT_PHY_ERR_OP_SCHEDULING_CONFLICT if a conflict is detected.
 * 
 * This helps catch scheduling bugs during development.
 */
extern bool g_strict_scheduling_mode;

/* ========================================================================
 * DATA STRUCTURES
 * ======================================================================== */

/* Forward declaration */
struct mock_phy_context_t;

/** @brief PHY Internal States */
/**
 * @brief PHY internal states
 * 
 * These states mirror the Nordic DECT NR+ PHY state machine:
 * DEINITIALIZED → INITIALIZED → ACTIVE → INITIALIZED → DEINITIALIZED
 */
typedef enum {
    PHY_STATE_DEINITIALIZED,
    PHY_STATE_INITIALIZED,
    PHY_STATE_ACTIVE,
} mock_phy_internal_state_t;

/** @brief Operation Types */
typedef enum {
    MOCK_OP_TYPE_NONE = 0,
    MOCK_OP_TYPE_TX,
    MOCK_OP_TYPE_RX,
    MOCK_OP_TYPE_RSSI,
} mock_op_type_t;

/**
 * @brief Operation lifecycle states
 * 
 * Tracks the lifecycle of an operation from scheduling to completion.
 * This mirrors how the real Nordic PHY manages operations.
 */
typedef enum {
    OP_STATE_IDLE = 0,        /**< Slot is free */
    OP_STATE_SCHEDULED,       /**< Operation accepted, waiting for start time */
    OP_STATE_RUNNING,         /**< Operation is executing */
    OP_STATE_COMPLETING,      /**< Generating completion events */
    OP_STATE_COMPLETED,       /**< Done, ready to be freed */
    OP_STATE_CANCELED,        /**< Canceled by user */
} operation_state_t;

/**
 * @brief Scheduled operation structure
 * 
 * Represents a single scheduled PHY operation (TX, RX, or RSSI).
 * Includes all state needed to simulate realistic operation behavior.
 */

/** 
 * @brief Scheduled Operation (Timeline Item)
 */
typedef struct {
    bool active;
    bool running;
    
    uint32_t handle;
    mock_op_type_t type;
    
    /* Time window (us) */
    uint64_t start_time_us;
    uint64_t end_time_us;
    
    /* RF Params */
    uint16_t carrier;
    
    /* Data (for TX) */
    uint8_t pdu_data[CONFIG_DECT_MAC_PDU_MAX_SIZE];
    uint16_t pdu_len;
    
    void *context; /* Pointer to MAC context */
} mock_scheduled_operation_t;

/** 
 * @brief Queued RX Packet (Incoming from Peer)
 */
typedef struct {
    bool active;
    
    /* When the packet arrives at the antenna (us) */
    uint64_t reception_time_us;
    uint64_t duration_us; /* Calculated duration for collision detection */
    uint16_t carrier;
    
    /* Event Data */
    struct nrf_modem_dect_phy_pcc_event pcc_data;
    uint8_t pdc_payload[CONFIG_DECT_MAC_PDU_MAX_SIZE];
    size_t pdc_len;
} mock_rx_packet_t;


/**
 * @brief PHY context statistics
 * 
 * Tracks operation counts for debugging and performance analysis.
 */
typedef struct {
    uint32_t tx_count;              /**< Number of TX operations scheduled */
    uint32_t rx_count;              /**< Number of RX operations scheduled */
    uint32_t rssi_count;            /**< Number of RSSI measurements */
    uint32_t tx_rx_count;           /**< Number of TX+RX operations */
    uint32_t canceled_count;        /**< Number of canceled operations */
    uint32_t failed_count;          /**< Number of failed operations */
    uint32_t rx_packets_delivered;  /**< Number of RX packets delivered */
    uint32_t rx_packets_dropped;    /**< Number of RX packets dropped */
} mock_phy_stats_t;

/** 
 * @brief PHY Context (The Virtual Hardware)
 */
typedef struct mock_phy_context_t {
    mock_phy_internal_state_t state;
    
    struct dect_mac_context *mac_ctx;
    
    /* Multi-device simulation links */
    struct mock_phy_context_t **peers;
    size_t num_peers;
    
    /* Operation Timeline */
    mock_scheduled_operation_t timeline[MOCK_TIMELINE_MAX_EVENTS];
    
    /* Incoming Packet Queue */
    mock_rx_packet_t rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];
    
    /* Internal Counters */
    uint16_t transaction_id_counter;
    
    /* Helpers for tracking active ops */
    mock_scheduled_operation_t *active_rx_ops[MOCK_TIMELINE_MAX_EVENTS];
    size_t num_active_rx_ops;   
} mock_phy_context_t;


/* ========================================================================
 * PUBLIC CONTROL API
 * ======================================================================== */

void mock_phy_complete_reset(mock_phy_context_t *ctx);
void mock_phy_init_context(mock_phy_context_t *ctx, struct dect_mac_context *mac_ctx, mock_phy_context_t **peers, size_t num_peers);
void mock_phy_set_active_context(mock_phy_context_t *ctx);

/* Time & Event Management */
uint64_t mock_phy_get_next_event_time(mock_phy_context_t *const phy_contexts[], size_t num_contexts);
void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us);

/* Peer-to-Peer Communication */
int mock_phy_queue_rx_packet(mock_phy_context_t *dest_ctx, const mock_rx_packet_t *packet);

/* ========================================================================
 * TEST CONFIGURATION API (NEW)
 * ======================================================================== */

/**
 * @brief Set the background noise (RSSI) for a specific carrier.
 * @param carrier Carrier index (0-9)
 * @param dbm RSSI value in dBm (e.g., -100 for clean, -60 for noisy)
 */
void mock_phy_test_set_noise_floor(uint16_t carrier, int8_t dbm);

/**
 * @brief Configure probabilistic packet loss.
 * @param loss_rate_percent 0 to 100. (0 = perfect link).
 */
void mock_phy_test_config_error_injection(uint8_t loss_rate_percent);

/**
 * @brief Enable or disable collision simulation.
 * @param enabled If true, overlapping packets result in CRC errors.
 */
void mock_phy_test_config_collisions(bool enabled);
/* ========================================================================
 * NORDIC PHY API
 * 
 * These functions implement the Nordic DECT NR+ PHY API.
 * See <mac/nrf_modem_dect_phy.h> for detailed documentation.
 * ======================================================================== */

/**
 * @brief Set PHY event handler
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_event_handler_set(nrf_modem_dect_phy_event_handler_t handler);

/**
 * @brief Activate PHY
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_activate(enum nrf_modem_dect_phy_radio_mode mode);

/**
 * @brief Deactivate PHY
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_deactivate(void);

/**
 * @brief Deinitialize PHY
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_deinit(void);

/**
 * @brief Schedule TX operation
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_tx(const struct nrf_modem_dect_phy_tx_params *params);

/**
 * @brief Schedule RX operation
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_rx(const struct nrf_modem_dect_phy_rx_params *params);

/**
 * @brief Schedule RSSI measurement
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_rssi(const struct nrf_modem_dect_phy_rssi_params *params);

/**
 * @brief Cancel operation
 * @see nrf_modem_dect_phy.h for full documentation
 */
int nrf_modem_dect_phy_cancel(uint32_t handle);

/* ========================================================================
 * NORDIC PHY API - STUB IMPLEMENTATIONS
 * 
 * These functions are present for API compatibility but not fully implemented.
 * They return success (0) but don't perform full simulation.
 * ======================================================================== */

int nrf_modem_dect_phy_configure(const struct nrf_modem_dect_phy_config_params *params);
int nrf_modem_dect_phy_tx_harq(const struct nrf_modem_dect_phy_tx_params *params);
int nrf_modem_dect_phy_tx_rx(const struct nrf_modem_dect_phy_tx_rx_params *params);
int nrf_modem_dect_phy_capability_get(void);
int nrf_modem_dect_phy_band_get(void);
int nrf_modem_dect_phy_time_get(void);
int nrf_modem_dect_phy_radio_config(const struct nrf_modem_dect_phy_radio_config_params *params);
int nrf_modem_dect_phy_link_config(const struct nrf_modem_dect_phy_link_config_params *params);
int nrf_modem_dect_phy_stf_cover_seq_control(bool rx_enable, bool tx_enable);
int nrf_modem_dect_phy_latency_get(void);

#endif /* MOCK_NRF_MODEM_DECT_PHY_H__ */