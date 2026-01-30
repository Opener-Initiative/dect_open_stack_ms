/* include/mocks/mock_nrf_modem_dect_phy.h */
/**
 * @file mock_nrf_modem_dect_phy.h
 * @brief Enhanced Mock PHY for Nordic DECT NR+ with improved realism
 * 
 * This mock PHY provides a realistic simulation of the Nordic DECT NR+ modem
 * while using microseconds for timing to aid development and testing.
 * 
 * Key Features:
 * - Proper PHY state machine matching Nordic hardware
 * - Realistic operation lifecycle and event sequencing
 * - Hardware latency simulation
 * - Multi-device communication support
 * - Scheduling conflict detection
 * - Enhanced debugging and statistics
 */

#ifndef MOCK_NRF_MODEM_DECT_PHY_H__
#define MOCK_NRF_MODEM_DECT_PHY_H__

#include <mac/nrf_modem_dect_phy.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================
 * CONFIGURATION
 * ======================================================================== */

/** @brief Maximum number of concurrent scheduled operations */
#define MOCK_TIMELINE_MAX_EVENTS 64

/** @brief Maximum number of packets that can be queued for reception */
#define MOCK_RX_QUEUE_MAX_PACKETS 32

/* ========================================================================
 * GLOBAL TEST CONTROL FLAGS
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
 * TYPE DEFINITIONS
 * ======================================================================== */

/* Forward declaration for self-referential peer pointers */
struct mock_phy_context_t;

/**
 * @brief PHY internal states
 * 
 * These states mirror the Nordic DECT NR+ PHY state machine:
 * DEINITIALIZED → INITIALIZED → ACTIVE → INITIALIZED → DEINITIALIZED
 */
typedef enum {
    PHY_STATE_DEINITIALIZED,  /**< PHY not initialized, no handler set */
    PHY_STATE_INITIALIZED,    /**< Event handler set, not activated */
    PHY_STATE_ACTIVE,         /**< PHY activated, can schedule operations */
} mock_phy_internal_state_t;

/**
 * @brief Types of PHY operations
 */
typedef enum {
    MOCK_OP_TYPE_NONE = 0,
    MOCK_OP_TYPE_TX,       /**< Transmission operation */
    MOCK_OP_TYPE_RX,       /**< Reception operation */
    MOCK_OP_TYPE_RSSI,     /**< RSSI measurement operation */
    MOCK_OP_TYPE_TX_RX,    /**< Combined TX+RX operation */
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
typedef struct {
    /* Basic state */
    bool active;                    /**< Slot is in use */
    bool running;                   /**< Operation is executing (legacy compat) */
    operation_state_t state;        /**< Detailed lifecycle state */
    
    /* Operation identification */
    uint32_t handle;                /**< User-provided handle */
    mock_op_type_t type;            /**< Operation type */
    uint64_t transaction_id;        /**< Internal transaction ID */
    
    /* Timing (all in microseconds) */
    uint64_t scheduled_time_us;     /**< When operation was scheduled */
    uint64_t start_time_us;         /**< When execution starts */
    uint64_t end_time_us;           /**< When execution ends */
    uint64_t duration_us;           /**< Expected duration */
    
    /* RF parameters */
    uint16_t carrier;               /**< RF carrier frequency */
    uint16_t network_id;            /**< Network ID */
    
    /* TX-specific data */
    uint8_t pdu_data[CONFIG_DECT_MAC_PDU_MAX_SIZE];  /**< TX payload */
    uint16_t pdu_len;               /**< TX payload length */
    
    /* RX-specific data */
    bool rx_filter_enabled;         /**< Whether RX filter is active */
    uint32_t rx_filter;             /**< RX filter value (network ID) */
    
    /* Error tracking */
    enum nrf_modem_dect_phy_err error_code;  /**< Error code if failed */
    
    /* Context for MAC layer */
    void *context;                  /**< User context pointer */
    
} mock_scheduled_operation_t;

/**
 * @brief RX packet structure
 * 
 * Represents a packet queued for reception. Used in multi-device simulation
 * to pass packets from one device's TX to another device's RX.
 */
typedef struct {
    bool active;                    /**< Packet is in queue */
    uint64_t reception_time_us;     /**< When packet should be received */
    uint16_t carrier;               /**< RF carrier */
    uint16_t network_id;            /**< Network ID */
    
    /* Packet data */
    struct nrf_modem_dect_phy_pcc_event pcc_data;  /**< PCC event data */
    uint8_t pdc_payload[CONFIG_DECT_MAC_PDU_MAX_SIZE];  /**< Packet payload */
    size_t pdc_len;                 /**< Payload length */
    
    /* Reception quality (for realistic simulation) */
    int8_t rssi_dbm;                /**< Received signal strength */
    int8_t snr_db;                  /**< Signal-to-noise ratio */
    
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
 * @brief PHY context structure
 * 
 * Represents the complete state of a single virtual PHY instance.
 * Supports multi-device simulation through peer connections.
 */
typedef struct mock_phy_context_t {
    /* State management */
    mock_phy_internal_state_t state;        /**< Current PHY state */
    uint64_t state_change_time_us;          /**< When state last changed */
    
    /* MAC layer connection */
    struct dect_mac_context *mac_ctx;       /**< Associated MAC context */
    
    /* Multi-device simulation */
    struct mock_phy_context_t **peers;      /**< Array of peer PHY contexts */
    size_t num_peers;                       /**< Number of peers */
    
    /* Operation scheduling */
    mock_scheduled_operation_t timeline[MOCK_TIMELINE_MAX_EVENTS];  /**< Scheduled operations */
    uint32_t next_handle;                   /**< Next auto-generated handle */
    uint16_t transaction_id_counter;        /**< Transaction ID counter */
    
    /* RX operation tracking (for packet matching) */
    mock_scheduled_operation_t *active_rx_ops[MOCK_TIMELINE_MAX_EVENTS];  /**< Active RX ops */
    size_t num_active_rx_ops;               /**< Number of active RX ops */
    
    /* RX packet queue */
    mock_rx_packet_t rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];  /**< Queued packets */
    
    /* Current RF state */
    uint32_t current_carrier;               /**< Current carrier frequency */
    enum nrf_modem_dect_phy_radio_mode current_mode;  /**< Current radio mode */
    
    /* Statistics */
    mock_phy_stats_t stats;                 /**< Operation statistics */
    
} mock_phy_context_t;

/* ========================================================================
 * PUBLIC API - MOCK CONTROL FUNCTIONS
 * ======================================================================== */

/**
 * @brief Initialize PHY context
 * 
 * Initializes a PHY context for use in testing. The context must be initialized
 * before any Nordic PHY API calls are made.
 * 
 * @param ctx PHY context to initialize
 * @param mac_ctx Associated MAC context (can be NULL for testing)
 * @param peers Array of peer PHY contexts for multi-device simulation (can be NULL)
 * @param num_peers Number of peer contexts in the array
 * 
 * @note After initialization, state is PHY_STATE_DEINITIALIZED.
 *       Call nrf_modem_dect_phy_event_handler_set() to move to INITIALIZED.
 */
void mock_phy_init_context(mock_phy_context_t *ctx, 
                           struct dect_mac_context *mac_ctx,
                           mock_phy_context_t **peers, 
                           size_t num_peers);

/**
 * @brief Set the active PHY context
 * 
 * The active context is used for all subsequent Nordic PHY API calls
 * (nrf_modem_dect_phy_tx, _rx, etc.) until a different context is made active.
 * 
 * This allows testing multi-device scenarios by switching between contexts.
 * 
 * @param ctx PHY context to make active
 */
void mock_phy_set_active_context(mock_phy_context_t *ctx);

/**
 * @brief Reset PHY context to initial state
 * 
 * Cancels all scheduled operations, clears all queued packets, and resets
 * the PHY to PHY_STATE_DEINITIALIZED.
 * 
 * @param ctx PHY context to reset
 */
void mock_phy_complete_reset(mock_phy_context_t *ctx);

/**
 * @brief Queue RX packet for delivery
 * 
 * Queues a packet to be delivered to a matching RX operation on the destination
 * PHY context. Used in multi-device simulation to pass packets between devices.
 * 
 * When an RX operation is running that matches the packet's carrier and filter
 * criteria, the packet will be delivered via an RX_PDC event.
 * 
 * @param dest_ctx Destination PHY context
 * @param packet Packet to queue
 * @return 0 on success, -EINVAL if parameters invalid, -ENOMEM if queue full
 * 
 * @note The packet is copied into the queue, so the caller can free/reuse
 *       the packet structure after this call returns.
 */
int mock_phy_queue_rx_packet(mock_phy_context_t *dest_ctx,
                              const mock_rx_packet_t *packet);

/**
 * @brief Get next event time across multiple PHY contexts
 * 
 * Returns the earliest time at which any of the provided PHY contexts has
 * a pending event (operation start, operation end, or packet reception).
 * 
 * This is used by test frameworks to efficiently advance simulation time
 * to the next interesting moment.
 * 
 * @param phy_contexts Array of PHY context pointers
 * @param num_contexts Number of contexts in the array
 * @return Next event time in microseconds, or UINT64_MAX if no events pending
 * 
 * @example
 * mock_phy_context_t *contexts[] = { &device_a, &device_b };
 * uint64_t next = mock_phy_get_next_event_time(contexts, 2);
 * if (next != UINT64_MAX) {
 *     k_sleep(K_USEC(next - current_time));
 * }
 */
uint64_t mock_phy_get_next_event_time(mock_phy_context_t *const phy_contexts[],
                                       size_t num_contexts);

/**
 * @brief Process pending events for a PHY context
 * 
 * Processes all events that are ready at the specified time:
 * - Transitions operations through their lifecycle (SCHEDULED→RUNNING→COMPLETING)
 * - Generates PHY events (TX_DONE, RX_PDC, RSSI, OP_COMPLETE, etc.)
 * - Matches queued RX packets to active RX operations
 * - Simulates packet transmission to peer devices
 * 
 * Should be called when simulation time reaches an event time returned by
 * mock_phy_get_next_event_time(), or periodically in real-time tests.
 * 
 * @param ctx PHY context to process
 * @param current_time_us Current simulation time in microseconds
 * 
 * @note This function may generate multiple events via the registered event handler.
 *       Make sure the active context is set correctly before calling.
 * 
 * @example
 * uint64_t current = k_ticks_to_us_floor64(k_uptime_ticks());
 * mock_phy_process_events(&phy_ctx, current);
 */
void mock_phy_process_events(mock_phy_context_t *ctx, uint64_t current_time_us);

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
