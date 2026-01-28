/* lib/dect_nrplus/tests/mocks/mock_nrf_modem_dect_phy.h */
#ifndef MOCK_NRF_MODEM_DECT_PHY_H_
#define MOCK_NRF_MODEM_DECT_PHY_H_

#include <stdbool.h>
#include <zephyr/kernel.h> // For k_fifo, k_timeout_t
#include <stdint.h>        // For uintxx_t types
#include <stddef.h>        // For size_t
#include <mac/nrf_modem_dect_phy.h>

/* --- Public Mock Data Structures --- */

typedef enum {
    MOCK_OP_TYPE_TX,
    MOCK_OP_TYPE_RX,
    MOCK_OP_TYPE_RSSI,
} mock_op_type_t;

typedef struct {
    bool active;
    uint32_t handle;
    mock_op_type_t type;
    uint64_t start_time_us;
    uint64_t end_time_us;
    uint8_t pdu_data[2048]; // Store the PDU data for this op
    uint16_t pdu_len;
} mock_scheduled_operation_t;

typedef struct {
    bool active;
    uint64_t reception_time_us;
    struct nrf_modem_dect_phy_pcc_event pcc_data;
    uint8_t pdc_payload[2048];
    size_t pdc_len;
} mock_rx_packet_t;

/**
 * @brief Global flag to force a scheduling conflict in the mock PHY.
 */
extern bool g_force_scheduling_conflict;

/**
 * @brief Global variable to set the mock LBT RSSI value (in dBm).
 */
extern int8_t g_mock_lbt_rssi;

/**
 * @brief Reset the mock PHY to its initial state.
 *
 * Clears all internal state, resets the timeline and RX queue, and sets the PHY state
 * to deinitialized. Used to prepare for a new test case.
 */
void mock_phy_reset(void);

/**
 * @brief Advance the mock PHY's internal clock by a specified duration.
 *
 * Processes any scheduled operations or queued RX packets that fall within the
 * time window up to the new time.
 *
 * @param time_to_advance_us Duration to advance the clock (in microseconds).
 */
void mock_phy_advance_time_us(uint32_t time_to_advance_us);

/**
 * @brief Advance the mock PHY's internal clock to a specific time.
 *
 * Processes any scheduled operations or queued RX packets that fall within the
 * time window up to the target time.
 *
 * @param target_time_us Target time to advance to (in microseconds).
 */
void mock_phy_advance_time_to_us(uint64_t target_time_us);

/**
 * @brief Queue an RX packet for the mock PHY to process at a specified time.
 *
 * The packet will be processed when the mock PHY's internal clock reaches or exceeds
 * the packet's reception time, provided it aligns with an active RX operation.
 *
 * @param packet Pointer to the RX packet to queue.
 * @return 0 on success, negative error code if the RX queue is full.
 */
int mock_phy_queue_rx_packet(const mock_rx_packet_t *packet);


uint64_t mock_phy_get_time_us(void);
mock_scheduled_operation_t* mock_phy_get_last_scheduled_op(void);
void mock_phy_capture_last_tx_pdu(uint8_t *buf, uint16_t *len);


#endif /* MOCK_NRF_MODEM_DECT_PHY_H_ */