/* lib/dect_nrplus/tests/mocks/mock_nrf_modem_dect_phy.c */
/* It enhances the original by adding realistic duration calculations for TX operations based on MCS, 
* a functional implementation of `nrf_modem_dect_phy_tx_harq` that simulates ACK/NACK flipping, 
* and a handler for `nrf_modem_dect_phy_latency_get` to provide realistic timing values to the MAC layer. 
*/

/**
 * @file mock_nrf_modem_dect_phy.c
 * @brief An enhanced mock implementation of the nrf_modem_dect_phy.h API.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "mock_nrf_modem_dect_phy.h" // Include its own new header
#include <mac/nrf_modem_dect_phy.h>

/* -----------------------------------------------------------------------------
 * Mock-specific definitions and state
 * -----------------------------------------------------------------------------
 */

#define MOCK_TIMELINE_MAX_EVENTS 32
#define MOCK_RX_QUEUE_MAX_PACKETS 16
#define MOCK_PDC_MAX_SIZE 2048

typedef enum {
    PHY_STATE_DEINITIALIZED,
    PHY_STATE_INITIALIZED,
    PHY_STATE_ACTIVE,
} mock_phy_internal_state_t;

typedef struct {
    bool active;
    uint32_t handle;
    enum {
        OP_TYPE_TX,
        OP_TYPE_RX,
        OP_TYPE_RSSI,
    } type;
    uint64_t start_time_us;
    uint64_t end_time_us;
} scheduled_operation_t;

typedef struct {
    bool active;
    uint64_t reception_time_us;
    bool force_pcc_crc_error;
    bool force_pdc_crc_error;
    struct nrf_modem_dect_phy_pcc_event pcc_data;
    uint8_t pdc_payload[MOCK_PDC_MAX_SIZE];
    size_t pdc_len;
} mock_rx_packet_t;

typedef struct {
    mock_phy_internal_state_t state;
    uint64_t current_time_us;
    nrf_modem_dect_phy_event_handler_t event_handler;
    scheduled_operation_t timeline[MOCK_TIMELINE_MAX_EVENTS];
    mock_rx_packet_t rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];
    uint16_t transaction_id_counter;
} mock_phy_state_t;

static mock_phy_state_t g_mock_phy;

bool g_force_scheduling_conflict = false;
int8_t g_mock_lbt_rssi = -120;


static mock_scheduled_operation_t g_timeline[MOCK_TIMELINE_MAX_EVENTS];
static mock_rx_packet_t g_rx_queue[MOCK_RX_QUEUE_MAX_PACKETS];

/* -----------------------------------------------------------------------------
 * Forward declarations for internal functions
 * -----------------------------------------------------------------------------
 */
static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event);
static int find_free_timeline_slot(void);
static void process_timeline(uint64_t new_time_us);
static void process_rx_queue(uint64_t new_time_us);
static uint32_t mock_calculate_tx_duration_us(size_t pdu_len, uint8_t mcs);

/* -----------------------------------------------------------------------------
 * Mock control functions (for the test harness to use)
 * -----------------------------------------------------------------------------
 */
uint64_t mock_phy_get_time_us(void) { return g_mock_phy.current_time_us; }

mock_scheduled_operation_t* mock_phy_get_last_scheduled_op(void)
{
    // Simple implementation: returns the last active op in the timeline
    for (int i = MOCK_TIMELINE_MAX_EVENTS - 1; i >= 0; i--) {
        if (g_timeline[i].active) {
            return &g_timeline[i];
        }
    }
    return NULL;
}

void mock_phy_capture_last_tx_pdu(uint8_t *buf, uint16_t *len)
{
    mock_scheduled_operation_t *op = mock_phy_get_last_scheduled_op();
    if (op && op->type == MOCK_OP_TYPE_TX) {
        *len = op->pdu_len;
        memcpy(buf, op->pdu_data, op->pdu_len);
    } else {
        *len = 0;
    }
}

void mock_phy_reset(void) {
    memset(&g_mock_phy, 0, sizeof(g_mock_phy));
    g_mock_phy.state = PHY_STATE_DEINITIALIZED;
    g_force_scheduling_conflict = false;
    g_mock_lbt_rssi = -120;
    printf("[PHY Mock] State reset.\n");
}

void mock_phy_advance_time_us(uint32_t time_to_advance_us) {
    if (g_mock_phy.state == PHY_STATE_DEINITIALIZED) return;
    uint64_t new_time = g_mock_phy.current_time_us + time_to_advance_us;
    process_timeline(new_time);
    process_rx_queue(new_time);
    g_mock_phy.current_time_us = new_time;
}

void mock_phy_advance_time_to_us(uint64_t target_time_us) {
    if (g_mock_phy.state == PHY_STATE_DEINITIALIZED || target_time_us <= g_mock_phy.current_time_us) return;
    process_timeline(target_time_us);
    process_rx_queue(target_time_us);
    g_mock_phy.current_time_us = target_time_us;
}

int mock_phy_queue_rx_packet(const mock_rx_packet_t *packet) {
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
        if (!g_mock_phy.rx_queue[i].active) {
            g_mock_phy.rx_queue[i] = *packet;
            g_mock_phy.rx_queue[i].active = true;
            printf("[PHY Mock] Queued RX packet to be received at %lluus.\n", (unsigned long long)packet->reception_time_us);
            return 0;
        }
    }
    printf("[PHY Mock] ERROR: RX packet queue is full.\n");
    return -1;
}

/* -----------------------------------------------------------------------------
 * Mock implementation of the nrf_modem_dect_phy.h API
 * -----------------------------------------------------------------------------
 */

int nrf_modem_dect_phy_event_handler_set(nrf_modem_dect_phy_event_handler_t handler) {
    g_mock_phy.event_handler = handler;
    printf("[PHY Mock] Event handler set.\n");
    return 0;
}

int nrf_modem_dect_phy_init(void) {
    if (g_mock_phy.state != PHY_STATE_DEINITIALIZED) return -1;
    mock_phy_reset();
    g_mock_phy.state = PHY_STATE_INITIALIZED;
    printf("[PHY Mock] State -> INITIALIZED.\n");
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_INIT, .time = g_mock_phy.current_time_us,
        .init = { .err = NRF_MODEM_DECT_PHY_SUCCESS, .temp = 25, .voltage = 3300, .temperature_limit = 85, }
    };
    mock_phy_send_event(&event);
    return 0;
}

int nrf_modem_dect_phy_configure(const struct nrf_modem_dect_phy_config_params *params) {
    if (g_mock_phy.state != PHY_STATE_INITIALIZED) return -1;
    printf("[PHY Mock] PHY configured.\n");
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_CONFIGURE, .time = g_mock_phy.current_time_us,
        .configure = { .err = NRF_MODEM_DECT_PHY_SUCCESS }
    };
    mock_phy_send_event(&event);
    return 0;
}

int nrf_modem_dect_phy_activate(enum nrf_modem_dect_phy_radio_mode mode) {
    if (g_mock_phy.state != PHY_STATE_INITIALIZED) return -1;
    g_mock_phy.state = PHY_STATE_ACTIVE;
    printf("[PHY Mock] State -> ACTIVE.\n");
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_ACTIVATE, .time = g_mock_phy.current_time_us,
        .activate = { .err = NRF_MODEM_DECT_PHY_SUCCESS }
    };
    mock_phy_send_event(&event);
    return 0;
}

int nrf_modem_dect_phy_deactivate(void) {
    if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
    g_mock_phy.state = PHY_STATE_INITIALIZED;
    printf("[PHY Mock] State -> INITIALIZED.\n");
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_DEACTIVATE, .time = g_mock_phy.current_time_us,
        .deactivate = { .err = NRF_MODEM_DECT_PHY_SUCCESS }
    };
    mock_phy_send_event(&event);
    return 0;
}

int nrf_modem_dect_phy_deinit(void) {
    g_mock_phy.state = PHY_STATE_DEINITIALIZED;
    printf("[PHY Mock] State -> DEINITIALIZED.\n");
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_DEINIT, .time = g_mock_phy.current_time_us,
        .deinit = { .err = NRF_MODEM_DECT_PHY_SUCCESS }
    };
    mock_phy_send_event(&event);
    return 0;
}

int nrf_modem_dect_phy_tx(const struct nrf_modem_dect_phy_tx_params *params) {
    if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
    if (params->lbt_period > 0) {
        if (g_mock_lbt_rssi >= params->lbt_rssi_threshold_max) {
            struct nrf_modem_dect_phy_event event = {
                .id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .time = g_mock_phy.current_time_us,
                .op_complete = { .handle = params->handle, .err = NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY }
            };
            mock_phy_send_event(&event);
            return 0;
        }
    }
    int slot = find_free_timeline_slot();
    if (slot < 0) return -1;
        g_timeline[slot].type = MOCK_OP_TYPE_TX; // Use the mock enum
        // Store the PDU data
        g_timeline[slot].pdu_len = params->data_size + 1; // +1 for MAC Hdr Type
        g_timeline[slot].pdu_data[0] = 0; // Placeholder for MAC Hdr Type
        memcpy(&g_timeline[slot].pdu_data[1], params->data, params->data_size);
        
        uint8_t mcs = params->phy_header->hdr_type_1.df_mcs;
        uint64_t duration_us = mock_calculate_tx_duration_us(params->data_size, mcs);
        g_mock_phy.timeline[slot] = (scheduled_operation_t){
            .active = true, .handle = params->handle, .type = OP_TYPE_TX,
            .start_time_us = params->start_time, .end_time_us = params->start_time + duration_us
    };
    printf("[PHY Mock] TX scheduled with handle %u at %lluus for %lluus.\n", params->handle, (unsigned long long)params->start_time, (unsigned long long)duration_us);
    return 0;
}

int nrf_modem_dect_phy_tx_harq(const struct nrf_modem_dect_phy_tx_params *params) {
    printf("[PHY Mock] TX HARQ called. Simulating ACK/NACK flip.\n");
    /* This is a simplified simulation. A real PHY would check the CRC of the received PDC
     * that this HARQ is responding to. We'll just assume it passes. */
    union nrf_modem_dect_phy_hdr *hdr = params->phy_header;
    hdr->hdr_type_2.feedback.format1.transmission_feedback0 = 1; /* Flip to ACK */
    return nrf_modem_dect_phy_tx(params);
}

int nrf_modem_dect_phy_rx(const struct nrf_modem_dect_phy_rx_params *params) {
    if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
    int slot = find_free_timeline_slot();
    if (slot < 0) return -1;
    g_mock_phy.timeline[slot] = (scheduled_operation_t){
        .active = true, .handle = params->handle, .type = OP_TYPE_RX,
        .start_time_us = params->start_time, .end_time_us = params->start_time + params->duration
    };
    printf("[PHY Mock] RX scheduled with handle %u from %lluus to %lluus.\n", params->handle, (unsigned long long)params->start_time, (unsigned long long)g_mock_phy.timeline[slot].end_time_us);
    return 0;
}

int nrf_modem_dect_phy_cancel(uint32_t handle) {
    if (g_mock_phy.state != PHY_STATE_ACTIVE) return -1;
    bool found = false;
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        if (g_mock_phy.timeline[i].active && g_mock_phy.timeline[i].handle == handle) {
            g_mock_phy.timeline[i].active = false;
            found = true;
            struct nrf_modem_dect_phy_event op_event = {
                .id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .time = g_mock_phy.current_time_us,
                .op_complete = { .handle = handle, .err = NRF_MODEM_DECT_PHY_ERR_OP_CANCELED }
            };
            mock_phy_send_event(&op_event);
            break;
        }
    }
    struct nrf_modem_dect_phy_event cancel_event = {
        .id = NRF_MODEM_DECT_PHY_EVT_CANCELED, .time = g_mock_phy.current_time_us,
        .cancel = { .handle = handle, .err = found ? NRF_MODEM_DECT_PHY_SUCCESS : NRF_MODEM_DECT_PHY_ERR_NOT_FOUND }
    };
    mock_phy_send_event(&cancel_event);
    printf("[PHY Mock] Cancel requested for handle %u. Found: %s\n", handle, found ? "yes" : "no");
    return 0;
}

int nrf_modem_dect_phy_latency_get(void) {
    struct nrf_modem_dect_phy_latency_info info = {
        .radio_mode[NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY_WITH_STANDBY] = {
            .scheduled_operation_startup = 200,
            .scheduled_operation_transition = 100,
        },
        .operation.receive.idle_to_active = 500,
        .operation.transmit.idle_to_active = 500,
    };
    struct nrf_modem_dect_phy_event event = {
        .id = NRF_MODEM_DECT_PHY_EVT_LATENCY, .time = g_mock_phy.current_time_us,
        .latency_get = { .err = NRF_MODEM_DECT_PHY_SUCCESS, .latency_info = &info }
    };
    mock_phy_send_event(&event);
    return 0;
}

/* Dummy implementations for other functions */
int nrf_modem_dect_phy_tx_rx(const struct nrf_modem_dect_phy_tx_rx_params *params) { return 0; }
int nrf_modem_dect_phy_rssi(const struct nrf_modem_dect_phy_rssi_params *params) { return 0; }
int nrf_modem_dect_phy_capability_get(void) { return 0; }
int nrf_modem_dect_phy_band_get(void) { return 0; }
int nrf_modem_dect_phy_time_get(void) { return 0; }
int nrf_modem_dect_phy_radio_config(const struct nrf_modem_dect_phy_radio_config_params *params) { return 0; }
int nrf_modem_dect_phy_link_config(const struct nrf_modem_dect_phy_link_config_params *params) { return 0; }
int nrf_modem_dect_phy_stf_cover_seq_control(bool rx_enable, bool tx_enable) { return 0; }

/* -----------------------------------------------------------------------------
 * Internal mock helper functions
 * -----------------------------------------------------------------------------
 */

static void mock_phy_send_event(const struct nrf_modem_dect_phy_event *event) {
    if (g_mock_phy.event_handler) {
        g_mock_phy.event_handler(event);
    }
}

static int find_free_timeline_slot(void) {
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        if (!g_mock_phy.timeline[i].active) {
            return i;
        }
    }
    return -1;
}

static void process_timeline(uint64_t new_time_us) {
    for (int i = 0; i < MOCK_TIMELINE_MAX_EVENTS; ++i) {
        scheduled_operation_t *op = &g_mock_phy.timeline[i];
        if (!op->active) continue;
        if (op->end_time_us > g_mock_phy.current_time_us && op->end_time_us <= new_time_us) {
            struct nrf_modem_dect_phy_event event = {
                .id = NRF_MODEM_DECT_PHY_EVT_COMPLETED, .time = op->end_time_us,
                .op_complete = { .handle = op->handle, .err = NRF_MODEM_DECT_PHY_SUCCESS }
            };
            mock_phy_send_event(&event);
            op->active = false;
        }
    }
}

static void process_rx_queue(uint64_t new_time_us) {
    for (int i = 0; i < MOCK_RX_QUEUE_MAX_PACKETS; ++i) {
        mock_rx_packet_t *pkt = &g_mock_phy.rx_queue[i];
        if (!pkt->active) continue;
        if (pkt->reception_time_us > g_mock_phy.current_time_us && pkt->reception_time_us <= new_time_us) {
            for (int j = 0; j < MOCK_TIMELINE_MAX_EVENTS; ++j) {
                scheduled_operation_t *rx_op = &g_mock_phy.timeline[j];
                if (rx_op->active && rx_op->type == OP_TYPE_RX &&
                    pkt->reception_time_us >= rx_op->start_time_us &&
                    pkt->reception_time_us <= rx_op->end_time_us) {
                    uint16_t transaction_id = g_mock_phy.transaction_id_counter++;
                    struct nrf_modem_dect_phy_event event;
                    if (pkt->force_pcc_crc_error) {
                        event.id = NRF_MODEM_DECT_PHY_EVT_PCC_ERROR;
                        event.pcc_crc_err = (struct nrf_modem_dect_phy_pcc_crc_failure_event){.handle = rx_op->handle, .transaction_id = transaction_id};
                    } else {
                        event.id = NRF_MODEM_DECT_PHY_EVT_PCC;
                        event.pcc = pkt->pcc_data;
                        event.pcc.handle = rx_op->handle;
                        event.pcc.transaction_id = transaction_id;
                    }
                    mock_phy_send_event(&event);
                    if (pkt->pdc_len > 0 && !pkt->force_pcc_crc_error) {
                         if (pkt->force_pdc_crc_error) {
                            event.id = NRF_MODEM_DECT_PHY_EVT_PDC_ERROR;
                            event.pdc_crc_err = (struct nrf_modem_dect_phy_pdc_crc_failure_event){.handle = rx_op->handle, .transaction_id = transaction_id};
                         } else {
                            event.id = NRF_MODEM_DECT_PHY_EVT_PDC;
                            event.pdc = (struct nrf_modem_dect_phy_pdc_event){.handle = rx_op->handle, .transaction_id = transaction_id, .data = pkt->pdc_payload, .len = pkt->pdc_len};
                         }
                         mock_phy_send_event(&event);
                    }
                    pkt->active = false;
                    break;
                }
            }
        }
    }
}

static uint32_t mock_calculate_tx_duration_us(size_t pdu_len, uint8_t mcs)
{
    /* Simplified calculation based on a fictional bitrate for each MCS */
    uint32_t bits_per_second = 500000 + (mcs * 100000); /* e.g., 500kbps for MCS0 */
    uint64_t total_bits = (uint64_t)pdu_len * 8;
    return (total_bits * 1000000) / bits_per_second;
}