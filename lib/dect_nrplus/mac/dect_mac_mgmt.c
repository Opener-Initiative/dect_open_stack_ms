/* dect_mac/dect_mac_mgmt.c */
#include <zephyr/logging/log.h>
#include <mac/dect_mac_mgmt.h>

LOG_MODULE_REGISTER(dect_mac_mgmt, CONFIG_DECT_MAC_MGMT_LOG_LEVEL);

// Static variable to hold the registered management service callback.
static mgmt_service_callback_t g_mgmt_callback = NULL;

void dect_mac_mgmt_service_init(void)
{
    g_mgmt_callback = NULL;
    LOG_INF("DECT MAC Management Service Initialized (callback cleared).");
}

void dect_mac_mgmt_service_register_callback(mgmt_service_callback_t cb)
{
    if (cb != NULL) {
        LOG_INF("Registering MAC management service callback.");
    } else {
        LOG_INF("Unregistering MAC management service callback.");
    }
    g_mgmt_callback = cb;
}

/**
 * @brief Internal function called by MAC layer to pass up a received management PDU.
 *
 * This function is not part of the public API in dect_mac_mgmt.h but would be
 * called by the MAC's PDU processing logic (e.g., in a state machine's PDC handler)
 * when a PDU identified as management data is successfully received and processed
 * (e.g., after security checks).
 *
 * @param data Pointer to the management PDU payload.
 * @param len Length of the payload.
 * @param source_long_rd_id Long RD ID of the sender.
 */
void dect_mac_mgmt_deliver_pdu(const uint8_t *data, size_t len, uint32_t source_long_rd_id)
{
    if (g_mgmt_callback != NULL) {
        LOG_DBG("Delivering management PDU (len %zu from 0x%08X) to registered callback.", len, source_long_rd_id);
        g_mgmt_callback(data, len, source_long_rd_id);
    } else {
        LOG_WRN("Received management PDU (len %zu from 0x%08X), but no management callback registered. PDU dropped.", len, source_long_rd_id);
    }
}