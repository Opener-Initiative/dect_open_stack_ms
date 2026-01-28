/* dect_mac/dect_mac_mgmt.h */
#ifndef DECT_MAC_MGMT_H__
#define DECT_MAC_MGMT_H__

#include <stdint.h> // For uint8_t
#include <stddef.h> // For size_t

/**
 * @brief Callback function prototype for the MAC management service.
 *
 * The MAC layer will invoke this callback when it receives a PDU that has been
 * identified as a management PDU (e.g., based on a specific MAC IE Type or
 * a pre-agreed identifier within a user data flow that is not handled by DLC/CVG).
 * This allows an upper-level management entity (part of the application or a separate module)
 * to process these frames for purposes like Over-The-Air (OTA) firmware updates, remote
 * device configuration, diagnostics, etc.
 *
 * @param data Pointer to the buffer containing the payload of the management PDU.
 *             This data is typically the content of the management IE or message,
 *             after MAC headers (and potentially security) have been processed and removed.
 * @param len Length of the management PDU payload in `data`.
 * @param source_long_rd_id The Long RD ID of the device that sent the management PDU.
 */
typedef void (*mgmt_service_callback_t)(const uint8_t *data, size_t len, uint32_t source_long_rd_id);

/**
 * @brief Registers the management service callback function with the MAC layer.
 *
 * The application's management entity should call this function once during its
 * initialization sequence if it needs to receive and process management frames
 * passed up by the MAC layer. Only one management callback can be registered.
 * Subsequent calls will overwrite the previous registration.
 *
 * @param cb The callback function to be registered. If NULL, it effectively
 *           unregisters any previously set callback.
 */
void dect_mac_mgmt_service_register_callback(mgmt_service_callback_t cb);

/**
 * @brief Initializes the MAC management service module.
 *
 * This function should be called once during the overall DECT stack initialization.
 * It prepares the management service for operation, primarily by clearing any
 * previously registered callback.
 */
void dect_mac_mgmt_service_init(void);


#endif /* DECT_MAC_MGMT_H__ */