#ifndef DECT_MAC_NVS_H__
#define DECT_MAC_NVS_H__

#include <stdint.h>

/**
 * @brief Initializes the MAC NVS module and loads the persisted HPC.
 *
 * This should be called once during MAC core initialization. It mounts the NVS
 * filesystem partition and attempts to load the last saved HPC value.
 * If no value is found, it defaults to 1.
 *
 * @return 0 on success, or a negative error code on failure to init NVS.
 */
int dect_mac_nvs_init(void);

/**
 * @brief Gets the last saved/loaded HPC value from the NVS module.
 *
 * @return The loaded HPC value.
 */
uint32_t dect_mac_nvs_get_hpc(void);

/**
 * @brief Saves the current HPC value to non-volatile storage.
 *
 * This function should be called whenever the HPC is incremented to ensure
 * it persists across reboots, which is critical for security.
 *
 * @param hpc The HPC value to save.
 * @return 0 on success, or a negative error code on failure.
 */
int dect_mac_nvs_save_hpc(uint32_t hpc);

#endif /* DECT_MAC_NVS_H__ */