/* lib/dect_nrplus/mac/include/dect_mac_random.h */
/* Overview: New header file to abstract random number generation, allowing the MAC to build without a crypto backend if security is disabled.
--- CREATE NEW FILE --- */
#ifndef DECT_MAC_RANDOM_H__
#define DECT_MAC_RANDOM_H__

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Fills a buffer with random bytes.
 *
 * This function is an abstraction over the system's random number generator.
 * If MAC security is enabled (CONFIG_DECT_MAC_SECURITY_ENABLE), it will use
 * the cryptographically secure PSA API. Otherwise, it will fall back to the
 * standard (potentially non-secure) Zephyr random API.
 *
 * @param buf Pointer to the buffer to fill.
 * @param len Number of random bytes to generate.
 */
void dect_mac_rand_get(uint8_t *buf, size_t len);

#endif /* DECT_MAC_RANDOM_H__ */