/* lib/dect_nrplus/mac/dect_mac_random.c */
// Overview: New file to provide a conditional random number generation implementation.
#include <zephyr/random/random.h>
#include <mac/dect_mac_random.h>
#include <zephyr/logging/log.h>

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
#include <psa/crypto.h>
#endif

LOG_MODULE_REGISTER(dect_mac_random, LOG_LEVEL_INF);

void dect_mac_rand_get(uint8_t *buf, size_t len)
{
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
	/* Use cryptographically secure generator when security is enabled */
	psa_status_t status = psa_generate_random(buf, len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("PSA random generation failed! Status: %d. Falling back to non-secure RNG.",
			(int)status);
		sys_rand_get(buf, len);
	}
#else
	/* Fallback to standard (potentially non-secure) generator */
	sys_rand_get(buf, len);
#endif
}