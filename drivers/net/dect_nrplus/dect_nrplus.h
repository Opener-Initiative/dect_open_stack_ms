/* drivers/net/dect_nrplus.h */
#ifndef ZEPHYR_DRIVERS_NET_DECT_NRPLUS_H_
#define ZEPHYR_DRIVERS_NET_DECT_NRPLUS_H_

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/kernel.h> 
#include <dect_cvg.h> /* For cvg_service_type_t */

struct dect_nrplus_dev_ctx {
	struct net_if *iface;
	cvg_service_type_t cvg_service_type;
	struct k_thread rx_thread_data;
	k_tid_t rx_thread_id;
	volatile bool shutdown_req;
	bool iid_is_set;
	k_thread_stack_t *rx_stack;
};

/**
 * @brief Sets a 6LoWPAN context for the DECT NR+ interface.
 *
 * This function is called by a higher-level management entity (like a future
 * CDD service) to configure a prefix for 6LoWPAN header compression.
 *
 * @param dev The DECT NR+ device instance.
 * @param cid The Context ID (0-15).
 * @param prefix Pointer to the IPv6 prefix.
 * @return 0 on success, negative error code on failure.
 */
int dect_nrplus_l2_set_sixlowpan_context(const struct device *dev, uint8_t cid,
					const struct in6_addr *prefix);

#endif /* ZEPHYR_DRIVERS_NET_DECT_NRPLUS_H_ */