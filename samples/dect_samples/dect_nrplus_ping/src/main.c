/* samples/dect_nrplus_ping/src/main.c */
// Overview: The main application entry point for the ping sample. It uses only the clean, public stack API to initialize and start the DECT NR+ stack, then yields to the network shell.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>


/* This is the ONLY header needed to interact with the DECT NR+ stack */
#include <dect_stack_api.h>
// #include "../../../lib/dect_nrplus/include/dect_stack_api.h"

// #include <nrf_modem_dect_phy.h>
// #include <modem/nrf_modem_lib.h>


LOG_MODULE_REGISTER(dect_ping_sample, LOG_LEVEL_INF);

static void stack_state_cb(dect_mac_public_state_t new_state)
{
	if (new_state == MAC_STATE_PUB_ASSOCIATED) {
		LOG_INF("DECT NR+ Link is UP. Network interface is ready.");
		LOG_INF("Use the 'net' shell command to configure IP addresses and ping.");
	} else {
		LOG_WRN("DECT NR+ Link is DOWN.");
	}
}

int main(void)
{
	LOG_INF("Starting DECT NR+ Ping Sample...");
	printk("Starting DECT NR+ Ping Sample...");

	int err = dect_stack_init();
	if (err) {
		LOG_ERR("Failed to initialize DECT NR+ stack: %d", err);
		printk("Failed to initialize DECT NR+ stack: %d", err);
		return 0;
	}

	dect_stack_register_state_change_cb(stack_state_cb);

#if IS_ENABLED(CONFIG_DECT_MAC_ROLE_IS_FT)
	struct in6_addr ft_prefix;

	err = net_addr_pton(AF_INET6, "2001:db8:cafe::", &ft_prefix);
	if (err) {
		LOG_ERR("Invalid prefix string");
		printk("Invalid prefix string");
		return 0;
	}

	/* dect_cdd_ft_build_own_config is a public API function included via dect_stack_api.h */
	err = dect_cdd_ft_build_own_config(&ft_prefix);
	if (err) {
		LOG_ERR("FT failed to build CDD config: %d", err);
		printk("FT failed to build CDD config: %d", err);
		return 0;
	}
#endif

	dect_stack_start();

	/* The main thread can now sleep, as all operations are handled by the
	 * DECT stack threads and the Zephyr network shell.
	 */
	return 0;
}