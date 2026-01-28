/* lib/dect_nrplus/dect_stack_api.c */
// Overview: Implements the new, abstracted public API for the DECT NR+ stack, providing a clean separation between the application and internal stack layers.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dect_stack_api.h>
#include <dect_cvg.h>
#include <mac/dect_mac.h>

LOG_MODULE_REGISTER(dect_stack_api, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Registers a callback to be notified of stack state changes.
 *
 * This is a simple wrapper around the internal MAC layer's callback registration.
 */
void dect_stack_register_state_change_cb(dect_stack_state_change_cb_t cb)
{
	dect_mac_register_state_change_cb(cb);
}

/**
 * @brief Initializes the entire DECT NR+ protocol stack.
 *
 * This function calls the top-most layer of the internal stack (CVG), which
 * is responsible for initializing all layers below it in the correct order.
 */
int dect_stack_init(void)
{
	int err;

	/* The CVG layer initializes the layers below it (DLC -> MAC) */
	err = dect_cvg_init();
	if (err) {
		LOG_ERR("Failed to initialize the DECT NR+ stack (CVG layer): %d", err);
		return err;
	}

	LOG_INF("DECT NR+ Stack Initialized.");
	return 0;
}

/**
 * @brief Starts the DECT NR+ stack's operation.
 *
 * This is a wrapper around the internal MAC layer's start function.
 */
void dect_stack_start(void)
{
	LOG_INF("Starting DECT NR+ MAC layer operations.");
	dect_mac_start();
}