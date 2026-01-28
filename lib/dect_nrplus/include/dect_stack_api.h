/* lib/dect_nrplus/include/dect_stack.h */
// Overview: New top-level public API header for the entire DECT NR+ stack. This is the single point of entry for applications.
#ifndef DECT_STACK_API_H__
#define DECT_STACK_API_H__

#include <mac/dect_mac_types.h> /* For dect_mac_public_state_t */
#include <dect_cdd.h>           /* For public CDD configuration functions */

/**
 * @brief Callback function prototype for stack state changes.
 *
 * @param new_state The new public state of the MAC layer.
 */
typedef void (*dect_stack_state_change_cb_t)(dect_mac_public_state_t new_state);

/**
 * @brief Initializes the entire DECT NR+ protocol stack.
 *
 * This function is the single entry point for an application to initialize
 * the MAC, DLC, and CVG layers in the correct order.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int dect_stack_init(void);

/**
 * @brief Starts the DECT NR+ stack's operation.
 *
 * This function starts the MAC state machine (scanning for a PT, or
 * beaconing for an FT). It must be called after a successful init.
 */
void dect_stack_start(void);

/**
 * @brief Registers a callback to be notified of stack state changes.
 *
 * @param cb The callback function to register.
 */
void dect_stack_register_state_change_cb(dect_stack_state_change_cb_t cb);

#endif /* DECT_STACK_API_H__ */