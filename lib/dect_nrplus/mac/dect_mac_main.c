/* dect_mac/dect_mac_main.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h> // For settings subsystem (optional)
#include <string.h>                 // For strlen if used
// #include <hw_id.h>                  // For hw_id_get() if needed by core_init directly

#include <mac/dect_mac_mgmt.h>      // For dect_mac_mgmt_service_register_callback, _init
#include <mac/dect_mac_core.h>      // For dect_mac_core_init, get_mac_context
#include <mac/dect_mac_context.h>   // For dect_mac_context_t and role access
#include <mac/dect_mac_main_dispatcher.h> // For dect_mac_event_dispatch
#include <mac/dect_mac_sm_pt.h>     // For dect_mac_sm_pt_start_operation
#include <mac/dect_mac_sm_ft.h>     // For dect_mac_sm_ft_start_operation
#include <mac/dect_mac_phy_if.h>    // For dect_mac_phy_if_init()
#include <mac/dect_mac_data_path.h> // For dect_mac_data_path_service_tx()

// For DLC API usage by this "application" main thread
#include <dect_cvg.h>
#include <mac/dect_mac_api.h>
#include <dect_dlc.h>
#include <zephyr/pm/device.h>
#include <psa/crypto.h>

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

LOG_MODULE_REGISTER(dect_mac_main, CONFIG_DECT_APP_MAIN_LOG_LEVEL);

/**
 * Message queue for events from PHY/timers to the MAC thread.
 * Defined in dect_mac_phy_if.c
 */
extern struct k_msgq mac_event_msgq;

// This now needs to be visible to main for passing to dect_mac_api_init
extern struct k_fifo g_dlc_internal_mac_rx_fifo;

/**
 * @brief External declaration of the DLC's application-facing receive queue.
 *
 * This dlist is defined in dect_dlc.c and is used to pass completed SDUs
 * from the DLC layer up to the application/CVG layer. The main init function
 * needs access to it to pass it down to the MAC API.
 */
extern sys_dlist_t g_dlc_to_app_rx_dlist;


/**
 * The MAC layer's dedicated thread stack area.
 * Size needs to be determined based on actual usage, including ISR/callback contexts
 * that might put messages into the queue.
 */
K_THREAD_STACK_DEFINE(dect_mac_stack_area, CONFIG_DECT_MAC_THREAD_STACK_SIZE); // Use Kconfig for size

/**
 * The MAC layer's thread control block.
 */
static struct k_thread dect_mac_thread_data;
static k_tid_t dect_mac_thread_id;


/**
 * The main entry point for the MAC layer's dedicated Zephyr thread.
 * This thread waits for events on `mac_event_msgq` and dispatches them.
 * It also periodically services TX queues and scheduled operations.
 */
void dect_mac_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);


    struct dect_mac_event_msg msg;
    // const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(dect_nrplus0));
    const struct device *dev = DEVICE_DT_GET_ANY(nordic_nrf_dect);
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    if (!device_is_ready(dev)) {
        LOG_ERR("DECT NR+ device not ready for PM control.");
        return;
    }

    LOG_INF("DECT_MAC_MAIN xxx  DECT MAC Thread Started. TID: %p", k_current_get());

    while (1) {
        pm_device_busy_set(dev);

        if (ctx->state >= MAC_STATE_PT_ASSOCIATING) {
            dect_mac_data_path_service_tx();
        }

        pm_device_busy_clear(dev);

        int ret = k_msgq_get(&mac_event_msgq, &msg, K_MSEC(CONFIG_DECT_MAC_THREAD_SERVICE_INTERVAL_MS));

        pm_device_busy_set(dev);

        if (ret == 0) {
			printk("*********************************************Sent from MAC THREAD dect_mac_thread_entry************************************ \n ");
            dect_mac_event_dispatch(&msg);
        } else if (ret != -EAGAIN) {
            LOG_ERR("Error reading from MAC event queue: %d", ret);
        }
    }
}


/**
 * Example application-level handler for management frames.
 */
static void app_management_handler(const uint8_t *data, size_t len, uint32_t source_long_rd_id)
{
    LOG_INF("APP_MGMT_CB: Received management frame (len %zu) from 0x%08X", len, source_long_rd_id);
    // Example: Print payload as hex
    LOG_HEXDUMP_INF(data, len, "Mgmt Payload:");
    // Application-specific logic for OTA, config, diagnostics would go here.
}


int dect_nrplus_stack_init(void)
{
	int err;

	err = dect_mac_phy_if_init();
	if (err) {
		LOG_ERR("MAC-PHY interface init failed: %d", err);
		return err;
	}

	// err = dect_mac_api_init(&g_dlc_to_app_rx_dlist);
	// if (err) {
	// 	LOG_ERR("MAC API init failed: %d", err);
	// 	return err;
	// }

#if defined(CONFIG_DECT_MAC_ROLE_FT)
	dect_mac_role_t my_role = MAC_ROLE_FT;
#elif defined(CONFIG_DECT_MAC_ROLE_PT)
	dect_mac_role_t my_role = MAC_ROLE_PT;
#else
#error "A DECT MAC role must be selected in Kconfig"
#endif

	err = dect_mac_core_init(my_role, 0);
	if (err) {
		LOG_ERR("MAC Core init failed: %d", err);
		return err;
	}

	err = dect_cvg_init();
	if (err) {
		LOG_ERR("CVG/DLC init failed: %d", err);
		return err;
	}

	dect_mac_thread_id = k_thread_create(&dect_mac_thread_data, dect_mac_stack_area,
					 K_THREAD_STACK_SIZEOF(dect_mac_stack_area),
					 dect_mac_thread_entry,
					 NULL, NULL, NULL,
					 CONFIG_DECT_MAC_THREAD_PRIORITY,
					 0, K_NO_WAIT);
	if (dect_mac_thread_id == NULL) {
		LOG_ERR("CRITICAL: Failed to create DECT MAC thread!");
		return -EAGAIN;
	}
	k_thread_name_set(dect_mac_thread_id, "dect_mac");
	LOG_INF("DECT MAC Thread created.");

	return 0;
}

void dect_nrplus_stack_start(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (ctx->role == MAC_ROLE_PT) {
		dect_mac_sm_pt_start_operation();
	} else {
		dect_mac_sm_ft_start_operation();
	}
}