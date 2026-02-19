/* lib/dect_nrplus/mac/dect_mac.c */
// Overview: New file to implement the unified public MAC API. It acts as a facade, calling the appropriate internal functions.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_types.h>
#include <mac/dect_mac_main_dispatcher.h>


#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#endif

LOG_MODULE_REGISTER(dect_mac, CONFIG_DECT_MAC_API_LOG_LEVEL);


/************************************************************** */
/* Special Tst functions to overrid the existing function calls */
#if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
/* --- Internal function pointer for test spy --- */
static int (*g_send_spy_cb)(mac_sdu_t *sdu, mac_flow_id_t flow) = NULL;
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API) */

#if IS_ENABLED(CONFIG_DECT_MAC_OWN_THREAD)
K_THREAD_STACK_DEFINE(dect_mac_stack_area, CONFIG_DECT_MAC_THREAD_STACK_SIZE);
static struct k_thread dect_mac_thread_data;
static k_tid_t dect_mac_thread_id;

static void mac_service_timer_handler(struct k_timer *timer_id);
K_TIMER_DEFINE(mac_service_timer, mac_service_timer_handler, NULL);

static void mac_service_timer_handler(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);
	/* This handler runs in the timer thread context.
	 * It calls dect_mac_service(), which is designed to be callable
	 * from different contexts.
	 */
	dect_mac_service();
}

void dect_mac_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[MAC_THREAD_DBG] DECT MAC Thread has started execution.\n");
	LOG_INF("DECT MAC Thread Started. TID: %p", k_current_get());

	/* Start the periodic timer that will service the data path */
	k_timer_start(&mac_service_timer, K_MSEC(CONFIG_DECT_MAC_THREAD_SERVICE_INTERVAL_MS),
		      K_MSEC(CONFIG_DECT_MAC_THREAD_SERVICE_INTERVAL_MS));

	while (1) {
		/* Block indefinitely until an event is received from PHY or other timers */
		dect_mac_process_event_timeout(K_FOREVER);
	}
}
// static void dect_mac_thread_entry(void *p1, void *p2, void *p3)
// {
// 	ARG_UNUSED(p1);
// 	ARG_UNUSED(p2);
// 	ARG_UNUSED(p3);

// 	printk("[MAC_THREAD_DBG] DECT MAC Thread has started execution.\n");
// 	LOG_INF("DECT MAC Thread Started. TID: %p", k_current_get());

// 	while (1) {
// 		/* Process a single event, waiting with a timeout */
// 		int ret = dect_mac_process_event_timeout(K_MSEC(CONFIG_DECT_MAC_THREAD_SERVICE_INTERVAL_MS));

// 		/* If no event was processed (timeout), this is a chance for periodic service.
// 		 * If an event was processed, service any resulting actions immediately.
// 		 */
// 		if (ret == 0 || ret == -EAGAIN) {
// 			dect_mac_service();
// 		}
// 	}
// }




#endif /* IS_ENABLED(CONFIG_DECT_MAC_OWN_THREAD) */

// #if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
// /* --- Internal function pointer for init spy --- */
// #endif /*  */



/* This is defined in dect_mac_phy_if.c */
extern struct k_msgq mac_event_msgq;

int dect_mac_init(struct k_queue *rx_queue_from_dlc, dlc_tx_status_cb_t status_cb)
{
	// printk("[INIT_DBG] Entering dect_mac_init. Checking for spy callback...\n");
// #if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
// printk("g_init_spy_cb is %s \n", g_init_spy_cb ? "OK":"NULL");
// 	if (g_init_spy_cb) {
// 		printk("[INIT_DBG] Spy callback for dect_mac_init is registered. Calling it.\n");
// 		return g_init_spy_cb(rx_queue_from_dlc, status_cb);
// 	}
// #endif
	
	int err;

	printk("[DECT_MAC] Entering dect_mac_init. Current active context: %p\n", (void *)dect_mac_get_active_context());

	#if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
	printk("[INIT_DBG] Entering dect_mac_init. Current spy pointer: %p\n", (void *)g_send_spy_cb);
	/* Restore the real send function pointer on every init to ensure clean test state */
	g_send_spy_cb = NULL;
	printk("[INIT_DBG] Spy pointer has been reset to: %p\n", (void *)g_send_spy_cb);
	#endif /*  */

	err = dect_mac_phy_if_init();
	if (err) {
		LOG_ERR("MAC-PHY interface init failed: %d", err);
		return err;
	}

	printk("[MAC_INIT_PROPAGATION_DBG] About to call dect_mac_api_init with queue pointer: %p\n",
	       (void *)rx_queue_from_dlc);

	err = dect_mac_api_init(rx_queue_from_dlc);
	if (err) {
		LOG_ERR("MAC API init failed: %d", err);
		return err;
	}

	dect_mac_data_path_register_dlc_callback(status_cb);

// #if IS_ENABLED(CONFIG_DECT_MAC_ROLE_FT)
// 	dect_mac_role_t my_role = MAC_ROLE_FT;
// #elif IS_ENABLED(CONFIG_DECT_MAC_ROLE_PT)
// 	dect_mac_role_t my_role = MAC_ROLE_PT;
// #else
// #error "A DECT MAC role must be selected in Kconfig"
// #endif

// 	err = dect_mac_core_init(my_role, CONFIG_DECT_MAC_PROVISIONED_LONG_RD_ID);
// 	if (err) {
// 		LOG_ERR("MAC Core init failed: %d", err);
// 		return err;
// 	}

#if IS_ENABLED(CONFIG_DECT_MAC_OWN_THREAD)
	dect_mac_thread_id = k_thread_create(&dect_mac_thread_data, dect_mac_stack_area,
					 K_THREAD_STACK_SIZEOF(dect_mac_stack_area),
					 dect_mac_thread_entry, NULL, NULL, NULL,
					 CONFIG_DECT_MAC_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (dect_mac_thread_id == NULL) {
		LOG_ERR("CRITICAL: Failed to create DECT MAC thread!");
		return -EAGAIN;
	}
	k_thread_name_set(dect_mac_thread_id, "dect_mac");
#endif /* IS_ENABLED(CONFIG_DECT_MAC_OWN_THREAD) */

	return 0;
}

void dect_mac_start(void)
{
	printk("[MAC_START_DBG] Entering dect_mac_start...\n");

	dect_mac_context_t *ctx = dect_mac_get_active_context();

	/* Activate the PHY before starting any operations */
	int err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	if (err) {
		LOG_ERR("Failed to activate DECT PHY: %d", err);
		dect_mac_change_state(MAC_STATE_ERROR);
	
		return;
	}

	/* Enable STF cover sequence for improved robustness (TS 103 636-3 Cl. 6.3.7) */
	err = nrf_modem_dect_phy_stf_cover_seq_control(true, true);
	if (err) {
		LOG_WRN("Failed to configure STF cover sequence: %d. Continuing with default.", err);
	}


	if (ctx->role == MAC_ROLE_PT) {
		// printk("[MAC_START_DBG] Entering dect_mac_start...\n");
		dect_mac_sm_pt_start_operation();
	} else {
		// printk("[MAC_START_DBG] Calling dect_mac_sm_ft_start_operation...\n");
		dect_mac_sm_ft_start_operation();
	}
}

int dect_mac_send(mac_sdu_t *sdu, mac_flow_id_t flow)
{

#if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
	printk("[MAC_SEND_DBG] TEST dect_mac_send() function was called. \n");
	if (g_send_spy_cb) {
		return g_send_spy_cb(sdu, flow);
	}
#else
	printk("[MAC_SEND_DBG] REAL dect_mac_send() function was called.\n");
#endif
	return dect_mac_api_send(sdu, flow);
}

#if IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API)
void dect_mac_test_set_send_spy(int (*handler)(mac_sdu_t *sdu, mac_flow_id_t flow))
{
	printk("[SPY_DBG] dect_mac_test_set_send_spy called. Handler set to: %p\n", (void *)handler);
	g_send_spy_cb = handler;
}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SEND_MOCK_API) */





int dect_mac_process_event_timeout(k_timeout_t timeout)
{
	struct dect_mac_event_msg msg;
	int ret = k_msgq_get(&mac_event_msgq, &msg, timeout);

	switch (ret)
	{
	case 0:
		// printk("[MAC_PROCESS_DBG] k_msgq_get returned error: Found Message (%d) \n", ret);
		break;
	case -42:
		printk("[MAC_PROCESS_DBG] k_msgq_get returned error: NO Message (%d) \n", ret);
		break;
	case -11:
		printk("[MAC_PROCESS_DBG] k_msgq_get returned error: Try Again (%d) \n", ret);
		break;			
	default:
		printk("[MAC_PROCESS_DBG] k_msgq_get returned error: (%d)... \n", ret);
		break;
	}

	if (ret == 0) {
		printk("[MAC]: dect_mac_process_event_timeout(): msg->type: %s(%d) err:%d\n",dect_mac_event_to_str(msg.type), msg.type, ret);
		dect_mac_event_dispatch(&msg);
	}

	return ret;
}

void dect_mac_service(void)
{
	dect_mac_data_path_service_tx();
}

mac_sdu_t *dect_mac_buffer_alloc(k_timeout_t timeout) { return dect_mac_api_buffer_alloc(timeout); }
// void dect_mac_buffer_free(mac_sdu_t *sdu) 
// { 
// 	LOG_DBG("[%s] Attempting to free a SDU buffer.", __func__);
// 	dect_mac_api_buffer_free(sdu); 
// }
int dect_mac_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id) { return dect_mac_api_ft_send_to_pt(sdu, flow, target_pt_short_rd_id); }
void dect_mac_register_state_change_cb(dect_mac_state_change_cb_t cb)
{
	dect_mac_core_register_state_change_cb(cb);
}
uint32_t dect_mac_get_own_long_id(void) 
{ 
	return dect_mac_get_active_context()->own_long_rd_id; 
}

uint32_t dect_mac_get_associated_ft_long_id(void)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (ctx && ctx->role == MAC_ROLE_PT && ctx->role_ctx.pt.associated_ft.is_valid) {
		return ctx->role_ctx.pt.associated_ft.long_rd_id;
	}
	return 0;
}


// #if IS_ENABLED(CONFIG_ZTEST)
// // int dect_mac_test_inject_event(dect_mac_context_t *ctx, dect_mac_event_type_t event_type, int timer_id)
// // {
// // 	struct dect_mac_event_msg msg = {
// // 		.ctx = ctx,
// // 		.type = event_type,
// // 		.modem_time_of_event = 0, /* Time is not critical for this test's timer events */
// // 		.data.timer_data.id = timer_id
// // 	};

// // 	return k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);
// // }
// #endif /* IS_ENABLED(CONFIG_ZTEST) */