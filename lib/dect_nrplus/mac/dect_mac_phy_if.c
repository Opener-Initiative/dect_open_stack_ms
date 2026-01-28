/* dect_mac/dect_mac_phy_if.c */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>             // For memcpy

#include <mac/dect_mac_sm.h>        // For struct dect_mac_event_msg definition and event types
#include <mac/dect_mac_core.h>      // For get_mac_context()
#include <mac/dect_mac_context.h>   // For dect_phy_latency_values_t, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ
#include <mac/dect_mac_main_dispatcher.h>



#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

LOG_MODULE_REGISTER(dect_mac_phy_if, CONFIG_DECT_MAC_PHY_IF_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZTEST) && IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)
#define MAX_CONCURRENT_PHY_OPS 64
#else
#define MAX_CONCURRENT_PHY_OPS 16
#endif

typedef struct {
	uint32_t handle;
	struct dect_mac_context *ctx;
} op_handle_map_entry_t;

static op_handle_map_entry_t g_op_handle_map[MAX_CONCURRENT_PHY_OPS] = {0};


// Helper to convert modem ticks to microseconds for storing latency values
static inline uint32_t modem_ticks_to_us_phy_if(uint32_t ticks) {
    if (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ == 0) return 0;
    return (uint32_t)(((uint64_t)ticks * 1000U) / NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
}

/**
 * @brief The single event handler callback registered with the nRF modem DECT PHY library.
 *
 * Executes in the modem library's event callback context.
 * Copies relevant event data into a `dect_mac_event_msg` and posts it to `mac_event_msgq`.
 */
static void phy_event_handler_callback(const struct nrf_modem_dect_phy_event *event)
{
	struct dect_mac_context *event_ctx = NULL;

	if (!event) {
		printk("[PHY_IF]: Received NULL event pointer from modem library!\n");
		return;
	}

	struct dect_mac_event_msg msg_to_queue;
	memset(&msg_to_queue, 0, sizeof(struct dect_mac_event_msg));

	/* Find the context associated with this event's handle */
	uint32_t event_handle = 0;
	bool is_op_event = false;

	/* Extract handle from events that have one */
	switch (event->id) {
	case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
		event_handle = event->op_complete.handle;
		is_op_event = true;
		break;
	case NRF_MODEM_DECT_PHY_EVT_CANCELED:
		event_handle = event->cancel.handle;
		is_op_event = true;
		break;
	case NRF_MODEM_DECT_PHY_EVT_RSSI:
		event_handle = event->rssi.handle;
		is_op_event = true;
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC:
		event_handle = event->pcc.handle;
		is_op_event = true;
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:
		event_handle = event->pcc_crc_err.handle;
		is_op_event = true;
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC:
		event_handle = event->pdc.handle;
		is_op_event = true;

        // /* Find the cached PCC to get the phy_type for this PDC */
        // for (int i = 0; i < MAX_PENDING_PCC_TRANSACTIONS; i++) {
        //     if (event_ctx->pcc_transaction_cache[i].is_valid &&
        //         event_ctx->pcc_transaction_cache[i].transaction_id == event->pdc.transaction_id) {
        //         msg_to_queue.data.pcc.phy_type = event_ctx->pcc_transaction_cache[i].pcc_data.phy_type;
        //         break;
        //     }
        // }
        const uint8_t *data = event->pdc.data;
        size_t len = 8; /* Adjust this based on your needs */
        printk("[MOCK_PHY] RX_QUEUE:PDC Payload Hexdump (first %zu bytes): ", len);
        for (size_t i = 0; i < len && i < sizeof(event->pdc); i++) {
            printk("%02x ", data[i]);
        }
        printk("\n");


		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:
		event_handle = event->pdc_crc_err.handle;
		is_op_event = true;
		break;
	default:
		/* Events without a handle (like INIT) use the currently active context */
		if (event_ctx == NULL) { /* Only if override was not used */
			event_ctx = dect_mac_get_active_context();
		}
		break;
	}

	if (is_op_event && event_ctx == NULL) { /* Only search map if override was not used */
		
		int last = 0;
		for (int i = 0; i < MAX_CONCURRENT_PHY_OPS; i++) {
			// printk("g_op_handle_map[%d].handle:%d == event_handle:%d \n", i, g_op_handle_map[i].handle, event_handle);
			last = i;
			if (g_op_handle_map[i].handle == event_handle) {
				event_ctx = g_op_handle_map[i].ctx;
				break;
			}
		}
		// printk("g_op_handle_map last:%d < %d\n", last, MAX_CONCURRENT_PHY_OPS);
	}

	if (!event_ctx) {
		printk("[PHY_IF HANDLER] ***WARNING***: Could not find context for event %d (handle %u). Using globally active context.\n",
		       event->id, event_handle);
		event_ctx = dect_mac_get_active_context();
	}

	msg_to_queue.ctx = event_ctx;
	msg_to_queue.modem_time_of_event = event->time;

	bool should_queue_msg = true;
	int err;

	printk("[PHY_IF] Rcvd event from modem: type=%d\n", event->id);

	switch (event->id) {
	case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
		msg_to_queue.type = MAC_EVENT_PHY_OP_COMPLETE;
		memcpy(&msg_to_queue.data.op_complete, &event->op_complete, sizeof(event->op_complete));
		break;

	case NRF_MODEM_DECT_PHY_EVT_PCC:
		msg_to_queue.type = MAC_EVENT_PHY_PCC;
		memcpy(&msg_to_queue.data.pcc, &event->pcc, sizeof(event->pcc));
		break;

	case NRF_MODEM_DECT_PHY_EVT_PDC:
		msg_to_queue.type = MAC_EVENT_PHY_PDC;
		memcpy(&msg_to_queue.data.pdc, &event->pdc, sizeof(event->pdc));
		
		// TODO:
		// /* Early filtering for Unicast packets not addressed to us */
		// dect_mac_header_type_octet_t *hdr_type = (void *)event->pdc.data;
		// if (hdr_type->mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		// 	dect_mac_unicast_header_t *uch = (void *)(event->pdc.data + sizeof(*hdr_type));
		// 	uint32_t dest_id = sys_be32_to_cpu(uch->receiver_long_rd_id_be);
		// 	if (dest_id != dect_mac_get_active_context()->own_long_rd_id) {
		// 		LOG_DBG("PHY_IF: Discarding Unicast packet for another destination (0x%08X)", dest_id);
		// 		should_queue_msg = false; /* Do not queue this event */
		// 	}
		// }
		break;

	case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:
		msg_to_queue.type = MAC_EVENT_PHY_PCC_ERROR;
		memcpy(&msg_to_queue.data.pcc_crc_err, &event->pcc_crc_err, sizeof(event->pcc_crc_err));
		break;

	case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:
		msg_to_queue.type = MAC_EVENT_PHY_PDC_ERROR;
		memcpy(&msg_to_queue.data.pdc_crc_err, &event->pdc_crc_err, sizeof(event->pdc_crc_err));
		break;

	case NRF_MODEM_DECT_PHY_EVT_RSSI:
		msg_to_queue.type = MAC_EVENT_PHY_RSSI_RESULT;
		memcpy(&msg_to_queue.data.rssi, &event->rssi, sizeof(event->rssi));
		break;

	case NRF_MODEM_DECT_PHY_EVT_INIT:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_INIT result: %d (Temp: %dC, Volt: %umV, Limit: %uC)",
			event->init.err, event->init.temp, event->init.voltage, event->init.temperature_limit);
		if (event->init.err == NRF_MODEM_DECT_PHY_SUCCESS) {
			LOG_INF("[PHY_IF]  DECT PHY Initialized by modem. Requesting latency information.");
			err = nrf_modem_dect_phy_latency_get();
			if (err != 0) {
				LOG_ERR("[PHY_IF]  Failed to request nrf_modem_dect_phy_latency_get(): %d", err);
			}
		} else {
			LOG_ERR("[PHY_IF]  Modem DECT PHY Initialization failed with error %d.", event->init.err);
		}
		should_queue_msg = false;
		break;

	case NRF_MODEM_DECT_PHY_EVT_LATENCY:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_LATENCY result: %d", event->latency_get.err);
		if (event->latency_get.err == NRF_MODEM_DECT_PHY_SUCCESS && event->latency_get.latency_info) {
			dect_mac_context_t *ctx = dect_mac_get_active_context();
			if (ctx) {
				const struct nrf_modem_dect_phy_latency_info *modem_lat = event->latency_get.latency_info;
				enum nrf_modem_dect_phy_radio_mode mode_to_use = NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY_WITH_STANDBY;

				ctx->phy_latency.scheduled_operation_startup_us = modem_ticks_to_us_phy_if(
					modem_lat->radio_mode[mode_to_use].scheduled_operation_startup);
				ctx->phy_latency.scheduled_operation_transition_us = modem_ticks_to_us_phy_if(
					modem_lat->radio_mode[mode_to_use].scheduled_operation_transition);
				ctx->phy_latency.idle_to_active_rx_us = modem_ticks_to_us_phy_if(modem_lat->operation.receive.idle_to_active);
				ctx->phy_latency.idle_to_active_tx_us = modem_ticks_to_us_phy_if(modem_lat->operation.transmit.idle_to_active);
				ctx->phy_latency.active_to_idle_rx_us = modem_ticks_to_us_phy_if(modem_lat->operation.receive.active_to_idle_rx);
				ctx->phy_latency.active_to_idle_tx_us = modem_ticks_to_us_phy_if(modem_lat->operation.transmit.active_to_idle);
				LOG_INF("[PHY_IF]  Latency info stored in MAC context (e.g., sched_startup_us: %u, idle_to_rx_us: %u).",
					ctx->phy_latency.scheduled_operation_startup_us, ctx->phy_latency.idle_to_active_rx_us);
			} else {
				LOG_ERR("[PHY_IF]  MAC context NULL, cannot store latency info.");
			}
		} else if (event->latency_get.err != NRF_MODEM_DECT_PHY_SUCCESS) {
			LOG_ERR("[PHY_IF]  Failed to get latency info from modem: %d", event->latency_get.err);
		} else {
			LOG_ERR("[PHY_IF]  Latency info event success, but latency_info pointer is NULL from modem.");
		}
		should_queue_msg = false;
		break;

	case NRF_MODEM_DECT_PHY_EVT_DEINIT:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_DEINIT result: %d", event->deinit.err);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_CONFIGURE result: %d", event->configure.err);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_RADIO_CONFIG:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_RADIO_CONFIG result: %d (handle %u)",
			event->radio_config.err, event->radio_config.handle);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:
		 LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_ACTIVATE result: %d (Temp: %dC, Volt: %umV)",
			 event->activate.err, event->activate.temp, event->activate.voltage);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_DEACTIVATE:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_DEACTIVATE result: %d", event->deactivate.err);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_CANCELED:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_CANCELED (for cancel op itself) result: %d for handle %u",
			event->cancel.err, event->cancel.handle);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_TIME:
		LOG_DBG("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_TIME: current modem time is %llu", event->time);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_CAPABILITY:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_CAPABILITY result: %d. Capability data ptr: %p", event->capability_get.err, event->capability_get.capability);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_BANDS:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_BANDS result: %d. Band count: %u", event->band_get.err, event->band_get.band_count);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_LINK_CONFIG:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_LINK_CONFIG result: %d", event->link_config.err);
		should_queue_msg = false;
		break;
	case NRF_MODEM_DECT_PHY_EVT_STF_CONFIG:
		LOG_INF("[PHY_IF]  NRF_MODEM_DECT_PHY_EVT_STF_CONFIG result: %d", event->stf_cover_seq_control.err);
		should_queue_msg = false;
		break;

	default:
		LOG_WRN("[PHY_IF]  Received unknown/unhandled PHY event ID: 0x%02X from modem. Not queueing.", event->id);
		should_queue_msg = false;
		break;
	}

	if (should_queue_msg) {
		printk("[PHY_IF] PRE-QUEUE CHECK: msg.type = %d (%s)\t", msg_to_queue.type, dect_mac_event_to_str(msg_to_queue.type));
		err = k_msgq_put(&mac_event_msgq, &msg_to_queue, K_NO_WAIT);
		printk("Queued MAC event %d: ", msg_to_queue.type);

		switch (err)
		{
		case 0:
			printk("Found Message (%d) \n", err);
			break;
		case -42:
			printk("NO Message (%d) \n", err);
			break;
		case -11:
			printk("Try Again (%d) \n", err);
			break;			
		default:
			printk("(%d)... \n", err);
			break;
		}		

		if (err != 0) {
			printk("[PHY_IF]  MAC event message queue FULL! Event type %d (PHY ID 0x%02X) was dropped. Error: %d \n",
				msg_to_queue.type, event->id, err);
		}
	}
}


int dect_mac_phy_if_init(void)
{
    int err = nrf_modem_dect_phy_event_handler_set(phy_event_handler_callback);
    if (err) {
        LOG_ERR("[PHY_IF]  Failed to set nRF DECT PHY event handler, err: %d", err);
    } else {
        LOG_INF("[PHY_IF]  nRF DECT PHY event handler registered successfully.");
    }
    // Note: nrf_modem_dect_phy_init() is called by dect_mac_phy_ctrl_init() or similar
    // after the modem library itself (nrf_modem_init) is up.
    // This function only sets the handler.
    return err;
}




static int count_active_handles(void)
{
    int count = 0;
    for (int i = 0; i < MAX_CONCURRENT_PHY_OPS; i++) {
        if (g_op_handle_map[i].handle != 0) {
            count++;
        }
    }
    return count;
}

void debug_operation_handle_map(const char *test_name)
{
    printk("[DEBUG] %s - Operation Handle Map State:\n", test_name);
    for (int i = 0; i < MAX_CONCURRENT_PHY_OPS; i++) {
        if (g_op_handle_map[i].handle != 0) {
            printk("  Slot %d: Handle %u, Context %p\n", 
                   i, g_op_handle_map[i].handle, g_op_handle_map[i].ctx);
        }
    }
    printk("[DEBUG] Total active handles: %d\n", count_active_handles());
}

void dect_mac_phy_if_reset_handle_map(void)
{
	memset(g_op_handle_map, 0, sizeof(g_op_handle_map));
	LOG_DBG("PHY_IF_MAP: Handle map reset for new test case.");
}


void dect_mac_phy_if_register_op_handle(uint32_t handle, struct dect_mac_context *ctx)
{
	for (int i = 0; i < MAX_CONCURRENT_PHY_OPS; i++) {
		// printk("  - g_op_handle_map[%d].handle:%d \n",i ,g_op_handle_map[i].handle);
		if (g_op_handle_map[i].handle == 0 || !g_op_handle_map[i].handle) {
			g_op_handle_map[i].handle = handle;
			g_op_handle_map[i].ctx = ctx;

			printk("[PHY_IF_MAP] Registered handle %u to context %p (Role %s) - g_op_handle_map[%d].handle:%d \n",
			       handle, (void *)ctx, (ctx->role == MAC_ROLE_PT ? "PT" : "FT"), i ,g_op_handle_map[i].handle);
			return;
		}
		// printk("  - g_op_handle_map[%d].handle:%d \n",i ,g_op_handle_map[i].handle);
	}
	printk("[PHY_IF_MAP] ***ERROR***: No free slots to register handle %u\n", handle);
}

void dect_mac_phy_if_deregister_op_handle(uint32_t handle)
{
	for (int i = 0; i < MAX_CONCURRENT_PHY_OPS; i++) {
		if (g_op_handle_map[i].handle == handle) {
            printk("[PHY_IF_MAP] Deregistering handle %u (context %p) from slot %d\n",
			       handle, (void *)g_op_handle_map[i].ctx, i);
			g_op_handle_map[i].handle = 0;
			g_op_handle_map[i].ctx = NULL;
			// memset(&g_op_handle_map[i], 0, sizeof(op_handle_map_entry_t));
			return;
		}
		// printk("  - Registered g_op_handle_map[%d].handle:%d \n",i ,g_op_handle_map[i].handle);
	}
}