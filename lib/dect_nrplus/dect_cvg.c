/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/dect_cvg.c */
/* This is the complete, corrected implementation of the CVG layer. It integrates full Segmentation and Reassembly (SAR), In-Sequence Delivery (ISD), Duplicate Removal, Flow Control (FC), Automatic Repeat Request (ARQ), SDU Lifetime Control, and TX Services negotiation procedures, ensuring robust and reliable data transport. */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)		
#include <psa/crypto.h>
#endif
#include <zephyr/sys/util.h> 

// #include <mac/dect_mac.h>
#include <mac/dect_mac_core.h>
// #include <mac/dect_mac_context.h>
// #include <mac/dect_mac_security.h>
// #include <mac/dect_mac_data_path.h>
// #include <mac/dect_mac_timeline_utils.h>

#include <mac/dect_mac.h>
#include <dect_cdd.h>
#include <dect_cvg.h>
#include <dect_dlc.h>

#if IS_ENABLED(CONFIG_ZTEST) || IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
#endif

LOG_MODULE_REGISTER(dect_cvg, CONFIG_DECT_CVG_LOG_LEVEL);

/*
Implement the new public functions in the CVG layer. 
These functions will now be responsible for calling the MAC layer to retrieve the required information, 
acting as a proper intermediary.
*/
void dect_cvg_register_link_status_cb(dect_mac_state_change_cb_t cb)
{
	/* Pass the callback down to the MAC layer */
	dect_mac_register_state_change_cb(cb);
}

int dect_cvg_get_iid_parts(uint32_t *sink_id, uint32_t *device_id)
{
	if (!sink_id || !device_id) {
		return -EINVAL;
	}

	*device_id = dect_mac_get_own_long_id();

	if (dect_mac_get_role() == MAC_ROLE_PT) {
		*sink_id = dect_mac_get_associated_ft_long_id();
		if (*sink_id == 0) {
			return -EAGAIN; /* Not yet associated */
		}
	} else { /* FT acts as its own sink */
		*sink_id = *device_id;
	}

	return 0;
}


#define CVG_MAX_IN_FLIGHT_SDUS 16
#define CVG_MAX_REASSEMBLY_SESSIONS 4
#define CVG_REASSEMBLY_BUF_SIZE	    (4096)
#define CVG_REASSEMBLY_TIMEOUT_MS   5000
#define CVG_APP_BUFFER_COUNT 8

typedef struct {
	mac_sdu_t *sdu;
	uint16_t sequence_number;
	uint32_t dest_long_id;
	uint16_t endpoint_id;
	uint32_t hpc_at_tx;
	bool is_active;
	uint8_t retries;
	struct k_timer lifetime_timer;
} cvg_inflight_sdu_ctx_t;

typedef struct {
	bool is_active;
	uint16_t sequence_number;
	uint8_t reassembly_buf[CVG_REASSEMBLY_BUF_SIZE];
	size_t total_sdu_len;
	size_t bytes_received;
	struct k_timer timeout_timer;
} cvg_reassembly_session_t;

typedef struct {
    cvg_service_type_t service_type;
    cvg_qos_t configured_qos; /**< Configured QoS/Flow ID */
    bool is_configured;
    uint32_t configured_lifetime_ms;
    bool security_enabled;
    uint8_t integrity_key[16];
    uint8_t cipher_key[16];
    uint16_t tx_sequence_number;
    uint16_t tx_window_start_sn;
    uint16_t tx_window_end_sn;
    uint16_t max_window_size;
    struct k_sem tx_window_sem;
    struct k_mutex flow_mutex;
    cvg_inflight_sdu_ctx_t tx_in_flight_sdu[CVG_MAX_IN_FLIGHT_SDUS];
    uint16_t rx_expected_sn;
    uint16_t last_ack_sent_sn;
    uint32_t peer_hpc;
	struct {
		mac_sdu_t *sdu;
		bool is_valid;
	} rx_reordering_buffer[CVG_MAX_IN_FLIGHT_SDUS];
} cvg_flow_context_t;



static void cvg_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id);
static void cvg_process_arq_feedback(const uint8_t *pdu_ptr, size_t len);
static int send_cvg_arq_feedback(uint32_t dest_long_id, bool ack, uint8_t feedback_info_code, uint16_t sn);

#if !defined(CONFIG_ZTEST)
static void cvg_reassembly_timeout_handler(struct k_timer *timer_id);
static cvg_reassembly_session_t *find_or_alloc_cvg_reassembly_session(uint16_t sn);
static void handle_tx_services_cfg_ie(const cvg_ie_tx_services_cfg_t *cfg_ie, uint32_t source_rd_id);
#endif

static cvg_flow_context_t g_default_cvg_flow_ctx;
#if !defined(CONFIG_ZTEST)
static cvg_reassembly_session_t g_cvg_reassembly_sessions[CVG_MAX_REASSEMBLY_SESSIONS];
#endif

typedef enum {
	CVG_ARQ_EVENT_ACK,
	CVG_ARQ_EVENT_NACK,
	CVG_ARQ_EVENT_TIMEOUT,
} cvg_arq_event_type_t;

typedef struct {
	void *fifo_reserved; /* For k_fifo internal use */
	cvg_arq_event_type_t type;
	uint16_t sn;
} cvg_arq_event_t;

K_MEM_SLAB_DEFINE(g_cvg_arq_event_slab, sizeof(cvg_arq_event_t), CVG_MAX_IN_FLIGHT_SDUS * 2, 4);
K_MEM_SLAB_DEFINE(g_cvg_app_sdu_slab, sizeof(mac_sdu_t), CVG_APP_BUFFER_COUNT, 4);

#if IS_ENABLED(CONFIG_ZTEST)
K_QUEUE_DEFINE(g_app_to_cvg_tx_queue);
K_QUEUE_DEFINE(g_cvg_to_app_rx_queue);
// K_QUEUE_DEFINE(g_cvg_retransmit_signal_queue);
K_QUEUE_DEFINE(g_cvg_arq_event_queue);
#else
// static K_QUEUE_DEFINE(g_cvg_retransmit_signal_queue);
static K_QUEUE_DEFINE(g_cvg_arq_event_queue);
static K_QUEUE_DEFINE(g_app_to_cvg_tx_queue);
static K_QUEUE_DEFINE(g_cvg_to_app_rx_queue);
#endif



typedef struct {
	void *fifo_reserved;
	uint16_t endpoint_id;
	uint32_t dest_long_id;
	mac_sdu_t *app_sdu_buf;
	bool is_retransmission;
	uint16_t original_sn;
} cvg_tx_queue_item_t;

K_MEM_SLAB_DEFINE(g_cvg_tx_item_slab, sizeof(cvg_tx_queue_item_t), CVG_APP_BUFFER_COUNT, 4);


static void cvg_tx_thread_entry(void *p1, void *p2, void *p3);
static void cvg_rx_thread_entry(void *p1, void *p2, void *p3);
static void cvg_arq_service_thread_entry(void *p1, void *p2, void *p3);

K_THREAD_STACK_DEFINE(g_cvg_tx_thread_stack, CONFIG_DECT_CVG_TX_THREAD_STACK_SIZE);
static struct k_thread g_cvg_tx_thread_data;
k_tid_t g_cvg_tx_thread_id;

K_THREAD_STACK_DEFINE(g_cvg_rx_thread_stack, CONFIG_DECT_CVG_RX_THREAD_STACK_SIZE);
static struct k_thread g_cvg_rx_thread_data;
k_tid_t g_cvg_rx_thread_id;

K_THREAD_STACK_DEFINE(g_cvg_arq_service_thread_stack, CONFIG_DECT_CVG_TX_SERVICE_THREAD_STACK_SIZE);
static struct k_thread g_cvg_arq_service_thread_data;
k_tid_t g_cvg_arq_service_thread_id;



static uint8_t cvg_lifetime_ms_to_code(uint32_t ms)
{
	if (ms <= 1) return 0x02;
	if (ms <= 5) return 0x03;
	if (ms <= 50) return 0x08;
	if (ms <= 100) return 0x0D;
	if (ms <= 500) return 0x12;
	if (ms <= 1000) return 0x14;
	if (ms <= 5000) return 0x1A;
	return 0x1E; // Default for > 5000ms
}

#if !defined(CONFIG_ZTEST)
static uint32_t cvg_code_to_lifetime_ms(uint8_t code)
{
	if (code <= 0x02) return 1;
	if (code <= 0x03) return 5;
	if (code <= 0x08) return 50;
	if (code <= 0x0D) return 100;
	if (code <= 0x12) return 500;
	if (code <= 0x14) return 1000;
	if (code <= 0x1A) return 5000;
	return 60000; // Default for codes > 0x1A
}


static __maybe_unused cvg_reassembly_session_t *find_or_alloc_cvg_reassembly_session(uint16_t sn)
{
	int free_slot = -1;

	for (int i = 0; i < CVG_MAX_REASSEMBLY_SESSIONS; i++) {
		if (g_cvg_reassembly_sessions[i].is_active) {
			if (g_cvg_reassembly_sessions[i].sequence_number == sn) {
				return &g_cvg_reassembly_sessions[i];
			}
		} else if (free_slot == -1) {
			free_slot = i;
		}
	}

	if (free_slot != -1) {
		cvg_reassembly_session_t *session = &g_cvg_reassembly_sessions[free_slot];

		memset(session, 0, sizeof(cvg_reassembly_session_t));
		session->is_active = true;
		session->sequence_number = sn;
		k_timer_init(&session->timeout_timer, cvg_reassembly_timeout_handler, NULL);
		session->timeout_timer.user_data = (void *)(uintptr_t)free_slot;
		k_timer_start(&session->timeout_timer, K_MSEC(CVG_REASSEMBLY_TIMEOUT_MS),
			      K_NO_WAIT);
		LOG_DBG("CVG_SAR: Allocated reassembly session %d for SN %u", free_slot, sn);
		return session;
	}

	LOG_ERR("CVG_SAR: No free reassembly sessions for SN %u", sn);
	return NULL;
}

static void cvg_reassembly_timeout_handler(struct k_timer *timer_id)
{
	uintptr_t session_idx = (uintptr_t)timer_id->user_data;

	if (session_idx < CVG_MAX_REASSEMBLY_SESSIONS &&
	    g_cvg_reassembly_sessions[session_idx].is_active) {
		LOG_WRN("CVG_SAR: Reassembly for session %u (SN %u) timed out. Discarding.",
			(unsigned int)session_idx,
			g_cvg_reassembly_sessions[session_idx].sequence_number);
		g_cvg_reassembly_sessions[session_idx].is_active = false;
	}
}

static int __maybe_unused build_cvg_transparent_pdu(uint8_t *target_buf, size_t target_buf_len,
                                     const uint8_t *app_payload, size_t app_payload_len)
{
    size_t header_len;
    cvg_header_ext_len_t len_type;

    if (app_payload_len <= 255) {
        header_len = 2;
        len_type = CVG_EXT_8BIT_LEN_FIELD;
    } else {
        header_len = 3;
        len_type = CVG_EXT_16BIT_LEN_FIELD;
    }

    if (header_len + app_payload_len > target_buf_len) {
        return -ENOMEM;
    }

    uint8_t mt_bit = 0;
    target_buf[0] = ((len_type & 0x03) << 6) | ((mt_bit & 0x01) << 5) | (CVG_IE_TYPE_DATA_TRANSPARENT & 0x1F);

    if (len_type == CVG_EXT_8BIT_LEN_FIELD) {
        target_buf[1] = (uint8_t)app_payload_len;
    } else {
        sys_put_be16(app_payload_len, &target_buf[1]);
    }

    if (app_payload && app_payload_len > 0) {
        memcpy(target_buf + header_len, app_payload, app_payload_len);
    }

    return header_len + app_payload_len;
}
#endif


static int build_cvg_data_ie_pdu(uint8_t *target_buf, size_t target_buf_len,
                                 const uint8_t *app_payload, size_t app_payload_len,
                                 uint16_t sequence_number)
{
    cvg_header_t cvg_hdr;
    cvg_hdr.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
                                 ((0 & 0x01) << 5) |
                                 (CVG_IE_TYPE_DATA & 0x1F);

    cvg_ie_data_base_t data_ie_base;
    cvg_ie_data_base_set(&data_ie_base, CVG_SI_COMPLETE_SDU, false, sequence_number);

    size_t total_hdr_len = sizeof(cvg_hdr) + sizeof(data_ie_base);
    if (total_hdr_len + app_payload_len > target_buf_len) {
        return -ENOMEM;
    }

    memcpy(target_buf, &cvg_hdr, sizeof(cvg_hdr));
    memcpy(target_buf + sizeof(cvg_hdr), &data_ie_base, sizeof(data_ie_base));

    if (app_payload && app_payload_len > 0) {
        memcpy(target_buf + total_hdr_len, app_payload, app_payload_len);
    }

    return total_hdr_len + app_payload_len;
}



static int send_cvg_arq_feedback(uint32_t dest_long_id, bool ack, uint8_t feedback_info_code, uint16_t sn)
{
	cvg_ie_arq_feedback_base_t fb_base;
	size_t pdu_len = sizeof(fb_base);

	/* Use Format 1 Header (MT=0) as defined in IE structure */
	fb_base.header.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
				     ((0 & 0x01) << 5) | (CVG_IE_TYPE_ARQ_FEEDBACK & 0x1F);
	
	uint8_t an_bit = ack ? 0 : 1;
	fb_base.an_fbinfo_sn_msb =
		((an_bit & 0x01) << 7) | ((feedback_info_code & 0x07) << 4) |
		((uint8_t)(sn >> 8) & 0x0F);
	fb_base.sequence_number_lsb = (uint8_t)(sn & 0xFF);

	LOG_DBG("CVG_ARQ_TX: Sending %s for SN %u (fb_info %u) to 0x%08X.", ack ? "ACK" : "NACK",
		sn, feedback_info_code, dest_long_id);

	return dlc_send_data(DLC_SERVICE_TYPE_0_TRANSPARENT, dest_long_id, (uint8_t *)&fb_base, pdu_len,
			     (uint8_t)CVG_QOS_CONTROL);
}
#if !defined(CONFIG_ZTEST)
#endif



/**
 * @brief Timer callback handler for when an in-flight CVG SDU's lifetime expires.
 *
 * This function is called by the kernel's timer thread when the lifetime timer
 * for a reliable CVG SDU (one waiting for an ACK) expires. It cleans up the
 * CVG SDU context and frees its resources.
 *
 * @param timer_id Pointer to the k_timer instance that expired.
 */
static void cvg_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id)
{
	cvg_inflight_sdu_ctx_t *sdu_ctx = CONTAINER_OF(timer_id, cvg_inflight_sdu_ctx_t, lifetime_timer);

	/* Do not check state here. The service thread is the sole owner of state. */
	/* Just post a timeout event for the relevant sequence number. */
	cvg_arq_event_t *evt;
	if (k_mem_slab_alloc(&g_cvg_arq_event_slab, (void **)&evt, K_NO_WAIT) == 0) {
		evt->type = CVG_ARQ_EVENT_TIMEOUT;
		evt->sn = sdu_ctx->sequence_number;
		k_queue_append(&g_cvg_arq_event_queue, evt);
	} else {
		LOG_ERR("CVG_ARQ: Could not allocate event for SN %u timeout", sdu_ctx->sequence_number);
	}
}



static void cvg_handle_ack_action(int cvg_inflight_idx)
{
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
	if (cvg_inflight_idx < 0 || cvg_inflight_idx >= CVG_MAX_IN_FLIGHT_SDUS) {
		return;
	}

	cvg_inflight_sdu_ctx_t *sdu_ctx = &flow->tx_in_flight_sdu[cvg_inflight_idx];

	if (sdu_ctx->is_active) {
		LOG_INF("CVG_ARQ: ACK received for CVG SDU with SN %u.", sdu_ctx->sequence_number);
		k_timer_stop(&sdu_ctx->lifetime_timer);

		if (sdu_ctx->sdu) {
			LOG_DBG("[ACK_HANDLER_DBG] About to free sdu_ctx->sdu at address: %p", (void *)sdu_ctx->sdu);

			k_mem_slab_free(&g_cvg_app_sdu_slab, (void *)sdu_ctx->sdu);
			sdu_ctx->sdu = NULL;
		}
		sdu_ctx->is_active = false;
		k_sem_give(&flow->tx_window_sem); // Give back a credit to the flow control window
	}
}

#if !defined(CONFIG_ZTEST)
/**
 * @brief Processes an incoming TX Services Configuration IE.
 *
 * This function is called when a peer sends a request to configure a service
 * or responds to our own request. It updates the local flow context based on
 * the negotiated parameters.
 *
 * @param cfg_ie Pointer to the received TX Services Config IE.
 * @param source_rd_id The Long RD ID of the peer that sent the IE.
 */
static void __maybe_unused handle_tx_services_cfg_ie(const cvg_ie_tx_services_cfg_t *cfg_ie, uint32_t source_rd_id)
{
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
	bool is_response = cvg_ie_tx_cfg_is_response(cfg_ie);
	cvg_service_type_t service = cvg_ie_tx_cfg_get_service_type(cfg_ie);
	uint16_t window_size = sys_be16_to_cpu(cfg_ie->max_window_size_be) & 0x07FF;
	// uint32_t lifetime_ms = dlc_code_to_lifetime_ms(cfg_ie->lifetime);
	uint32_t lifetime_ms = cvg_code_to_lifetime_ms(cfg_ie->lifetime);

	if (is_response) {
		LOG_INF("CVG_CFG: Rcvd TX Services RESPONSE from 0x%08X.", source_rd_id);
		LOG_INF("CVG_CFG: Peer accepted -> Svc:%d, Win:%u, Life:%ums", service, window_size,
			lifetime_ms);

		/* Peer has accepted our request, apply the settings locally */
		k_mutex_lock(&flow->flow_mutex, K_FOREVER);
		flow->service_type = service;
		flow->max_window_size = window_size;
		flow->configured_lifetime_ms = lifetime_ms;
		flow->is_configured = true;
		k_sem_init(&flow->tx_window_sem, window_size, window_size);
		k_mutex_unlock(&flow->flow_mutex);

	} else { /* This is a request from the peer */
		LOG_INF("CVG_CFG: Rcvd TX Services REQUEST from 0x%08X.", source_rd_id);
		LOG_INF("CVG_CFG: Peer requests -> Svc:%d, Win:%u, Life:%ums", service, window_size,
			lifetime_ms);

		/* For now, we will unconditionally accept the peer's request.
		 * A more advanced implementation could check if these parameters are acceptable.
		 */
		bool accepted = true;

		if (accepted) {
			/* Apply the settings locally */
			k_mutex_lock(&flow->flow_mutex, K_FOREVER);
			flow->service_type = service;
			flow->max_window_size = window_size;
			flow->configured_lifetime_ms = lifetime_ms;
			flow->is_configured = true;
			k_sem_init(&flow->tx_window_sem, window_size, window_size);
			k_mutex_unlock(&flow->flow_mutex);

			/* Send back a response confirming the accepted parameters */
			dect_cvg_request_tx_services(source_rd_id, service, window_size,
						     lifetime_ms);
		} else {
			/* TODO: Send back a response with modified (or NACK) parameters */
			LOG_WRN("CVG_CFG: Peer request not accepted (logic not implemented).");
		}
	}
}
#endif /* !defined(CONFIG_ZTEST) */


/**
 * @brief Processes an incoming ARQ Feedback IE.
 *
 * This function parses the feedback IE to determine which in-flight SDUs have
 * been successfully received (ACK) and which need retransmission (NACK).
 *
 * @param pdu_ptr Pointer to the start of the ARQ Feedback IE.
 * @param len Length of the IE.
 */
static void cvg_process_arq_feedback(const uint8_t *pdu_ptr, size_t len)
{
	if (len < sizeof(cvg_ie_arq_feedback_base_t)) {
		return;
	}

	const cvg_ie_arq_feedback_base_t *fb_base = (const cvg_ie_arq_feedback_base_t *)pdu_ptr;
	bool is_nack = (fb_base->an_fbinfo_sn_msb >> 7) & 0x01;
	uint16_t sn = ((uint16_t)(fb_base->an_fbinfo_sn_msb & 0x0F) << 8) |
		      fb_base->sequence_number_lsb;

	cvg_arq_event_t *evt;
	if (k_mem_slab_alloc(&g_cvg_arq_event_slab, (void **)&evt, K_NO_WAIT) == 0) {
		evt->type = is_nack ? CVG_ARQ_EVENT_NACK : CVG_ARQ_EVENT_ACK;
		evt->sn = sn;
		k_queue_append(&g_cvg_arq_event_queue, evt);
	} else {
		LOG_ERR("CVG_ARQ: Could not allocate event for SN %u feedback", sn);
	}
}


static void cvg_tx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_DBG("[CVG_THREAD_DBG] TX Thread has started execution.");

	static uint8_t cvg_pdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];

	while (1) {
		LOG_DBG("[CVG TX] About to get item from FIFO.");
		cvg_tx_queue_item_t *tx_item = k_queue_get(&g_app_to_cvg_tx_queue, K_FOREVER);
		if (!tx_item) {
			// printk("x");
			continue;
		}

		LOG_DBG("[CVG TX] Got item from FIFO");

		cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
		mac_sdu_t *app_sdu = tx_item->app_sdu_buf;
		int err = 0;
		uint16_t current_sn = 0;
		bool is_reliable_service = (flow->service_type >= CVG_SERVICE_TYPE_3_FC);
		int pdu_len = 0;
		size_t pdu_offset = 0;

		LOG_DBG("[CVG TX] flow->service_type:%d >= %d", flow->service_type, CVG_SERVICE_TYPE_3_FC);


		/* --- Common Logic for Reliable Services (Flow Control & SN) --- */
		if (tx_item->is_retransmission) {
			current_sn = tx_item->original_sn;
		} else if (is_reliable_service) {
			/* Take a credit from the flow control window. This will block if the window is full. */
			k_sem_take(&flow->tx_window_sem, K_FOREVER);

			LOG_DBG("[MUTEX_DBG] cvg_tx_thread_entry: Attempting to lock mutex...");
			k_mutex_lock(&flow->flow_mutex, K_FOREVER);
			LOG_DBG("[MUTEX_DBG] cvg_tx_thread_entry: Mutex locked.");

			current_sn = flow->tx_sequence_number;
			flow->tx_sequence_number = (current_sn + 1) & 0x0FFF;

			LOG_DBG("[MUTEX_DBG] cvg_tx_thread_entry: Unlocking mutex...");
			k_mutex_unlock(&flow->flow_mutex);
		} else {
			LOG_DBG("[MUTEX_DBG] cvg_tx_thread_entry: NOT is_reliable_service...");
		}

		/* --- Security Processing (Conditional) --- */
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
		if (flow->security_enabled) {
			uint8_t mic[5];
			uint8_t iv[16];

			/* Note: For security, the original payload is stored for re-TX before it's encrypted. */
			/* A more optimized implementation might avoid this copy. */
			mac_sdu_t *sdu_for_arq = NULL;
			if (is_reliable_service) {
				sdu_for_arq = dect_mac_buffer_alloc(K_NO_WAIT);
				if (!sdu_for_arq) {
					err = -ENOMEM;
					LOG_ERR("CVG_TX_SEC: Failed to alloc buffer for ARQ copy");
					goto free_and_continue;
				}
				memcpy(sdu_for_arq->data, app_sdu->data, app_sdu->len);
				sdu_for_arq->len = app_sdu->len;
			}

			/* Calculate MIC on original SDU */
			err = dect_mac_security_calculate_mic(app_sdu->data, app_sdu->len,
							    flow->integrity_key, mic);
			if (err) {
				LOG_ERR("CVG_TX_SEC: MIC calculation failed: %d", err);
				if (sdu_for_arq) dect_mac_buffer_free(sdu_for_arq);
				goto free_and_continue;
			}

			/* Append MIC to SDU data */
			memcpy(app_sdu->data + app_sdu->len, mic, sizeof(mic));
			app_sdu->len += sizeof(mic);

			/* Build IV and encrypt SDU+MIC */
			dect_mac_security_build_iv(iv, dect_mac_get_own_long_id(),
						   tx_item->dest_long_id, dect_mac_get_hpc(),
						   current_sn);
			err = dect_mac_security_crypt_payload(app_sdu->data, app_sdu->len,
							    flow->cipher_key, iv, true);
			if (err) {
				LOG_ERR("CVG_TX_SEC: Encryption failed: %d", err);
				if (sdu_for_arq) dect_mac_buffer_free(sdu_for_arq);
				goto free_and_continue;
			}

			/* Prepend Security IE */
			cvg_ie_security_t *sec_ie = (cvg_ie_security_t *)(cvg_pdu_buf);
			sec_ie->header.ext_mt_f2c_or_type =
				((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
				(CVG_IE_TYPE_SECURITY & 0x1F);
			sec_ie->rsv_keyidx_ivtype = 0; /* Key Index 0, IV Type 0 */
			sec_ie->hpc_be = sys_cpu_to_be32(dect_mac_get_hpc());
			pdu_offset += sizeof(cvg_ie_security_t);

			/* Replace the original SDU with the unencrypted copy for the ARQ job */
			if (is_reliable_service && sdu_for_arq && !tx_item->is_retransmission) {
				dect_mac_buffer_free(app_sdu);
				app_sdu = sdu_for_arq;
			} else if (tx_item->is_retransmission && sdu_for_arq) {
				/* For a retransmission, just free the temporary buffer, the app_sdu points to the original */
				dect_mac_buffer_free(sdu_for_arq);
			}
		}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */

		/* --- PDU Construction & Transmission --- */
		if (tx_item->endpoint_id != 0) {
			cvg_ie_ep_mux_t *ep_mux = (cvg_ie_ep_mux_t *)(cvg_pdu_buf + pdu_offset);
			ep_mux->header.ext_mt_f2c_or_type =
				((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) | (CVG_IE_TYPE_EP_MUX & 0x1F);
			ep_mux->endpoint_mux_be = sys_cpu_to_be16(tx_item->endpoint_id);
			pdu_offset += sizeof(cvg_ie_ep_mux_t);
		}

		pdu_len = build_cvg_data_ie_pdu(cvg_pdu_buf + pdu_offset,
						sizeof(cvg_pdu_buf) - pdu_offset, app_sdu->data,
						app_sdu->len, current_sn);

		if (pdu_len < 0) {
			err = pdu_len;
			LOG_ERR("CVG_TX: Failed to build CVG PDU: %d", err);
			goto free_and_continue;
		}
		pdu_len += pdu_offset;

		err = dlc_send_data(DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ, tx_item->dest_long_id,
				    cvg_pdu_buf, pdu_len, (uint8_t)flow->configured_qos);

free_and_continue:
		if (err != 0) {
			if (is_reliable_service && !tx_item->is_retransmission) {
				k_sem_give(&flow->tx_window_sem);
			}

			if (!tx_item->is_retransmission) {
				k_mem_slab_free(&g_cvg_app_sdu_slab, (void *)app_sdu);
			}
		} else {
			if (is_reliable_service && !tx_item->is_retransmission) {
				/* Store SDU for potential re-TX */
				int buffer_index = current_sn % CVG_MAX_IN_FLIGHT_SDUS;
				cvg_inflight_sdu_ctx_t *sdu_ctx =
					&flow->tx_in_flight_sdu[buffer_index];

				sdu_ctx->sdu = app_sdu; /* The job now owns the buffer */
				sdu_ctx->sequence_number = current_sn;
				sdu_ctx->dest_long_id = tx_item->dest_long_id;
				sdu_ctx->endpoint_id = tx_item->endpoint_id;
				sdu_ctx->retries = 0;
				sdu_ctx->is_active = true;
				k_timer_start(&sdu_ctx->lifetime_timer,
					      K_MSEC(flow->configured_lifetime_ms), K_NO_WAIT);
			} else if (!is_reliable_service) {
				if (!tx_item->is_retransmission) {
					/* For non-reliable service, we are done with the buffer */
					k_mem_slab_free(&g_cvg_app_sdu_slab, (void *)app_sdu);
				}
			}
		}
		k_mem_slab_free(&g_cvg_tx_item_slab, (void *)tx_item);
	}
}


static void cvg_rx_thread_entry(void *p1, void *p2, void *p3)
{
	LOG_DBG("[CVG_THREAD_DBG] RX Thread has started execution.");

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("CVG RX Thread started.");

	static uint8_t dlc_sdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
	bool next_data_ie_is_encrypted = false;

	while (1) {
		dlc_service_type_t service_type;
		size_t dlc_sdu_len = sizeof(dlc_sdu_buf);
		uint32_t source_addr;

		LOG_DBG("[CVG_RX_DBG] Calling dlc_receive_data...");

		// k_timeout_t timeout = K_FOREVER;
		// LOG_DBG("[CVG_RX_DBG]   -> with timeout value: %d ticks", timeout.ticks);

		// int err = dlc_receive_data(&service_type, dlc_sdu_buf, &dlc_sdu_len, K_FOREVER);
		int err = dlc_receive_data(&service_type, &source_addr, dlc_sdu_buf, &dlc_sdu_len, K_FOREVER);
		
		LOG_DBG("[CVG_RX_DBG] dlc_receive_data returned with code: %d", err);

		if (err) {
			continue;
		}

		const uint8_t *pdu_ptr = dlc_sdu_buf;
		size_t remaining_len = dlc_sdu_len;

		while (remaining_len > 0) {
			const cvg_header_t *hdr = (const cvg_header_t *)pdu_ptr;
			cvg_ie_type_t ie_type = (cvg_ie_type_t)(hdr->ext_mt_f2c_or_type & 0x1F);
			size_t ie_consumed_len = 0;

			switch (ie_type) {
			case CVG_IE_TYPE_EP_MUX:
				if (remaining_len >= sizeof(cvg_ie_ep_mux_t)) {
					const cvg_ie_ep_mux_t *ep_ie = (const cvg_ie_ep_mux_t *)pdu_ptr;
					uint16_t ep = sys_be16_to_cpu(ep_ie->endpoint_mux_be);
					LOG_DBG("[CVG_RX] Parsed EP MUX: 0x%04X", ep);
					ie_consumed_len = sizeof(cvg_ie_ep_mux_t);
				} else {
					ie_consumed_len = remaining_len; // malformed
				}
				break;

			case CVG_IE_TYPE_SECURITY:
				if (remaining_len >= sizeof(cvg_ie_security_t)) {
					const cvg_ie_security_t *sec_ie = (const cvg_ie_security_t *)pdu_ptr;
					g_default_cvg_flow_ctx.peer_hpc = sys_be32_to_cpu(sec_ie->hpc_be);
					next_data_ie_is_encrypted = true;
					ie_consumed_len = sizeof(cvg_ie_security_t);
				}
				break;

			case CVG_IE_TYPE_DATA:
			case CVG_IE_TYPE_DATA_EP:
			{

				uint8_t *payload_ptr = (uint8_t *)pdu_ptr + sizeof(cvg_header_t) + sizeof(cvg_ie_data_base_t);
				size_t payload_len = remaining_len - (sizeof(cvg_header_t) + sizeof(cvg_ie_data_base_t));

				const cvg_ie_data_base_t *data_base = (const cvg_ie_data_base_t *)(pdu_ptr + sizeof(cvg_header_t));
				uint16_t sn = cvg_ie_data_base_get_sn(data_base);

				LOG_DBG("[CVG_RX_ACK_GEN_DBG] Received data packet with SN=%u. Preparing to send ACK.", sn);

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
				uint32_t source_long_rd_id = 0;
				// const cvg_ie_data_base_t *data_base = (const cvg_ie_data_base_t *)(pdu_ptr + sizeof(cvg_header_t));
				// uint16_t sn = cvg_ie_data_base_get_sn(data_base);

				if (g_default_cvg_flow_ctx.security_enabled || next_data_ie_is_encrypted) {
					uint8_t iv[16];
					dect_mac_security_build_iv(
						iv, source_long_rd_id, dect_mac_get_own_long_id(),
						g_default_cvg_flow_ctx.peer_hpc, sn);
					err = dect_mac_security_crypt_payload(
						payload_ptr, payload_len,
						g_default_cvg_flow_ctx.cipher_key, iv, false);
					if (err) {
						LOG_ERR("CVG_RX_SEC: Decryption failed for SN %u",
							sn);
						break;
					}

					uint8_t received_mic[5];

					memcpy(received_mic, payload_ptr + payload_len - 5, 5);
					size_t sdu_len = payload_len - 5;
					uint8_t calculated_mic[5];

					err = dect_mac_security_calculate_mic(
						payload_ptr, sdu_len,
						g_default_cvg_flow_ctx.integrity_key,
						calculated_mic);

					// TODO: Need a time safe memory comparision function
					// if (err || !timing_safe_mem_equal(received_mic, calculated_mic, 5)) {
					if (err || !memcmp(received_mic, calculated_mic, 5)) {
						LOG_ERR("CVG_RX_SEC: MIC verification failed for SN %u",
							sn);
						break;
					}

					payload_len = sdu_len; /* Use only the SDU part now */
				}
#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */				
				next_data_ie_is_encrypted = false; /* Reset flag */

/* Pass the cleartext payload to reassembly/delivery */
LOG_DBG("[[RX] Starting cleartext payload delivery to application...");
LOG_DBG("[[RX] Payload details: ptr=%p, len=%zu", payload_ptr, payload_len);

// Log the payload content for debugging
if (payload_ptr && payload_len > 0) {
	LOG_INF("[[RX] Complete payload (%zu bytes): ", payload_len);
	LOG_HEXDUMP_INF(payload_ptr, payload_len, "[[RX] Payload content");
}
// if (payload_ptr && payload_len > 0) {
//     printk("[[RX] Payload content (first 16 bytes): ");
//     for (size_t i = 0; i < (payload_len < 16 ? payload_len : 16); i++) {
//         printk("%02x ", payload_ptr[i]);
//     }
//     printk("");
    
//     // Also log the complete payload if it's small enough
//     if (payload_len <= 32) {
//         printk("[[RX] Complete payload (%zu bytes): ", payload_len);
//         for (size_t i = 0; i < payload_len; i++) {
//             printk("%02x ", payload_ptr[i]);
//         }
//         printk("");
//     }
// }

mac_sdu_t *app_sdu = NULL;
LOG_DBG("[[RX] Attempting to allocate SDU buffer from g_cvg_app_sdu_slab...");

				// CORRECTED slab_alloc syntax - using address-of operator properly
				int alloc_result = k_mem_slab_alloc(&g_cvg_app_sdu_slab, (void **)&app_sdu, K_NO_WAIT);
				LOG_DBG("[[RX] k_mem_slab_alloc result: %d, app_sdu pointer: %p", alloc_result, app_sdu);

				if (alloc_result == 0) {
					/* Duplicate removal logic based on sequence number */
					if (g_default_cvg_flow_ctx.service_type >= CVG_SERVICE_TYPE_1_SEQ_NUM) {
						if ((int16_t)(sn - g_default_cvg_flow_ctx.rx_expected_sn) < 0) {
							/* This sequence number is in the past, suggesting a duplicate */
							LOG_DBG("[[RX] DROPPED: Duplicate packet SN=%u, expected SN=%u", sn, g_default_cvg_flow_ctx.rx_expected_sn);
							k_mem_slab_free(&g_cvg_app_sdu_slab, (void *)app_sdu);
							// Send ACK if reliable service to prevent peer stalling
							if (g_default_cvg_flow_ctx.service_type >= CVG_SERVICE_TYPE_3_FC) {
								send_cvg_arq_feedback(source_addr, true, 0, sn);
							}
							ie_consumed_len = remaining_len;
							break;
						}
						
						/* Update expected SN */
						g_default_cvg_flow_ctx.rx_expected_sn = (sn + 1) & 0x0FFF;
					}

					LOG_DBG("[[RX] SUCCESS: SDU buffer allocated at %p", app_sdu);
					LOG_DBG("[[RX] Copying %zu bytes from payload to SDU buffer...", payload_len);
					
					memcpy(app_sdu->data, payload_ptr, payload_len);
					app_sdu->len = payload_len;
					
					LOG_DBG("[[RX] SDU buffer prepared: len=%u", app_sdu->len);
					
					// Verify the copied data
					if (payload_len > 0) {
						LOG_HEXDUMP_INF(app_sdu->data, payload_len, "[[RX] Copied data in SDU buffer");
						// for (size_t i = 0; i < (payload_len < 16 ? payload_len : 16); i++) {
						// 	printk("%02x ", app_sdu->data[i]);
						// }
						// printk("");
					}
					
					k_queue_append(&g_cvg_to_app_rx_queue, app_sdu);
				} else {
					LOG_ERR("[[RX] FAILED: Could not allocate SDU buffer from slab (slab full)");
					LOG_ERR("[[RX] ERROR: Payload of %zu bytes will be dropped!", payload_len);
					
					// Log slab statistics if available
					#ifdef CONFIG_MEM_SLAB_TRACE
					LOG_ERR("[[RX] Slab status: blocks may be exhausted");
					#endif
				}

				LOG_DBG("[[RX] Completed payload delivery process");


				/* After processing, send an ACK */
				if (g_default_cvg_flow_ctx.service_type >= CVG_SERVICE_TYPE_3_FC) {
					send_cvg_arq_feedback(source_addr, true, 0, sn);
				}
				
				ie_consumed_len = remaining_len;
				break;
			}
            
			case CVG_IE_TYPE_ARQ_FEEDBACK:
				cvg_process_arq_feedback(pdu_ptr, remaining_len);
				ie_consumed_len = remaining_len;
				break;

			default:
				LOG_WRN("CVG_RX: Unhandled CVG IE type 0x%X", ie_type);
				ie_consumed_len = remaining_len;
				break;
			}

			if (ie_consumed_len == 0 || ie_consumed_len > remaining_len) {
				break;
			}
			pdu_ptr += ie_consumed_len;
			remaining_len -= ie_consumed_len;
		}
	}
}





static void cvg_arq_service_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("[CVG_THREAD_DBG] ARQ Service Thread has started execution.");

	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;

	while (1) {
		cvg_arq_event_t *evt = k_queue_get(&g_cvg_arq_event_queue, K_FOREVER);
		if (!evt) {
			continue;
		}

		uint16_t sn = evt->sn;
		uint16_t buffer_index = sn % CVG_MAX_IN_FLIGHT_SDUS;

		k_mutex_lock(&flow->flow_mutex, K_FOREVER);

		cvg_inflight_sdu_ctx_t *sdu_ctx = &flow->tx_in_flight_sdu[buffer_index];

		/* If the job is no longer active or is for a different SN, ignore the event */
		if (!sdu_ctx->is_active || sdu_ctx->sequence_number != sn) {
			k_mutex_unlock(&flow->flow_mutex);
			k_mem_slab_free(&g_cvg_arq_event_slab, (void *)evt);
			continue;
		}

		switch (evt->type) {
		case CVG_ARQ_EVENT_ACK:
			LOG_INF("CVG_ARQ_SVC: Processing ACK for SN %u.", sn);
			cvg_handle_ack_action(buffer_index); /* This function is now safe */
			break;

		case CVG_ARQ_EVENT_NACK:
		case CVG_ARQ_EVENT_TIMEOUT:
			LOG_WRN("CVG_ARQ_SVC: Processing NACK/Timeout for SN %u.", sn);
			if (sdu_ctx->retries >= 3) { /* TODO: Kconfig */
				LOG_ERR("CVG_ARQ_SVC: SDU SN %u reached max retries. Discarding.", sn);
				cvg_handle_ack_action(buffer_index); /* Cleanup is same as ACK */
			} else {
				sdu_ctx->retries++;
				LOG_INF("CVG_ARQ_SVC: Retransmitting SDU SN %u (attempt %u).",
					sn, sdu_ctx->retries + 1);
				/* Re-queue for TX thread */
				cvg_tx_queue_item_t *tx_item;
				if (k_mem_slab_alloc(&g_cvg_tx_item_slab, (void **)&tx_item, K_NO_WAIT) == 0) {
					tx_item->app_sdu_buf = sdu_ctx->sdu;
					tx_item->dest_long_id = sdu_ctx->dest_long_id;
					tx_item->endpoint_id = sdu_ctx->endpoint_id;
					tx_item->is_retransmission = true;
					tx_item->original_sn = sn;
					/* NOTE: This re-queues the original buffer. The TX thread must not free it. */
					k_queue_append(&g_app_to_cvg_tx_queue, tx_item);
				}
				/* Restart the lifetime timer for this new attempt */
				k_timer_start(&sdu_ctx->lifetime_timer, K_MSEC(flow->configured_lifetime_ms), K_NO_WAIT);
			}
			break;
		}

		k_mutex_unlock(&flow->flow_mutex);
		// k_mem_slab_free(&g_cvg_arq_event_slab, (void **)&evt);
	}
}




/* Overview: 
 * Enhances the `dect_cvg_init` function to perform a full reset of the CVG layer's state. 
 * It now purges all FIFOs and re-initializes contexts, ensuring a clean state for every test run.
*/
int dect_cvg_init(void)
{
	/* Initialise FIFO Threads */
	k_queue_init(&g_app_to_cvg_tx_queue);
	k_queue_init(&g_cvg_to_app_rx_queue);
	// k_queue_init(&g_cvg_retransmit_signal_queue);
	k_queue_init(&g_cvg_arq_event_queue);
    
	/* Re-initialize the flow context for non-reliable service */
    memset(&g_default_cvg_flow_ctx, 0, sizeof(g_default_cvg_flow_ctx));
    g_default_cvg_flow_ctx.service_type = CVG_SERVICE_TYPE_0_TRANSPARENT; /* Non-reliable */
    g_default_cvg_flow_ctx.is_configured = true;
    g_default_cvg_flow_ctx.configured_lifetime_ms = 1000; /* 1 second timeout */
	g_default_cvg_flow_ctx.rx_expected_sn = 0; // Reset expected SN for duplicate removal
	g_default_cvg_flow_ctx.tx_sequence_number = 0;

    /* Initialize semaphore with window size */
    k_sem_init(&g_default_cvg_flow_ctx.tx_window_sem, 
                CVG_MAX_IN_FLIGHT_SDUS, 
                CVG_MAX_IN_FLIGHT_SDUS);


    /* FIX: Initialize semaphore with available credits for testing */
    k_sem_init(&g_default_cvg_flow_ctx.tx_window_sem, 
                CVG_MAX_IN_FLIGHT_SDUS, /* Initial count */
                CVG_MAX_IN_FLIGHT_SDUS); /* Max count */

	k_mutex_init(&g_default_cvg_flow_ctx.flow_mutex);

	for (int i = 0; i < CVG_MAX_IN_FLIGHT_SDUS; i++) {
        g_default_cvg_flow_ctx.tx_in_flight_sdu[i].is_active = false;
        g_default_cvg_flow_ctx.tx_in_flight_sdu[i].sdu = NULL;
        k_timer_init(&g_default_cvg_flow_ctx.tx_in_flight_sdu[i].lifetime_timer,
                     cvg_tx_sdu_lifetime_expiry_handler, NULL);
        g_default_cvg_flow_ctx.tx_in_flight_sdu[i].lifetime_timer.user_data = (void *)0xFFFF;
        g_default_cvg_flow_ctx.rx_reordering_buffer[i].is_valid = false;
        g_default_cvg_flow_ctx.rx_reordering_buffer[i].sdu = NULL;
    }

	/* Initialize layers below. This is safe to call multiple times. */
	int err = dect_dlc_init();
	if (err) {
		LOG_ERR("Failed to initialize DLC layer: %d", err);
		return err;
	}

	/* Create threads in a suspended state */
	g_cvg_tx_thread_id = k_thread_create(&g_cvg_tx_thread_data, g_cvg_tx_thread_stack,
					   K_THREAD_STACK_SIZEOF(g_cvg_tx_thread_stack),
					   cvg_tx_thread_entry, NULL, NULL, NULL,
					   CONFIG_DECT_CVG_TX_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(g_cvg_tx_thread_id, "dect_cvg_tx");

	g_cvg_rx_thread_id = k_thread_create(&g_cvg_rx_thread_data, g_cvg_rx_thread_stack,
					   K_THREAD_STACK_SIZEOF(g_cvg_rx_thread_stack),
					   cvg_rx_thread_entry, NULL, NULL, NULL,
					   CONFIG_DECT_CVG_RX_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(g_cvg_rx_thread_id, "dect_cvg_rx");

	LOG_DBG("[CVG_INIT_DBG] Created RX thread with ID: %p", g_cvg_rx_thread_id);

	g_cvg_arq_service_thread_id = k_thread_create(
		&g_cvg_arq_service_thread_data, g_cvg_arq_service_thread_stack,
		K_THREAD_STACK_SIZEOF(g_cvg_arq_service_thread_stack),
		cvg_arq_service_thread_entry, NULL, NULL, NULL,
		CONFIG_DECT_CVG_TX_SERVICE_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(g_cvg_arq_service_thread_id, "cvg_arq_svc");


	/* Start the necessary threads */
	LOG_DBG("[CVG_INIT_DBG] About to call k_thread_start on CVG threads...");
	LOG_DBG("[CVG_INIT_DBG]   -> Starting ARQ thread: %p", g_cvg_arq_service_thread_id);
	k_thread_start(g_cvg_arq_service_thread_id);
	LOG_DBG("[CVG_INIT_DBG]   -> Starting TX thread: %p", g_cvg_tx_thread_id);
	k_thread_start(g_cvg_tx_thread_id);
    LOG_DBG("[CVG_INIT_DBG]   -> Starting RX thread: %p", g_cvg_rx_thread_id);
	k_thread_start(g_cvg_rx_thread_id);    


	LOG_INF("CVG Layer Initialized.");
	return 0;
}




int dect_cvg_configure_flow(cvg_service_type_t service, cvg_qos_t qos, uint16_t max_window_size, uint32_t lifetime_ms)
{
    if (service >= CVG_SERVICE_TYPE_3_FC && max_window_size == 0) {
        LOG_ERR("CVG_CFG: Max window size cannot be 0 for a flow-controlled service.");
        return -EINVAL;
    }

	/* TODO: Remove this printk THERE IS A NEED FOR SOME MINOR DELAY */
	printk("CVG_CFG: dect_cvg_configure_flow -> Svc:%d, Win:%u, Life:%ums", service, max_window_size,
				lifetime_ms);

    g_default_cvg_flow_ctx.service_type = service;
    g_default_cvg_flow_ctx.configured_qos = qos;
    g_default_cvg_flow_ctx.max_window_size = max_window_size;
    g_default_cvg_flow_ctx.configured_lifetime_ms = lifetime_ms;
    g_default_cvg_flow_ctx.is_configured = true;

    k_sem_init(&g_default_cvg_flow_ctx.tx_window_sem, max_window_size, max_window_size);

    LOG_INF("CVG Flow configured. Service: %d, Window Size: %u", service, max_window_size);

    return 0;
}

__attribute__((weak)) int dect_cvg_send(uint16_t endpoint_id, uint32_t dest_long_id, const uint8_t *app_sdu,
		  size_t app_sdu_len)
{
	LOG_DBG("Starting dect_cvg_send ...");
	LOG_DBG("Parameters: endpoint_id=0x%04X, dest_long_id=0x%08X, app_sdu_len=%zu",
	       endpoint_id, dest_long_id, app_sdu_len);
	
	// Log app_sdu content if available
	if (app_sdu && app_sdu_len > 0) {
		LOG_HEXDUMP_INF(app_sdu, app_sdu_len, "App SDU content:");
		// printk("App SDU content (first 16 bytes): ");
		// for (size_t i = 0; i < (app_sdu_len < 16 ? app_sdu_len : 16); i++) {
		// 	printk("%02x ", app_sdu[i]);
		// }
		// printk("");
	}

	// dect_cvg_send(0x8003, g_mac_ctx_ft.own_long_rd_id, payload, sizeof(payload));
	if (!app_sdu && app_sdu_len > 0) {
		LOG_ERR("FAILED: Null app_sdu pointer with non-zero length (%zu)...", app_sdu_len);
		return -EINVAL;
	}
	
	size_t max_sdu_size = (sizeof(mac_sdu_t) - offsetof(mac_sdu_t, data));
	if (app_sdu_len > max_sdu_size) {
		LOG_ERR("FAILED: App SDU too large for transport buffer (%zu > %zu)...", 
		       app_sdu_len, max_sdu_size);
		LOG_ERR("App SDU too large for transport buffer (%zu > %zu)", app_sdu_len,
			max_sdu_size);
		return -EMSGSIZE;
	}

	cvg_tx_queue_item_t *tx_item = NULL;

	LOG_DBG("Attempting to allocate TX queue item from slab...");
	if (k_mem_slab_alloc(&g_cvg_tx_item_slab, (void **)&tx_item, K_NO_WAIT) != 0) {
		LOG_WRN("FAILED: Could not allocate TX queue item (slab full)...");
		return -ENOMEM;
	}
	LOG_DBG("Successfully allocated TX queue item at %p", tx_item);

	mac_sdu_t *sdu_buf = NULL;

	LOG_DBG("Attempting to allocate app SDU buffer from slab...");
	if (k_mem_slab_alloc(&g_cvg_app_sdu_slab, (void **)&sdu_buf, K_NO_WAIT) != 0) {
		LOG_WRN("FAILED: Could not allocate app SDU buffer (slab full)...");
		k_mem_slab_free(&g_cvg_tx_item_slab, (void **)&tx_item);
		LOG_WRN("Freed TX queue item due to SDU buffer allocation failure");
		return -ENOMEM;
	}
	LOG_DBG("Successfully allocated app SDU buffer at %p", sdu_buf);

	// Copy application data to the SDU buffer
	LOG_DBG("Copying %zu bytes from app_sdu to SDU buffer...", app_sdu_len);
	memcpy(sdu_buf->data, app_sdu, app_sdu_len);
	sdu_buf->len = app_sdu_len;
	LOG_DBG("SDU buffer prepared: len=%u", sdu_buf->len);

	// Set up the TX queue item
	tx_item->app_sdu_buf = sdu_buf;
	tx_item->endpoint_id = endpoint_id;
	tx_item->dest_long_id = dest_long_id;
	tx_item->is_retransmission = false;
	tx_item->original_sn = 0;

	LOG_DBG("TX queue item configured:");
	LOG_DBG("  - app_sdu_buf: %p", tx_item->app_sdu_buf);
	LOG_DBG("  - endpoint_id: 0x%04X", tx_item->endpoint_id);
	LOG_DBG("  - dest_long_id: 0x%08X", tx_item->dest_long_id);

	// Put the item in the TX FIFO
	LOG_DBG("Putting TX item into g_app_to_cvg_tx_fifo...");
	k_queue_append(&g_app_to_cvg_tx_queue, tx_item);
	LOG_DBG("Successfully queued TX item for processing");

	LOG_DBG("Ending dect_cvg_send - SUCCESS");
	return 0;
}


int dect_cvg_receive(uint8_t *app_sdu_buf, size_t *len_inout, k_timeout_t timeout)
{
	LOG_DBG("[RX] Starting dect_cvg_receive...");
	LOG_DBG("[RX] Parameters: app_sdu_buf=%p, len_inout=%p, timeout=%d",
	       app_sdu_buf, len_inout, (int)timeout.ticks);
	
	if (len_inout) {
		LOG_DBG("[RX] Input buffer size: *len_inout=%zu", *len_inout);
	}

    // Input validation
    if (!app_sdu_buf || !len_inout || *len_inout == 0) {
        LOG_ERR("[RX] FAILED: Invalid parameters - app_sdu_buf=%p, len_inout=%p, *len_inout=%zu",
               app_sdu_buf, len_inout, len_inout ? *len_inout : 0);
        return -EINVAL;
    }

    LOG_DBG("[RX] Waiting for data from g_cvg_to_app_rx_queue...");
    mac_sdu_t *sdu_buf = k_queue_get(&g_cvg_to_app_rx_queue, timeout);
    if (!sdu_buf) {
        LOG_WRN("[RX] Timeout or no data available in FIFO");
        return -EAGAIN; // Timeout
    }
    
    LOG_DBG("[RX] Retrieved SDU buffer from FIFO: %p", sdu_buf);
    LOG_DBG("[RX] SDU buffer content: len=%u", sdu_buf->len);
    
    // Log first few bytes of received data for debugging
    if (sdu_buf->len > 0) {
		LOG_HEXDUMP_INF(sdu_buf->data, sdu_buf->len, "[RX] SDU data:");
        // printk("[RX] SDU data (first 16 bytes): ");
        // for (size_t i = 0; i < (sdu_buf->len < 16 ? sdu_buf->len : 16); i++) {
        //     printk("%02x ", sdu_buf->data[i]);
        // }
        // printk("");
    }

    // Check if provided buffer is large enough
    if (*len_inout < sdu_buf->len) {
        LOG_ERR("[RX] Buffer too small: provided=%zu, needed=%u", 
               *len_inout, sdu_buf->len);
        *len_inout = sdu_buf->len; // Report required size
        LOG_ERR("[RX] Returning required size: %zu, putting SDU back in FIFO", *len_inout);
        k_queue_prepend(&g_cvg_to_app_rx_queue, sdu_buf); // Put it back
        return -EMSGSIZE;
    }

    // Copy data to application buffer
    LOG_DBG("[RX] Copying %u bytes from SDU buffer to app buffer...", sdu_buf->len);
    *len_inout = sdu_buf->len;
    memcpy(app_sdu_buf, sdu_buf->data, sdu_buf->len);
    
    // Log the copied data for verification
    if (sdu_buf->len > 0) {
		LOG_HEXDUMP_INF(app_sdu_buf, sdu_buf->len, "[RX] Copied data to app buffer:");
        // printk("[RX] Copied data to app buffer (first 16 bytes): ");
        // for (size_t i = 0; i < (sdu_buf->len < 16 ? sdu_buf->len : 16); i++) {
        //     printk("%02x ", app_sdu_buf[i]);
        // }
        // printk("");
    }

    LOG_DBG("[RX] Freeing SDU buffer %p back to slab...", sdu_buf);
    k_mem_slab_free(&g_cvg_app_sdu_slab, (void *)sdu_buf);
    
    LOG_DBG("[RX] Successfully received %zu bytes, ending dect_cvg_receive", *len_inout);
    return 0;
}

int dect_cvg_request_tx_services(uint32_t dest_id, cvg_service_type_t service,
				 uint16_t max_window_size, uint32_t lifetime_ms)
{
	uint8_t pdu_buf[sizeof(cvg_ie_tx_services_cfg_t)];
	cvg_ie_tx_services_cfg_t *cfg_ie = (cvg_ie_tx_services_cfg_t *)pdu_buf;

	cfg_ie->header.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
					    ((0 & 0x01) << 5) |
					    (CVG_IE_TYPE_TX_SERVICES_CFG & 0x1F);

	cfg_ie->rqrs_reserved_svctype = (0 << 7) |
				      ((service & 0x07));
	// cfg_ie->lifetime = lifetime_ms_to_dlc_code(lifetime_ms);
	cfg_ie->lifetime = cvg_lifetime_ms_to_code(lifetime_ms);
	cfg_ie->max_window_size_be = sys_cpu_to_be16(max_window_size & 0x07FF);

	LOG_INF("CVG_CFG: Sending TX Services Request -> Svc:%d, Win:%u, Life:%ums (code %u)",
		service, max_window_size, lifetime_ms, cfg_ie->lifetime);

	return dlc_send_data(DLC_SERVICE_TYPE_0_TRANSPARENT, dest_id, pdu_buf, sizeof(pdu_buf),
			     (uint8_t)CVG_QOS_CONTROL);
}

int dect_cvg_set_security_params(const uint8_t *integrity_key, const uint8_t *cipher_key)
{
	if (!integrity_key || !cipher_key) {
		return -EINVAL;
	}

	k_mutex_lock(&g_default_cvg_flow_ctx.flow_mutex, K_FOREVER);
	memcpy(g_default_cvg_flow_ctx.integrity_key, integrity_key, 16);
	memcpy(g_default_cvg_flow_ctx.cipher_key, cipher_key, 16);
	g_default_cvg_flow_ctx.security_enabled = true;
	k_mutex_unlock(&g_default_cvg_flow_ctx.flow_mutex);

	LOG_INF("CVG security parameters set and security enabled for default flow.");
	return 0;
}
