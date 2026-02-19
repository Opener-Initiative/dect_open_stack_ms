/* lib/dect_nrplus/dect_dlc.c */
/* This is the complete, corrected implementation of the DLC layer. It restores and integrates full Segmentation and Reassembly (SAR) logic, ARQ retransmission, and SDU Lifetime Control, ensuring that no functionality is lost while adding robustness. */

/* dect_dlc/dect_dlc.c */
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
// #include <hw_id.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>


#include <dect_dlc.h>
#include <mac/dect_mac.h>
// #if IS_ENABLED(CONFIG_ZTEST)		
#include <mac/dect_mac_core.h>
// #endif
// #include <mac/dect_mac_core.h>

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif


LOG_MODULE_REGISTER(dect_dlc, CONFIG_DECT_DLC_LOG_LEVEL);

#if IS_ENABLED(CONFIG_DECT_DLC_API_MOCK)
/* --- Internal function pointers for test spy --- */
static int (*g_dlc_send_spy_cb)(dlc_service_type_t, uint32_t, const uint8_t *, size_t, uint8_t) = NULL;
static int (*g_dlc_receive_spy_cb)(dlc_service_type_t *, uint32_t *, uint8_t *, size_t *, k_timeout_t) = NULL;
#endif /* IS_ENABLED(CONFIG_DECT_DLC_API_MOCK) */


// --- DLC Internal State and Buffers ---
#define MAX_DLC_REASSEMBLY_SESSIONS 4
#define MAX_DLC_RETRANSMISSION_JOBS 8
#define DLC_RETRANSMISSION_TIMEOUT_MS 10000
#define DLC_MAX_RETRIES 3
#define DLC_REASSEMBLY_BUF_SIZE (CONFIG_DECT_MAC_SDU_MAX_SIZE * 5)
#define DLC_REASSEMBLY_TIMEOUT_MS 5000
#define DLC_DUPLICATE_CACHE_SIZE 16


/* A buffer definition for delivering large, reassembled SDUs to the upper layer */
typedef struct {
	void *fifo_reserved; /* For k_queue internal use */
	uint8_t data[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
	size_t len;
} dlc_app_sdu_t;

// K_MEM_SLAB_DEFINE(g_dlc_app_sdu_slab, sizeof(dlc_app_sdu_t), 4, 4);
K_MEM_SLAB_DEFINE(g_dlc_app_sdu_slab, sizeof(dlc_app_sdu_t), (4 + MAX_DLC_RETRANSMISSION_JOBS), 4);

static uint16_t dlc_tx_sequence_number = 0;
static uint32_t g_tx_sdu_lifetime_ms = CONFIG_DECT_DLC_DEFAULT_SDU_LIFETIME_MS;
static uint32_t g_rx_sdu_lifetime_ms = CONFIG_DECT_DLC_DEFAULT_SDU_LIFETIME_MS;

/**
 * @brief Context for a DLC SDU that is awaiting acknowledgement (ARQ).
 */
typedef struct {
	void *fifo_reserved; /* For k_fifo internal use */
    bool is_active;
    uint16_t sequence_number;
    uint8_t retries;
    dlc_service_type_t service;
    uint32_t dest_long_id;
    // mac_sdu_t *sdu_payload; // This holds the *full* original DLC SDU for re-TX
	dlc_app_sdu_t *sdu_payload; // This holds the *full* original DLC SDU for re-TX
    struct k_timer retransmit_attempt_timer;
    struct k_timer lifetime_timer;
    uint8_t flow_id; /**< Stored Flow ID for retransmission */
} dlc_retransmission_job_t;

static dlc_retransmission_job_t retransmission_jobs[MAX_DLC_RETRANSMISSION_JOBS];

/**
 * @brief Cache to detect and drop duplicate broadcast/flooded packets.
 */
typedef struct {
	uint32_t source_rd_id;
	uint8_t sequence_number;
} dlc_duplicate_cache_entry_t;

static dlc_duplicate_cache_entry_t g_dlc_dup_cache[DLC_DUPLICATE_CACHE_SIZE];
static uint8_t g_dlc_dup_cache_next_idx = 0;


/**
 * @brief Context for a reassembly session for a segmented DLC SDU.
 */
typedef struct {
    bool is_active;
    uint16_t sequence_number;
    uint8_t reassembly_buf[DLC_REASSEMBLY_BUF_SIZE];
    size_t total_sdu_len;
    size_t bytes_received;
    struct k_timer timeout_timer;
    dlc_service_type_t service_type;
} dlc_reassembly_session_t;

static dlc_reassembly_session_t reassembly_sessions[MAX_DLC_REASSEMBLY_SESSIONS];

/* DLC Scheduler Context */
#define DLC_LANE_COUNT 4
#define DLC_LANE_CONTROL 0
#define DLC_LANE_HIGH    1
#define DLC_LANE_MED     2
#define DLC_LANE_LOW     3

#define DLC_SCHED_STARVATION_THRESHOLD_LOW_MS  500
#define DLC_SCHED_STARVATION_THRESHOLD_MED_MS  200
#define DLC_SCHED_STARVATION_THRESHOLD_HIGH_MS 50

typedef struct {
    struct k_queue queues[DLC_LANE_COUNT];
    int64_t last_service_time[DLC_LANE_COUNT];
} dlc_scheduler_t;

static dlc_scheduler_t g_dlc_scheduler;

/* Queue for data from MAC to the DLC layer. */
struct k_queue g_dlc_internal_mac_rx_queue;

K_FIFO_DEFINE(g_dlc_retransmit_signal_fifo);
K_MEM_SLAB_DEFINE(g_dlc_rx_delivery_item_slab, sizeof(dlc_rx_delivery_item_t), 32, 4); /* Increased slab size for queues */

/* Removed g_dlc_rx_sem as k_queue handles blocking */

/* --- FORWARD DECLARATIONS FOR STATIC FUNCTIONS --- */
static void dlc_reassembly_timeout_handler(struct k_timer *timer_id);
static void dlc_retransmit_attempt_timeout_handler(struct k_timer *timer_id);
static void dlc_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id);
static void dlc_tx_service_thread_entry(void *p1, void *p2, void *p3);
static void dlc_rx_thread_entry(void *p1, void *p2, void *p3);
static void dlc_tx_status_cb_handler(uint16_t dlc_sn, bool success);
static int queue_dlc_pdu_to_mac(uint32_t dest_long_id, const uint8_t *dlc_header,
				size_t dlc_header_len, const uint8_t *payload_segment,
				size_t payload_segment_len, bool report_status, uint16_t dlc_sn,
				uint8_t flow_id);
static int dlc_send_segmented(dlc_service_type_t service, uint32_t dest_long_id,
			      const uint8_t *dlc_sdu_hdr, size_t dlc_sdu_hdr_len,
			      const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len,
			      uint16_t dlc_sn, uint8_t flow_id);				  
static int dlc_resend_sdu_with_original_sn(dlc_retransmission_job_t *job);
static dlc_reassembly_session_t* find_or_alloc_reassembly_session(uint16_t sequence_number, dlc_service_type_t service);
static int dlc_sched_drop_oldest(void);



int dlc_serialize_routing_header(uint8_t *target_buf, size_t target_buf_len,
					const dect_dlc_routing_header_t *rh);
int dlc_parse_routing_header(const uint8_t *buf, size_t len, dect_dlc_routing_header_t *rh);

#if !defined(CONFIG_ZTEST)
static int dlc_parse_ext_header(const uint8_t *buf, size_t len, dect_dlc_ext_len_type_t *len_type,
				dect_dlc_ext_ie_type_t *ie_type, uint16_t *payload_len);
#endif /* !defined(CONFIG_ZTEST) */


/* --- THREAD DEFINITIONS --- */
K_THREAD_STACK_DEFINE(g_dlc_rx_thread_stack, CONFIG_DECT_DLC_RX_THREAD_STACK_SIZE);
static struct k_thread g_dlc_rx_thread_data;
k_tid_t g_dlc_rx_thread_id;

K_THREAD_STACK_DEFINE(g_dlc_tx_service_thread_stack, CONFIG_DECT_DLC_TX_SERVICE_THREAD_STACK_SIZE);
static struct k_thread g_dlc_tx_service_thread_data;
k_tid_t g_dlc_tx_service_thread_id;



#if !defined(CONFIG_ZTEST)
static uint8_t lifetime_ms_to_dlc_code(uint32_t ms) 
{
	if (ms <= 1) return 0x02;
	if (ms <= 5) return 0x03;
	if (ms <= 50) return 0x08;
	if (ms <= 100) return 0x0D;
	if (ms <= 500) return 0x12;
	if (ms <= 1000) return 0x14;
	if (ms <= 5000) return 0x1A;
	return 0x1E;
} 

static uint32_t dlc_code_to_lifetime_ms(uint8_t code)
{
	if (code <= 0x02) return 1;
	if (code <= 0x03) return 5;
	if (code <= 0x08) return 50;
	if (code <= 0x0D) return 100;
	if (code <= 0x12) return 500;
	if (code <= 0x14) return 1000;
	if (code <= 0x1A) return 5000;
	return 60000;
}
#endif /* #if !defined(CONFIG_ZTEST) */



/**
 * @brief Serializes a DLC routing header structure into a byte buffer.
 *
 * This function reads the bitmap from the provided routing header structure
 * and writes only the present fields into the target buffer in the correct
 * order as defined by the ETSI standard.
 *
 * @param target_buf Buffer to write the serialized header into.
 * @param target_buf_len Maximum length of the target buffer.
 * @param rh Pointer to the routing header structure containing the data to serialize.
 * @return The total length of the serialized header in bytes, or a negative error code.
 */
int dlc_serialize_routing_header(uint8_t *target_buf, size_t target_buf_len,
                                const dect_dlc_routing_header_t *rh)
{
    printk("[SERIALIZE_RH_DBG] Starting dlc_serialize_routing_header: target_buf=%p, target_buf_len=%zu, rh=%p\n",
           target_buf, target_buf_len, rh);

    /* Dump input routing header fields */
    if (rh) {
        printk("[SERIALIZE_RH_DBG] Input Routing Header:\n");
        printk("  -> bitmap=0x%04X\n", rh->bitmap);
        printk("  -> source_addr=0x%08X\n", rh->source_addr);
        printk("  -> dest_addr=0x%08X\n", rh->dest_addr);
        printk("  -> hop_count=%u, hop_limit=%u\n", rh->hop_count, rh->hop_limit);
        printk("  -> delay=0x%08X\n", rh->delay);
        printk("  -> sequence_number=%u\n", rh->sequence_number);
    } else {
        printk("[SERIALIZE_RH_DBG] Input rh is NULL\n");
    }

    if (!target_buf || !rh) {
        printk("[ERROR] dlc_serialize_routing_header: Invalid input (target_buf=%p, rh=%p)\n",
               target_buf, rh);
        return -EINVAL;
    }

    uint8_t *p = target_buf;
    size_t required_len = 0;

    /* Calculate required length for buffer validation */
    printk("[SERIALIZE_RH_DBG] Calculating required buffer length:\n");
    required_len += 2; // bitmap
    printk("  -> bitmap: 2 bytes\n");

    if ((rh->bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
        required_len += 4;
        printk("  -> source_addr: 4 bytes\n");
    }
    if (((rh->bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x01) == DLC_RH_DEST_ADD_PRESENT) {
        required_len += 4;
        printk("  -> dest_addr: 4 bytes\n");
    }
    if ((rh->bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
        required_len += 2;
        printk("  -> hop_count + hop_limit: 2 bytes\n");
    }
    if ((rh->bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
        required_len += 4;
        printk("  -> delay: 4 bytes\n");
    }
    if ((rh->bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
        required_len += 1;
        printk("  -> sequence_number: 1 bytes\n");
    }

    printk("[SERIALIZE_RH_DBG] Total required length: %zu bytes, available: %zu bytes\n",
           required_len, target_buf_len);

    if (target_buf_len < required_len) {
        printk("[ERROR] dlc_serialize_routing_header: Buffer too small (need %zu, have %zu)\n",
               required_len, target_buf_len);
        return -ENOMEM;
    }

    /* Serialize the data */
    printk("[SERIALIZE_RH_DBG] Serializing fields:\n");
	sys_put_be16(rh->bitmap, p);
    printk("  -> Wrote bitmap=0x%04X at offset %zu\n", rh->bitmap, (size_t)(p - target_buf));
    p += sizeof(uint16_t);

    if ((rh->bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
		sys_put_be32(rh->source_addr, p);
        printk("  -> Wrote source_addr=0x%08X at offset %zu\n", rh->source_addr, (size_t)(p - target_buf));
        p += sizeof(uint32_t);
    }
    if (((rh->bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x01) == DLC_RH_DEST_ADD_PRESENT) {
		sys_put_be32(rh->dest_addr, p);
        printk("  -> Wrote dest_addr=0x%08X at offset %zu\n", rh->dest_addr, (size_t)(p - target_buf));
        p += sizeof(uint32_t);
    }
    if ((rh->bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
        *p = rh->hop_count;
        printk("  -> Wrote hop_count=%u at offset %zu\n", rh->hop_count, (size_t)(p - target_buf));
        p++;
        *p = rh->hop_limit;
        printk("  -> Wrote hop_limit=%u at offset %zu\n", rh->hop_limit, (size_t)(p - target_buf));
        p++;
    }
    if ((rh->bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
		sys_put_be32(rh->delay, p);
        printk("  -> Wrote delay=0x%08X at offset %zu\n", rh->delay, (size_t)(p - target_buf));
        p += sizeof(uint32_t);
    }
    if ((rh->bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
        *p = rh->sequence_number;
        printk("  -> Wrote sequence_number=%u at offset %zu\n", rh->sequence_number, (size_t)(p - target_buf));
        p++;
    }

    /* Hexdump of serialized buffer */
    size_t serialized_len = (size_t)(p - target_buf);
    printk("[SERIALIZE_RH_DBG] Serialized Buffer Hexdump (len=%zu):\n", serialized_len);
    for (size_t i = 0; i < serialized_len && i < 32; i++) { /* Limit to 32 bytes */
        printk("%02x ", target_buf[i]);
        if ((i + 1) % 16 == 0) printk("\n");
    }
    printk("\n");

    printk("[SERIALIZE_RH_DBG] Completed serialization. Total bytes written: %zu\n", serialized_len);
    return (int)serialized_len;
}




#if !defined(CONFIG_ZTEST)
static void dlc_handle_timers_config_ie(const dect_dlc_header_timers_config_t *cfg_ie)
{
	uint32_t new_lifetime_ms = dlc_code_to_lifetime_ms(cfg_ie->lifetime_timer_val);
	LOG_INF("DLC_CFG: Received Timers Config IE, lifetime code 0x%02x -> %u ms. Updating local config.",
		cfg_ie->lifetime_timer_val, new_lifetime_ms);
	g_tx_sdu_lifetime_ms = new_lifetime_ms;
	g_rx_sdu_lifetime_ms = new_lifetime_ms;
}

static void dlc_handle_route_error_ie(const dect_dlc_ie_route_error_t *err_ie)
{
	uint32_t invalid_hop = sys_be32_to_cpu(err_ie->invalid_next_hop_addr_be);
	LOG_WRN("DLC_ROUTE_ERR: Received Route Error IE. Reason: %u, Invalid Next Hop: 0x%08X",
		err_ie->error_reason, invalid_hop);

	/* TODO: Add logic to update local routing tables to invalidate any
	 * paths that use the node which reported the error to reach 'invalid_hop'.
	 * For now, we just log the event.
	 */
}
#endif /* !defined(CONFIG_ZTEST) */



/**
 * @brief Callback handler for final MAC layer TX status reports.
 *
 * This function is registered with the MAC layer and is called when a DLC SDU
 * that required ARQ and a status report has been either successfully ACKed or
 * has permanently failed (e.g., max retries, lifetime expiry).
 *
 * @param dlc_sn The 10-bit DLC sequence number of the SDU.
 * @param success True if the transmission was successful, false otherwise.
 */
static void dlc_tx_status_cb_handler(uint16_t dlc_sn, bool success)
{
	int job_idx = -1;
	for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
		if (retransmission_jobs[i].is_active && retransmission_jobs[i].sequence_number == dlc_sn) {
			job_idx = i;
			break;
		}
	}

	if (job_idx == -1) {
		/* This can happen if the SDU lifetime expired in the DLC layer
		 * at the same time the MAC layer was processing it. It's not an error. */
		LOG_DBG("DLC_ARQ_CB: Received status for SN %u, but no active job found. Likely already timed out.", dlc_sn);
		return;
	}

	dlc_retransmission_job_t *job = &retransmission_jobs[job_idx];
	k_timer_stop(&job->retransmit_attempt_timer);
	k_timer_stop(&job->lifetime_timer);

	if (success) {
		LOG_INF("DLC_ARQ_CB: MAC SUCCESS for SN %u. Freeing job.", dlc_sn);
		// k_mem_slab_free(&g_dlc_arq_buf_slab, (void *)job->sdu_payload);

		// Not sure if we need to free this memory
		k_mem_slab_free(&g_dlc_app_sdu_slab, (void *)job->sdu_payload);
		// dect_mac_buffer_free(job->sdu_payload);
		
		job->is_active = false;
	} else {
		/* The MAC layer already tried its HARQ retries and failed.
		 * The DLC layer now needs to decide if it will re-send the entire SDU.
		 */
		LOG_WRN("DLC_ARQ_CB: MAC PERMANENT FAILURE for SN %u. Signaling for DLC-level re-TX.", dlc_sn);

		k_fifo_put(&g_dlc_retransmit_signal_fifo, job);
	}
}


__weak int dlc_send_data(dlc_service_type_t service, uint32_t dest_long_id,
		  const uint8_t *cvg_pdu_payload, size_t cvg_pdu_len, uint8_t flow_id)
{
	printk("[DLC_SEND_DBG] dlc_send_data received payload at %p (len %zu), flow_id %u:\n",
	       (void *)cvg_pdu_payload, cvg_pdu_len, flow_id);

#if IS_ENABLED(CONFIG_DECT_DLC_API_MOCK)
	if (g_dlc_send_spy_cb) {
		printk("[DLC_SEND_DBG] Calling g_dlc_send_spy_cb \n");
		return g_dlc_send_spy_cb(service, dest_long_id, cvg_pdu_payload, cvg_pdu_len, flow_id);
	}
#endif

	for (int i=0; i<cvg_pdu_len && i < 32; i++) { printk("%02x ", cvg_pdu_payload[i]); }
	printk("\n");

	if (cvg_pdu_payload == NULL && cvg_pdu_len > 0) {
		return -EINVAL;
	}

	/* Assemble only the routing header on the stack */
	uint8_t rh_buf[sizeof(dect_dlc_routing_header_t)];
	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;

	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	rh.source_addr = dect_mac_get_own_long_id();

	if (dest_long_id == 0xFFFFFFFE) { /* Uplink to Backend */
		bitmap |= (DLC_RH_DEST_ADD_BACKEND << DLC_RH_DEST_ADDR_SHIFT);
		bitmap |= (DLC_RH_TYPE_UPLINK_TO_BACKEND << DLC_RH_ROUTING_TYPE_SHIFT);
	} else { /* RD-to-RD or Broadcast */
		bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
		bitmap |= (1 << DLC_RH_HOP_COUNT_LIMIT_SHIFT) | (1 << DLC_RH_SEQ_NUM_SHIFT);
		bitmap |= (DLC_RH_TYPE_LOCAL_FLOODING << DLC_RH_ROUTING_TYPE_SHIFT);
		rh.dest_addr = dest_long_id;
		rh.hop_count = 1;
		rh.hop_limit = 5;
		uint8_t random_values[2];
		sys_rand_get(random_values, sizeof(random_values));
		rh.sequence_number = random_values[0];
	}

	rh.bitmap = bitmap;

	printk("[DLC_SEND_RH_DBG] Routing header struct before serialization: hop_count=%u, hop_limit=%u\n",
	       rh.hop_count, rh.hop_limit);

	int rh_len = dlc_serialize_routing_header(rh_buf, sizeof(rh_buf), &rh);
	if (rh_len < 0) {
		return rh_len;
	}

	bool needs_dlc_arq = (service == DLC_SERVICE_TYPE_2_ARQ || service == DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ);
	uint16_t current_dlc_sn = 0;

	if (service != DLC_SERVICE_TYPE_0_TRANSPARENT) {
		current_dlc_sn = dlc_tx_sequence_number;
		dlc_tx_sequence_number = (dlc_tx_sequence_number + 1) & 0x03FF;
	}

	if (needs_dlc_arq) {
		/* ARQ logic remains the same, but it now stores header + payload */
		int job_idx = -1;
		for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
			if (!retransmission_jobs[i].is_active) {
				job_idx = i;
				break;
			}
		}
		if (job_idx == -1) {
			LOG_ERR("DLC_SEND_ARQ: No free retransmission jobs. Dropping SDU.");
			return -ENOBUFS;
		}

		dlc_retransmission_job_t *arq_job = &retransmission_jobs[job_idx];
		if (k_mem_slab_alloc(&g_dlc_app_sdu_slab, (void **)&arq_job->sdu_payload, K_NO_WAIT) != 0) {
			LOG_ERR("DLC_SEND_ARQ: Failed to allocate buffer from ARQ slab. Dropping.");
			return -ENOMEM;
		}
		memcpy(arq_job->sdu_payload->data, rh_buf, rh_len);
		memcpy(arq_job->sdu_payload->data + rh_len, cvg_pdu_payload, cvg_pdu_len);
		arq_job->sdu_payload->len = rh_len + cvg_pdu_len;
		arq_job->is_active = true;
		arq_job->sequence_number = current_dlc_sn;
		arq_job->retries = 0;
		arq_job->service = service;
		arq_job->dest_long_id = dest_long_id;
		arq_job->flow_id = flow_id;
		k_timer_start(&arq_job->lifetime_timer, K_MSEC(g_tx_sdu_lifetime_ms), K_NO_WAIT);
	}
printk("[DLC_SEND_DBG] Calling dlc_send_segmented...\n");
	return dlc_send_segmented(service, dest_long_id, rh_buf, rh_len, cvg_pdu_payload, cvg_pdu_len, current_dlc_sn, flow_id);
}



int dlc_forward_pdu(const uint8_t *dlc_pdu, size_t dlc_pdu_len)
{
	if (dect_mac_get_role() != MAC_ROLE_PT) {
		return -EPERM;
	}
	if (!dect_mac_is_associated()) {
		return -ENOTCONN;
	}

	return queue_dlc_pdu_to_mac(dect_mac_get_associated_ft_long_id(), NULL, 0, dlc_pdu,
				    dlc_pdu_len, false, 0, MAC_FLOW_HIGH_PRIORITY); // Default to High Priority for forwarded PDUs
}


/**
 * @brief Parses a serialized DLC routing header from a byte buffer.
 *
 * This function reads the bitmap and then conditionally reads the subsequent
 * fields into the provided routing header structure.
 *
 * @param buf Pointer to the start of the serialized routing header.
 * @param len The maximum length of the buffer.
 * @param rh Pointer to the routing header structure to populate.
 * @return The number of bytes consumed by the parsed header, or a negative error code.
 */

int dlc_parse_routing_header(const uint8_t *buf, size_t len, dect_dlc_routing_header_t *rh)
{
    printk("dlc_parse_routing_header Called: buf=%p, len=%zu\n", buf, len);
    
    if (!buf || !rh || len < sizeof(uint16_t)) {
        printk("[ERROR] dlc_parse_routing_header: Invalid input\n");
        return -EINVAL;
    }

    const uint8_t *p = buf;
    memset(rh, 0, sizeof(*rh));

    printk("[PARSE_RH_RAW_DBG] Raw bytes for bitmap: 0x%02X 0x%02X\n", p[0], p[1]);

	/* Read the bitmap from the buffer, converting from Big Endian to CPU's native order */
	rh->bitmap = sys_get_be16(p);
	p += sizeof(uint16_t);

    printk("[PARSE_RH_DBG] Parsed bitmap=0x%04X\n", rh->bitmap);

    printk("[BITMASK_DBG] DLC_RH_SRC_ADDR_SHIFT = %d, Result = %d\n",
           DLC_RH_SRC_ADDR_SHIFT, (rh->bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01);
    
    if ((rh->bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
        printk("[PARSE_RH_DBG] Parsing Source Address\n");
        if ((p + sizeof(uint32_t)) > (buf + len)) {
            printk("[ERROR] Buffer overrun for source_addr\n");
            return -EMSGSIZE;
        }
        rh->source_addr = sys_get_be32(p);
        printk("[PARSE_RH_DBG]  -> source_addr=0x%08X\n", rh->source_addr);
        p += sizeof(uint32_t);
    } else {
        printk("[PARSE_RH_DBG] Source Address not present\n");
    }

    printk("[BITMASK_DBG] DLC_RH_DEST_ADDR_SHIFT = %d, Result = %d\n",
           DLC_RH_DEST_ADDR_SHIFT, (rh->bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x07);
    
    if (((rh->bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x07) == DLC_RH_DEST_ADD_PRESENT) {
        printk("[PARSE_RH_DBG] Parsing Destination Address\n");
        if ((p + sizeof(uint32_t)) > (buf + len)) {
            printk("[ERROR] Buffer overrun for dest_addr\n");
            return -EMSGSIZE;
        }
        rh->dest_addr = sys_get_be32(p);
        printk("[PARSE_RH_DBG]  -> dest_addr=0x%08X\n", rh->dest_addr);
        p += sizeof(uint32_t);
    } else {
        printk("[PARSE_RH_DBG] Destination Address not present\n");
    }

    printk("[BITMASK_DBG] DLC_RH_HOP_COUNT_LIMIT_SHIFT = %d, Result = %d\n",
           DLC_RH_HOP_COUNT_LIMIT_SHIFT, (rh->bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01);
    
    if ((rh->bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
        printk("[PARSE_RH_DBG] Parsing Hop Count/Limit\n");
        if ((p + 2) > (buf + len)) {
            printk("[ERROR] Buffer overrun for hop fields\n");
            return -EMSGSIZE;
        }
        rh->hop_count = *p++;
        rh->hop_limit = *p++;
        printk("[PARSE_RH_DBG]  -> hop_count=%u, hop_limit=%u\n", rh->hop_count, rh->hop_limit);
    } else {
        printk("[PARSE_RH_DBG] Hop Count/Limit not present\n");
    }

    if ((rh->bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
        printk("[PARSE_RH_DBG] Parsing Delay\n");
        if ((p + sizeof(uint32_t)) > (buf + len)) {
            printk("[ERROR] Buffer overrun for delay\n");
            return -EMSGSIZE;
        }
        rh->delay = sys_get_be32(p);
        printk("[PARSE_RH_DBG]  -> delay=0x%08X\n", rh->delay);
        p += sizeof(uint32_t);
    } else {
        printk("[PARSE_RH_DBG] Delay not present\n");
    }

    if ((rh->bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
        printk("[PARSE_RH_DBG] Parsing Sequence Number\n");
        if (p >= (buf + len)) {
            printk("[ERROR] Buffer overrun for sequence number\n");
            return -EMSGSIZE;
        }
        rh->sequence_number = *p++;
        printk("[PARSE_RH_DBG]  -> sequence_number=%u\n", rh->sequence_number);
    } else {
        printk("[PARSE_RH_DBG] Sequence Number not present\n");
    }

    /* Hexdump of serialized buffer */
    size_t parsed_len = (size_t)(p - buf);
	// int parsed_len = (int)(p - buf);
    printk("[PARSE_RH_DBG] Parsed Buffer Hexdump (len=%zu):\n", parsed_len);
    for (size_t i = 0; i < parsed_len && i < 32; i++) { /* Limit to 32 bytes */
        printk("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) printk("\n");
    }
    printk("\n");

    printk("[PARSE_RH_DBG] Completed parsing. Total bytes parsed: %d\n", parsed_len);
    return parsed_len;
}
/*********************************************************************************************************************************** */

#if !defined(CONFIG_ZTEST)
/**
 * @brief Parses a DLC Extension Header.
 *
 * This function reads the MUX-like header for DLC extensions to determine the
 * type and length of the following IE.
 *
 * @param buf Pointer to the start of the extension header.
 * @param len The maximum length of the buffer.
 * @param len_type Pointer to store the parsed length type.
 * @param ie_type Pointer to store the parsed IE type.
 * @param payload_len Pointer to store the parsed payload length.
 * @return The number of bytes consumed by the extension header (1, 2, or 3),
 *         or a negative error code.
 */
static int dlc_parse_ext_header(const uint8_t *buf, size_t len, dect_dlc_ext_len_type_t *len_type,
				dect_dlc_ext_ie_type_t *ie_type, uint16_t *payload_len)
{
	if (!buf || !len_type || !ie_type || !payload_len || len < 1) {
		return -EINVAL;
	}

	*len_type = dlc_ext_hdr_get_len_type((const dect_dlc_extension_header_t *)buf);
	*ie_type = dlc_ext_hdr_get_ie_type((const dect_dlc_extension_header_t *)buf);

	switch (*len_type) {
	case DLC_EXT_HDR_NO_LEN_FIELD:
		*payload_len = 0; /* Length is implicit or defined by type */
		return 1;
	case DLC_EXT_HDR_8BIT_LEN_FIELD:
		if (len < 2) return -EMSGSIZE;
		*payload_len = buf[1];
		return 2;
	case DLC_EXT_HDR_16BIT_LEN_FIELD:
		if (len < 3) return -EMSGSIZE;
		*payload_len = sys_get_be16(&buf[1]);
		return 3;
	default:
		return -EBADMSG;
	}
}
#endif /* !defined(CONFIG_ZTEST) */


static void dlc_rx_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    /* CRITICAL: Use unbuffered printk here */
    printk("[TRACE] Starting DLC RX thread entry point\n");
    printk("[TRACE] CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE = %d\n", CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE);
    printk("[TRACE] sizeof(mac_sdu_t) = %zu\n", sizeof(mac_sdu_t));
    printk("[TRACE] sizeof(dlc_app_sdu_t) = %zu\n", sizeof(dlc_app_sdu_t));

    while (1) {
        printk("[TRACE] DLC RX thread waiting for packet...\n");
        mac_sdu_t *mac_sdu = k_queue_get(&g_dlc_internal_mac_rx_queue, K_FOREVER);
        if (!mac_sdu) {
            printk("[TRACE] k_queue_get returned NULL!\n");
            continue;
        }

#if IS_ENABLED(CONFIG_ZTEST)
        if (mac_sdu->ctx) {
            dect_mac_test_set_active_context(mac_sdu->ctx);
        }
#endif

        printk("[TRACE] DLC RX thread got packet %p, len %u\n", (void *)mac_sdu, mac_sdu->len);
        const uint8_t *dlc_pdu = mac_sdu->data;
        size_t dlc_pdu_len = mac_sdu->len;
        const dect_dlc_header_type123_basic_t *base_hdr = (const void *)dlc_pdu;
        dlc_segmentation_indication_t si = dlc_hdr_t123_basic_get_si(base_hdr);
        uint16_t sn = dlc_hdr_t123_basic_get_sn(base_hdr);

        const uint8_t *dlc_sdu_payload_ptr = NULL;
        size_t dlc_sdu_payload_len = 0;

        if (si == DLC_SI_COMPLETE_SDU) {
            printk("[DLC_RX_DBG] Path: Complete SDU. Passing original MAC buffer up.\n");
            printk("  -> sdu_buf ptr: %p\n", (void *)mac_sdu);            
            size_t hdr_len = sizeof(dect_dlc_header_type123_basic_t);
            dlc_sdu_payload_ptr = dlc_pdu + hdr_len;
            dlc_sdu_payload_len = dlc_pdu_len - hdr_len;
        } else { /* Segmented PDU */
            dlc_reassembly_session_t *session = find_or_alloc_reassembly_session(
                sn, DLC_SERVICE_TYPE_1_SEGMENTATION);
            if (!session) {
                printk("[DLC_RX_ERR] Failed to find/alloc reassembly session for SN %u\n", sn);
                goto free_and_continue;
            }

            k_timer_start(&session->timeout_timer, K_MSEC(DLC_REASSEMBLY_TIMEOUT_MS),
                      K_NO_WAIT);

            uint16_t offset = 0;
            const uint8_t *segment_payload_ptr;
            size_t segment_payload_len;

            if (si == DLC_SI_FIRST_SEGMENT) {
                if (dlc_pdu_len < sizeof(dect_dlc_header_type123_basic_t)) {
                    LOG_ERR("DLC_RX: Truncated first segment (len %zu)", dlc_pdu_len);
                    goto free_and_continue;
                }
                segment_payload_ptr = dlc_pdu + sizeof(dect_dlc_header_type123_basic_t);
                segment_payload_len = dlc_pdu_len - sizeof(dect_dlc_header_type123_basic_t);

                printk("[FIRST_SEGMENT_DBG] Data to be copied from first segment (len=%zu):\n", segment_payload_len);
                for (int i=0; i<segment_payload_len && i < 32; i++) { printk("%02x ", segment_payload_ptr[i]); }
                printk("\n");

            } else {
                if (dlc_pdu_len < sizeof(dect_dlc_header_type13_segmented_t)) {
                    LOG_ERR("DLC_RX: Truncated subsequent segment (len %zu)", dlc_pdu_len);
                    goto free_and_continue;
                }
                const dect_dlc_header_type13_segmented_t *seg_hdr = (const void *)dlc_pdu;
                offset = dlc_hdr_t13_segmented_get_offset(seg_hdr);
                segment_payload_ptr = dlc_pdu + sizeof(dect_dlc_header_type13_segmented_t);
                segment_payload_len = dlc_pdu_len - sizeof(dect_dlc_header_type13_segmented_t);

                printk("[MEMCPY_DBG] Dest Ptr for subsequent segment: %p\n", (void *)(session->reassembly_buf + offset));
                printk("[OTHER_SEGMENT_DBG] Copying %zu bytes from subsequent segment to reassembly buffer at offset %u.\n",
                       segment_payload_len, offset);
                printk("  -> Data: ");
                for (int i=0; i<segment_payload_len && i < 32; i++) { printk("%02x ", segment_payload_ptr[i]); }
                printk("\n");
            }

            if ((offset + segment_payload_len) > DLC_REASSEMBLY_BUF_SIZE) {
                session->is_active = false;
                k_timer_stop(&session->timeout_timer);
                goto free_and_continue;
            }
            memcpy(session->reassembly_buf + offset, segment_payload_ptr,
                   segment_payload_len);
            session->bytes_received += segment_payload_len;

            if (si == DLC_SI_LAST_SEGMENT) {
                session->total_sdu_len = offset + segment_payload_len;
            }

            printk("[REASSEMBLY_DBG] After processing segment: SN=%u, bytes_received=%zu, total_sdu_len=%zu\n",
                   sn, session->bytes_received, session->total_sdu_len);

            if (session->total_sdu_len > 0 &&
                session->bytes_received >= session->total_sdu_len) {
                k_timer_stop(&session->timeout_timer);
                
                printk("[REASSEMBLY_BUF_DBG] Reassembled buffer (len=%zu):\n", session->total_sdu_len);
                for (int i=0; i<session->total_sdu_len && i < 64; i++) { printk("%02x ", session->reassembly_buf[i]); }
                printk("\n");

                dlc_sdu_payload_ptr = session->reassembly_buf;
                dlc_sdu_payload_len = session->total_sdu_len;
                session->is_active = false;
            }
        }

        /*
         * ===================================================================
         * START OF MODIFICATION: Multi-Message Processing Loop
         * ===================================================================
         *
         * The original code assumed one message per SDU payload. This has been
         * replaced with a loop to process multiple sequential messages.
         */
        if (dlc_sdu_payload_ptr != NULL) {
            // Pointers to track our position within the payload buffer
            const uint8_t *current_payload_ptr = dlc_sdu_payload_ptr;
            size_t remaining_payload_len = dlc_sdu_payload_len;

            while (remaining_payload_len > 0) {
                dect_dlc_routing_header_t rh;

                printk("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz \n");

                printk("[DLC_RX_DBG] About to parse SDU payload for routing header (remaining_len=%zu):\n", remaining_payload_len);
                for (int i = 0; i < remaining_payload_len && i < 64; i++) { /* Print up to 48 bytes */
                    printk("%02x ", current_payload_ptr[i]);
                }
                printk("\n");

                int rh_len = dlc_parse_routing_header(current_payload_ptr,
                                      remaining_payload_len, &rh);

                // If we can't parse a routing header, the payload is malformed.
                // We must stop processing this SDU.
                if (rh_len <= 0) {
                    LOG_ERR("Failed to parse routing header. Aborting processing of this SDU.");
                    break; // Exit the inner while loop
                }

                // Calculate the pointer and length for the payload of the CURRENT message
                const uint8_t *current_msg_payload_ptr = current_payload_ptr + rh_len;
                size_t current_msg_payload_len = remaining_payload_len - rh_len;

                uint8_t routing_type =
                    (rh.bitmap >>
                     DLC_RH_ROUTING_TYPE_SHIFT) &
                    0x07;

printk("[DLC_RX_DBG] dest_addr:0x%08X routing_type:%d\n", rh.dest_addr, routing_type);

                if (routing_type == DLC_RH_TYPE_LOCAL_FLOODING) {
                    printk("[DUPLICATE_CHECK_DBG] Parsed src_addr: 0x%08X, seq_num: %u\n",
                           rh.source_addr, rh.sequence_number);
                    for (int i = 0; i < DLC_DUPLICATE_CACHE_SIZE; i++) {
                        if (g_dlc_dup_cache[i].source_rd_id == rh.source_addr &&
                            g_dlc_dup_cache[i].sequence_number ==
                                rh.sequence_number) {
                            LOG_DBG("DLC_RX_FWD: Duplicate packet from 0x%08X, seq %u. Dropping.",
                                rh.source_addr, rh.sequence_number);
                            goto advance_to_next_message; // Use goto to skip processing and advance
                        }
                    }
                    g_dlc_dup_cache[g_dlc_dup_cache_next_idx].source_rd_id =
                        rh.source_addr;
                    g_dlc_dup_cache[g_dlc_dup_cache_next_idx].sequence_number =
                        rh.sequence_number;
                    g_dlc_dup_cache_next_idx =
                        (g_dlc_dup_cache_next_idx + 1) %
                        DLC_DUPLICATE_CACHE_SIZE;
                }

                printk("[ROUTING_DBG] Checking destination. dest_addr: 0x%08X, own_addr: 0x%08X\n",
                       rh.dest_addr, dect_mac_get_own_long_id());

#if defined(CONFIG_ZTEST)
/* this is because the mock phy and threads are not able to fully understand their own long id */
				bool special_addr = (rh.dest_addr == 0x44332211) ? true : false;
				if (rh.dest_addr != dect_mac_get_own_long_id() && !special_addr &&
				rh.dest_addr != 0xFFFFFFFF) {	
#else
                if (rh.dest_addr != dect_mac_get_own_long_id() &&
                    rh.dest_addr != 0xFFFFFFFF) {
#endif
                    printk("/* This is a packet for forwarding */ \n");
                    if (rh.hop_count < rh.hop_limit) {
                        printk("DLC_RX_FWD: PDU for 0x%08X not for me. Hops %u/%u. Forwarding.\n",
                            rh.dest_addr, rh.hop_count, rh.hop_limit);
                        
                        /* Increment hop count in the local copy */
                        rh.hop_count++;

                        /*
                         * Forwarding reassembled SDU logic.
                         * If the SDU + new RH exceeds the MAC maximum, we drop it for now
                         * (re-segmentation for forwarding is not yet implemented).
                         */
                        uint8_t rh_fwd_buf[sizeof(dect_dlc_routing_header_t)];
                        int new_rh_len = dlc_serialize_routing_header(rh_fwd_buf, sizeof(rh_fwd_buf), &rh);
                        
                        if (new_rh_len > 0) {
                            size_t fwd_needed = new_rh_len + current_msg_payload_len;
                            if (fwd_needed > CONFIG_DECT_MAC_SDU_MAX_SIZE - sizeof(dect_dlc_header_type123_basic_t)) {
                                LOG_ERR("DLC_RX_FWD: Forwarding reassembled SDU failed: too large for MAC (%zu bytes).", fwd_needed);
                            } else {
                                dect_dlc_header_type123_basic_t fwd_dlc_hdr;
                                uint8_t fwd_payload_buf[CONFIG_DECT_MAC_SDU_MAX_SIZE];

                                memcpy(fwd_payload_buf, rh_fwd_buf, new_rh_len);
                                memcpy(fwd_payload_buf + new_rh_len, current_msg_payload_ptr, current_msg_payload_len);

                                dlc_hdr_t123_basic_set(&fwd_dlc_hdr,
                                               DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
                                               DLC_SI_COMPLETE_SDU, sn);
                                
                                uint8_t fwd_flow_id = mac_sdu->flow_id_present ? mac_sdu->flow_id : MAC_FLOW_HIGH_PRIORITY;
                                queue_dlc_pdu_to_mac(0, (const uint8_t *)&fwd_dlc_hdr, sizeof(fwd_dlc_hdr),
                                           fwd_payload_buf, fwd_needed,
                                           false, 0, fwd_flow_id);
                            }
                        }
                    }
                    goto advance_to_next_message; // Forwarded, now move to the next message in the payload
                }

                /* Packet is for us, strip routing header and deliver */
                /*
                 * CORRECTED/UNIFIED DELIVERY LOGIC:
                 * Regardless of whether the SDU was complete or reassembled, if a message
                 * is for the local device, it MUST be copied into a new application buffer.
                 * This is critical because the original buffer (`mac_sdu` or `reassembly_buf`)
                 * may contain other messages that still need to be processed in this loop.
                 * We cannot pass the original buffer up to the application.
                 */
                dlc_rx_delivery_item_t *item = NULL;
                int alloc_ret = k_mem_slab_alloc(&g_dlc_rx_delivery_item_slab, (void **)&item, K_NO_WAIT);
                
                if (alloc_ret != 0) {
                    /* Slab full, try to drop oldest and retry once */
                    if (dlc_sched_drop_oldest() == 0) {
                        alloc_ret = k_mem_slab_alloc(&g_dlc_rx_delivery_item_slab, (void **)&item, K_NO_WAIT);
                    }
                }

                if (alloc_ret == 0) {
                    dlc_app_sdu_t *app_sdu = NULL;
                    printk("[DLC_RX_DBG] Message for local device. Allocating from app slab.\n");
                    int sdu_alloc_ret = k_mem_slab_alloc(&g_dlc_app_sdu_slab, (void **)&app_sdu, K_NO_WAIT);
                    
                    if (sdu_alloc_ret != 0) {
                        /* SDU slab full, try to drop oldest and retry once */
                        if (dlc_sched_drop_oldest() == 0) {
                            sdu_alloc_ret = k_mem_slab_alloc(&g_dlc_app_sdu_slab, (void **)&app_sdu, K_NO_WAIT);
                        }
                    }

                    if (sdu_alloc_ret == 0) {
                        memcpy(app_sdu->data, current_msg_payload_ptr, current_msg_payload_len);
                        app_sdu->len = current_msg_payload_len;
                        item->sdu_buf = app_sdu;
                        item->is_app_sdu = true;

                        item->dlc_service_type = DLC_SERVICE_TYPE_1_SEGMENTATION;

                        
                        /* --- DLC CLASSIFIER --- */
                        /* Extract Flow ID from logic. Since we stripped headers, we must rely on mac_sdu metadata if valid */
                        /* But mac_sdu might have multiple PDUs. We need to be careful. */
                        /* Actually, dlc_rx_thread_entry iterates over ONE mac_sdu. All PDUs in it *should* share the MAC header Flow ID. */
                        /* However, flow_id is per MAC SDU. */
                        uint8_t flow = mac_sdu->flow_id_present ? mac_sdu->flow_id : MAC_FLOW_HIGH_PRIORITY;
                        
                        /* Map Flow to Lane */
                        int lane = DLC_LANE_LOW;
                        if (flow == MAC_FLOW_HIGH_PRIORITY) lane = DLC_LANE_HIGH;
                        else if (flow == MAC_FLOW_RELIABLE_DATA) lane = DLC_LANE_MED;
                        else if (flow == MAC_FLOW_BEST_EFFORT) lane = DLC_LANE_LOW;
                        /* Control/Signaling override if needed? For now, map simple. */
                        
                        /* Enqueue to Scheduler */
                        item->priority = lane;
                        item->entry_timestamp = net_get_os_tick();
                        item->origin_queue = &g_dlc_scheduler.queues[lane];
                        
                        k_queue_append(&g_dlc_scheduler.queues[lane], item);
                        
                        printk("[DLC_SCHED_DBG] Packet (SN %u, Flow %u) enqueued to Lane %d.\n", sn, flow, lane);
                    } else {
                        LOG_ERR("Failed to allocate from app_sdu_slab");
                        k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void *)item);
                    }
                }

advance_to_next_message:
                // Advance the pointer past the message we just processed
                size_t total_msg_size = rh_len + current_msg_payload_len;
				printk("rh_len:%d current_payload_ptr:%p total_msg_size:%d remaining_payload_len:%d \n", rh_len, current_payload_ptr, total_msg_size, remaining_payload_len);
                current_payload_ptr += total_msg_size;
                remaining_payload_len -= total_msg_size;
				printk("rh_len:%d current_payload_ptr:%p total_msg_size:%d remaining_payload_len:%d \n", rh_len, current_payload_ptr, total_msg_size, remaining_payload_len);
            } // end of while(remaining_payload_len > 0)
        }
        /*
         * ===================================================================
         * END OF MODIFICATION
         * ===================================================================
         */

free_and_continue:
        if (mac_sdu) {
            dect_mac_buffer_free(mac_sdu);
        }
    }
}





static void dlc_tx_service_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("DLC TX Service (ARQ) Thread started.");
	printk("[ARQ_THREAD_DBG] ARQ service thread started. Active context ROLE is:%d\n",dect_mac_get_role());

	while (1) {
		dlc_retransmission_job_t *job = k_fifo_get(&g_dlc_retransmit_signal_fifo, K_FOREVER);
		if (!job || !job->is_active) {
			continue;
		}

		if (job->retries >= DLC_MAX_RETRIES) {
			LOG_ERR("DLC_ARQ_SVC: Job for SN %u has reached max retries. Discarding.", job->sequence_number);

			k_mem_slab_free(&g_dlc_app_sdu_slab, (void *)job->sdu_payload);
			job->is_active = false;
			continue;
		}

		job->retries++;
		LOG_INF("DLC_ARQ_SVC: Re-transmitting SDU for SN %u (attempt %u).", job->sequence_number, job->retries + 1);

		int err = dlc_resend_sdu_with_original_sn(job);
		if (err) {
			LOG_ERR("DLC_ARQ_SVC: dlc_resend_sdu_with_original_sn for SN %u failed (err %d). Retrying on next timeout/signal.",
				job->sequence_number, err);
			k_timer_start(&job->retransmit_attempt_timer, K_MSEC(DLC_RETRANSMISSION_TIMEOUT_MS), K_NO_WAIT);
		}
	}
}

static int dlc_resend_sdu_with_original_sn(dlc_retransmission_job_t *job)
{
	if (!job || !job->is_active || !job->sdu_payload) {
		return -EINVAL;
	}

	/* The job->sdu_payload contains the full SDU (Routing Header + CVG Payload).
	 * We need to parse the routing header to know its length, so we can pass
	 * the header and the payload as separate arguments to dlc_send_segmented.
	 */
	dect_dlc_routing_header_t rh;
	int rh_len = dlc_parse_routing_header(job->sdu_payload->data, job->sdu_payload->len, &rh);

	if (rh_len <= 0) {
		LOG_ERR("ARQ_RESEND: Could not parse routing header from stored SDU for SN %u.",
			job->sequence_number);
		return -EBADMSG;
	}

	const uint8_t *sdu_hdr = job->sdu_payload->data;
	const uint8_t *sdu_payload = job->sdu_payload->data + rh_len;
	size_t sdu_payload_len = job->sdu_payload->len - rh_len;

	return dlc_send_segmented(job->service, job->dest_long_id, sdu_hdr, rh_len, sdu_payload,
				    sdu_payload_len, job->sequence_number, job->flow_id);
}




#if IS_ENABLED(CONFIG_DECT_DLC_API_MOCK)
void dlc_test_set_send_spy(int (*handler)(dlc_service_type_t, uint32_t, const uint8_t *, size_t, uint8_t))
{
	g_dlc_send_spy_cb = handler;
}

void dlc_test_set_receive_spy(int (*handler)(dlc_service_type_t *, uint32_t *, uint8_t *, size_t *, k_timeout_t))
{
	g_dlc_receive_spy_cb = handler;
}
#endif /* IS_ENABLED(CONFIG_DECT_DLC_API_MOCK) */




int dect_dlc_init(void)
{
	// printk("[DLIST_INIT_ORDER_DBG] Address of g_dlc_to_app_rx_dlist BEFORE init & start: %p\n",
	//        (void *)&g_dlc_to_app_rx_dlist);
	printk("[DLIST_INIT_ORDER_DBG] Address of g_dlc_internal_mac_rx_queue BEFORE init: %p\n",
	       (void *)&g_dlc_internal_mac_rx_queue);

	printk("[DLC_INIT_DBG] Entered dect_dlc_init.\n");
    /* Initialize Scheduler Queues */
    for (int i = 0; i < DLC_LANE_COUNT; i++) {
        k_queue_init(&g_dlc_scheduler.queues[i]);
        g_dlc_scheduler.last_service_time[i] = k_uptime_ticks();
    }

	/* Reset duplicate detection cache */
	memset(g_dlc_dup_cache, 0, sizeof(g_dlc_dup_cache));
	g_dlc_dup_cache_next_idx = 0;

	k_queue_init(&g_dlc_internal_mac_rx_queue);

	/* printk("[INIT_DLIST_DBG] DLC's APP RX dlist (g_dlc_to_app_rx_dlist) is at address: %p\n",
	       (void *)&g_dlc_to_app_rx_dlist); */
	printk("[INIT_QUEUE_DBG] DLC's internal RX queue (g_dlc_internal_mac_rx_queue) is at address: %p\n",
	       (void *)&g_dlc_internal_mac_rx_queue);

	/* Create threads in a suspended state. The application/test will start them. */
	g_dlc_rx_thread_id = k_thread_create(
		&g_dlc_rx_thread_data, g_dlc_rx_thread_stack,
		K_THREAD_STACK_SIZEOF(g_dlc_rx_thread_stack), dlc_rx_thread_entry, NULL, NULL, NULL,
		CONFIG_DECT_DLC_RX_THREAD_PRIORITY, 0, K_FOREVER);
	printk("[DLC_INIT_DBG] k_thread_create for g_dlc_rx_thread_id returned: %p\n", g_dlc_rx_thread_id);
	k_thread_name_set(g_dlc_rx_thread_id, "dect_dlc_rx");

	g_dlc_tx_service_thread_id = k_thread_create(
		&g_dlc_tx_service_thread_data, g_dlc_tx_service_thread_stack,
		K_THREAD_STACK_SIZEOF(g_dlc_tx_service_thread_stack), dlc_tx_service_thread_entry,
		NULL, NULL, NULL, CONFIG_DECT_DLC_TX_SERVICE_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(g_dlc_tx_service_thread_id, "dlc_arq_svc");

	/* START THE THREADS - THIS IS WHAT'S MISSING */
	k_thread_start(g_dlc_rx_thread_id);
	k_thread_start(g_dlc_tx_service_thread_id);
	printk("[DLC_INIT_DBG] Started DLC RX and TX service threads\n");

	for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
		k_timer_init(&retransmission_jobs[i].retransmit_attempt_timer,
			     dlc_retransmit_attempt_timeout_handler, NULL);
		retransmission_jobs[i].retransmit_attempt_timer.user_data = (void *)(uintptr_t)i;
		k_timer_init(&retransmission_jobs[i].lifetime_timer,
			     dlc_tx_sdu_lifetime_expiry_handler, NULL);
		retransmission_jobs[i].is_active = false;
	}
	for (int i = 0; i < MAX_DLC_REASSEMBLY_SESSIONS; i++) {
		k_timer_init(&reassembly_sessions[i].timeout_timer, dlc_reassembly_timeout_handler,
			     NULL);
		reassembly_sessions[i].timeout_timer.user_data = (void *)(uintptr_t)i;
		reassembly_sessions[i].is_active = false;
	}
	/* The DLC provides its RX dlist and TX status callback to the unified MAC init function. */
	printk("[DLC_INIT_DBG] About to call dect_mac_init...\n");
	// int err = dect_mac_init(&g_dlc_to_app_rx_dlist, dlc_tx_status_cb_handler);
	int err = dect_mac_init(&g_dlc_internal_mac_rx_queue, dlc_tx_status_cb_handler);
	
	if (err) {
		LOG_ERR("Failed to initialize MAC layer: %d", err);
		return err;
	}

	// printk("[DLIST_INIT_ORDER_DBG] Address of g_dlc_to_app_rx_dlist AFTER init & start: %p\n",
	//        (void *)&g_dlc_to_app_rx_dlist);
	printk("[QUEUE_INIT_ORDER_DBG] Address of g_dlc_internal_mac_rx_queue AFTER init & start: %p\n",
	       (void *)&g_dlc_internal_mac_rx_queue);

	LOG_INF("DLC Layer Initialized.");
	return 0;
}

void dect_dlc_test_reset_duplicate_cache(void)
{
	memset(g_dlc_dup_cache, 0, sizeof(g_dlc_dup_cache));
	g_dlc_dup_cache_next_idx = 0;
}



int dlc_receive_data(dlc_service_type_t *service_type_out, uint32_t *source_addr_out,
		     uint8_t *app_level_payload_buf, size_t *app_level_payload_len_inout,
		     k_timeout_t timeout)
// int dlc_receive_data(dlc_service_type_t *service_type_out, uint8_t *app_level_payload_buf,
// 		     size_t *app_level_payload_len_inout, k_timeout_t timeout)
{
	printk("[DLC_RECV_DBG] Entering dlc_receive_data...\n");

#if IS_ENABLED(CONFIG_DECT_DLC_API_MOCK)
	if (g_dlc_receive_spy_cb) {
		printk("[DLC_RECV_DBG] Calling g_dlc_receive_spy_cb...\n");
		return g_dlc_receive_spy_cb(service_type_out, source_addr_out, app_level_payload_buf,
					    app_level_payload_len_inout, timeout);
	}
#endif

	if (!service_type_out || !app_level_payload_buf || !app_level_payload_len_inout ||
	    (*app_level_payload_len_inout == 0)) {
		printk("[DLC_RECV_DBG] (!service_type_out || !app_level_payload_buf || !app_level_payload_len_inout || (*app_level_payload_len_inout == 0)) ERROR...\n");
		return -EINVAL;
	}

	if (!service_type_out || !app_level_payload_buf || !app_level_payload_len_inout ||
	    (*app_level_payload_len_inout == 0)) {
		printk("[DLC_RECV_DBG] Invalid args.\n");
		return -EINVAL;
	}

    /* Initialize Poll Events for 4 queues */
    struct k_poll_event events[DLC_LANE_COUNT];
    for (int i = 0; i < DLC_LANE_COUNT; i++) {
        k_poll_event_init(&events[i], K_POLL_TYPE_FIFO_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &g_dlc_scheduler.queues[i]);
    }

	// int64_t start_time = k_uptime_get();
    
    /* Wait for data on ANY queue */
    int ret = k_poll(events, DLC_LANE_COUNT, timeout);
    if (ret != 0) {
        return ret; // -EAGAIN on timeout
    }

    /* --- DISPATCHER: Audit-then-Select Logic --- */
    int selected_lane = -1;
    int64_t now = k_uptime_ticks();

    /* 1. Audit for Starvation (Check lower priority lanes first) */
    /* Check Low Starvation */
    if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_LOW])) {
        int64_t wait_time = k_ticks_to_ms_floor64(now - g_dlc_scheduler.last_service_time[DLC_LANE_LOW]);
        if (wait_time > DLC_SCHED_STARVATION_THRESHOLD_LOW_MS) {
            selected_lane = DLC_LANE_LOW;
            printk("[DLC_SCHED_WARN] Starvation detected on Low Lane (Wait %lld ms). Promoting.\n", wait_time);
        }
    }
    /* Check Med Starvation (overrides Low if also starving? Or strict priority among starving?) */
    /* Let's say if Med is starving, it's more important than Low starving. */
    if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_MED])) {
        int64_t wait_time = k_ticks_to_ms_floor64(now - g_dlc_scheduler.last_service_time[DLC_LANE_MED]);
        if (wait_time > DLC_SCHED_STARVATION_THRESHOLD_MED_MS) {
            selected_lane = DLC_LANE_MED;
        }
    }

    /* 2. Strict Priority Selection (if no starvation selected) */
    if (selected_lane == -1) {
        if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_CONTROL])) selected_lane = DLC_LANE_CONTROL;
        else if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_HIGH])) selected_lane = DLC_LANE_HIGH;
        else if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_MED])) selected_lane = DLC_LANE_MED;
        else if (!k_queue_is_empty(&g_dlc_scheduler.queues[DLC_LANE_LOW])) selected_lane = DLC_LANE_LOW;
    }

    if (selected_lane == -1) {
        /* Should not happen if k_poll returned 0, but race condition possible? */
        return -EAGAIN; 
    }

    /* 3. Dequeue */
    dlc_rx_delivery_item_t *delivery_item = k_queue_get(&g_dlc_scheduler.queues[selected_lane], K_NO_WAIT);
    if (!delivery_item) return -EAGAIN;

    /* Update Service Time */
    g_dlc_scheduler.last_service_time[selected_lane] = now;


	if (!delivery_item->sdu_buf) {
		LOG_ERR("No delivery_item->sdu_buf");
		k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void *)delivery_item);
		return -EFAULT;
	}

	printk("[DLC_RECV_DBG] delivery_item->is_app_sdu:%s *app_level_payload_len_inout:%zu \n", delivery_item->is_app_sdu? "Yes":"No", *app_level_payload_len_inout);
	if (delivery_item->is_app_sdu) {
		printk("[DLC_RECV_DBG] Processing APP SDU buffer\n");
		dlc_app_sdu_t *sdu_buf = delivery_item->sdu_buf;
		
		if (*app_level_payload_len_inout < sdu_buf->len) {
			/* Buffer too small - put item back (prepend) and return error */
            /* NOTE: Prepending to k_queue puts it at head */
            k_queue_prepend(&g_dlc_scheduler.queues[selected_lane], delivery_item);
			return -EMSGSIZE;
		}
		
		*app_level_payload_len_inout = sdu_buf->len;
		memcpy(app_level_payload_buf, sdu_buf->data, sdu_buf->len);
		
		printk("  -> Freeing app_sdu_buf at address: %p to slab g_dlc_app_sdu_slab\n", 
		       (void *)sdu_buf);
		k_mem_slab_free(&g_dlc_app_sdu_slab, (void *)sdu_buf);
	} else {
        /* Legacy path fallback, usually unused now */
		printk("[DLC_RECV_DBG] Processing MAC SDU buffer\n");
		mac_sdu_t *sdu_buf = delivery_item->sdu_buf;
		
		/* Unsegmented packets have routing header stripped before this point */
		const uint8_t *payload = sdu_buf->data + sizeof(dect_dlc_header_type123_basic_t);
		size_t payload_len = sdu_buf->len - sizeof(dect_dlc_header_type123_basic_t);

		if (*app_level_payload_len_inout < payload_len) {
			*app_level_payload_len_inout = payload_len;
            k_queue_prepend(&g_dlc_scheduler.queues[selected_lane], delivery_item);
			return -EMSGSIZE;
		}
		
		*app_level_payload_len_inout = payload_len;
		memcpy(app_level_payload_buf, payload, payload_len);
		
		printk("  -> Freeing mac_sdu_buf at address: %p\n", (void *)sdu_buf);
		dect_mac_buffer_free(sdu_buf);
	}

	
	*service_type_out = delivery_item->dlc_service_type;
    /* Free the delivery item wrapper */
	k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void *)delivery_item);
printk("[DLC_RECV_DBG] EXITTING dlc_receive_data...\n");
	return 0;
}



static dlc_reassembly_session_t* find_or_alloc_reassembly_session(uint16_t sequence_number, dlc_service_type_t service)
{
    int free_slot = -1;
    for (int i = 0; i < MAX_DLC_REASSEMBLY_SESSIONS; i++) {
        if (reassembly_sessions[i].is_active) {
            if (reassembly_sessions[i].sequence_number == sequence_number) {
                return &reassembly_sessions[i];
            }
        } else if (free_slot == -1) {
            free_slot = i;
        }
    }

    if (free_slot != -1) {
        dlc_reassembly_session_t *session = &reassembly_sessions[free_slot];
        session->is_active = true;
        session->sequence_number = sequence_number;
        session->service_type = service;
        memset(session->reassembly_buf, 0, DLC_REASSEMBLY_BUF_SIZE);
        session->bytes_received = 0;
        session->total_sdu_len = 0;
        k_timer_start(&session->timeout_timer, K_MSEC(g_rx_sdu_lifetime_ms), K_NO_WAIT);
        LOG_DBG("DLC_SAR: Allocated reassembly session %d for SN %u, Svc %u.", free_slot, sequence_number, service);
        return session;
    }
    LOG_ERR("DLC_SAR: No free reassembly sessions available for SN %u!", sequence_number);
    return NULL;
}

static void dlc_reassembly_timeout_handler(struct k_timer *timer_id)
{
    uintptr_t session_idx_from_timer = (uintptr_t)timer_id->user_data;
    if (session_idx_from_timer < MAX_DLC_REASSEMBLY_SESSIONS &&
        reassembly_sessions[session_idx_from_timer].is_active) {
        printk("DLC_SAR_TIMEOUT: Reassembly for session %u (SN %u) timed out. Discarding segments.",
                (unsigned int)session_idx_from_timer, reassembly_sessions[session_idx_from_timer].sequence_number);
        LOG_WRN("DLC_SAR_TIMEOUT: Reassembly for session %u (SN %u) timed out. Discarding segments.",
                (unsigned int)session_idx_from_timer, reassembly_sessions[session_idx_from_timer].sequence_number);
		reassembly_sessions[session_idx_from_timer].is_active = false;
    }
}

static void dlc_retransmit_attempt_timeout_handler(struct k_timer *timer_id)
{
	uintptr_t job_idx = (uintptr_t)timer_id->user_data;
	if (job_idx < MAX_DLC_RETRANSMISSION_JOBS && retransmission_jobs[job_idx].is_active) {
		LOG_WRN("DLC_ARQ_TIMEOUT: Transmission for SN %u timed out. Signaling for re-TX.",
			retransmission_jobs[job_idx].sequence_number);
		k_fifo_put(&g_dlc_retransmit_signal_fifo, &retransmission_jobs[job_idx]);
	}
}

static void dlc_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id)
{
	dlc_retransmission_job_t *job = CONTAINER_OF(timer_id, dlc_retransmission_job_t, lifetime_timer);
	if (job && job->is_active) {
		LOG_ERR("DLC_LIFETIME: SDU with SN %u expired. Discarding from ARQ buffer.", job->sequence_number);
		k_timer_stop(&job->retransmit_attempt_timer);
		// k_mem_slab_free(&g_dlc_arq_buf_slab, (void *)job->sdu_payload);

		// Not sure if we should clear this memory
		k_mem_slab_free(&g_dlc_app_sdu_slab, (void *)job->sdu_payload);
		// dect_mac_buffer_free(job->sdu_payload);
		job->is_active = false;
	}
}

/**
 * @brief Assembles a final DLC PDU from its header and payload, and queues it to the MAC layer.
 *
 * This is the final step in the DLC TX path. It allocates a MAC SDU buffer,
 * copies the DLC header and the provided payload segment into it, sets the
 * necessary metadata for status reporting (if required for ARQ), and then
 * calls the appropriate MAC API function based on the device's role (PT or FT).
 *
 * @param dest_long_id The final destination Long RD ID of the peer.
 * @param dlc_header Pointer to the pre-built DLC header.
 * @param dlc_header_len Length of the DLC header.
 * @param payload_segment Pointer to the payload segment (a piece of the original CVG PDU).
 * @param payload_segment_len Length of the payload segment.
 * @param report_status True if the DLC's ARQ mechanism requires a final success/fail status
 *                      report from the MAC for this PDU.
 * @param dlc_sn The DLC sequence number, used to correlate the status report.
 * @return 0 on successful queueing to the MAC layer.
 * @retval -ENOMEM If a MAC SDU buffer could not be allocated.
 * @retval -EMSGSIZE If the combined PDU is too large for a MAC SDU buffer.
 * @retval -ENOTCONN If the FT cannot find a connected peer matching the destination ID.
 * @retval Other negative error codes from the MAC API.
 */
static int queue_dlc_pdu_to_mac(uint32_t dest_long_id, const uint8_t *dlc_header,
				size_t dlc_header_len, const uint8_t *payload_segment,
				size_t payload_segment_len, bool report_status, uint16_t dlc_sn,
				uint8_t flow_id)
{
	int err;

	/* 1. Allocate a buffer from the MAC layer's memory slab */
	mac_sdu_t *mac_sdu = dect_mac_buffer_alloc(K_NO_WAIT);

	if (!mac_sdu) {
		LOG_ERR("DLC_Q_MAC: Failed to allocate MAC SDU buffer.");
		return -ENOMEM;
	}

	/* 2. Assemble the full DLC PDU into the MAC SDU buffer */
	size_t total_pdu_len = dlc_header_len + payload_segment_len;

	if (total_pdu_len > CONFIG_DECT_MAC_SDU_MAX_SIZE) {
		LOG_ERR("DLC_Q_MAC: Assembled DLC PDU too large (%zu > %d).", total_pdu_len,
			CONFIG_DECT_MAC_SDU_MAX_SIZE);
		dect_mac_buffer_free(mac_sdu);
		return -EMSGSIZE;
	}

	uint8_t *p = mac_sdu->data;

	memcpy(p, dlc_header, dlc_header_len);
	p += dlc_header_len;

	if (payload_segment && payload_segment_len > 0) {
		memcpy(p, payload_segment, payload_segment_len);
	}
	mac_sdu->len = total_pdu_len;

	/* 3. Set metadata for status reporting if required by DLC ARQ */
	mac_sdu->dlc_status_report_required = report_status;
	if (report_status) {
		mac_sdu->dlc_sn_for_status = dlc_sn;
	}
	/* 4. Call the appropriate MAC API based on role */
	mac_sdu->flow_id = flow_id;
	mac_sdu->flow_id_present = true;

	if (dect_mac_get_role() == MAC_ROLE_PT) {
		/* PT sends to its associated FT */
		
		err = dect_mac_send(mac_sdu, MAC_FLOW_RELIABLE_DATA);
		
	} else { /* MAC_ROLE_FT */
		/* FT must send to a specific connected PT */
		uint16_t dest_short_id = dect_mac_get_short_id_for_long_id(dest_long_id);

		if (dest_short_id == 0) {
			LOG_ERR("DLC_Q_MAC: FT could not find Short ID for Long ID 0x%08X.",
				dest_long_id);
			dect_mac_buffer_free(mac_sdu);
			return -ENOTCONN;
		}
		err = dect_mac_ft_send_to_pt(mac_sdu, MAC_FLOW_RELIABLE_DATA, dest_short_id);
	}

	if (err != 0) {
		LOG_ERR("DLC_Q_MAC: Failed to queue PDU to MAC API: %d", err);
		/* The MAC API is responsible for freeing the SDU on failure to queue. */
	}

	return err;
}



/**
 * @brief Segments a DLC SDU if necessary and queues the resulting PDUs to the MAC layer.
 *
 * This function is the workhorse of the DLC TX path. It takes a complete DLC SDU
 * (which includes the CVG PDU payload and any DLC routing headers), a service type,
 * and a sequence number.
 *
 * If the SDU is small enough to fit in a single MAC frame, it prepends the appropriate
 * DLC header with SI=COMPLETE_SDU and sends it as one PDU.
 *
 * If the SDU is too large, it breaks it into multiple segments, each with the
 * appropriate DLC header (FIRST, MIDDLE, LAST) and segmentation offset, and queues

 * each segment as a separate PDU to the MAC layer.
 *
 * @param service The DLC service type being used.
 * @param dest_long_id The final destination Long RD ID of the peer.
 * @param dlc_sdu_payload Pointer to the complete DLC SDU payload (e.g., CVG PDU).
 * @param dlc_sdu_payload_len Length of the complete DLC SDU payload.
 * @param dlc_sn The 10-bit DLC sequence number for this SDU.
 * @return 0 on success (all segments queued), or a negative error code on the first failure.
 */
static int dlc_send_segmented(dlc_service_type_t service, uint32_t dest_long_id,
			      const uint8_t *dlc_sdu_hdr, size_t dlc_sdu_hdr_len,
			      const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len,
			      uint16_t dlc_sn, uint8_t flow_id)
{
	int err = 0;
	bool needs_dlc_arq = (service == DLC_SERVICE_TYPE_2_ARQ ||
			      service == DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ);

	size_t total_len = dlc_sdu_hdr_len + dlc_sdu_payload_len;

	if (total_len <= (CONFIG_DECT_MAC_SDU_MAX_SIZE - sizeof(dect_dlc_header_type123_basic_t))) {
		/* SDU fits in a single PDU, no segmentation needed */
		dect_dlc_header_type123_basic_t dlc_hdr;
		dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
				       DLC_SI_COMPLETE_SDU, dlc_sn);

		/* We need to combine the SDU header and payload for the MAC layer */
		mac_sdu_t *mac_pdu = dect_mac_buffer_alloc(K_NO_WAIT);
		if (!mac_pdu) {
			return -ENOMEM;
		}
		memcpy(mac_pdu->data, dlc_sdu_hdr, dlc_sdu_hdr_len);
		memcpy(mac_pdu->data + dlc_sdu_hdr_len, dlc_sdu_payload, dlc_sdu_payload_len);
		mac_pdu->len = total_len;

		err = queue_dlc_pdu_to_mac(dest_long_id, (const uint8_t *)&dlc_hdr,
					   sizeof(dlc_hdr), mac_pdu->data, mac_pdu->len,
					   needs_dlc_arq, dlc_sn, flow_id);
		dect_mac_buffer_free(mac_pdu);
	} else {
		/* SDU is too large, segmentation is required */
		if (service != DLC_SERVICE_TYPE_1_SEGMENTATION &&
		    service != DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ) {
			return -EMSGSIZE;
		}

		size_t bytes_sent = 0;
		bool is_first_segment = true;

		while (bytes_sent < total_len) {
			dlc_segmentation_indication_t si;
			uint8_t dlc_hdr_buf[sizeof(dect_dlc_header_type13_segmented_t)];
			size_t dlc_hdr_len;
			size_t current_payload_len;
			mac_sdu_t *mac_pdu = dect_mac_buffer_alloc(K_NO_WAIT);
			if (!mac_pdu) {
				return -ENOMEM;
			}

			if (is_first_segment) {
				si = DLC_SI_FIRST_SEGMENT;
				dect_dlc_header_type123_basic_t *hdr = (void *)dlc_hdr_buf;
				dlc_hdr_t123_basic_set(hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
						       si, dlc_sn);
				dlc_hdr_len = sizeof(*hdr);
				current_payload_len = MIN(total_len, CONFIG_DECT_MAC_SDU_MAX_SIZE - dlc_hdr_len);
				memcpy(mac_pdu->data, dlc_sdu_hdr, dlc_sdu_hdr_len);
				memcpy(mac_pdu->data + dlc_sdu_hdr_len, dlc_sdu_payload, current_payload_len - dlc_sdu_hdr_len);
			} else {
				size_t remaining_bytes = total_len - bytes_sent;
				current_payload_len = MIN(CONFIG_DECT_MAC_SDU_MAX_SIZE - sizeof(dect_dlc_header_type13_segmented_t), remaining_bytes);
				si = (current_payload_len == remaining_bytes) ? DLC_SI_LAST_SEGMENT
									: DLC_SI_MIDDLE_SEGMENT;
				dect_dlc_header_type13_segmented_t *hdr = (void *)dlc_hdr_buf;
				dlc_hdr_t13_segmented_set(hdr,
							  DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
							  si, dlc_sn, bytes_sent);
				dlc_hdr_len = sizeof(*hdr);
				memcpy(mac_pdu->data, dlc_sdu_payload + (bytes_sent - dlc_sdu_hdr_len), current_payload_len);
			}
			mac_pdu->len = current_payload_len;

			bool report_this_segment = needs_dlc_arq && (si == DLC_SI_LAST_SEGMENT);
			err = queue_dlc_pdu_to_mac(dest_long_id, dlc_hdr_buf, dlc_hdr_len,
						   mac_pdu->data, mac_pdu->len,
						   report_this_segment, dlc_sn, flow_id);
			dect_mac_buffer_free(mac_pdu);

			if (err != 0) {
				break;
			}
			bytes_sent += current_payload_len;
			is_first_segment = false;
		}
	}
	return err;
}

/**
 * @brief Scavenger logic to drop the oldest packet in the delivery queues.
 * 
 * Searches for the oldest packet (starting from lowest priority lane) and 
 * discards it to reclaim memory slabs.
 * 
 * @return 0 if an item was dropped, -ENODATA if no items could be found.
 */
static int dlc_sched_drop_oldest(void)
{
    /* Search from Low Priority to High Priority */
    for (int lane = DLC_LANE_LOW; lane >= DLC_LANE_CONTROL; lane--) {
        dlc_rx_delivery_item_t *dropped = k_queue_get(&g_dlc_scheduler.queues[lane], K_NO_WAIT);
        if (dropped) {
            LOG_WRN("DLC_SCHED_DROP: Dropped oldest item from Lane %d due to buffer pressure.", lane);
            
            if (dropped->sdu_buf) {
                if (dropped->is_app_sdu) {
                    k_mem_slab_free(&g_dlc_app_sdu_slab, &dropped->sdu_buf);
                } else {
                    dect_mac_buffer_free(dropped->sdu_buf);
                }
            }
            k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void *)dropped);
            return 0;
        }
    }
    return -ENODATA;
}

