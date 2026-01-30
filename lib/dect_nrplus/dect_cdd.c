/* lib/dect_nrplus/dect_cdd.c */
// Overview: New file implementing the core logic for the CDD service.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string.h>
#include <zephyr/sys/byteorder.h>

// #include <mac/dect_mac_core.h>
// #include <mac/dect_mac_context.h>
#include <mac/dect_mac.h>
#include <dect_cdd.h>
#include <dect_cvg.h>
#include <dect_dlc.h>

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

#include <dect_cdd.h> /* This header now defines the cdd_ipv6_prefix_handler_t */
// #include <dect_stack_api.h> /* For dect_nrplus_l2_set_sixlowpan_context */
/* No direct includes of upper layers. The handler is the interface. */

LOG_MODULE_REGISTER(dect_cdd, CONFIG_DECT_CDD_LOG_LEVEL);


static struct {
	cdd_content_pdu_t content;
	bool is_valid;
	cdd_ipv6_prefix_handler_t prefix_handler;
} g_cdd_ctx;


static void pt_send_cdd_request(void)
{
	if (!dect_mac_is_associated()) {
		LOG_ERR("CDD_REQ: Cannot send, PT not associated.");
		return;
	}

	uint8_t pdu_buf[sizeof(cvg_ie_ep_mux_t) + sizeof(cdd_request_pdu_t)];
	cvg_ie_ep_mux_t *ep_mux = (cvg_ie_ep_mux_t *)pdu_buf;

	ep_mux->header.ext_mt_f2c_or_type =
		((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) | (CVG_IE_TYPE_EP_MUX & 0x1F);
	ep_mux->endpoint_mux_be = sys_cpu_to_be16(CVG_EP_MANAGEMENT_CDD);

	cdd_request_pdu_t *req = (cdd_request_pdu_t *)(pdu_buf + sizeof(cvg_ie_ep_mux_t));

	req->reserved_type = CDD_REQUEST_TYPE_COMPLETE_CONFIG;

	LOG_INF("CDD_REQ: Sending Configuration Data Request to FT.");
	dect_cvg_send(CVG_SERVICE_TYPE_1_SEQ_NUM, dect_mac_get_associated_ft_long_id(), pdu_buf,
		      sizeof(pdu_buf));
}

/**
 * @brief Builds the FT's own configuration data to be served via CDD.
 *
 * This function should be called by the FT when it initializes. It populates
 * the CDD content with network parameters, like the IPv6 prefix.
 */
/**
 * @brief Builds the FT's own configuration data to be served via CDD.
 *
 * This function should be called by the FT when it initializes. It populates
 * the CDD content with network parameters, like the IPv6 prefix. This version
 * uses fixed-size buffers to prevent overflows and includes bounds checking.
 */
int dect_cdd_ft_build_own_config(const struct in6_addr *ft_prefix)
{
	LOG_DBG("Entering function...");

	if (dect_mac_get_role() != MAC_ROLE_FT) {
		LOG_ERR("Function called on a non-FT node.");
		return -EPERM;
	}
	if (!ft_prefix) {
		LOG_ERR("Input ft_prefix is NULL.");
		return -EINVAL;
	}

	LOG_INF("CDD_FT: Building own configuration data.");

	/* Reset the content structure before populating */
	memset(&g_cdd_ctx.content, 0, sizeof(g_cdd_ctx.content));

	g_cdd_ctx.content.type = 0; /* Full Configuration Data Content PDU */
	g_cdd_ctx.content.sink_addr_be = sys_cpu_to_be32(dect_mac_get_own_long_id());
	g_cdd_ctx.content.app_seq_num = 1; /* Initial sequence number */
	g_cdd_ctx.content.num_data_items = 0;

	LOG_DBG("  - PDU Type: %u", g_cdd_ctx.content.type);
	LOG_DBG("  - Sink Addr: 0x%08X", sys_be32_to_cpu(g_cdd_ctx.content.sink_addr_be));
	LOG_DBG("  - App Seq Num: %u", g_cdd_ctx.content.app_seq_num);

	/* --- Add IPv6 Prefix Data Item --- */
	LOG_DBG("Adding IPv6 Prefix Data Item...");
	uint8_t context_id = 1; /* Use Context ID 1 for this prefix */

	if (g_cdd_ctx.content.num_data_items >= DECT_CDD_MAX_ITEMS) {
		LOG_ERR("CDD_FT: Cannot add new data item, CDD content is full.");
		return -ENOMEM;
	}

	cdd_data_item_t *item = &g_cdd_ctx.content.items[g_cdd_ctx.content.num_data_items];

	sys_put_be16(CVG_EP_IPV6_PROFILE, item->ep_address);
	LOG_DBG("  - Item[%u] EP: 0x%04X", g_cdd_ctx.content.num_data_items, CVG_EP_IPV6_PROFILE);

	size_t required_payload_len =
		sizeof(cdd_ipv6_addr_element_hdr_t) + 8; /* 8 bytes for a /64 prefix */

	if (required_payload_len > DECT_CDD_MAX_ITEM_PAYLOAD) {
		LOG_ERR("CDD_FT: IPv6 prefix data (%zu bytes) is too large for item payload buffer (%d bytes).",
			required_payload_len, DECT_CDD_MAX_ITEM_PAYLOAD);
		return -EMSGSIZE;
	}

	cdd_ipv6_addr_element_hdr_t *hdr = (cdd_ipv6_addr_element_hdr_t *)item->payload;

	hdr->element_type = 1;  /* IPv6 Address Element */
	hdr->element_version = 0;
	hdr->rfu = 0;
	hdr->prefix_type = 0;   /* 64-bit prefix */
	hdr->context_usage = 1; /* Used for header compression */
	hdr->context_id = context_id;
	hdr->service_id = 0;    /* RFU */

	LOG_DBG("  - Item[%u] IPv6 Element Hdr: ElemType=%u, Ver=%u, PrefixType=%u, CtxUse=%u, CID=%u",
		g_cdd_ctx.content.num_data_items, hdr->element_type, hdr->element_version,
		hdr->prefix_type, hdr->context_usage, hdr->context_id);

	uint8_t *prefix_ptr = item->payload + sizeof(*hdr);

	memcpy(prefix_ptr, &ft_prefix->s6_addr, 8); /* Copy 64-bit /64 prefix */
	LOG_HEXDUMP_DBG(prefix_ptr, 8, "  - Item Prefix Data:");

	item->length = required_payload_len;
	g_cdd_ctx.content.num_data_items++;
	g_cdd_ctx.is_valid = true;

	LOG_INF("CDD_FT: Added IPv6 prefix to CDD content for CID %u.", context_id);
	LOG_HEXDUMP_DBG(ft_prefix, sizeof(struct in6_addr), "IPv6 prefix:");
	return 0;
}


void dect_cdd_init(void)
{
	memset(&g_cdd_ctx, 0, sizeof(g_cdd_ctx));
	LOG_INF("CDD Service Initialized.");
}

void dect_cdd_register_prefix_handler(cdd_ipv6_prefix_handler_t handler)
{
	g_cdd_ctx.prefix_handler = handler;
}

int dect_cdd_ft_set_data(const cdd_content_pdu_t *data)
{
	if (!data) {
		return -EINVAL;
	}
	memcpy(&g_cdd_ctx.content, data, sizeof(cdd_content_pdu_t));
	g_cdd_ctx.content.app_seq_num++; /* Increment sequence on new data */
	g_cdd_ctx.is_valid = true;
	LOG_INF("CDD_FT: New configuration data set. AppSeqNum is now %u.",
		g_cdd_ctx.content.app_seq_num);
	return 0;
}

void dect_cdd_pt_process_beacon_info(uint32_t sink_addr, uint8_t app_seq_num)
{
	if (!g_cdd_ctx.is_valid || g_cdd_ctx.content.sink_addr_be != sys_cpu_to_be32(sink_addr) ||
	    g_cdd_ctx.content.app_seq_num != app_seq_num) {
		LOG_INF("CDD_PT: Stale or missing config data. Local(valid:%d, sink:0x%08X, seq:%u), Beacon(sink:0x%08X, seq:%u). Requesting update.",
			g_cdd_ctx.is_valid, sys_be32_to_cpu(g_cdd_ctx.content.sink_addr_be),
			g_cdd_ctx.content.app_seq_num, sink_addr, app_seq_num);
		pt_send_cdd_request();
	}
}


void dect_cdd_handle_incoming_pdu(const uint8_t *data, size_t len, uint32_t source_rd_id)
{
	if (dect_mac_get_role() == MAC_ROLE_FT) {
		/* This must be a request from a PT */
		if (len == sizeof(cdd_request_pdu_t)) {
			const cdd_request_pdu_t *req = (const cdd_request_pdu_t *)data;

			if ((req->reserved_type & 0x1F) == CDD_REQUEST_TYPE_COMPLETE_CONFIG) {
				LOG_INF("CDD_FT: Rcvd Config Request from 0x%08X. Sending response.",
					source_rd_id);

				if (g_cdd_ctx.is_valid) {
					/* Calculate actual content size based on variable item lengths */
					size_t data_items_total_len = 0;

					for (int i = 0; i < g_cdd_ctx.content.num_data_items;
					     i++) {
						/* Each item's size is header + actual payload length */
						data_items_total_len +=
							offsetof(cdd_data_item_t, payload) +
							g_cdd_ctx.content.items[i].length;
					}

					size_t content_len =
						offsetof(cdd_content_pdu_t, items) +
						data_items_total_len;
					size_t pdu_len = sizeof(cvg_ie_ep_mux_t) + content_len;

					/* Allocate buffer from heap to avoid stack overflow with variable-size PDU */
					uint8_t *pdu_buf = k_malloc(pdu_len);

					if (!pdu_buf) {
						LOG_ERR("CDD_FT: Failed to allocate %zu bytes for response PDU.",
							pdu_len);
						return;
					}

					cvg_ie_ep_mux_t *ep_mux = (cvg_ie_ep_mux_t *)pdu_buf;

					ep_mux->header.ext_mt_f2c_or_type =
						((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
						(CVG_IE_TYPE_EP_MUX & 0x1F);
					ep_mux->endpoint_mux_be =
						sys_cpu_to_be16(CVG_EP_MANAGEMENT_CDD);

					memcpy(pdu_buf + sizeof(cvg_ie_ep_mux_t),
					       &g_cdd_ctx.content, content_len);

					dect_cvg_send(CVG_EP_MANAGEMENT_CDD, source_rd_id,
						      pdu_buf, pdu_len);

					k_free(pdu_buf);
				} else {
					LOG_WRN("CDD_FT: Rcvd request, but no valid CDD data to send.");
				}
			}
		}
	} else { /* PT Role */
		/* This must be a response from the FT */
		if (len >= offsetof(cdd_content_pdu_t, items)) {
			const cdd_content_pdu_t *resp = (const cdd_content_pdu_t *)data;

			LOG_INF("CDD_PT: Rcvd Config Content from FT. Sink:0x%08X, Seq:%u, Items:%u",
				sys_be32_to_cpu(resp->sink_addr_be), resp->app_seq_num,
				resp->num_data_items);

			/* Store the new configuration metadata */
			g_cdd_ctx.content.sink_addr_be = resp->sink_addr_be;
			g_cdd_ctx.content.app_seq_num = resp->app_seq_num;
			g_cdd_ctx.is_valid = true;

			/* Parse items and invoke callback if an IPv6 prefix is found */
			const cdd_data_item_t *item = resp->items;
			size_t remaining_len = len - offsetof(cdd_content_pdu_t, items);

			for (int i = 0; i < resp->num_data_items; i++) {
				if (remaining_len < offsetof(cdd_data_item_t, payload)) {
					LOG_ERR("CDD_PT: Malformed PDU, not enough data for item %d header.", i);
					break;
				}
				if (remaining_len < offsetof(cdd_data_item_t, payload) + item->length) {
					LOG_ERR("CDD_PT: Malformed PDU, item %d length %u exceeds remaining data %zu.",
						i, item->length, remaining_len);
					break;
				}

				uint16_t ep = sys_be16_to_cpu(*(uint16_t *)item->ep_address);

				if (ep == CVG_EP_IPV6_PROFILE) {
					const cdd_ipv6_addr_element_hdr_t *hdr =
						(const cdd_ipv6_addr_element_hdr_t *)item->
						payload;
					size_t prefix_bytes = (hdr->prefix_type == 0) ? 8 : 16;
					size_t expected_item_len = sizeof(*hdr) + prefix_bytes;

					if (item->length < expected_item_len) {
						LOG_WRN("CDD_PT: IPv6 data item too short (len %u).",
							item->length);
						continue;
					}

					if (hdr->context_usage == 1 && hdr->prefix_type == 0) {
						if (g_cdd_ctx.prefix_handler) {
							struct in6_addr prefix;
							uint8_t prefix_len = 64;
							uint8_t context_id = hdr->context_id;
							const uint8_t *prefix_ptr =
								item->payload + sizeof(*hdr);

							memset(&prefix, 0, sizeof(prefix));
							memcpy(&prefix.s6_addr, prefix_ptr, 8);

							LOG_INF("CDD_PT: Found IPv6 prefix for 6LoWPAN context CID %u. Invoking handler.",
								context_id);
							g_cdd_ctx.prefix_handler(&prefix,
									       prefix_len,
									       context_id);
						} else {
							LOG_WRN("CDD_PT: Found IPv6 prefix but no handler is registered.");
						}
					}
				}

				size_t current_item_size =
					offsetof(cdd_data_item_t, payload) + item->length;
				item = (const cdd_data_item_t *)((const uint8_t *)item +
								 current_item_size);
				remaining_len -= current_item_size;
			}
		}
	}
}
