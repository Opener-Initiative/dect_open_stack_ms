/* lib/dect_nrplus/dect_cdd.c */
// Overview: Implementation of the spec-pure CDD service logic.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include <mac/dect_mac.h>
#include <dect_cdd.h>
#include <dect_cvg.h>

LOG_MODULE_REGISTER(dect_cdd, CONFIG_DECT_CDD_LOG_LEVEL);

static struct {
	cdd_content_pdu_t content;
	bool is_valid;
	cdd_data_item_handler_t handler;
} g_cdd_ctx;

static void pt_send_cdd_request(void)
{
	if (!dect_mac_is_associated()) {
		LOG_ERR("CDD_REQ: Cannot send, PT not associated.");
		return;
	}

	cdd_request_pdu_t req;
	req.reserved_type = CDD_REQUEST_TYPE_COMPLETE_CONFIG;

	LOG_INF("CDD_REQ: Sending Configuration Data Request to FT.");
	dect_cvg_send(CVG_EP_MANAGEMENT_CDD, dect_mac_get_associated_ft_long_id(), (uint8_t *)&req,
		      sizeof(req));
}

void dect_cdd_init(void)
{
	memset(&g_cdd_ctx, 0, sizeof(g_cdd_ctx));
	LOG_INF("CDD Service Initialized.");
}

void dect_cdd_register_handler(cdd_data_item_handler_t handler)
{
	g_cdd_ctx.handler = handler;
}

int dect_cdd_ft_add_item(uint16_t endpoint, const uint8_t *data, size_t len)
{
	if (g_cdd_ctx.content.num_data_items >= DECT_CDD_MAX_ITEMS) {
		LOG_ERR("CDD_FT: Content full.");
		return -ENOMEM;
	}
	if (len > DECT_CDD_MAX_ITEM_PAYLOAD) {
		LOG_ERR("CDD_FT: Item too large.");
		return -EMSGSIZE;
	}

	cdd_data_item_t *item = &g_cdd_ctx.content.items[g_cdd_ctx.content.num_data_items];
	sys_put_be16(endpoint, item->ep_address);
	item->length = (uint8_t)len;
	memcpy(item->payload, data, len);

	g_cdd_ctx.content.num_data_items++;
	g_cdd_ctx.is_valid = true;
	g_cdd_ctx.content.type = 0; /* Full Configuration */
	
	LOG_INF("CDD_FT: Added item for EP 0x%04X.", endpoint);
	return 0;
}

void dect_cdd_ft_set_sink_addr(uint32_t sink_addr)
{
	sys_put_be32(sink_addr, g_cdd_ctx.content.sink_addr_be);
}

void dect_cdd_pt_process_beacon_info(uint32_t sink_addr, uint8_t app_seq_num)
{
	uint32_t local_sink = sys_get_be32(g_cdd_ctx.content.sink_addr_be);

	if (!g_cdd_ctx.is_valid || local_sink != sink_addr ||
	    g_cdd_ctx.content.app_seq_num != app_seq_num) {
		LOG_INF("CDD_PT: Update needed. Local(sink:0x%08X, seq:%u), Beacon(sink:0x%08X, seq:%u).",
			local_sink, g_cdd_ctx.content.app_seq_num, sink_addr, app_seq_num);
		pt_send_cdd_request();
	}
}

void dect_cdd_handle_incoming_pdu(const uint8_t *data, size_t len, uint32_t source_rd_id)
{
	if (dect_mac_get_role() == MAC_ROLE_FT) {
		if (len == sizeof(cdd_request_pdu_t)) {
			const cdd_request_pdu_t *req = (const cdd_request_pdu_t *)data;
			if ((req->reserved_type & 0x1F) == CDD_REQUEST_TYPE_COMPLETE_CONFIG) {
				LOG_INF("CDD_FT: Rcvd Config Request from 0x%08X.", source_rd_id);
				if (g_cdd_ctx.is_valid) {
					size_t items_len = 0;
					for (int i = 0; i < g_cdd_ctx.content.num_data_items; i++) {
						items_len += offsetof(cdd_data_item_t, payload) +
							     g_cdd_ctx.content.items[i].length;
					}
					size_t total_len = offsetof(cdd_content_pdu_t, items) + items_len;

					LOG_INF("CDD_FT: Sending response (len %zu).", total_len);
					dect_cvg_send(CVG_EP_MANAGEMENT_CDD, source_rd_id,
						      (uint8_t *)&g_cdd_ctx.content, total_len);
				}
			}
		}
	} else { /* PT Role */
		if (len >= offsetof(cdd_content_pdu_t, items)) {
			const cdd_content_pdu_t *resp = (const cdd_content_pdu_t *)data;
			uint32_t sink = sys_get_be32(resp->sink_addr_be);

			LOG_INF("CDD_PT: Rcvd Content from FT 0x%08X. Seq:%u, Items:%u",
				sink, resp->app_seq_num, resp->num_data_items);

			memcpy(g_cdd_ctx.content.sink_addr_be, resp->sink_addr_be, 4);
			g_cdd_ctx.content.app_seq_num = resp->app_seq_num;
			g_cdd_ctx.is_valid = true;

			const cdd_data_item_t *item = resp->items;
			size_t rem = len - offsetof(cdd_content_pdu_t, items);

			for (int i = 0; i < resp->num_data_items; i++) {
				if (rem < offsetof(cdd_data_item_t, payload)) break;
				
				size_t item_len = offsetof(cdd_data_item_t, payload) + item->length;
				if (rem < item_len) break;

				uint16_t ep = sys_get_be16(item->ep_address);
				if (g_cdd_ctx.handler) {
					g_cdd_ctx.handler(ep, item->payload, item->length);
				}

				rem -= item_len;
				item = (const cdd_data_item_t *)((const uint8_t *)item + item_len);
			}
		}
	}
}

#if IS_ENABLED(CONFIG_ZTEST)
void *dect_cdd_test_get_state_ptr(void) { return &g_cdd_ctx; }
size_t dect_cdd_test_get_state_size(void) { return sizeof(g_cdd_ctx); }
#endif