/* drivers/net/dect_nrplus.c */
/* This change refactors the L2 driver to use a callback from the MAC layer for state changes. This ensures the IID is set at the correct time (upon association) and makes the RX thread more efficient. */

#define LOG_LEVEL CONFIG_DECT_NRPLUS_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dect_nrplus_l2, LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/ipv6.h>
#include <zephyr/net/icmp.h>
#include <zephyr/net/sixlowpan.h>
#include <zephyr/net/ip.h>
#include <zephyr/sys/byteorder.h>

// #include "dect_nrplus.h"
#include "dect_nrplus.h" /* Use the local driver header */
#include <dect_nrplus.h> /* Use the public library API header */
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <dect_cvg.h>
#include <dect_cdd.h>


static void set_link_addr(struct net_if *iface)
{
	struct dect_nrplus_dev_ctx *ctx = net_if_get_device(iface)->data;
	dect_mac_context_t *mac_ctx = get_mac_context();
	uint8_t iid[8];
	uint32_t sink_id;

	if (mac_ctx->role == MAC_ROLE_PT) {
		if (!mac_ctx->role_ctx.pt.associated_ft.is_valid) {
			LOG_WRN("Cannot set IID: PT is not associated.");
			return;
		}
		sink_id = mac_ctx->role_ctx.pt.associated_ft.long_rd_id;
	} else { /* FT Role */
		sink_id = mac_ctx->own_long_rd_id;
	}

	sys_put_be32(sink_id, &iid[0]);
	sys_put_be32(mac_ctx->own_long_rd_id, &iid[4]);

	net_if_set_link_addr(iface, iid, sizeof(iid), NET_LINK_IEEE802154);
	ctx->iid_is_set = true;

	LOG_INF("DECT NR+ IID set to %08x-%08x", sink_id, mac_ctx->own_long_rd_id);
}

static void mac_state_change_cb(dect_mac_state_t new_state)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(dect_nrplus0));
	struct dect_nrplus_dev_ctx *ctx = dev->data;

	if (!ctx || !ctx->iface) {
		return;
	}

	if (new_state == MAC_STATE_ASSOCIATED) {
		if (!ctx->iid_is_set) {
			set_link_addr(ctx->iface);
		}
	} else if (ctx->iid_is_set && new_state < MAC_STATE_ASSOCIATED) {
		LOG_WRN("Lost association, IID is now invalid.");
		ctx->iid_is_set = false;
	}
}

static void cdd_prefix_handler_cb(const struct in6_addr *prefix, uint8_t len, uint8_t context_id)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(dect_nrplus0));

	if (!device_is_ready(dev)) {
		LOG_ERR("CDD_CB: DECT NR+ device not ready, cannot set context.");
		return;
	}

	if (len != 64) {
		LOG_WRN("CDD_CB: Received prefix with len %u, only /64 is supported.", len);
	}

	dect_nrplus_l2_set_sixlowpan_context(dev, context_id, prefix);
}

static void dect_nrplus_rx_thread(void *p1, void *p2, void *p3)
{
	struct dect_nrplus_dev_ctx *ctx = p1;
	struct net_if *iface = ctx->iface;
	struct net_pkt *pkt;
	int ret;

	LOG_INF("DECT NR+ RX thread started for iface %p.", iface);

	while (!ctx->shutdown_req) {
		pkt = net_pkt_rx_alloc_with_buffer(iface, 256, AF_UNSPEC, 0, K_MSEC(1000));
		if (!pkt) {
			continue;
		}

		size_t received_len = net_buf_tailroom(pkt->buffer);

		ret = dect_cvg_receive(pkt->buffer->data, &received_len, K_MSEC(500));
		if (ret < 0) {
			if (ret != -EAGAIN) {
				LOG_WRN("dect_cvg_receive returned error %d. Retrying.", ret);
			}
			net_pkt_unref(pkt);
			continue;
		}

		if (received_len == 0) {
			net_pkt_unref(pkt);
			continue;
		}

		net_buf_add(pkt->buffer, received_len);

		LOG_DBG("RX: Received %zu bytes from CVG, passing to 6LoWPAN for decompression",
			received_len);

		ret = sixlowpan_uncompress(pkt);
		if (ret < 0) {
			LOG_ERR("Failed to decompress and inject packet into network stack: %d", ret);
			net_pkt_unref(pkt);
		}
	}

	LOG_INF("DECT NR+ RX thread exiting.");
}

static void dect_nrplus_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dect_nrplus_dev_ctx *ctx = dev->data;

	LOG_INF("Initializing DECT NR+ L2 network interface %p", iface);
	ctx->iface = iface;
	ctx->iid_is_set = false;

	sixlowpan_set_iface(iface);
}

static int dect_nrplus_iface_send(struct net_if *iface, struct net_pkt *pkt)
{
	dect_mac_context_t *mac_ctx = get_mac_context();
	int ret;
	uint32_t dest_long_id = 0;

	if (net_pkt_family(pkt) != AF_INET6) {
		return -EPROTOTYPE;
	}

	if (mac_ctx->state < MAC_STATE_ASSOCIATED) {
		LOG_WRN("L2_SEND: Cannot send, MAC not associated.");
		return -ENETDOWN;
	}

	const struct in6_addr *dest_addr = &NET_IPV6_HDR(pkt)->dst;

	if (net_ipv6_is_addr_mcast(dest_addr)) {
		dest_long_id = 0xFFFFFFFF; /* Broadcast address for DLC */
	} else {
		uint32_t dest_sink_id = sys_get_be32(&dest_addr->s6_addr[0]);
		uint32_t dest_device_id = sys_get_be32(&dest_addr->s6_addr[8]);
		uint32_t my_sink_id = (mac_ctx->role == MAC_ROLE_PT)
					      ? mac_ctx->role_ctx.pt.associated_ft.long_rd_id
					      : mac_ctx->own_long_rd_id;

		if (dest_sink_id == my_sink_id) {
			dest_long_id = dest_device_id;
		} else {
			dest_long_id = 0xFFFFFFFE; /* Backend address for DLC */
		}
	}

	if (dest_long_id == 0) {
		LOG_ERR("L2_SEND: Could not determine destination Long RD ID. Dropping packet.");
		return -EHOSTUNREACH;
	}

	ret = sixlowpan_compress(pkt);
	if (ret < 0) {
		LOG_ERR("6LoWPAN compression failed: %d", ret);
		return ret;
	}

	LOG_DBG("L2_SEND: Sending compressed packet (%zu bytes) to CVG for EP 0x%04X, Dest 0x%08X",
		net_pkt_get_len(pkt), CVG_EP_IPV6_PROFILE, dest_long_id);

	ret = dect_cvg_send(CVG_EP_IPV6_PROFILE, dest_long_id, pkt->buffer->data,
			    net_pkt_get_len(pkt));
	if (ret < 0) {
		LOG_ERR("dect_cvg_send failed: %d", ret);
	}

	net_pkt_unref(pkt);
	return ret;
}

static enum net_l2_flags dect_nrplus_l2_flags(struct net_if *iface)
{
	ARG_UNUSED(iface);
	return NET_L2_POINTOPOINT | NET_L2_NO_ARP;
}

static int dect_nrplus_iface_enable(struct net_if *iface, bool enable)
{
	const struct device *dev = net_if_get_device(iface);
	struct dect_nrplus_dev_ctx *ctx = dev->data;

	if (enable) {
		if (ctx->rx_thread_id != 0) {
			LOG_WRN("RX thread already started for iface %p", iface);
			return -EALREADY;
		}

		ctx->shutdown_req = false;
		ctx->rx_thread_id =
			k_thread_create(&ctx->rx_thread_data, ctx->rx_stack,
					K_THREAD_STACK_SIZEOF(ctx->rx_stack), dect_nrplus_rx_thread,
					ctx, NULL, NULL, CONFIG_DECT_NRPLUS_RX_THREAD_PRIORITY, 0,
					K_NO_WAIT);
		if (!ctx->rx_thread_id) {
			LOG_ERR("Failed to create DECT NR+ RX thread for iface %p", iface);
			return -ENOMEM;
		}
		k_thread_name_set(ctx->rx_thread_id, "dect_nrplus_rx");
		LOG_INF("DECT NR+ RX thread started for iface %p", iface);
	} else {
		if (ctx->rx_thread_id != 0) {
			ctx->shutdown_req = true;
			int ret = k_thread_join(ctx->rx_thread_id, K_SECONDS(5));

			if (ret) {
				LOG_ERR("Failed to join DECT NR+ RX thread (err %d).", ret);
			} else {
				LOG_INF("DECT NR+ RX thread stopped for iface %p", iface);
			}
			ctx->rx_thread_id = 0;
			ctx->iid_is_set = false;
		}
	}
	return 0;
}

static struct net_if_api dect_nrplus_if_api = {
	.iface_api.init = dect_nrplus_iface_init,
	.iface_api.send = dect_nrplus_iface_send,
	.iface_api.enable = dect_nrplus_iface_enable,
};

static int dect_nrplus_driver_init(const struct device *dev)
{
	struct dect_nrplus_dev_ctx *ctx = dev->data;

	ctx->cvg_service_type = DT_PROP(dev->of_node, cvg_service_type);

	dect_cdd_init();
	dect_cdd_register_prefix_handler(cdd_prefix_handler_cb);

	/* Register a callback with the MAC to be notified of state changes */
	// dect_mac_sm_register_state_change_cb(mac_state_change_cb);
	printk("dect_nrplus_driver_init \n");
	dect_mac_register_state_change_cb(mac_state_change_cb);

	LOG_INF("DECT NR+ L2 driver initialized, device: %s, CVG Service: %d",
		dev->name, ctx->cvg_service_type);
	return 0;
}

static int dect_nrplus_pm_action(const struct device *dev, enum pm_device_action action)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(action);
	return 0;
}

NET_L2_INIT(DECT_NRPLUS_L2, dect_nrplus_l2_flags, NULL, NULL);

#define DECT_NRPLUS_INIT(inst)							\
	/* Define the stack memory area for this device instance */		\
	static K_THREAD_STACK_DEFINE(dect_nrplus_rx_stack_##inst,		\
				     CONFIG_DECT_NRPLUS_RX_THREAD_STACK_SIZE);	\
										\
	/* Define the context struct for this device instance and initialize the stack pointer */ \
	static struct dect_nrplus_dev_ctx dect_nrplus_ctx_##inst = {		\
		.rx_stack = dect_nrplus_rx_stack_##inst,			\
	};									\
										\
	NET_DEVICE_DT_INST_DEFINE(inst,						\
				  dect_nrplus_driver_init,			\
				  dect_nrplus_pm_action,			\
				  &dect_nrplus_ctx_##inst,			\
				  NULL,						\
				  CONFIG_DECT_NRPLUS_INIT_PRIORITY,		\
				  &dect_nrplus_if_api,				\
				  &DECT_NRPLUS_L2,				\
				  1500); /* MTU */

DT_INST_FOREACH_STATUS_OKAY(DECT_NRPLUS_INIT)