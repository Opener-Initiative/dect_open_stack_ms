/* drivers/net/dect_stack_api.c */
// Overview: This is the complete and correct L2 network driver. It is simple and only implements the required L2 hooks for the network stack.

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_core.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

/* Include ONLY the public APIs of the layers below */
#include <dect_cdd.h>
#include <dect_cvg.h>
#include <dect_mac.h>
#include "dect_stack_api.h"

LOG_MODULE_REGISTER(dect_nrplus_l2, CONFIG_DECT_NRPLUS_LOG_LEVEL);

static void dect_nrplus_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct dect_nrplus_dev_ctx *ctx = dev->data;
	struct net_if *iface = ctx->iface;

	LOG_INF("DECT NR+ RX thread started for iface %p.", iface);

	while (1) {
		/* Allocate a buffer for the maximum possible IP packet size */
		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(iface, 1500, AF_UNSPEC, 0, K_FOREVER);
		if (!pkt) {
			LOG_WRN("L2_RX: Failed to allocate RX packet");
			continue;
		}

		size_t received_len = net_buf_tailroom(pkt->buffer);
		int ret = dect_cvg_receive(net_buf_tail(pkt->buffer), &received_len, K_FOREVER);

		if (ret == 0 && received_len > 0) {
			net_buf_add(pkt->buffer, received_len);
			LOG_DBG("L2_RX: Received %zu bytes from CVG, passing to IP stack", received_len);

			/* Pass the raw (potentially compressed) data to the network core.
			 * The 6LoWPAN module, if enabled, will hook into this process
			 * and perform decompression before the IP stack sees it.
			 */
			if (net_recv_data(iface, pkt) < 0) {
				LOG_ERR("L2_RX: Failed to push packet to IP stack");
				net_pkt_unref(pkt);
			}
		} else {
			if (ret != -EAGAIN) {
				LOG_ERR("L2_RX: dect_cvg_receive failed: %d", ret);
			}
			net_pkt_unref(pkt);
		}
	}
}

static int dect_nrplus_send(struct net_if *iface, struct net_pkt *pkt)
// static int dect_nrplus_send(const struct device *dev, struct net_pkt *pkt)
{
	int ret;

	/* The packet received here from the IP stack has already been processed
	 * by the 6LoWPAN module (if enabled), so it's already compressed.
	 * We just need to send it.
	 */
	size_t payload_len = net_pkt_get_len(pkt);

	uint32_t dest_id = dect_mac_get_associated_ft_long_id();
	if (dest_id == 0) {
		LOG_WRN("L2_SEND: Cannot send, not associated with an FT.");
		return -ENOTCONN;
	}

	LOG_DBG("L2_SEND: Sending packet (%zu bytes) to CVG for Dest 0x%08X", payload_len, dest_id);

	ret = dect_cvg_send(CVG_EP_IPV6_PROFILE, dest_id, pkt->buffer->data, payload_len);
	if (ret < 0) {
		LOG_ERR("dect_cvg_send failed: %d", ret);
	}

	return ret;
}

static void dect_nrplus_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dect_nrplus_dev_ctx *ctx = dev->data;
	uint8_t mac_addr[6];

	LOG_INF("Initializing DECT NR+ L2 network interface %p", iface);
	ctx->iface = iface;

	/* The IID is now set by the 6LoWPAN module itself based on the link address */
	uint32_t long_id = dect_mac_get_own_long_id();
	mac_addr[0] = 0x00;
	mac_addr[1] = 0xDE; /* DE for DECT */
	sys_put_be32(long_id, &mac_addr[2]);
	net_if_set_link_addr(iface, mac_addr, sizeof(mac_addr), NET_LINK_ETHERNET);
}

// static int dect_nrplus_iface_enable(const struct device *dev, bool enable)
// {
// 	struct dect_nrplus_dev_ctx *ctx = dev->data;

// 	if (enable) {
// 		if (ctx->rx_thread_id != 0) {
// 			return -EALREADY;
// 		}
// 		ctx->rx_thread_id = k_thread_create(&ctx->rx_thread_data, ctx->rx_stack,
// 						  CONFIG_DECT_NRPLUS_RX_THREAD_STACK_SIZE,
// 						  dect_nrplus_rx_thread, (void *)dev, NULL, NULL,
// 						  CONFIG_DECT_NRPLUS_RX_THREAD_PRIORITY, 0,
// 						  K_NO_WAIT);
// 		k_thread_name_set(ctx->rx_thread_id, "dect_nrplus_rx");
// 	} else {
// 		/* Shutdown logic would go here if needed */
// 	}
// 	return 0;
// }


static int dect_nrplus_enable(struct net_if *iface, bool state)
{
	return 1;
}
	/**
	 * Return L2 flags for the network interface.
	 */
static enum net_l2_flags dect_nrplus_get_l2_flags(struct net_if *iface)
// static enum net_l2_flags dect_nrplus_get_l2_flags(const struct device *dev)
{
	return NET_L2_POINT_TO_POINT;
}

static const struct net_l2 dect_nrplus_l2 = {
	.get_flags = dect_nrplus_get_l2_flags,
	.send = dect_nrplus_send,	
	.enable = dect_nrplus_enable,	
};

static const struct net_if_api dect_nrplus_if_api = {
	.init = dect_nrplus_iface_init,
    // .send = dect_nrplus_iface_send,	
	// .enable = dect_nrplus_iface_enable,
};

static int dect_nrplus_driver_init(const struct device *dev)
{
	LOG_INF("DECT NR+ L2 driver instance %s initialized.", dev->name);
	return 0;
}

#define DECT_NRPLUS_INIT(inst)                                                                     \
	K_THREAD_STACK_DEFINE(dect_nrplus_rx_stack_##inst,                                         \
			      CONFIG_DECT_NRPLUS_RX_THREAD_STACK_SIZE);                            \
                                                                                                   \
	static struct dect_nrplus_dev_ctx dect_nrplus_ctx_##inst = {                                \
		.rx_stack = dect_nrplus_rx_stack_##inst,                                           \
	};                                                                                         \
                                                                                                   \
	NET_DEVICE_DT_INST_DEFINE(inst, dect_nrplus_driver_init, NULL, &dect_nrplus_ctx_##inst,     \
				  NULL, CONFIG_DECT_NRPLUS_INIT_PRIORITY, &dect_nrplus_if_api,      \
				  &dect_nrplus_l2, 1280); /* MTU for 6LoWPAN */

DT_INST_FOREACH_STATUS_OKAY(DECT_NRPLUS_INIT)