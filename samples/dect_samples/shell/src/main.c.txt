/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* --- CORRECTED DECT NR+ STACK INCLUDES --- */
// Use the public, top-level headers provided by the library.
#include <mac/dect_mac_main_dispatcher.h> // For dect_mac_state_to_str
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_sm.h>
#include <dect_cvg.h>

/* This new function will be our clean, top-level API for the stack */
int dect_nrplus_stack_init(void);
void dect_nrplus_stack_start(void);


LOG_MODULE_REGISTER(dect_app, LOG_LEVEL_INF);

/* --- Globals for Ping Command --- */
static K_SEM_DEFINE(g_ping_reply_sem, 0, 1);
static uint64_t g_rtt_ms;
static uint8_t g_received_payload[128];
static size_t g_received_len;

/* --- RX Thread to handle incoming data and echo requests --- */
static void rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t rx_buf[256];
	uint8_t tx_buf[256];

	LOG_INF("Application RX thread started.");

	while (1) {
		size_t len = sizeof(rx_buf);
		int ret = dect_cvg_receive(rx_buf, &len, K_FOREVER);

		if (ret < 0) {
			if (ret != -EAGAIN) {
				LOG_WRN("dect_cvg_receive failed with error: %d", ret);
			}
			continue;
		}

		/* Check if this is a PING request or a PONG reply */
		if (len > 5 && strncmp((const char *)rx_buf, "PING:", 5) == 0) {
			/* This is a PING request, we must echo it back as a PONG */
			LOG_INF("Received PING request, sending PONG reply...");

			dect_mac_context_t *mac_ctx = get_mac_context();
			uint32_t peer_long_id = 0;

			if (mac_ctx->role == MAC_ROLE_PT) {
				peer_long_id = mac_ctx->role_ctx.pt.associated_ft.long_rd_id;
			} else {
				/* For an FT, find the first connected PT to echo back to. */
				for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
					if (mac_ctx->role_ctx.ft.connected_pts[i].is_valid) {
						peer_long_id = mac_ctx->role_ctx.ft.connected_pts[i].long_rd_id;
						break;
					}
				}
			}

			if (peer_long_id != 0) {
				int tx_len = snprintk(tx_buf, sizeof(tx_buf), "PONG:%s", rx_buf + 5);
				dect_cvg_send(CVG_EP_IPV6_PROFILE, peer_long_id, tx_buf, (size_t)tx_len);
			}

		} else if (len > 5 && strncmp((const char *)rx_buf, "PONG:", 5) == 0) {
			/* This is a PONG reply to our ping */
			char *timestamp_str = (char *)(rx_buf + 5);
			uint64_t original_timestamp = strtoull(timestamp_str, NULL, 10);
			uint64_t reply_timestamp = k_uptime_get();

			g_rtt_ms = reply_timestamp - original_timestamp;
			g_received_len = MIN(len, sizeof(g_received_payload));
			memcpy(g_received_payload, rx_buf, g_received_len);

			k_sem_give(&g_ping_reply_sem);
		}
	}
}

K_THREAD_DEFINE(rx_tid, 2048, rx_thread_entry, NULL, NULL, NULL, 7, 0, 0);

/* --- Custom Shell Command Implementation --- */
static int cmd_ping(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Missing destination Long RD ID.");
		// shell_help(sh, "Usage: ping <destination_long_rd_id_hex>");
		return -EINVAL;
	}

	char *endptr;
	uint32_t dest_id = strtoul(argv[1], &endptr, 16);
	if (*endptr != '\0') {
		shell_error(sh, "Invalid hex value for destination ID: %s", argv[1]);
		return -EINVAL;
	}

	shell_print(sh, "Pinging 0x%08X...", dest_id);

	char payload[64];
	uint64_t send_time = k_uptime_get();
	int len = snprintk(payload, sizeof(payload), "PING:%llu", send_time);

	int ret = dect_cvg_send(CVG_EP_IPV6_PROFILE, dest_id, (uint8_t *)payload, (size_t)len);
	if (ret < 0) {
		shell_error(sh, "Failed to send ping: dect_cvg_send error %d", ret);
		return ret;
	}

	/* Wait for the reply from the RX thread */
	ret = k_sem_take(&g_ping_reply_sem, K_SECONDS(5));
	if (ret != 0) {
		shell_error(sh, "Request timed out.");
		return -ETIMEDOUT;
	}

	g_received_payload[g_received_len] = '\0'; /* Null-terminate for printing */
	shell_print(sh, "Reply from 0x%08X: len=%zu time=%llums", dest_id, g_received_len,
		    g_rtt_ms);
	shell_print(sh, "  Payload: %s", g_received_payload);

	return 0;
}

SHELL_CMD_REGISTER(ping, NULL, "Send a DECT NR+ echo request <dest_id_hex>", cmd_ping);

/* --- Main Application Entry Point --- */
int main(void)
{
	int err;
	LOG_INF("DECT NR+ Ping Test Application Started");

	/* --- 1. Initialize the full DECT NR+ Stack with a single call --- */
	err = dect_nrplus_stack_init();
	if (err) {
		LOG_ERR("Failed to initialize DECT NR+ stack: %d", err);
		return -1;
	}

	/* --- 2. Start the MAC state machine's operation --- */
	dect_nrplus_stack_start();

	/* --- 3. Wait for the link to be established --- */
	dect_mac_context_t *ctx = get_mac_context();
	LOG_INF("Waiting for MAC to become associated...");
	while (ctx->state != MAC_STATE_ASSOCIATED) {
		k_sleep(K_SECONDS(1));
		LOG_INF("Current MAC state: %s", dect_mac_state_to_str(ctx->state));
	}

	LOG_INF("MAC is associated! Shell is ready.");
	LOG_INF("Your Long RD ID is: 0x%08X", ctx->own_long_rd_id);
	if (ctx->role == MAC_ROLE_PT) {
		LOG_INF("Connected to FT with Long RD ID: 0x%08X",
			ctx->role_ctx.pt.associated_ft.long_rd_id);
	}

	// The main thread can now sleep or do other work.
	// The RX thread and shell will handle the ping test.
	return 0;
}