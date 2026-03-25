/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/tests/cdd_service/src/main.c */
/* This is a Ztest suite for verifying the end-to-end flow of the Configuration Data Distribution (CDD) service. */
#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>

#include "dect_cdd.h"
#include "dect_cvg.h"

#include <mac/dect_mac.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_if.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mocks/mock_nrf_modem_dect_phy.h>
#include "../../tests/utils/test_harness_helpers.h"
#include <zephyr/drivers/timer/system_timer.h> 

/* Ensure access to the test harness context setter */
extern void dect_mac_test_set_active_context(struct dect_mac_context *ctx);

LOG_MODULE_REGISTER(test_cdd_service, LOG_LEVEL_DBG);

/* --- Test Globals --- */
static dect_mac_context_t g_mac_ctx_pt;
static dect_mac_context_t g_mac_ctx_ft;

/* --- Mocks --- */
static struct {
	uint16_t endpoint_id;
	uint32_t dest_long_id;
	uint8_t payload[256];
	size_t len;
	int call_count;
} g_mock_cvg_send;

static struct {
	uint8_t cid;
	struct in6_addr prefix;
	int call_count;
} g_mock_l2_set_context;

/* Re-define the IPv6 element header in the test (application land) */
typedef struct {
	uint8_t element_type : 2;
	uint8_t element_version : 2;
	uint8_t rfu : 2;
	uint8_t prefix_type : 1;
	uint8_t context_usage : 1;
	uint8_t context_id : 4;
	uint8_t service_id : 4;
} __attribute__((packed)) cdd_ipv6_addr_element_hdr_t;

int dect_cvg_send(uint16_t endpoint_id, uint32_t dest_long_id, const uint8_t *app_sdu,
		  size_t app_sdu_len)
{
	g_mock_cvg_send.call_count++;
	g_mock_cvg_send.endpoint_id = endpoint_id;
	g_mock_cvg_send.dest_long_id = dest_long_id;
	g_mock_cvg_send.len = MIN(app_sdu_len, sizeof(g_mock_cvg_send.payload));
	memcpy(g_mock_cvg_send.payload, app_sdu, g_mock_cvg_send.len);
	return 0;
}

/* Generic handler for CDD items (Simulation of Zephyr integration) */
static void test_cdd_item_handler(uint16_t endpoint, const uint8_t *data, size_t len)
{
	if (endpoint == CVG_EP_IPV6_PROFILE) {
		const cdd_ipv6_addr_element_hdr_t *hdr = (const cdd_ipv6_addr_element_hdr_t *)data;
		if (hdr->element_type == 1 && hdr->context_usage == 1) {
			const uint8_t *prefix_bytes = data + sizeof(*hdr);
			struct in6_addr prefix;
			memset(&prefix, 0, sizeof(prefix));
			memcpy(&prefix.s6_addr, prefix_bytes, 8);

			g_mock_l2_set_context.call_count++;
			g_mock_l2_set_context.cid = hdr->context_id;
			memcpy(&g_mock_l2_set_context.prefix, &prefix, sizeof(struct in6_addr));
			
			printk("[TEST_CB] Applied 6LoWPAN context for CID %u\n", hdr->context_id);
		}
	}
}

/* --- Enhanced Test Setup & Teardown --- */

static void *cdd_suite_setup(void)
{
	dect_cdd_init();
	return NULL;
}

static void cdd_suite_before(void *fixture)
{
	ARG_UNUSED(fixture);
	int err;

	/* Reset shared resources and global state */
	memset(&g_mac_ctx_pt, 0, sizeof(g_mac_ctx_pt));
	memset(&g_mac_ctx_ft, 0, sizeof(g_mac_ctx_ft));
	memset(&g_mock_cvg_send, 0, sizeof(g_mock_cvg_send));
	memset(&g_mock_l2_set_context, 0, sizeof(g_mock_l2_set_context));

	/* Initialize FT MAC core */
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	err = dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	zassert_ok(err, "FT dect_mac_core_init failed");

	/* Initialize PT MAC core */
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	err = dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);
	zassert_ok(err, "PT dect_mac_core_init failed");

	/* Mock association */
	g_mac_ctx_pt.role_ctx.pt.associated_ft.is_valid = true;
	g_mac_ctx_pt.role_ctx.pt.associated_ft.long_rd_id = 0x11223344;
	
	/* Register the test handler */
	dect_cdd_register_handler(test_cdd_item_handler);

	printk("[TEST_SETUP] CDD Test setup completed\n");
}

static void cdd_suite_after(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&g_mock_cvg_send, 0, sizeof(g_mock_cvg_send));
	memset(&g_mock_l2_set_context, 0, sizeof(g_mock_l2_set_context));
	printk("[TEST_CLEANUP] CDD Test cleanup completed\n");
}

/* --- Test Cases --- */
extern void *dect_cdd_test_get_state_ptr(void);
extern size_t dect_cdd_test_get_state_size(void);

ZTEST(cdd_service_tests, test_cdd_request_response_flow)
{
	printk("\n--- RUNNING TEST: %s ---\n", __func__);

	uint32_t ft_id = 0x11223344;
	uint32_t pt_id = 0xAABBCCDD;
	
	uint8_t ft_cdd_state[512];
	uint8_t pt_cdd_state[512];
	size_t state_size = dect_cdd_test_get_state_size();
	zassert_true(state_size <= sizeof(ft_cdd_state), "State buffer too small");

	struct in6_addr ft_prefix;
	net_addr_pton(AF_INET6, CONFIG_NET_CONFIG_MY_IPV6_PREFIX, &ft_prefix);
	
	/* 1. FT builds its configuration */
	printk("\n[TEST] 1. FT builds its configuration\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	dect_cdd_init();
	dect_cdd_ft_set_sink_addr(ft_id);

	/* Build IPv6 item payload manually in test (simulating app land) */
	uint8_t ipv6_item_buf[32];
	cdd_ipv6_addr_element_hdr_t *hdr = (cdd_ipv6_addr_element_hdr_t *)ipv6_item_buf;
	hdr->element_type = 1;
	hdr->element_version = 0;
	hdr->rfu = 0;
	hdr->prefix_type = 0;
	hdr->context_usage = 1;
	hdr->context_id = 1;
	hdr->service_id = 0;
	memcpy(ipv6_item_buf + sizeof(*hdr), &ft_prefix.s6_addr, 8);
	
	dect_cdd_ft_add_item(CVG_EP_IPV6_PROFILE, ipv6_item_buf, sizeof(*hdr) + 8);
	memcpy(ft_cdd_state, dect_cdd_test_get_state_ptr(), state_size);

	/* 2. PT is notified of a new App Sequence Number and sends a request */
	printk("\n[TEST] 2. PT processes beacon info and sends CDD request\n");
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	dect_cdd_init();
	dect_cdd_register_handler(test_cdd_item_handler);
	memcpy(pt_cdd_state, dect_cdd_test_get_state_ptr(), state_size);

	dect_cdd_pt_process_beacon_info(ft_id, 1);

	zassert_equal(g_mock_cvg_send.call_count, 1, "PT did not send CDD request");
	zassert_equal(g_mock_cvg_send.endpoint_id, CVG_EP_MANAGEMENT_CDD, "Wrong EP");

	uint8_t captured_req[256];
	size_t req_len = g_mock_cvg_send.len;
	memcpy(captured_req, g_mock_cvg_send.payload, req_len);
	memcpy(pt_cdd_state, dect_cdd_test_get_state_ptr(), state_size);
	memset(&g_mock_cvg_send, 0, sizeof(g_mock_cvg_send));

	/* 3. FT receives the request and sends back the content */
	printk("\n[TEST] 3. FT receives request and sends CDD content response\n");
	dect_mac_test_set_active_context(&g_mac_ctx_ft);
	memcpy(dect_cdd_test_get_state_ptr(), ft_cdd_state, state_size);
	dect_cdd_handle_incoming_pdu(captured_req, req_len, pt_id);

	zassert_equal(g_mock_cvg_send.call_count, 1, "FT did not respond");
	zassert_equal(g_mock_cvg_send.endpoint_id, CVG_EP_MANAGEMENT_CDD, "Wrong EP");

	uint8_t captured_resp[256];
	size_t resp_len = g_mock_cvg_send.len;
	memcpy(captured_resp, g_mock_cvg_send.payload, resp_len);

	/* 4. PT receives the content and sets the 6LoWPAN context */
	printk("\n[TEST] 4. PT receives CDD content and applies 6LoWPAN context\n");
	dect_mac_test_set_active_context(&g_mac_ctx_pt);
	memcpy(dect_cdd_test_get_state_ptr(), pt_cdd_state, state_size);
	dect_cdd_handle_incoming_pdu(captured_resp, resp_len, ft_id);

	/* Final assertions */
	printk("\n[TEST] 5. Final assertions\n");
	zassert_equal(g_mock_l2_set_context.call_count, 1, "6LoWPAN context not applied");
	zassert_equal(g_mock_l2_set_context.cid, 1, "Wrong CID");
	zassert_mem_equal(&g_mock_l2_set_context.prefix.s6_addr, &ft_prefix.s6_addr, 8, "Wrong prefix");
}

ZTEST_SUITE(cdd_service_tests, 
	NULL,
	cdd_suite_setup,
	cdd_suite_before,
	cdd_suite_after,
	NULL);