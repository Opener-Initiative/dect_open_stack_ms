/* lib/dect_nrplus/tests/cdd_service/src/main.c */
/* This is a new file. It contains a Ztest suite for verifying the end-to-end flow of the Configuration Data Distribution (CDD) service. */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <zephyr/net/ip.h>

#include "dect_cdd.h"
#include "dect_cvg.h"
#include "dect_mac_core.h"
#include "dect_mac_context.h"

LOG_MODULE_REGISTER(test_cdd_service, LOG_LEVEL_DBG);

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

int dect_nrplus_l2_set_sixlowpan_context(const struct device *dev, uint8_t cid,
					 const struct in6_addr *prefix)
{
	ARG_UNUSED(dev);
	g_mock_l2_set_context.call_count++;
	g_mock_l2_set_context.cid = cid;
	memcpy(&g_mock_l2_set_context.prefix, prefix, sizeof(struct in6_addr));
	return 0;
}

/* --- Test Fixture --- */
static void *setup(void)
{
	memset(&g_mock_cvg_send, 0, sizeof(g_mock_cvg_send));
	memset(&g_mock_l2_set_context, 0, sizeof(g_mock_l2_set_context));
	dect_cdd_init();
	return NULL;
}

/* --- Test Cases --- */

ZTEST(cdd_service_tests, test_cdd_request_response_flow)
{
	dect_mac_context_t ft_ctx, pt_ctx;
	uint32_t ft_id = 0x11223344;
	uint32_t pt_id = 0xAABBCCDD;

	/* 1. Setup FT and PT contexts */
	g_current_mac_ctx = &ft_ctx;
	dect_mac_core_init(MAC_ROLE_FT, ft_id);
	g_current_mac_ctx = &pt_ctx;
	dect_mac_core_init(MAC_ROLE_PT, pt_id);
	pt_ctx.role_ctx.pt.associated_ft.is_valid = true;
	pt_ctx.role_ctx.pt.associated_ft.long_rd_id = ft_id;

	/* 2. FT builds its configuration */
	g_current_mac_ctx = &ft_ctx;
	dect_cdd_ft_build_own_config();

	/* 3. PT is notified of a new App Sequence Number and sends a request */
	g_current_mac_ctx = &pt_ctx;
	dect_cdd_pt_process_beacon_info(ft_id, 1); /* Seq num 1 is different from default 0 */

	zassert_equal(g_mock_cvg_send.call_count, 1, "PT did not send CDD request");
	zassert_equal(g_mock_cvg_send.endpoint_id, CVG_EP_MANAGEMENT_CDD, "Request sent to wrong EP");
	zassert_equal(g_mock_cvg_send.dest_long_id, ft_id, "Request sent to wrong FT");

	/* 4. FT receives the request and sends back the content */
	uint8_t captured_req[g_mock_cvg_send.len];
	memcpy(captured_req, g_mock_cvg_send.payload, g_mock_cvg_send.len);
	memset(&g_mock_cvg_send, 0, sizeof(g_mock_cvg_send));

	g_current_mac_ctx = &ft_ctx;
	dect_cdd_handle_incoming_pdu(captured_req, sizeof(captured_req), pt_id);

	zassert_equal(g_mock_cvg_send.call_count, 1, "FT did not send CDD content response");
	zassert_equal(g_mock_cvg_send.endpoint_id, CVG_EP_MANAGEMENT_CDD, "Response sent to wrong EP");
	zassert_equal(g_mock_cvg_send.dest_long_id, pt_id, "Response sent to wrong PT");

	/* 5. PT receives the content and sets the 6LoWPAN context */
	uint8_t captured_resp[g_mock_cvg_send.len];
	memcpy(captured_resp, g_mock_cvg_send.payload, g_mock_cvg_send.len);

	g_current_mac_ctx = &pt_ctx;
	dect_cdd_handle_incoming_pdu(captured_resp, sizeof(captured_resp), ft_id);

	zassert_equal(g_mock_l2_set_context.call_count, 1, "6LoWPAN context was not set");
	zassert_equal(g_mock_l2_set_context.cid, 1, "Context set with wrong CID");

	struct in6_addr expected_prefix;
	net_addr_pton(AF_INET6, CONFIG_NET_CONFIG_MY_IPV6_PREFIX, &expected_prefix);
	zassert_mem_equal(&g_mock_l2_set_context.prefix, &expected_prefix, sizeof(struct in6_addr),
			  "Context set with wrong prefix");
}

ZTEST_SUITE(cdd_service_tests, NULL, setup, NULL, NULL, NULL);