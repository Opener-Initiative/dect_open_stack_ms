/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DECT NR+ MAC Layer Debug Shell
 */

#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/util.h>

/* Include only the necessary public MAC headers */
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_phy_ctrl.h> // Needed for PCC calculation test
#include <mac/dect_mac_pdu.h>

#if IS_ENABLED(CONFIG_DECT_MAC_SHELL_ENABLE)

LOG_MODULE_REGISTER(dect_mac_shell, CONFIG_DECT_MAC_SHELL_LOG_LEVEL);

/* --- Command Implementations --- */

static int cmd_get_context(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	if (!ctx) {
		shell_error(sh, "MAC context not available.");
		return -EAGAIN;
	}

	shell_print(sh, "--- DECT MAC CONTEXT ---");
	shell_print(sh, "Role: %s, State: %s", (ctx->role == MAC_ROLE_PT ? "PT" : "FT"),
		    dect_mac_state_to_str(ctx->state));
	shell_print(sh, "Own Long ID: 0x%08X, Short ID: 0x%04X", ctx->own_long_rd_id,
		    ctx->own_short_rd_id);
	shell_print(sh, "Own HPC: %u, PSN: %u", ctx->hpc, ctx->psn);

	if (ctx->role == MAC_ROLE_PT) {
		shell_print(sh, "Associated FT: 0x%08X (Valid: %s, Secure: %s)",
			    ctx->role_ctx.pt.associated_ft.long_rd_id,
			    ctx->role_ctx.pt.associated_ft.is_valid ? "Yes" : "No",
			    ctx->role_ctx.pt.associated_ft.is_secure ? "Yes" : "No");
	} else {
		shell_print(sh, "Connected PTs:");
		int count = 0;
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (ctx->role_ctx.ft.connected_pts[i].is_valid) {
				shell_print(sh, "  - PT[%d]: 0x%08X (Secure: %s)", i,
					    ctx->role_ctx.ft.connected_pts[i].long_rd_id,
					    ctx->role_ctx.ft.connected_pts[i].is_secure ? "Yes"
											: "No");
				count++;
			}
		}
		if (count == 0) {
			shell_print(sh, "  (None)");
		}
	}
	shell_print(sh, "--- END OF CONTEXT ---");
	return 0;
}

static int cmd_send_mac_data(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: send_data <\"payload\"> [target_short_id_hex_if_ft]");
		return -EINVAL;
	}

	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (ctx->state != MAC_STATE_ASSOCIATED) {
		shell_error(sh, "Device is not in ASSOCIATED state.");
		return -ENETDOWN;
	}

	mac_sdu_t *sdu = dect_mac_buffer_alloc(K_SECONDS(1));
	if (!sdu) {
		shell_error(sh, "Failed to allocate SDU buffer.");
		return -ENOMEM;
	}

	size_t payload_len = strlen(argv[1]);
	if (payload_len > (CONFIG_DECT_MAC_SDU_MAX_SIZE - 2)) { // Reserve space for MUX header
		shell_error(sh, "Payload too large.");
		dect_mac_api_buffer_free(sdu);
		return -EMSGSIZE;
	}

	/* This shell command sends a raw user data IE.
	 * A real application would send a DLC/CVG PDU.
	 */
	int sdu_len = build_user_data_ie_muxed(sdu->data, CONFIG_DECT_MAC_SDU_MAX_SIZE,
					       (const uint8_t *)argv[1], payload_len,
					       IE_TYPE_USER_DATA_FLOW_1);
	if (sdu_len < 0) {
		shell_error(sh, "Failed to build user data IE: %d", sdu_len);
		dect_mac_api_buffer_free(sdu);
		return sdu_len;
	}
	sdu->len = sdu_len;

	int err;
	if (ctx->role == MAC_ROLE_FT) {
		if (argc < 3) {
			shell_error(sh, "FT role requires a target Short ID (hex).");
			dect_mac_api_buffer_free(sdu);
			return -EINVAL;
		}
		uint16_t target_id = (uint16_t)strtoul(argv[2], NULL, 16);
		err = dect_mac_api_ft_send_to_pt(sdu, MAC_FLOW_RELIABLE_DATA, target_id);
	} else {
		err = dect_mac_api_send(sdu, MAC_FLOW_RELIABLE_DATA);
	}

	if (err) {
		shell_error(sh, "Failed to queue SDU for TX: %d", err);
		/* The API frees the SDU on error */
	} else {
		shell_print(sh, "Queued SDU for transmission.");
	}

	return err;
}

static int cmd_test_pcc_params(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: test_pcc <payload_bytes> <mcs_code> [mu_code] [beta_code]");
		return -EINVAL;
	}

	size_t payload_bytes = (size_t)strtol(argv[1], NULL, 10);
	uint8_t mcs_code = (uint8_t)strtol(argv[2], NULL, 10);
	uint8_t mu_code = (argc > 3) ? (uint8_t)strtol(argv[3], NULL, 10) : 0;
	uint8_t beta_code = (argc > 4) ? (uint8_t)strtol(argv[4], NULL, 10) : 0;

	uint8_t out_pkt_len_field, out_pkt_len_type;
	uint8_t mcs_after_calc = mcs_code;

	shell_print(sh, "Testing PCC Calc: Payload: %zu B, MCS_in: %u, mu_code: %u, beta_code: %u",
		    payload_bytes, mcs_code, mu_code, beta_code);

	dect_mac_phy_ctrl_calculate_pcc_params(payload_bytes, mu_code, beta_code,
					       &out_pkt_len_field, &mcs_after_calc,
					       &out_pkt_len_type);

	shell_print(sh, "Output: PacketLenField: %u (0x%02X) => %u units", out_pkt_len_field,
		    out_pkt_len_field, out_pkt_len_field + 1);
	shell_print(sh, "        PacketLenType: %u (%s)", out_pkt_len_type,
		    out_pkt_len_type == 0 ? "subslots" : "slots");
	shell_print(sh, "        MCS_final: %u", mcs_after_calc);

	return 0;
}

static int cmd_show_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (!ctx) {
		shell_error(sh, "MAC context not available.");
		return -EAGAIN;
	}

	shell_print(sh, "=== DECT NR+ MAC/DLC/CVG STATISTICS ===\n");

	/* MAC Layer Statistics */
	shell_print(sh, "--- MAC Layer ---");
	shell_print(sh, "  Current State: %s", dect_mac_state_to_str(ctx->state));
	shell_print(sh, "  Role: %s", ctx->role == MAC_ROLE_FT ? "FT (Fixed Terminal)" : "PT (Portable Terminal)");
	shell_print(sh, "  Security Enabled: %s", ctx->security_enabled ? "Yes" : "No");
	
	/* PHY/MAC Counters */
	shell_print(sh, "  Beacons TX: %u, RX: %u", 
		    ctx->role == MAC_ROLE_FT ? ctx->role_ctx.ft.beacon_tx_count : 0,
		    ctx->role == MAC_ROLE_PT ? ctx->role_ctx.pt.beacon_rx_count : 0);
	
	/* HARQ Statistics */
	shell_print(sh, "  HARQ Processes:");
	int active_harq = 0;
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		if (ctx->harq[i].is_active) {
			active_harq++;
		}
	}
	shell_print(sh, "    Active: %d/%d", active_harq, MAX_HARQ_PROCESSES);

	/* Association Statistics */
	if (ctx->role == MAC_ROLE_PT) {
		shell_print(sh, "  Association Attempts: %u", ctx->role_ctx.pt.assoc_attempt_count);
		shell_print(sh, "  RACH Transmissions: %u", ctx->role_ctx.pt.rach_tx_count);
		shell_print(sh, "  Current RACH CW: %u", ctx->role_ctx.pt.current_contention_window);
	} else {
		int connected_count = 0;
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (ctx->role_ctx.ft.connected_pts[i].is_valid) {
				connected_count++;
			}
		}
		shell_print(sh, "  Connected PTs: %d/%d", connected_count, MAX_PEERS_PER_FT);
	}

	/* Security Statistics */
	if (ctx->security_enabled) {
		shell_print(sh, "  Security:");
		shell_print(sh, "    HPC: %u, PSN: %u", ctx->hpc, ctx->psn);
		shell_print(sh, "    MIC Failures: %u", ctx->consecutive_mic_failures);
	}

	/* DLC Layer Statistics (if available) */
	shell_print(sh, "\n--- DLC Layer ---");
	shell_print(sh, "  (Statistics would require DLC context access)");
	shell_print(sh, "  Service Types: Type 0-3 (Transparent, Segmentation, ARQ)");
	shell_print(sh, "  Max SDU Size: %d bytes", CONFIG_DECT_MAC_SDU_MAX_SIZE);

	/* CVG Layer Statistics (if available) */
	shell_print(sh, "\n--- CVG Layer ---");
	shell_print(sh, "  (Statistics would require CVG context access)");
	shell_print(sh, "  Service Types: Type 0-4 (Transparent to Full ARQ)");
	
	shell_print(sh, "\n=== END OF STATISTICS ===");
	return 0;
}

static int cmd_inject_ie(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: inject_ie <ie_type_hex> <hex_payload>");
		shell_print(sh, "Example: inject_ie 0x12 48656c6c6f");
		shell_print(sh, "IE Types: 0x1D=User Data, 0x12=Custom Test");
		return -EINVAL;
	}

	dect_mac_context_t *ctx = dect_mac_get_active_context();
	if (!ctx || ctx->state != MAC_STATE_ASSOCIATED) {
		shell_error(sh, "Device must be in ASSOCIATED state to inject IEs.");
		return -ENETDOWN;
	}

	/* Parse IE type */
	uint8_t ie_type = (uint8_t)strtoul(argv[1], NULL, 16);
	
	/* Parse hex payload */
	const char *hex_str = argv[2];
	size_t hex_len = strlen(hex_str);
	if (hex_len % 2 != 0) {
		shell_error(sh, "Hex payload must have even number of characters.");
		return -EINVAL;
	}

	size_t payload_len = hex_len / 2;
	if (payload_len > (CONFIG_DECT_MAC_SDU_MAX_SIZE - 10)) {
		shell_error(sh, "Payload too large.");
		return -EMSGSIZE;
	}

	/* Allocate SDU buffer */
	mac_sdu_t *sdu = dect_mac_buffer_alloc(K_SECONDS(1));
	if (!sdu) {
		shell_error(sh, "Failed to allocate SDU buffer.");
		return -ENOMEM;
	}

	/* Convert hex string to binary */
	uint8_t payload[CONFIG_DECT_MAC_SDU_MAX_SIZE];
	for (size_t i = 0; i < payload_len; i++) {
		char byte_str[3] = {hex_str[i*2], hex_str[i*2 + 1], '\0'};
		payload[i] = (uint8_t)strtoul(byte_str, NULL, 16);
	}

	/* Build MAC PDU with custom IE
	 * Format: [MAC Extension (2b) + IE Type (6b)] [Payload]
	 * For simplicity, use no length field (extension = 00)
	 */
	sdu->data[0] = ie_type & 0x3F; // 6-bit IE type, extension = 00
	memcpy(&sdu->data[1], payload, payload_len);
	sdu->len = payload_len + 1;

	shell_print(sh, "Injecting IE Type 0x%02X with %zu bytes payload...", ie_type, payload_len);

	/* Send the injected IE */
	int err;
	if (ctx->role == MAC_ROLE_FT) {
		/* For FT, need to specify target PT */
		uint16_t target_id = 0x0001; // Default to first PT
		
		/* Find first valid connected PT */
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (ctx->role_ctx.ft.connected_pts[i].is_valid) {
				target_id = ctx->role_ctx.ft.connected_pts[i].short_rd_id;
				break;
			}
		}
		
		shell_print(sh, "Sending to PT Short ID: 0x%04X", target_id);
		err = dect_mac_api_ft_send_to_pt(sdu, MAC_FLOW_UNRELIABLE_DATA, target_id);
	} else {
		/* For PT, send to associated FT */
		err = dect_mac_api_send(sdu, MAC_FLOW_UNRELIABLE_DATA);
	}

	if (err) {
		shell_error(sh, "Failed to send injected IE: %d", err);
	} else {
		shell_print(sh, "IE injection successful!");
	}

	return err;
}

/* --- Shell Command Structure Definition --- */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_dect_cmds,
	SHELL_CMD(context, NULL, "Print the current MAC context.", cmd_get_context),
	SHELL_CMD(send_data, NULL, "Queue a MAC data PDU for TX <\"payload\"> [target_id_if_ft]",
		  cmd_send_mac_data),
	SHELL_CMD(test_pcc, NULL, "Test PCC parameter calculation <bytes> <mcs> [mu] [beta]",
		  cmd_test_pcc_params),
	SHELL_CMD(stats, NULL, "Display MAC/DLC/CVG layer statistics", cmd_show_stats),
	SHELL_CMD(inject_ie, NULL, "Inject custom MAC IE <ie_type_hex> <hex_payload>", cmd_inject_ie),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dect, &sub_dect_cmds, "DECT MAC Debug Commands", NULL);

#endif /* CONFIG_DECT_MAC_SHELL_ENABLE */