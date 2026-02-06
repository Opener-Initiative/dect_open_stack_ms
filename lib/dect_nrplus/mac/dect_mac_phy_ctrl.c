/* dect_mac/dect_mac_phy_ctrl.c */
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/sys/util.h> // For ARRAY_SIZE, MIN, MAX
#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
#include <psa/crypto.h>
#endif


#include <mac/dect_mac_phy_ctrl.h>
#include <mac/dect_mac_core.h>      // For get_mac_context()
#include <mac/dect_mac_context.h>   // For dect_mac_context_t and its members, config constants
#include <mac/dect_mac_main_dispatcher.h> // For string utility functions (logging)
#include <mac/dect_mac_pdu.h>       // For dect_mac_header_type_octet_t (though nrf_modem provides its own PCC structs)
#include <mac/dect_mac_phy_tbs_tables.h> // Include the new header with TBS tables
#include <mac/dect_mac_timeline_utils.h>
#include <mac/dect_mac_phy_if.h> /* For handle registration functions */

#if IS_ENABLED(CONFIG_ZTEST)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

LOG_MODULE_REGISTER(dect_mac_phy_ctrl, CONFIG_DECT_MAC_PHY_CTRL_LOG_LEVEL);


static union nrf_modem_dect_phy_hdr g_phy_pcc_tx_constructor_buf;
static uint8_t g_phy_pdc_tx_constructor_buf_ctrl[MAX_MAC_PDU_SIZE_FOR_PCC_CALC];


void dect_mac_phy_ctrl_build_pcc_header(union nrf_modem_dect_phy_hdr *pcc_hdr,
				       const dect_mac_context_t *ctx,
				       uint16_t full_mac_pdu_len,
				       uint16_t target_receiver_short_id, bool is_beacon,
				       uint8_t mu_code,
				       const union nrf_modem_dect_phy_feedback *feedback,
				       pending_op_type_t op_type)
{

	printk("[PCC_BUILD] Building header. Active context role: %s, Network ID LSB to be used: 0x%02X\n",
	       (ctx->role == MAC_ROLE_PT ? "PT" : "FT"), (uint8_t)(ctx->network_id_32bit & 0xFF));


	uint8_t pcc_nrf_phy_type_val;
	uint8_t calculated_pcc_packet_len_field;
	uint8_t mcs_to_use_for_pcc_calc;
	uint8_t calculated_pcc_pkt_len_type_field;

	/* The PDC payload IS the full MAC PDU */
	// const uint8_t *pdc_content_for_phy = full_mac_pdu_to_send;
	uint16_t pdc_content_len_for_phy = full_mac_pdu_len;

	mcs_to_use_for_pcc_calc = ctx->config.default_data_mcs_code;
	if (is_beacon || op_type == PENDING_OP_PT_RACH_ASSOC_REQ) {
		mcs_to_use_for_pcc_calc = 0;
	}

	dect_mac_phy_ctrl_calculate_pcc_params(pdc_content_len_for_phy, mu_code, ctx->own_phy_params.beta,
					       &calculated_pcc_packet_len_field,
					       &mcs_to_use_for_pcc_calc,
					       &calculated_pcc_pkt_len_type_field);

	if (is_beacon) {
		pcc_nrf_phy_type_val = 0;
		struct nrf_modem_dect_phy_hdr_type_1 *pcc1 = &pcc_hdr->hdr_type_1;

		pcc1->header_format = 0b000;
		pcc1->packet_length_type = calculated_pcc_pkt_len_type_field;
		pcc1->packet_length = calculated_pcc_packet_len_field;
		pcc1->short_network_id = (uint8_t)(ctx->network_id_32bit & 0xFF);

		pcc1->transmitter_id_hi = (uint8_t)(ctx->own_short_rd_id >> 8);
		pcc1->transmitter_id_lo = (uint8_t)(ctx->own_short_rd_id & 0xFF);

		pcc1->transmit_power = ctx->config.default_tx_power_code;
		if (mcs_to_use_for_pcc_calc > 7) {
			LOG_ERR("PCC_BUILD: Beacon MCS %u is invalid for 3-bit field. Defaulting to 0.",
				mcs_to_use_for_pcc_calc);
			mcs_to_use_for_pcc_calc = 0;
		}
		pcc1->df_mcs = mcs_to_use_for_pcc_calc & 0x07;
		pcc1->reserved = 0;
	} else {
		pcc_nrf_phy_type_val = 1;
		struct nrf_modem_dect_phy_hdr_type_2 *pcc2 = &pcc_hdr->hdr_type_2;

		pcc2->header_format = 0b000;
		pcc2->packet_length_type = calculated_pcc_pkt_len_type_field;
		pcc2->packet_length = calculated_pcc_packet_len_field;
		pcc2->short_network_id = (uint8_t)(ctx->network_id_32bit & 0xFF);

		pcc2->transmitter_id_hi = (uint8_t)(ctx->own_short_rd_id >> 8);
		pcc2->transmitter_id_lo = (uint8_t)(ctx->own_short_rd_id & 0xFF);

		pcc2->transmit_power = ctx->config.default_tx_power_code;
		pcc2->df_mcs = mcs_to_use_for_pcc_calc & 0x0F;

		/* Directly assign bytes from the native uint16_t.
		 * The struct fields are defined as hi/lo, so this correctly
		 * places the bytes for the PHY, regardless of host endianness.
		 */
		pcc2->receiver_id_hi = (uint8_t)(target_receiver_short_id >> 8);
		pcc2->receiver_id_lo = (uint8_t)(target_receiver_short_id & 0xFF);

		pcc2->num_spatial_streams = 0;

		bool is_harq_data_op = (op_type >= PENDING_OP_PT_DATA_TX_HARQ0 &&
					op_type <= PENDING_OP_FT_DATA_TX_HARQ_MAX);
		if (is_harq_data_op) {
			int harq_idx_base = (op_type >= PENDING_OP_FT_DATA_TX_HARQ0)
						? PENDING_OP_FT_DATA_TX_HARQ0
						: PENDING_OP_PT_DATA_TX_HARQ0;
			int harq_idx = op_type - harq_idx_base;

			if (harq_idx >= 0 && harq_idx < MAX_HARQ_PROCESSES &&
			    ctx->harq_tx_processes[harq_idx].is_active) {
				pcc2->df_harq_process_num = harq_idx & 0x07;
				pcc2->df_new_data_indication =
					(ctx->harq_tx_processes[harq_idx].tx_attempts == 1) ? 1 : 0;
				pcc2->df_redundancy_version =
					ctx->harq_tx_processes[harq_idx].redundancy_version & 0x03;
			} else {
				LOG_WRN("PCC_BUILD: HARQ op_type %s but no active/valid HARQ proc %d. Using default HARQ fields.",
					dect_pending_op_to_str(op_type), harq_idx);
				pcc2->df_new_data_indication = 1;
				pcc2->df_redundancy_version = 0;
				pcc2->df_harq_process_num = 0;
			}
		} else {
			pcc2->df_new_data_indication = 1;
			pcc2->df_redundancy_version = 0;
			pcc2->df_harq_process_num = 0;
		}

		if (feedback) {
			memcpy(&pcc2->feedback, feedback, sizeof(union nrf_modem_dect_phy_feedback));
		}
		printk("[PHY_CTRL] pcc2->short_network_id =%0x04 \n", pcc2->short_network_id) ;
	}
}


int dect_mac_phy_ctrl_start_rx(uint32_t carrier, uint32_t duration_modem_units,
			       enum nrf_modem_dect_phy_rx_mode mode, uint32_t phy_op_handle,
			       uint16_t expected_receiver_id, pending_op_type_t op_type)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();

	printk("PHY_CTRL_RX: Starting RX op %s (H:%u), pending op %s (H:%u).\n",
				dect_pending_op_to_str(op_type), phy_op_handle,
				dect_pending_op_to_str(ctx->pending_op_type),
				ctx->pending_op_handle);

	if (ctx->pending_op_type != PENDING_OP_NONE && ctx->pending_op_handle != phy_op_handle) {
		if (ctx->pending_op_type != op_type || ctx->pending_op_handle != phy_op_handle) {
			printk("PHY_CTRL_RX: Cannot start RX op %s (H:%u), op %s (H:%u) already pending.\n",
				dect_pending_op_to_str(op_type), phy_op_handle,
				dect_pending_op_to_str(ctx->pending_op_type),
				ctx->pending_op_handle);
			// LOG_WRN("PHY_CTRL_RX: Cannot start RX op %s (H:%u), op %s (H:%u) already pending.",
			// 	dect_pending_op_to_str(op_type), phy_op_handle,
			// 	dect_pending_op_to_str(ctx->pending_op_type),
			// 	ctx->pending_op_handle);				
			return -EBUSY;
		}
	}
	ctx->pending_op_handle = phy_op_handle;
	ctx->pending_op_type = op_type;
	ctx->current_rx_op_carrier = carrier;

	printk("[PHY_CTRL_SET_PENDING] Set pending op to Type: %s, Handle: %u\n",
	       dect_pending_op_to_str(op_type), phy_op_handle);

	struct nrf_modem_dect_phy_rx_params rx_params = {
		.start_time = 0, /* Default to immediate start for RX initiated by MAC logic */
		.handle = phy_op_handle,
		.network_id = ctx->network_id_32bit,
		.mode = mode,
		.rssi_interval =
			NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF, /* Default, can be changed by caller if needed */
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED, /* TODO: Set if multi-link AFC is used */
		.rssi_level = 0, /* Auto AGC */
		.carrier = carrier,
		.duration = duration_modem_units,
		.filter = { .short_network_id = (uint8_t)(ctx->network_id_32bit & 0xFF),
			    .is_short_network_id_used = 1,
			    /* Use the passed-in receiver ID. This allows for broadcast (0xFFFF) or unicast. */
			    .receiver_identity = (expected_receiver_id) }
				// .receiver_identity = sys_cpu_to_be16(expected_receiver_id) }
	};

	printk("PHY_CTRL_RX: Sending RX. op %s (H:%u), pending op %s (H:%u).\n",
				dect_pending_op_to_str(op_type), phy_op_handle,
				dect_pending_op_to_str(ctx->pending_op_type),
				ctx->pending_op_handle);

	printk("PHY_CTRL_RX: Sending RX. Hdl: %u, C: %u, Mode: %d, Dur: %u TU, RxID_Filter: 0x%04X, OpT: %s \n",
		phy_op_handle, carrier, mode, duration_modem_units, expected_receiver_id,
		dect_pending_op_to_str(op_type));

	// LOG_DBG("PHY_CTRL_RX: Starting RX. Hdl: %u, C: %u, Mode: %d, Dur: %u TU, RxID_Filter: 0x%04X, OpT: %s",
	// 	phy_op_handle, carrier, mode, duration_modem_units, expected_receiver_id,
	// 	dect_pending_op_to_str(op_type));

	int ret = nrf_modem_dect_phy_rx(&rx_params);
	// printk("[PHY_CTRL_RX] nrf_modem_dect_phy_rx returned: %d\n", ret);
	if (ret == 0) {
		dect_mac_phy_if_register_op_handle(phy_op_handle, ctx);
	}

	if (ret != 0) {
		LOG_ERR("PHY_CTRL_RX: nrf_modem_dect_phy_rx() failed: %d. Hdl: %u, OpT: %s", ret,
			phy_op_handle, dect_pending_op_to_str(op_type));
		if (ctx->pending_op_handle == phy_op_handle && ctx->pending_op_type == op_type) {
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
	}
	return ret;
}


int dect_mac_phy_ctrl_assemble_final_pdu(
    uint8_t *target_final_mac_pdu_buf,
    size_t target_buf_max_len,
    uint8_t mac_hdr_type_octet_byte,
    const void *common_hdr_data,
    size_t common_hdr_len,
    const uint8_t *mac_sdu_area_data,
    size_t mac_sdu_area_len,
    uint16_t *out_assembled_pdu_len_cleartext)
{
	printk("[PDU_ASSEMBLY_DBG] Entering assemble_final_pdu.\n");
	printk("  -> mac_hdr_type_octet_byte: 0x%02X\n", mac_hdr_type_octet_byte);
	printk("  -> common_hdr_len: %zu\n", common_hdr_len);
	printk("  -> sdu_area_len: %zu\n", mac_sdu_area_len);


    size_t current_offset = 0;

    printk("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    printk("[PHY CTRL] dect_mac_phy_ctrl_assemble_final_pdu -> Using MAC Header Type: 0x%02X\n", mac_hdr_type_octet_byte);	

	if (!target_final_mac_pdu_buf || !out_assembled_pdu_len_cleartext) {
		if (!target_final_mac_pdu_buf ){printk("[PHY CTRL] ERROR:dect_mac_phy_ctrl_assemble_final_pdu -> (!target_final_mac_pdu_buf}\n");}
		if (!mac_hdr_type_octet_byte){printk("[PHY CTRL] ERROR:dect_mac_phy_ctrl_assemble_final_pdu -> (!mac_hdr_type_octet_byte}\n");}
		printk("[PHY CTRL] ERROR:dect_mac_phy_ctrl_assemble_final_pdu -> (!target_final_mac_pdu_buf || !mac_hdr_type_octet_byte || !out_assembled_pdu_len_cleartext) \n");
        return -EINVAL;
    }
    // Validate conditional parameters
    if ((common_hdr_data == NULL && common_hdr_len > 0) || (common_hdr_data != NULL && common_hdr_len == 0)) {
		printk("[PHY CTRL] ERROR:dect_mac_phy_ctrl_assemble_final_pdu -> ((common_hdr_data == NULL && common_hdr_len > 0) || (common_hdr_data != NULL && common_hdr_len == 0)) \n");
        // Allow common_hdr_data to be NULL if common_hdr_len is 0 (e.g. no common header after type octet)
        if (common_hdr_data != NULL || common_hdr_len != 0) return -EINVAL;
    }
    if ((mac_sdu_area_data == NULL && mac_sdu_area_len > 0) || (mac_sdu_area_data != NULL && mac_sdu_area_len == 0)) {
		printk("[PHY CTRL] ERROR:dect_mac_phy_ctrl_assemble_final_pdu -> ((mac_sdu_area_data == NULL && mac_sdu_area_len > 0) || (mac_sdu_area_data != NULL && mac_sdu_area_len == 0)) \n");
        if (mac_sdu_area_data != NULL || mac_sdu_area_len != 0) return -EINVAL;
    }

	/* Total required size = 1 (type octet) + common header + SDU area */
	size_t total_required = 1 + common_hdr_len + mac_sdu_area_len;
	if (total_required > target_buf_max_len) {
		LOG_ERR("[PHY CTRL] PDU assembly overflow: %zu > %zu", total_required, target_buf_max_len);
		return -ENOMEM;
	}

    /* 1. MAC Header Type (1 byte) */
    target_final_mac_pdu_buf[current_offset] = mac_hdr_type_octet_byte;
    current_offset += 1;

printk("[PHY CTRL] dect_mac_phy_ctrl_assemble_final_pdu - mac_hdr_type_octet_byte:0x%08X \n", mac_hdr_type_octet_byte);


    // 2. MAC Common Header
    if (common_hdr_data && common_hdr_len > 0) {
        if (current_offset + common_hdr_len > target_buf_max_len) return -ENOMEM;
        memcpy(target_final_mac_pdu_buf + current_offset, common_hdr_data, common_hdr_len);
        current_offset += common_hdr_len;
    }

    // 3. MAC SDU Area (MUXed IEs or MUXed User Data SDU)
    if (mac_sdu_area_data && mac_sdu_area_len > 0) {
        if (current_offset + mac_sdu_area_len > target_buf_max_len) return -ENOMEM;
        memcpy(target_final_mac_pdu_buf + current_offset, mac_sdu_area_data, mac_sdu_area_len);
        current_offset += mac_sdu_area_len;
    }

	printk("[PHY CTRL] dect_mac_phy_ctrl_assemble_final_pdu -> current_offset: %d \n", current_offset);
	printk("  -> SDU Area len: %zu\n", mac_sdu_area_len);
	printk("  -> Final PDU len: %zu\n", current_offset);

	printk("[PDU_ASSEMBLY_DBG] Final PDU len: %zu\n", current_offset);
	printk("  -> Final PDU hexdump (first 48 bytes): ");
	for (int i=0; i<current_offset && i < 48; i++) { printk("%02x ", target_final_mac_pdu_buf[i]); }
	printk("\n");

    *out_assembled_pdu_len_cleartext = current_offset;
    return 0;
}

int dect_mac_phy_ctrl_start_tx_assembled(uint32_t carrier,
					 const uint8_t *full_mac_pdu_to_send,
					 uint16_t full_mac_pdu_len,
					 uint16_t target_receiver_short_id, bool is_beacon,
					 uint32_t phy_op_handle, pending_op_type_t op_type,
					 bool use_lbt, uint64_t phy_op_target_start_time,
					 uint8_t mu_code,
					 const union nrf_modem_dect_phy_feedback *feedback)
{
	dect_mac_context_t *ctx = dect_mac_get_active_context();
	printk("[PHY_CTRL_TX] Function called with context %p (Role: %s)\n",
	       (void *)ctx, (ctx->role == MAC_ROLE_PT ? "PT" : "FT"));
	printk("[PHY_CTRL_TX] Calling nrf_modem_dect_phy_tx with phy_type: %u  pending_op_type: %u ctx->pending_op_handle:%u phy_op_handle:%u \n",
	       is_beacon ? 0 : 1 , ctx->pending_op_type, ctx->pending_op_handle, phy_op_handle);
	// printk("[PHY_CTRL_TX] Calling nrf_modem_dect_phy_tx with pending_op_type: %u\n",
	//        ctx->pending_op_type);
	// printk("[PHY_CTRL_TX] Calling nrf_modem_dect_phy_tx with ctx->pending_op_handle:%u phy_op_handle:%u \n",
	//        ctx->pending_op_handle, phy_op_handle);
	if (!ctx) {
		printk("[PHY_CTRL] dect_mac_phy_ctrl_start_tx_assembled FAILED-> !ctx..(-EFAULT).\n");
		return -EFAULT;
	}

	ctx->pending_op_handle = phy_op_handle;
	ctx->pending_op_type = op_type;

	printk("[PHY_CTRL_SET_PENDING] Set pending op to Type: %s, Handle: %u\n",
	       dect_pending_op_to_str(op_type), phy_op_handle);

	if (full_mac_pdu_to_send == NULL || full_mac_pdu_len == 0 ||
	    full_mac_pdu_len < sizeof(dect_mac_header_type_octet_t) ||
	    full_mac_pdu_len > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
		LOG_ERR("PHY_CTRL_TX: Invalid PDU or length: %p, len %u", full_mac_pdu_to_send,
			full_mac_pdu_len);
		if (ctx->pending_op_handle == phy_op_handle) {
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
		return -EINVAL;
	}

		printk("\n");
		printk("  - FULL MAC PDU Payload (MAC PDU, len %u):\n", full_mac_pdu_len);
		for (int i = 0; i < full_mac_pdu_len; i++) {
			printk("%02x ", full_mac_pdu_to_send[i]);
		}
		printk("\n");

	/* The PDC payload IS the full MAC PDU. Copy it to the constructor buffer. */
	if (full_mac_pdu_len > sizeof(g_phy_pdc_tx_constructor_buf_ctrl)) {
		LOG_ERR("PHY_CTRL_TX: Full MAC PDU for PHY TX too large (%u > %zu)",
			full_mac_pdu_len, sizeof(g_phy_pdc_tx_constructor_buf_ctrl));
		if (ctx->pending_op_handle == phy_op_handle) {
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
		return -ENOMEM;
	}
	if (full_mac_pdu_len > 0) {
		memcpy(g_phy_pdc_tx_constructor_buf_ctrl, full_mac_pdu_to_send,
		       full_mac_pdu_len);
	}

	memset(&g_phy_pcc_tx_constructor_buf, 0, sizeof(g_phy_pcc_tx_constructor_buf));

	dect_mac_phy_ctrl_build_pcc_header(&g_phy_pcc_tx_constructor_buf, ctx,
					   full_mac_pdu_len, target_receiver_short_id, is_beacon, mu_code,
					   feedback, op_type);

	struct nrf_modem_dect_phy_tx_params tx_params = {
		.start_time = phy_op_target_start_time,
		.handle = phy_op_handle,
		.network_id = ctx->network_id_32bit,
		.phy_type = is_beacon ? 0 : 1,
		.lbt_rssi_threshold_max = use_lbt ? ctx->config.rssi_threshold_min_dbm : 0,
		.carrier = carrier,
		.lbt_period = use_lbt ? NRF_MODEM_DECT_LBT_PERIOD_MIN : 0,
		.phy_header = &g_phy_pcc_tx_constructor_buf,
		.bs_cqi = NRF_MODEM_DECT_PHY_BS_CQI_NOT_USED,
		// .data = (pdc_content_len_for_phy > 0) ? g_phy_pdc_tx_constructor_buf_ctrl : NULL,
		// .data_size = pdc_content_len_for_phy
		.data = (full_mac_pdu_len > 0) ? g_phy_pdc_tx_constructor_buf_ctrl : NULL,
		.data_size = full_mac_pdu_len		
	};

	int ret = nrf_modem_dect_phy_tx(&tx_params);

	if (ret == 0) {

		printk("[TX_EVIDENCE] Scheduled TX for op %s (Hdl %u)\n",
		       dect_pending_op_to_str(op_type), phy_op_handle);
		printk("  - PCC Header (PHY Type %u):\n", tx_params.phy_type);
		size_t pcc_len = (tx_params.phy_type == 0) ? sizeof(struct nrf_modem_dect_phy_hdr_type_1)
							   : sizeof(struct nrf_modem_dect_phy_hdr_type_2);
		for (int i = 0; i < pcc_len; i++) {
			printk("%02x ", ((uint8_t *)tx_params.phy_header)[i]);
		}
		printk("\n");
		printk("  - PDC Payload (MAC PDU, len %u):\n", tx_params.data_size);
		for (int i = 0; i < tx_params.data_size; i++) {
			printk("%02x ", tx_params.data[i]);
		}
		printk("\n");

		

		dect_mac_phy_if_register_op_handle(phy_op_handle, ctx);

		uint8_t pcc_pkt_len_field = is_beacon
						  ? g_phy_pcc_tx_constructor_buf.hdr_type_1.packet_length
						  : g_phy_pcc_tx_constructor_buf.hdr_type_2.packet_length;
		uint8_t pcc_pkt_len_type = is_beacon
						 ? g_phy_pcc_tx_constructor_buf.hdr_type_1.packet_length_type
						 : g_phy_pcc_tx_constructor_buf.hdr_type_2.packet_length_type;
		uint32_t num_units = pcc_pkt_len_field + 1;
		uint32_t duration_ticks;

		if (pcc_pkt_len_type == 0) { /* subslots */
			duration_ticks = num_units * get_subslot_duration_ticks_for_mu(mu_code);
		} else { /* slots */
			duration_ticks = num_units * get_subslots_per_etsi_slot_for_mu(mu_code) *
				       get_subslot_duration_ticks_for_mu(mu_code);
		}

		ctx->last_phy_op_end_time = phy_op_target_start_time + duration_ticks;
		LOG_DBG("PHY_CTRL_TX: Scheduled Hdl:%u, OpEnd:%llu", phy_op_handle,
			ctx->last_phy_op_end_time);
	} else {
		LOG_ERR("PHY_CTRL_TX: nrf_modem_dect_phy_tx() failed: %d. Hdl:%u, OpT:%s", ret,
			phy_op_handle, dect_pending_op_to_str(op_type));
		if (ctx->pending_op_handle == phy_op_handle) {
			ctx->pending_op_type = PENDING_OP_NONE;
			ctx->pending_op_handle = 0;
		}
	}
	return ret;
}


int dect_mac_phy_ctrl_start_rssi_scan(uint32_t carrier, uint32_t duration_modem_units,
                                      enum nrf_modem_dect_phy_rssi_interval reporting_interval,
                                      uint32_t phy_op_handle, pending_op_type_t op_type) {

	// printk("[PHY_CTRL_RSSI_DBG] Entering dect_mac_phy_ctrl_start_rssi_scan...\n");

    dect_mac_context_t *ctx = dect_mac_get_active_context();
    if (ctx->pending_op_type != PENDING_OP_NONE && ctx->pending_op_handle != phy_op_handle) {
         if (ctx->pending_op_type != op_type || ctx->pending_op_handle != phy_op_handle) {
            LOG_WRN("PHY_CTRL_RSSI: Op %s (H:%u) busy with %s (H:%u).",
                    dect_pending_op_to_str(op_type), phy_op_handle,
                    dect_pending_op_to_str(ctx->pending_op_type), ctx->pending_op_handle);
            return -EBUSY;
         }
    }
    ctx->pending_op_handle = phy_op_handle;
    ctx->pending_op_type = op_type;

	printk("[PHY_CTRL_SET_PENDING] Set pending op to Type: %s, Handle: %u\n",
	       dect_pending_op_to_str(op_type), phy_op_handle);

    struct nrf_modem_dect_phy_rssi_params rssi_params = {
        .start_time = 0, // Immediate start for scans initiated by MAC logic
        .handle = phy_op_handle,
        .carrier = carrier,
        .duration = duration_modem_units,
        .reporting_interval = reporting_interval
    };
    LOG_INF("PHY_CTRL_RSSI: Starting RSSI. Hdl: %u, C: %u, Dur: %u TU, RepInt: %d, OpT: %s",
            phy_op_handle, carrier, duration_modem_units, reporting_interval, dect_pending_op_to_str(op_type));

printk("[PHY_CTRL_RSSI_DBG] About to call nrf_modem_dect_phy_rssi...\n");

    int ret = nrf_modem_dect_phy_rssi(&rssi_params);
printk("[PHY_CTRL_RSSI_DBG] Returned from nrf_modem_dect_phy_rssi with code: %d\n", ret);

	if (ret == 0) {
		dect_mac_phy_if_register_op_handle(phy_op_handle, ctx);
	}

    if (ret != 0) {
        LOG_ERR("PHY_CTRL_RSSI: nrf_modem_dect_phy_rssi() failed: %d. Hdl: %u, OpT: %s", ret, phy_op_handle, dect_pending_op_to_str(op_type));
        if (ctx->pending_op_handle == phy_op_handle && ctx->pending_op_type == op_type) {
             ctx->pending_op_type = PENDING_OP_NONE;
             ctx->pending_op_handle = 0;
        }
    }
    return ret;
}

int dect_mac_phy_ctrl_cancel_op(uint32_t phy_op_handle) {
    LOG_INF("PHY_CTRL: Requesting cancel for PHY op handle %u", phy_op_handle);
    // The NRF_MODEM_DECT_PHY_EVT_COMPLETED for the cancelled op will clear pending_op_type.
    // NRF_MODEM_DECT_PHY_EVT_CANCELED confirms the cancel request itself.
    return nrf_modem_dect_phy_cancel(phy_op_handle);
}

pending_op_type_t dect_mac_phy_ctrl_handle_op_complete(const struct nrf_modem_dect_phy_op_complete_event *event) {
    dect_mac_context_t *ctx = dect_mac_get_active_context();
    pending_op_type_t completed_type = PENDING_OP_NONE;

    if (event == NULL) {
        printk("{ERROR} PHY_CTRL_OP_DONE: NULL event pointer.\n");
		// LOG_ERR("PHY_CTRL_OP_DONE: NULL event pointer.");
        return PENDING_OP_NONE;
    }

	printk("[PHY_CTRL_OP_DONE] Received completion for Handle: %u. Current pending Handle: %u, Type: %s\n",
	       event->handle, ctx->pending_op_handle, dect_pending_op_to_str(ctx->pending_op_type));

	/*
	 * CRITICAL: Always deregister the handle for any completed operation to prevent leaks,
	 * regardless of whether it matches the MAC's currently tracked "pending" operation.
	 */
	dect_mac_phy_if_deregister_op_handle(event->handle);

    // Check if this completion matches the MAC's currently tracked pending operation
    // if (event->handle == ctx->pending_op_handle && ctx->pending_op_type != PENDING_OP_NONE) {
	if (ctx && ctx->pending_op_handle == event->handle) {
        completed_type = ctx->pending_op_type;
        printk("PHY_CTRL_OP_DONE: Matches pending op: Handle %u, Type %s, Err %d (%s) \n",
                event->handle, dect_pending_op_to_str(completed_type),
                event->err, nrf_modem_dect_phy_err_to_str(event->err));

		/* Handle critical PHY errors that require halting the stack */
		if (event->err == NRF_MODEM_DECT_PHY_ERR_TEMP_HIGH) {
			dect_mac_enter_error_state("PHY reports TEMP_HIGH - thermal shutdown");
		}

		if (ctx->state != MAC_STATE_PT_WAIT_ASSOC_RESP){
			dect_mac_phy_if_deregister_op_handle(event->handle);
		}
			

        // LOG_INF("PHY_CTRL_OP_DONE: Matches pending op: Handle %u, Type %s, Err %d (%s)",
        //         event->handle, dect_pending_op_to_str(completed_type),
        //         event->err, nrf_modem_dect_phy_err_to_str(event->err));				
        // ctx->pending_op_type = PENDING_OP_NONE;
        // ctx->pending_op_handle = 0; // Clear the global pending operation
		/*
		 * State is NO LONGER cleared here. The caller (state machine) is now responsible
		 * for calling dect_mac_core_clear_pending_op() when it has finished processing.
		 */
    } else {
        // This can happen if:
        // 1. An operation completed that MAC wasn't specifically tracking with pending_op_handle (e.g., a multi-part op where only the first part was tracked).
        // 2. An operation was cancelled, and this is its completion event, but pending_op_handle was already cleared by a new op starting.
        // 3. A stale completion event for an old handle.
        if (ctx->pending_op_type == PENDING_OP_NONE && event->err == NRF_MODEM_DECT_PHY_ERR_OP_CANCELED) {
             LOG_DBG("PHY_CTRL_OP_DONE: Op CANCELED (Hdl %u), no specific MAC op was pending with this handle (or already cleared).", event->handle);
        } else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS && event->err != NRF_MODEM_DECT_PHY_ERR_OP_CANCELED) {
            LOG_WRN("PHY_CTRL_OP_DONE: Error %d (%s) for unexpected/stale handle %u (current pending: Hdl %u, Type %s)",
                    event->err, nrf_modem_dect_phy_err_to_str(event->err),
                    event->handle, ctx->pending_op_handle, dect_pending_op_to_str(ctx->pending_op_type));
        } else { // Success or Canceled, but not matching current pending
             LOG_DBG("PHY_CTRL_OP_DONE: Success/Canceled for Hdl %u, but not matching current pending (Hdl %u, Type %s).",
                    event->handle, ctx->pending_op_handle, dect_pending_op_to_str(ctx->pending_op_type));
        }
    }
	// printk("completed_type:%d\n", completed_type);
	
    return completed_type;
}


void dect_mac_phy_ctrl_calculate_pcc_params(size_t mac_pdc_payload_len_bytes,
					    uint8_t mu_code, uint8_t beta_code,
					    uint8_t *out_packet_length_field,
					    uint8_t *in_out_selected_mcs_field,
					    uint8_t *out_packet_length_type_field)
{
	if (!out_packet_length_field || !in_out_selected_mcs_field ||
	    !out_packet_length_type_field) {
		LOG_ERR("PCC_CALC: NULL output pointers!");
		if (out_packet_length_field) {
			*out_packet_length_field = 0;
		}
		if (out_packet_length_type_field) {
			*out_packet_length_type_field = 0;
		}
		return;
	}

	uint32_t pdc_payload_len_bits = (uint32_t)mac_pdc_payload_len_bytes * 8;
	uint8_t num_subslots_needed = 0;
	bool found_fit = false;
	uint8_t selected_mcs = *in_out_selected_mcs_field;

	const uint32_t (*selected_tbs_table)[TBS_MAX_SUB_SLOTS_J];

	/* Select the correct TBS table based on mu and beta */
	if (beta_code == 0) { /* beta = 1 */
		switch (mu_code) {
		case 0: /* mu = 1 */
			selected_tbs_table = tbs_single_slot_mu1_beta1;
			break;
		case 1: /* mu = 2 */
			selected_tbs_table = tbs_single_slot_mu2_beta1;
			break;
		case 2: /* mu = 4 */
			selected_tbs_table = tbs_single_slot_mu4_beta1;
			break;
		default:
			LOG_ERR("PCC_CALC: Unsupported mu_code=%u for beta=1. Using default.",
				mu_code);
			selected_tbs_table = tbs_single_slot_mu1_beta1;
			break;
		}
	} else {
		LOG_ERR("PCC_CALC: Unsupported beta_code=%u. Using default table.", beta_code);
		selected_tbs_table = tbs_single_slot_mu1_beta1;
	}

	if (selected_mcs > TBS_MAX_MCS_INDEX) {
		LOG_ERR("PCC_CALC: Requested MCS %u out of range (max %u). Clamping to max.",
			selected_mcs, TBS_MAX_MCS_INDEX);
		selected_mcs = TBS_MAX_MCS_INDEX;
		*in_out_selected_mcs_field = selected_mcs;
	}

	if (mac_pdc_payload_len_bytes == 0) {
		num_subslots_needed = 1;
		found_fit = true;
	} else {
		for (uint8_t j_idx = 0; j_idx < TBS_MAX_SUB_SLOTS_J; j_idx++) {
			if (selected_tbs_table[selected_mcs][j_idx] == 0 && pdc_payload_len_bits > 0) {
				continue;
			}
			if (pdc_payload_len_bits <= selected_tbs_table[selected_mcs][j_idx]) {
				num_subslots_needed = j_idx + 1;
				found_fit = true;
				break;
			}
		}
	}

	if (!found_fit) {
		LOG_ERR("PCC_CALC: Payload %zu bytes (%u bits) too large for MCS %u even at max %u subslots (TBS: %u bits). Clamping length.",
			mac_pdc_payload_len_bytes, pdc_payload_len_bits, selected_mcs,
			TBS_MAX_SUB_SLOTS_J,
			selected_tbs_table[selected_mcs][TBS_MAX_SUB_SLOTS_J - 1]);
		num_subslots_needed = TBS_MAX_SUB_SLOTS_J;
	}

	uint8_t subslots_per_slot = get_subslots_per_etsi_slot_for_mu(mu_code);

	if (num_subslots_needed > 16) {
		/* Large payload: Try to use Slot mode (Type 1) if possible.
		 * Note: Currently limited by TBS tables spanning only 16 subslots total.
		 */
		uint8_t slots_needed = (num_subslots_needed + subslots_per_slot - 1) / subslots_per_slot;
		if (slots_needed > 16) {
			slots_needed = 16;
			LOG_WRN("PCC_CALC: Payload exceeds 16-slot max capacity. Clamping.");
		}
		*out_packet_length_field = slots_needed - 1;
		*out_packet_length_type_field = 1; /* Slot mode */
	} else {
		*out_packet_length_field = num_subslots_needed - 1;
		*out_packet_length_type_field = 0; /* Subslot mode */
	}

	LOG_DBG("PCC_CALC: Payload %zuB, mu_c %u, beta_c %u, MCS %u -> %u subslots (PCC len_f 0x%X, type %u)",
		mac_pdc_payload_len_bytes, mu_code, beta_code, selected_mcs, num_subslots_needed,
		*out_packet_length_field, *out_packet_length_type_field);
}

uint16_t dect_mac_phy_ctrl_get_transmitter_id(const union nrf_modem_dect_phy_hdr *hdr, uint8_t phy_type)
{
	if (phy_type == 0) { /* Type 1 / Beacon */
		return (uint16_t)((hdr->hdr_type_1.transmitter_id_hi << 8) | hdr->hdr_type_1.transmitter_id_lo);
	} else { /* Type 2 / Unicast */
		return (uint16_t)((hdr->hdr_type_2.transmitter_id_hi << 8) | hdr->hdr_type_2.transmitter_id_lo);
	}
}

uint16_t dect_mac_phy_ctrl_get_receiver_id(const union nrf_modem_dect_phy_hdr *hdr, uint8_t phy_type)
{
	if (phy_type == 0) {
		return 0xFFFF; /* Broadcast/None specifically for Type 1 in our usage */
	} else {
		return (uint16_t)((hdr->hdr_type_2.receiver_id_hi << 8) | hdr->hdr_type_2.receiver_id_lo);
	}
}

uint16_t dect_mac_phy_ctrl_get_packet_length_bytes(const union nrf_modem_dect_phy_hdr *hdr, uint8_t phy_type, uint8_t mu, uint8_t beta)
{
	uint8_t pkt_len_field;
	uint8_t pkt_len_type;
	uint8_t mcs;

	if (phy_type == 0) {
		pkt_len_field = hdr->hdr_type_1.packet_length;
		pkt_len_type = hdr->hdr_type_1.packet_length_type;
		mcs = hdr->hdr_type_1.df_mcs;
	} else {
		pkt_len_field = hdr->hdr_type_2.packet_length;
		pkt_len_type = hdr->hdr_type_2.packet_length_type;
		mcs = hdr->hdr_type_2.df_mcs;
	}

	uint8_t num_subslots = pkt_len_field + 1;

	if (pkt_len_type == 1) {
		num_subslots *= get_subslots_per_etsi_slot_for_mu(mu);
	}

	if (num_subslots == 0 || num_subslots > TBS_MAX_SUB_SLOTS_J) {
		return 0;
	}

	const uint32_t (*selected_tbs_table)[TBS_MAX_SUB_SLOTS_J];

	if (beta == 0) { /* beta = 1 */
		switch (mu) {
		case 0: selected_tbs_table = tbs_single_slot_mu1_beta1; break;
		case 1: selected_tbs_table = tbs_single_slot_mu2_beta1; break;
		case 2: selected_tbs_table = tbs_single_slot_mu4_beta1; break;
		default: return 0;
		}
	} else {
		return 0;
	}

	if (mcs > TBS_MAX_MCS_INDEX) {
		return 0;
	}

	/* TBS tables store bits. Divide by 8 for bytes. */
	return (uint16_t)(selected_tbs_table[mcs][num_subslots - 1] / 8);
}

int dect_mac_phy_ctrl_deactivate(void)
{
    dect_mac_context_t *ctx = dect_mac_get_active_context();

    if (ctx->state < MAC_STATE_IDLE) {
        LOG_WRN("PHY_CTRL_DEACT: Deactivate called but MAC is already deactivated or uninitialized.");
        return 0; // Nothing to do
    }

    LOG_INF("PHY_CTRL_DEACT: Requesting DECT PHY deactivation.");

    // The PHY deactivation is asynchronous. The result will be reported in the
    // NRF_MODEM_DECT_PHY_EVT_DEACTIVATE event, which is currently ignored by the
    // main dispatcher but logged by the PHY interface.
    int ret = nrf_modem_dect_phy_deactivate();
    if (ret != 0) {
        LOG_ERR("PHY_CTRL_DEACT: nrf_modem_dect_phy_deactivate() failed: %d", ret);
    }

    // Even if the call fails, we should clear the pending op state as we are
    // trying to halt operations.
    ctx->pending_op_type = PENDING_OP_NONE;
    ctx->pending_op_handle = 0;

    return ret;
}