/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* dect_mac/dect_mac_timeline_utils.c */
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_ZTEST) || IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)
#include <mocks/mock_nrf_modem_dect_phy.h> /* For g_mock_phy_context_override */
#include <mac/nrf_modem_dect_phy.h>
#else
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <nrf_modem_dect_phy.h>
// #include <zms.h>
#endif

#include <mac/dect_mac_timeline_utils.h>

LOG_MODULE_REGISTER(dect_mac_timeline, LOG_LEVEL_INF);

/**
 * @brief Converts a duration in microseconds to modem time ticks.
 *
 * @param us Duration in microseconds.
 * @param tick_rate_khz The modem's tick rate in kHz.
 * @return The equivalent duration in modem time ticks.
 */
uint64_t modem_us_to_ticks(uint64_t us, uint32_t tick_rate_khz)
{
	if (tick_rate_khz == 0) {
		return 0;
	}
	/* Use 64-bit arithmetic to prevent intermediate overflow */
	return (us * tick_rate_khz) / 1000U;
}


/**
 * @brief Converts a duration in modem time ticks to microseconds.
 *
 * @param ticks Duration in modem time ticks.
 * @param tick_rate_khz The modem's tick rate in kHz.
 * @return The equivalent duration in microseconds.
 */
uint64_t modem_ticks_to_us(uint64_t ticks, uint32_t tick_rate_khz)
{
	if (tick_rate_khz == 0) {
		return 0;
	}
	/* Use 64-bit arithmetic to prevent intermediate overflow */
	return (ticks * 1000U) / tick_rate_khz;
}

uint64_t dect_mac_estimate_modem_now(dect_mac_context_t *ctx)
{
	if (!ctx || ctx->last_event_system_uptime_ticks == 0) {
		return ctx ? ctx->last_known_modem_time : 0;
	}

	uint64_t now_ticks = k_uptime_ticks();
	uint64_t elapsed_system_ticks = 0;

	if (now_ticks >= ctx->last_event_system_uptime_ticks) {
		elapsed_system_ticks = now_ticks - ctx->last_event_system_uptime_ticks;
	} else {
		/* Handle 64-bit rollover (extremely unlikely for uptime, but for robustness) */
		elapsed_system_ticks = (UINT64_MAX - ctx->last_event_system_uptime_ticks) + now_ticks + 1;
	}

	uint64_t elapsed_us = k_ticks_to_us_floor64(elapsed_system_ticks);
	uint64_t elapsed_modem_ticks = modem_us_to_ticks(elapsed_us, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	return ctx->last_known_modem_time + elapsed_modem_ticks;
}


/**
 * @brief Calculates the duration of one subslot in modem time ticks for a given mu_code.
 *
 * A subslot is 5 OFDM symbols. Symbol duration depends on mu.
 * NRF_MODEM_DECT_SYMBOL_DURATION is for mu=1 (code 0).
 * mu_actual = 2^mu_code. Symbol_duration_mu = Symbol_duration_mu1 / (2^(mu_code)).
 *
 * @param mu_code The mu code (0-7).
 * @return Subslot duration in modem ticks, or 0 on error.
 */
uint32_t get_subslot_duration_ticks_for_mu(uint8_t mu_code)
{
    if (mu_code > 7) {
        LOG_ERR("PHY_TIMING: Invalid mu_code %u for subslot duration.", mu_code);
        return 0;
    }

    uint32_t base_symbol_duration_ticks = NRF_MODEM_DECT_SYMBOL_DURATION;
    uint32_t actual_symbol_duration_ticks = base_symbol_duration_ticks;

    if (mu_code > 0) {
        actual_symbol_duration_ticks = base_symbol_duration_ticks / (1U << mu_code);
    }
    
    if (actual_symbol_duration_ticks == 0 && base_symbol_duration_ticks != 0) {
        LOG_ERR("PHY_TIMING: Calculated symbol duration is 0 for mu_code %u.", mu_code);
        return 0;
    }
    return actual_symbol_duration_ticks * 5;
}


/**
 * @brief Gets the number of subslots per ETSI slot for a given mu_code.
 *
 * @param mu_code The mu code (0-3).
 * @return Number of subslots per ETSI slot.
 */
uint8_t get_subslots_per_etsi_slot_for_mu(uint8_t mu_code)
{
    if (mu_code > 3) {
        LOG_WRN("PHY_TIMING: mu_code %u > 3, N_slot_subslot may not be standard. Defaulting for mu=8.", mu_code);
        return 16;
    }
    return 2 * (1U << mu_code);
}


uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time,
                                            uint8_t sfn_of_anchor_relevance, uint8_t target_sfn_val,
                                            uint16_t target_subslot_idx,
                                            uint8_t link_mu_code, uint8_t link_beta_code)
{
    ARG_UNUSED(link_beta_code); // Beta currently not used in subslot/frame duration calculations here

    if (!ctx) {
        LOG_ERR("CALC_TIME: NULL MAC context provided!");
        return UINT64_MAX; 
    }

    if (sfn_zero_anchor_time == 0 && ctx->last_known_modem_time == 0) {
        LOG_WRN("CALC_TIME: SFN Zero Anchor and last_known_modem_time are both 0. Cannot calculate. Returning large future estimate.");
        return modem_us_to_ticks(500000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
    }
    if (sfn_zero_anchor_time == 0) {
        LOG_WRN("CALC_TIME: SFN Zero Anchor is 0. Using last_known_modem_time + fallback delay.");
        uint32_t fallback_delay_ticks = modem_us_to_ticks(FRAME_DURATION_MS_NOMINAL * 1000 * 2, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
        return ctx->last_known_modem_time + fallback_delay_ticks;
    }

    uint32_t frame_duration_ticks_val = FRAME_DURATION_TICKS;
    if (frame_duration_ticks_val == 0) {
        LOG_ERR("CALC_TIME: Calculated frame_duration_ticks is 0! Modem tick rate likely 0.");
        return UINT64_MAX;
    }

    // Use the provided link_mu_code for subslot duration calculation
    uint8_t mu_code_for_calc = link_mu_code;
    if (mu_code_for_calc > 7) { // Max mu_code for 2^7=128, typical DECT NR+ 0-3
        LOG_WRN("CALC_TIME: Invalid link_mu_code %u provided, defaulting to 0 (mu=1).", link_mu_code);
        mu_code_for_calc = 0;
    }

    uint32_t subslot_duration_ticks_val = get_subslot_duration_ticks_for_mu(mu_code_for_calc);
    if (subslot_duration_ticks_val == 0) {
        LOG_ERR("CALC_TIME: Calculated subslot_duration_ticks is 0 for mu_code %u!", mu_code_for_calc);
        return UINT64_MAX;
    }

    uint64_t anchor_relevance_frame_start_time = sfn_zero_anchor_time +
                                                 ((uint64_t)sfn_of_anchor_relevance * frame_duration_ticks_val);

    /* Determine how many full 256-frame epochs have passed since the anchor relevance time
     * to ensure we target a frame close to 'now'.
     */
    uint64_t now = ctx->last_known_modem_time;
    uint64_t epoch_duration_ticks = (uint64_t)256 * frame_duration_ticks_val;
    uint64_t cycles_since_relevance = 0;

    if (now > anchor_relevance_frame_start_time) {
        cycles_since_relevance = (now - anchor_relevance_frame_start_time) / epoch_duration_ticks;
    }

    uint64_t epoch_start_time = anchor_relevance_frame_start_time + (cycles_since_relevance * epoch_duration_ticks);

    /* SFN diff within an epoch (0-255) */
    int16_t sfn_diff = (int16_t)target_sfn_val - (int16_t)sfn_of_anchor_relevance;
    if (sfn_diff < 0) {
        sfn_diff += 256;
    }

    uint64_t target_frame_start_time = epoch_start_time + ((int64_t)sfn_diff * frame_duration_ticks_val);

    /* Ensure the result is in the future (at least 500us cushion for PHY processing) */
    uint32_t safety_margin_ticks = modem_us_to_ticks(500, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
    while (target_frame_start_time + safety_margin_ticks < now) {
        target_frame_start_time += epoch_duration_ticks;
    }

    uint64_t target_subslot_offset_in_frame_ticks = (uint64_t)target_subslot_idx * subslot_duration_ticks_val;
    uint64_t final_target_time = target_frame_start_time + target_subslot_offset_in_frame_ticks;

    LOG_DBG("CALC_TIME: AnchorSFN %u, TargetSFN %u, EpochCycles %llu, TargetStart %llu (Now %llu)",
            sfn_of_anchor_relevance, target_sfn_val, cycles_since_relevance, final_target_time, now);

    return final_target_time;
}


void update_next_occurrence(dect_mac_context_t *ctx, dect_mac_schedule_t *schedule,
			    uint64_t current_modem_time, uint8_t link_mu_code)
{
	if (!schedule || !schedule->is_active) {
		printk("[TIME_UTILs] update_next_occurrence: (!schedule || !schedule->is_active)  \n");
		return;
	}

	uint32_t subslot_duration_ticks = get_subslot_duration_ticks_for_mu(link_mu_code);
	uint32_t frame_duration_ticks = FRAME_DURATION_TICKS;
	uint64_t repetition_period_ticks = 0;
	uint32_t scheduled_duration_subslots = 0;
	uint16_t current_schedule_start_subslot = 0;

	if (schedule->alloc_type == RES_ALLOC_TYPE_DOWNLINK ||
	    (schedule->alloc_type == RES_ALLOC_TYPE_BIDIR && ctx->role == MAC_ROLE_FT)) {
		scheduled_duration_subslots = schedule->dl_duration_subslots;
		current_schedule_start_subslot = schedule->dl_start_subslot;
	} else if (schedule->alloc_type == RES_ALLOC_TYPE_UPLINK ||
		   (schedule->alloc_type == RES_ALLOC_TYPE_BIDIR && ctx->role == MAC_ROLE_PT)) {
		scheduled_duration_subslots = schedule->ul_duration_subslots;
		current_schedule_start_subslot = schedule->ul_start_subslot;
	} else {
		LOG_ERR("SCHED_UPD: Invalid alloc_type %d in schedule for Ch %u, SS %u. Deactivating.",
			schedule->alloc_type, schedule->channel, current_schedule_start_subslot);
		schedule->is_active = false;
		return;
	}
	if (scheduled_duration_subslots == 0 && schedule->repeat_type != RES_ALLOC_REPEAT_SINGLE) {
		LOG_ERR("SCHED_UPD: Zero duration for repeating schedule (Ch %u, SS %u). Deactivating.",
			schedule->channel, current_schedule_start_subslot);
		schedule->is_active = false;
		return;
	}

	if (schedule->repeat_type == RES_ALLOC_REPEAT_SINGLE) {
		uint64_t single_occurrence_end_time =
			schedule->next_occurrence_modem_time +
			((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);
		if (single_occurrence_end_time < current_modem_time &&
		    schedule->next_occurrence_modem_time != 0) {
			LOG_DBG("SCHED_UPD: Single occurrence schedule (Ch %u, SS %u @ %llu) has passed. Deactivating.",
				schedule->channel, current_schedule_start_subslot,
				schedule->next_occurrence_modem_time);
			schedule->is_active = false;
		}
		return;
	}

	if (schedule->repetition_value == 0) {
		LOG_ERR("SCHED_UPD: Repetition value 0 is undefined for schedule (Ch %u, SS %u). Deactivating.",
			schedule->channel, current_schedule_start_subslot);
		schedule->is_active = false;
		return;
	}

	if (schedule->repeat_type == RES_ALLOC_REPEAT_FRAMES ||
	    schedule->repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP) {
		repetition_period_ticks = (uint64_t)schedule->repetition_value * frame_duration_ticks;
	} else if (schedule->repeat_type == RES_ALLOC_REPEAT_SUBSLOTS ||
		   schedule->repeat_type == RES_ALLOC_REPEAT_SUBSLOTS_GROUP) {
		repetition_period_ticks =
			(uint64_t)schedule->repetition_value * subslot_duration_ticks;
	} else {
		LOG_ERR("SCHED_UPD: Unknown repeat type %d for schedule (Ch %u, SS %u). Deactivating.",
			schedule->repeat_type, schedule->channel, current_schedule_start_subslot);
		schedule->is_active = false;
		return;
	}

	if (repetition_period_ticks == 0) {
		schedule->is_active = false;
		return;
	}

	uint64_t current_slot_end_time_for_update =
		schedule->next_occurrence_modem_time +
		((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);

	if (schedule->next_occurrence_modem_time == 0 && schedule->schedule_init_modem_time != 0) {
		LOG_WRN("SCHED_UPD: next_occurrence_modem_time is 0 for active repeating schedule. Init needed.");
		schedule->next_occurrence_modem_time = current_modem_time;
		current_slot_end_time_for_update = current_modem_time;
	}

	while (schedule->next_occurrence_modem_time < current_modem_time ||
	       current_slot_end_time_for_update <= current_modem_time) {
		schedule->next_occurrence_modem_time += repetition_period_ticks;
		current_slot_end_time_for_update =
			schedule->next_occurrence_modem_time +
			((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);
	}

	if (schedule->validity_value != 0xFF && schedule->sfn_of_initial_occurrence != 0xFF &&
	    ctx->ft_sfn0_modem_time_anchor != 0) {
		/* Current time in frames relative to sfn0 anchor */
		uint64_t ticks_from_sfn0_anchor;
		if (schedule->next_occurrence_modem_time < ctx->ft_sfn0_modem_time_anchor) {
			/* Handle potential 64-bit rollover (modem ticks) */
			ticks_from_sfn0_anchor =
				(UINT64_MAX - ctx->ft_sfn0_modem_time_anchor) +
				schedule->next_occurrence_modem_time + 1;
		} else {
			ticks_from_sfn0_anchor = schedule->next_occurrence_modem_time - ctx->ft_sfn0_modem_time_anchor;
		}

		uint64_t frames_from_sfn0_anchor_now =
			ticks_from_sfn0_anchor / frame_duration_ticks;
		uint8_t current_sfn = (uint8_t)(frames_from_sfn0_anchor_now % 256);

		printk("[DEBUG_PROBE] frames_from_sfn0_anchor_now:%u ticks_from_sfn0_anchor:%u frame_duration_ticks:%u current_sfn:%u \n", (unsigned int)frames_from_sfn0_anchor_now, (unsigned int)ticks_from_sfn0_anchor, frame_duration_ticks, current_sfn);

		int16_t frames_elapsed_since_initial =
			(int16_t)current_sfn - (int16_t)schedule->sfn_of_initial_occurrence;

		printk("[DEBUG_PROBE] Schedule for Ch %u Init SFN %u, Current SFN %u, Validity %u frames, Elapsed ~%d, Initial Occurrence %d\n",
			schedule->channel, schedule->sfn_of_initial_occurrence, current_sfn,
			schedule->validity_value, frames_elapsed_since_initial, (int16_t)schedule->sfn_of_initial_occurrence);

		// if (frames_elapsed_since_initial < 0) {
		// 	frames_elapsed_since_initial += 256;
		// }

		// if ((uint8_t)frames_elapsed_since_initial >= schedule->validity_value) {
		// 	// printk("[DEBUG_PROBE] Schedule for Ch %u is being DEACTIVATED due to validity expiry. Init SFN %u, Current SFN %u, Validity %u frames, Elapsed ~%d\n",
		// 	// 	schedule->channel, schedule->sfn_of_initial_occurrence, current_sfn,
		// 	// 	schedule->validity_value, frames_elapsed_since_initial);
		// 	LOG_INF("SCHED_UPD: Schedule (Ch %u, SS %u) deactivated: validity expired. Init SFN %u, Current SFN %u, Validity %u frames, Elapsed ~%d",
		// 		schedule->channel, current_schedule_start_subslot,
		// 		schedule->sfn_of_initial_occurrence, current_sfn,
		// 		schedule->validity_value, frames_elapsed_since_initial);
		// 	schedule->is_active = false;
		// 	return;
		// }
#if 0
		/* Only perform validity check if the schedule has actually started */
		if (frames_elapsed_since_initial >= 0) {
			if ((uint8_t)frames_elapsed_since_initial >= schedule->validity_value) {
				LOG_INF("SCHED_UPD: Schedule (Ch %u, SS %u) deactivated: validity expired. Init SFN %u, Current SFN %u, Validity %u frames, Elapsed ~%d",
					schedule->channel, current_schedule_start_subslot,
					schedule->sfn_of_initial_occurrence, current_sfn,
					schedule->validity_value,
					frames_elapsed_since_initial);
				schedule->is_active = false;
				return;
			}
		}
#endif		
	}
	LOG_DBG("SCHED_UPD: Updated schedule (Ch %u, SS %u): next occurrence at %llu",
		schedule->channel, current_schedule_start_subslot,
		schedule->next_occurrence_modem_time);
}

uint8_t dect_mac_get_current_sfn(dect_mac_context_t *ctx)
{
	if (ctx->ft_sfn0_modem_time_anchor == 0) {
		return 0;
	}

	uint64_t now = ctx->last_known_modem_time;
	uint64_t ticks_from_anchor;

	if (now < ctx->ft_sfn0_modem_time_anchor) {
		/* Rollover case */
		ticks_from_anchor = (UINT64_MAX - ctx->ft_sfn0_modem_time_anchor) + now + 1;
	} else {
		ticks_from_anchor = now - ctx->ft_sfn0_modem_time_anchor;
	}

	uint32_t frame_duration_ticks = FRAME_DURATION_TICKS;
	uint64_t frames_since_anchor = ticks_from_anchor / frame_duration_ticks;
	
	/* SFN is 8-bit, wraps every 256 frames */
	return (uint8_t)((ctx->current_sfn_at_anchor_update + frames_since_anchor) % 256);
}

void dect_mac_timeline_sync_sfn(dect_mac_context_t *ctx)
{
	uint8_t current_sfn = dect_mac_get_current_sfn(ctx);

	if (ctx->role == MAC_ROLE_FT) {
		ctx->role_ctx.ft.sfn = current_sfn;
	} else if (ctx->role == MAC_ROLE_PT) {
		ctx->role_ctx.pt.current_ft_sfn = current_sfn;
	}
}
