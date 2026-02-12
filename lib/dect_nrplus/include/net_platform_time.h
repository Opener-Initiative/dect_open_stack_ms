/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file net_platform_time.h
 * @brief Platform abstraction for timing operations in DECT NR+ library.
 *
 * This file separates timing into two domains:
 * 1. Macro Timing (System Tick): Power-efficient, used for protocol logic.
 * 2. Micro Timing (Cycle Counter): High-precision, used for HW signaling and Mocks.
 */

#ifndef NET_PLATFORM_TIME_H__
#define NET_PLATFORM_TIME_H__

#include <zephyr/kernel.h> /**< dependency on Zephyr kernel */

#ifdef __cplusplus
extern "C" {
#endif

/* Use int64_t to prevent rollover issues */
typedef int64_t net_tick_t;

/* ============================================================
 * MACRO TIMING (32 kHz Domain) - Use for Protocol Logic
 * ============================================================ */

/**
 * @brief Get current OS tick count with 30.5us resolution.
 *
 * Uses the low-power RTC. Safe to call from any context.
 *
 * @return Current tick count.
 */
static inline net_tick_t net_get_os_tick(void) {
    return k_uptime_ticks();
}

/**
 * @brief Convert milliseconds to ticks (ceiling).
 *
 * @param ms Time in milliseconds.
 * @return Corresponding ticks.
 */
static inline net_tick_t net_ms_to_ticks(uint32_t ms) {
    return k_ms_to_ticks_ceil32(ms);
}

/**
 * @brief Convert microseconds to ticks (ceiling).
 *
 * WARNING: Input < 31us will result in 1 tick (jittery).
 * Use only for durations > 100us.
 *
 * @param us Time in microseconds.
 * @return Corresponding ticks.
 */
static inline net_tick_t net_us_to_ticks(uint32_t us) {
    return k_us_to_ticks_ceil32(us);
}

/**
 * @brief Check if a duration has elapsed since a start tick.
 *
 * Handles rollover safely using subtraction.
 *
 * @param start_tick The tick count at the start of the period.
 * @param duration_ticks The duration to check against.
 * @return true if elapsed, false otherwise.
 */
static inline bool net_is_timeout(net_tick_t start_tick, net_tick_t duration_ticks) {
    return (k_uptime_ticks() - start_tick) >= duration_ticks;
}

/* ============================================================
 * MICRO TIMING (64 MHz Domain) - Use for Hardware Signaling
 * ============================================================ */

/**
 * @brief High precision busy-wait delay.
 *
 * Blocks the CPU. Uses the 64 MHz system timer for nanosecond accuracy.
 * Power intensive - do not use for long waits (> 100us).
 *
 * @param us Time to wait in microseconds.
 */
static inline void net_delay_us_high_precision(uint32_t us) {
    k_busy_wait(us);
}

#ifdef __cplusplus
}
#endif

#endif /* NET_PLATFORM_TIME_H__ */
