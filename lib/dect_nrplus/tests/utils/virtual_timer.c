/* tests/utils/virtual_timer.c */
// Overview: Implementation of the new, independent simulation clock module.
#include <stdint.h>
#include "virtual_timer.h"


static uint64_t g_current_time_us = 0;

void virtual_timer_init(void)
{
	g_current_time_us = 0;
}

uint64_t virtual_timer_get_time(void)
{
	return g_current_time_us;
}

void virtual_timer_step(uint64_t step_us)
{
	g_current_time_us += step_us;
}

void virtual_timer_advance_to(uint64_t target_time_us)
{
	if (target_time_us > g_current_time_us) {
		g_current_time_us = target_time_us;
	}
}

void virtual_timer_advance_by(uint64_t duration_us)
{
	g_current_time_us += duration_us;
}