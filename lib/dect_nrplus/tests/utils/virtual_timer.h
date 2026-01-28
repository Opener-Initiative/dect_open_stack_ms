/* tests/utils/virtual_timer.h */
// Overview: Public API for the new, independent simulation clock module.
// --- CREATE NEW FILE ---
#ifndef VIRTUAL_TIMER_H__
#define VIRTUAL_TIMER_H__

#include <stdint.h>

void virtual_timer_init(void);
uint64_t virtual_timer_get_time(void);
void virtual_timer_step(uint64_t step_us);
void virtual_timer_advance_to(uint64_t target_time_us);
void virtual_timer_advance_by(uint64_t duration_us);

#endif /* VIRTUAL_TIMER_H__ */