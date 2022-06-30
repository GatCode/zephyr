/*
 * NOTE: parts of the code below and the corresponding source file
 * are copied from the Nordic nrf5340_audio sample (sdk-nrf on GitHub)
 */

#ifndef _SYNC_TIMER_H_
#define _SYNC_TIMER_H_

#include <stdint.h>

/**
 * @brief Get the currrent timer value
 *
 * @return Captured value
 */
uint32_t audio_sync_timer_curr_time_get(void);

#endif /* _SYNC_TIMER_H_ */
