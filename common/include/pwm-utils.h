#ifndef __PCM_UTILS_H__
#define __PCM_UTILS_H__

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);
uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, int d);

#ifdef __cplusplus
}
#endif
#endif // __PCM_UTILS_H__