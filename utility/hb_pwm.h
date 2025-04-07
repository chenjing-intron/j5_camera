/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef UTILITY_HB_PWM_H_
#define UTILITY_HB_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../inc/cam_common.h"

/*
 * APIs to control multiple lpwms
 * hb_vin_lpwm_config configures all 4 lpwms
 * num should be 4 for lpwm
 * offset_us: should be offset_us[4] array, for offset of each lpwm channel
 * offset_us[i] is in [1us 999999us], step is 1us
 * period_us: should be period_us[4] array, for period of each lpwm channel
 * period_us[i] is in [2us 1000000us], step is 1us, e.g. 30fps is 33333
 * duty_us: should be duty_us[4] array, duty_cycle of each lpwm channel
 * duty_us[i] is in [1us 4000us], step is 1us
 */
extern int32_t hb_vin_lpwm_config(lpwm_info_t *lpwm_info);
/*
 * Use bit map to start/stop multile pwm
 * bit0-3 map to lpwm0-3, eg. 0x3: lpwm0,1, 0x5:lpwm0,2
 */

extern int32_t hb_vin_lpwm_start(const lpwm_info_t *lpwm_info);
extern int32_t hb_vin_lpwm_stop(const lpwm_info_t *lpwm_info);
extern int32_t hb_vin_lpwm_deinit(lpwm_info_t *lpwm_info);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_PWM_H_
