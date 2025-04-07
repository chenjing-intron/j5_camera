/*
 *    COPYRIGHT NOTICE
 *   Copyright 2019 Horizon Robotics, Inc.
 *    All rights reserved.
 */

#ifndef UTILITY_SENSOR_INC_DS933_SETTING_H_
#define UTILITY_SENSOR_INC_DS933_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t ds933_ar0143_init_setting[] = {
	0x1, 0x33,  // delay 500 us
	0x3, 0xc5,  // delay 10us
	0xd, 0x59   // delay 10us
};

static uint32_t ds933_ix019_init_setting[] = {
	0x1, 0x33,  // delay 500 us
	0x3, 0xc5,  // delay 10us
	0xd, 0x99   // delay 10us   //config local to raise high to GPIO1
};

static uint32_t ds933_ov10635_ix019_init_setting[] = {
	0x1, 0x33,  // delay 500 us
	0x3, 0xc5,  // delay 10us
	0xd, 0x59   // delay 10us   //config GPIO1 to be controlled by remote
};

static uint32_t ds933_ar0144_init_setting[] = {
	0x1, 0x33,  // delay 500 us
	0x3, 0xc5,  // delay 10us
	0xd, 0x55   // delay 10us
};

static uint32_t ds933_ar0144AT_init_setting[] = {
    0x1, 0x03,  // delay 500 us
    0x3, 0xc5,  // delay 10us
    0xd, 0x99,  // delay 10us
    0xe, 0x19
};
#ifdef __cplusplus
}
#endif
#endif // UTILITY_SENSOR_INC_DS933_SETTING_H_
