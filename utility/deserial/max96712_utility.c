/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[max96712]:[%s][%d]" fmt, __func__, __LINE__

#include "hb_cam_utility.h"
#define INIT_STATE 1
#define DEINIT_STATE 2
#define DES_RESET_DELAY 2000

// PRQA S 3206++

static int32_t deserializer_deinit(deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}
static int32_t deserializer_init(deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

static int32_t deserializer_stream_on(deserial_info_t *max96712_info, uint32_t port)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

static int32_t deserializer_stream_off(deserial_info_t *max96712_info, uint32_t port)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

static int32_t deserializer_start_physical(const deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

static int32_t deserializer_reset(const deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	if(max96712_info->power_mode == 1u) {
		for(gpio = 0; gpio < max96712_info->gpio_num; gpio++) {
			if(max96712_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)max96712_info->gpio_pin[gpio],
							max96712_info->gpio_level[gpio]);
			}
		}
		(void)usleep(DES_RESET_DELAY);
		for(gpio = 0; gpio < max96712_info->gpio_num; gpio++) {
			if(max96712_info->gpio_pin[gpio] >= 0) {
				ret =(int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)max96712_info->gpio_pin[gpio],
							1-max96712_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -RET_ERROR;
				}
			}
		}
	}
	return ret;
}

deserial_module_t max96712 = {
	.module = "max96712",
	.init = deserializer_init,
	.stream_on = deserializer_stream_on,
	.stream_off = deserializer_stream_off,
	.start_physical = deserializer_start_physical,
	.deinit = deserializer_deinit,
	.reset = deserializer_reset,
};

// PRQA S --

