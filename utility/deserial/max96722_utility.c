/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <ctype.h>

#include "cJSON.h"
#include "hb_cam_utility.h"
#include "hb_i2c.h"
#define INIT_STATE 1
#define DEINIT_STATE 2

int32_t deserializer_deinit(deserial_info_t *max96722_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}
int32_t deserializer_init(deserial_info_t *max96722_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

int32_t deserializer_stream_on(deserial_info_t *max96722_info, uint32_t port)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

int32_t deserializer_stream_off(deserial_info_t *max96722_info, uint32_t port)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

int32_t deserializer_start_physical(const deserial_info_t *max96722_info)
{
	int32_t ret = RET_OK;
	// to do
	return ret;
}

int32_t deserializer_reset(const deserial_info_t *max96722_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	if(max96722_info->power_mode) {
		for(gpio = 0; gpio < max96722_info->gpio_num; gpio++) {
			if(max96722_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)max96722_info->gpio_pin[gpio],
							max96722_info->gpio_level[gpio]);
			}
		}
		usleep(20*1000);
		for(gpio = 0; gpio < max96722_info->gpio_num; gpio++) {
			if(max96722_info->gpio_pin[gpio] >= 0) {
				ret =(int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)max96722_info->gpio_pin[gpio],
							1-max96722_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -RET_ERROR;
				}
			}
		}
	}
	return ret;
}

deserial_module_t max96722 = {
	.module = "max96722",
	.init = deserializer_init,
	.stream_on = deserializer_stream_on,
	.stream_off = deserializer_stream_off,
	.start_physical = deserializer_start_physical,
	.deinit = deserializer_deinit,
	.reset = deserializer_reset,
};

