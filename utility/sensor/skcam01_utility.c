/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[skcam01]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "hb_cam_utility.h"


int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				vin_dbg("gpio_num %d  %d %d %d \n", sensor_info->gpio_num, sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio], 1-sensor_info->gpio_level[gpio]);
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay*1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio], 1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}
int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}
int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

sensor_module_t skcam01 = {
	.module = "skcam01",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};


