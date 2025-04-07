/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[v1080pRAW12_1936x1100]:" fmt

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
#include "inc/ds953_setting.h"
#include "inc/ds954_setting.h"

int sensor_v1080pRAW12_1936x1100_954_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	/*======954init======*/
	ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
			setting_size, ds954_mono_init_setting);
	if (ret < 0) {
		vin_err("write ds954_mono_init_setting error\n", sensor_info->serial_addr);
		return ret;
	}
	usleep(100*1000);
	setting_size = sizeof(ds954_mono_init_setting)/sizeof(uint32_t)/2 - 1;
	vin_dbg("deserial_if->bus_num= %d, deserial_if->deserial_addr = 0x%x, setting_size = %d, \n", deserial_if->bus_num, deserial_if->deserial_addr, setting_size);
	ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
			setting_size, &(ds954_mono_init_setting[2]));
	if (ret < 0) {
		vin_err("write ds954_mono_init_setting error\n", sensor_info->serial_addr);
		return ret;
	}
	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				vin_dbg("gpio_num %d  %d %d %d \n", sensor_info->gpio_num, sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio], 1-sensor_info->gpio_level[gpio]);
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
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
	int setting_size = 0;
	/*======power on======*/
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	/*======954init======*/
	ret = sensor_v1080pRAW12_1936x1100_954_init(sensor_info);
	if (ret < 0) {
		vin_err("v1080pRAW12_1936x1100_954_init fail\n");
		return ret;
	}
	/*======953init======*/
	setting_size = sizeof(ds953_imx390_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, ds953_imx390_init_setting);
	if (ret < 0) {
			vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
	}
	vin_dbg("v1080pRAW12_1936x1100 ub953 init done\n");
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	ret = ((deserial_module_t *)(deserial_if->deserial_ops))->stream_on(
								deserial_if, sensor_info->deserial_port);
	if (ret < 0) {
		  vin_err("v1080pRAW12_1936x1100 954 start fail\n");
		  return -HB_CAM_SERDES_STREAM_ON_FAIL;
	}
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

sensor_module_t v1080pRAW12_1936x1100 = {
	.module = "v1080pRAW12_1936x1100",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};


