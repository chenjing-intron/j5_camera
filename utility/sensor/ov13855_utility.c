/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ov13855]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "hb_cam_utility.h"
#include "inc/ov13855_setting.h"

int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(ov13855_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, ov13855_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(ov13855_stream_on_setting)/sizeof(uint32_t)/2;
	vin_dbg("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, ov13855_stream_on_setting);
	if(ret < 0) {
		vin_dbg("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(ov13855_stream_off_setting)/sizeof(uint32_t)/2;
	printf("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
		setting_size, ov13855_stream_off_setting);
	if(ret < 0) {
		vin_dbg("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

sensor_module_t ov13855 = {
	.module = "ov13855",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
