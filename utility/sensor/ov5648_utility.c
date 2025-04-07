/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ov5648]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "../hb_cam_utility.h"
#include "inc/ov5648_setting.h"

#define MCLK (24000000)
static int power_ref;
int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (power_ref > 0) {
		power_ref++;
		return ret;
	}
	if(sensor_info->power_mode) {
		/*extra_mode: xj3som-xj3board:0  j3som-x3board:1*/
		ret = hb_vin_set_mclk(sensor_info->extra_mode, MCLK);
		if (ret < 0) {
			vin_err("%d : set clock %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
		ret = hb_vin_enable_mclk(sensor_info->extra_mode);
		if (ret < 0) {
			vin_err("%d : enable clock %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
										1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(5*1000);
			}
		}
	}
	power_ref++;
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
				__LINE__, sensor_info->sensor_name);
		return ret;
	}

	if(sensor_info->config_index == 0) {
		setting_size =
				sizeof(ov5648_1lane_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_1lane_init_setting);
		if (ret < 0) {
			vin_dbg("%d : init %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->config_index == 1) {
		setting_size =
				sizeof(ov5648_2lane_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_2lane_init_setting);
		if (ret < 0) {
			vin_dbg("%d : init %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->config_index == 2) {
		setting_size =
				sizeof(ov5648_1lane_1080p_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_1lane_1080p_init_setting);
		if (ret < 0) {
			vin_dbg("%d : init %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->config_index == 3) {
		setting_size =
				sizeof(ov5648_2lane_1080p_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_2lane_1080p_init_setting);
		if (ret < 0) {
			vin_dbg("%d : init %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->config_index == 0 || sensor_info->config_index == 2) {
		setting_size =
				sizeof(ov5648_1lane_stream_on_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_1lane_stream_on_setting);
		if(ret < 0) {
			vin_dbg("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->config_index == 1 || sensor_info->config_index == 3) {
		setting_size =
				sizeof(ov5648_2lane_stream_on_setting)/sizeof(uint32_t)/2;
		vin_dbg("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_2lane_stream_on_setting);
		if(ret < 0) {
			vin_dbg("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->config_index == 0 || sensor_info->config_index == 2) {
		setting_size =
				sizeof(ov5648_1lane_stream_off_setting)/sizeof(uint32_t)/2;
		printf("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_1lane_stream_off_setting);
		if(ret < 0) {
			vin_dbg("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->config_index == 1 || sensor_info->config_index == 3) {
		setting_size =
				sizeof(ov5648_2lane_stream_off_setting)/sizeof(uint32_t)/2;
		printf("sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov5648_2lane_stream_off_setting);
		if(ret < 0) {
			vin_dbg("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if(sensor_info->power_mode) {
		power_ref--;
		if (power_ref == 0) {
			ret = hb_vin_disable_mclk(sensor_info->extra_mode);
			if (ret < 0) {
				vin_err("%d : disable clock %s fail\n",
						__LINE__, sensor_info->sensor_name);
			}
		}
	}
	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0) {
		vin_err("%d : deinit %s fail\n",
				__LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

sensor_module_t ov5648 = {
	.module = "ov5648",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
