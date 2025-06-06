/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[dual_ix019]:" fmt

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
#include "inc/ds933_setting.h"
#include "inc/ds954_setting.h"

int sensor_dual_ix019_954_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	/*======954init======*/
	setting_size = sizeof(ds954_dual_ix019_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("deserial_if->bus_num= %d, deserial_if->deserial_addr = 0x%x, setting_size = %d, \n", deserial_if->bus_num, deserial_if->deserial_addr, setting_size);
	ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1, setting_size, ds954_dual_ix019_init_setting);
	if (ret < 0) {
		vin_err("write ds954_imx390_init_setting error\n", sensor_info->serial_addr);
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
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	/*======sensor_poweron======*/
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	/*======954init======*/
	ret = sensor_dual_ix019_954_init(sensor_info);
	if (ret < 0) {
		vin_err("sensor_dual_ix019_954_init fail\n");
		return ret;
	}
	/*======933init======*/
	setting_size = sizeof(ds933_ix019_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, ds933_ix019_init_setting);
	if (ret < 0) {
		vin_err("ub933 write 0x%x error\n", sensor_info->serial_addr);
		return ret;
	}
	usleep(10*1000);
	setting_size = sizeof(ds933_ov10635_ix019_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr1, 1, setting_size, ds933_ov10635_ix019_init_setting);
	if (ret < 0) {
		vin_err("ub933 write 0x%x error\n", sensor_info->serial_addr1);
	}
	return ret;
}
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	ret = ((deserial_module_t *)(deserial_if->deserial_ops))->stream_on(deserial_if, sensor_info->deserial_port);
	if (ret < 0) {
	 	 vin_err("dual_ix019 954 stream_on fail\n");
	  	 return -HB_CAM_SERDES_STREAM_ON_FAIL;
	}
	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	deserial_info_t *deserial_info = NULL;

	deserial_info = (deserial_info_t *)sensor_info->deserial_info;
	ret = ((deserial_module_t *)(deserial_info->deserial_ops))->stream_off(deserial_info, sensor_info->deserial_port);
	if (ret < 0) {
		 vin_err("dual_ix019 s954 stream_off fail\n");
		 return -HB_CAM_SERDES_STREAM_OFF_FAIL;
	}

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

sensor_module_t dual_ix019 = {
	.module = "dual_ix019",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};


