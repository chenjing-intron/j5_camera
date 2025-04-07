/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0230]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "hb_cam_utility.h"
#include "inc/ar0230_setting.h"
#include "hb_i2c.h"

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio], 1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(5*1000);
			}
		}
	}
	return ret;
}
static int sensor_ar0230_bypass_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(onsemi0230_bypass_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
		setting_size, onsemi0230_bypass_on_setting);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}
static int sensor_ar0230_bypass_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	setting_size = sizeof(onsemi0230_bypass_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
		setting_size, onsemi0230_bypass_off_setting);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

static int sensor_xc9080_1080p_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int fps = sensor_info->fps;
	int setting_size = 0, i;

	switch (fps) {
		case AR0230_5FPS:
		case AR0230_15FPS:
		case AR0230_25FPS:
			setting_size = sizeof(xc9080_0230_init_setting)/sizeof(uint32_t)/2;
			vin_info("xc9080_1080p setting_size = %d\n", setting_size);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->isp_addr,
						xc9080_0230_init_setting[i*2], xc9080_0230_init_setting[i*2 + 1]);
				if(i == 74) {
					usleep(30000);
				}
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	return ret;
}
static int sensor_xc9080_720p_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int fps = sensor_info->fps;
	int setting_size = 0, i;

	switch (fps) {
		case AR0230_5FPS:
		case AR0230_15FPS:
		case AR0230_25FPS:
			setting_size = sizeof(xc9080_0230_720p_init_setting)/sizeof(uint32_t)/2;
			vin_info("xc9080_720p setting_size = %d\n", setting_size);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->isp_addr,
				xc9080_0230_720p_init_setting[i*2], xc9080_0230_720p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	return ret;
}

static int sensor_ar0230_1080p_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case AR0230_5FPS:
			setting_size = sizeof(onsemi0230_5fps_1080p_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_5fps_1080p_init_setting[i*2],
						onsemi0230_5fps_1080p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case AR0230_15FPS:
			setting_size = sizeof(onsemi0230_15fps_1080p_init_setting)/
								sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_15fps_1080p_init_setting[i*2],
						onsemi0230_15fps_1080p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case AR0230_25FPS:
			setting_size = sizeof(onsemi0230_25fps_1080p_init_setting)/
								sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
						onsemi0230_25fps_1080p_init_setting[i*2],
							onsemi0230_25fps_1080p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("xchip9080_AR0230_1080P_config fps %d x2dev OK!\n\n", fps);
	return ret;
}

static int sensor_ar0230_720p_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case AR0230_5FPS:
			setting_size = sizeof(onsemi0230_5fps_720p_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_5fps_720p_init_setting[i*2],
						onsemi0230_5fps_720p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case AR0230_15FPS:
			setting_size = sizeof(onsemi0230_15fps_720p_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_15fps_720p_init_setting[i*2],
							onsemi0230_15fps_720p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case AR0230_25FPS:
			setting_size = sizeof(onsemi0230_25fps_720p_init_setting)/
							sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
					onsemi0230_25fps_720p_init_setting[i*2],
					onsemi0230_25fps_720p_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("xchip9080_AR0230_config_720P fps %d x2dev OK!\n", fps);
	return ret;
}

static int sensor_ar0230_96board_1080p_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case AR0230_25FPS:
			setting_size = sizeof(onsemi0230_96board_init_setting)/sizeof(uint16_t)/2;
			vin_info("x2dev setting_size %d\n", setting_size);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_96board_init_setting[i*2],
						onsemi0230_96board_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			vin_info("xchip9080_AR0230_config fps %d 96board OK!\n", fps);
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	if((sensor_info->fps == AR0230_5FPS || sensor_info->fps == AR0230_15FPS ||
					sensor_info->fps == AR0230_25FPS)
			&& sensor_info->resolution == 1080 ) {
		ret = sensor_xc9080_1080p_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_xc9080_1080p_init fps %d fail\n",
					__LINE__, sensor_info->fps);
			return ret;
		}
	} else if ((sensor_info->fps == AR0230_5FPS || sensor_info->fps == AR0230_15FPS
					|| sensor_info->fps == AR0230_25FPS)
				&& sensor_info->resolution == 720) {
		ret = sensor_xc9080_720p_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_xc9080_720p_init fps %d fail\n",
						__LINE__, sensor_info->fps);
			return ret;
		}
	}
	ret = sensor_ar0230_bypass_on(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_0230_bypss_on %s fail\n", __LINE__,
					sensor_info->sensor_name);
		return ret;
	}
	if(sensor_info->extra_mode == 0) {/*x2dev config*/
		if((sensor_info->fps == AR0230_5FPS ||
			sensor_info->fps == AR0230_15FPS ||
			sensor_info->fps == AR0230_25FPS) &&
				sensor_info->resolution == 1080) {
			ret = sensor_ar0230_1080p_init(sensor_info);
			if(ret < 0) {
				vin_err("%d sensor_ar0230_1080p_init error\n", __LINE__);
			}
		} else if ((sensor_info->fps == AR0230_5FPS || sensor_info->fps == AR0230_15FPS ||
					sensor_info->fps == AR0230_25FPS) && sensor_info->resolution == 720) {
			ret = sensor_ar0230_720p_init(sensor_info);
			if(ret < 0) {
				vin_err("%d sensor_ar0230_720p_init error\n", __LINE__);
			}
		}
	}
	if(sensor_info->extra_mode == 1) { /*96board config*/
		if(sensor_info->fps == AR0230_25FPS && sensor_info->resolution == 1080) {
			ret = sensor_ar0230_96board_1080p_init(sensor_info);
			if(ret < 0) {
				vin_err("%d sensor_ar0230_96board_1080p_init error\n", __LINE__);
			}
		}
	}
	ret = sensor_ar0230_bypass_off(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_ar0230_bypss_off  %s fail\n", __LINE__,
					sensor_info->sensor_name);
	}
	return ret;
}
int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	sensor_ar0230_bypass_on(sensor_info);
	switch (fps) {
		case AR0230_5FPS:
			setting_size = sizeof(onsemi0230_5fps_setting)/sizeof(uint16_t)/2;
			vin_info("AR0230_5FPS  x2dev setting_size %d\n", setting_size);
			usleep(50*1000);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_5fps_setting[i*2],
							onsemi0230_5fps_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			usleep(50*1000);
			break;
		case AR0230_15FPS:
			setting_size = sizeof(onsemi0230_15fps_setting)/sizeof(uint16_t)/2;
			vin_info("AR0230_15FPS  x2dev setting_size %d\n", setting_size);
			usleep(50*1000);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_15fps_setting[i*2],
							onsemi0230_15fps_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			usleep(50*1000);
			break;
		case AR0230_25FPS:
			setting_size = sizeof(onsemi0230_25fps_setting)/sizeof(uint16_t)/2;
			vin_info("AR0230_25FPS  x2dev setting_size %d\n", setting_size);
			usleep(50*1000);
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						sensor_info->sensor_addr, onsemi0230_25fps_setting[i*2],
						onsemi0230_25fps_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			usleep(50*1000);
			break;
		default:
			vin_err("not suport fps type %d\n", fps);
			break;
	}
	ret = sensor_ar0230_bypass_off(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_ar0230_bypass_off fail\n", __LINE__);
		return ret;
	}
	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(onsemi0230_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("sensor_start sensor_name %s, setting_size = %d\n",
		sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
		setting_size, onsemi0230_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}
static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(onsemi0230_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("sensor_stop setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
			setting_size, onsemi0230_stream_off_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}
static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}
int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}
sensor_module_t ar0230 = {
	.module = "ar0230",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

