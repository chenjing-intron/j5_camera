/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[onsemi_dual]:" fmt

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
#include "inc/onsemi_dual_setting.h"
#include "hb_i2c.h"

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio,ret = RET_OK;

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

	setting_size = sizeof(xc9080_dual_bypass_on_0230_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_bypass_on_0230_setting);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}
static int sensor_ar0238_bypass_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(xc9080_dual_bypass_on_0237_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_bypass_on_0237_setting);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}
static int sensor_xc9080_bypass_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(xc9080_dual_bypass_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_bypass_off_setting);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}
static int sensor_xc9080_1080_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int fps = sensor_info->fps;
	int setting_size = 0;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(xc9080_dual_15fps_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_15fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(xc9080_dual_25fps_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_25fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_dbg("xc9080_dual1080P_init %dfps setting_size = %d success\n", fps, setting_size);
	return ret;
}
static int sensor_xc9080_720_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int fps = sensor_info->fps;
	int setting_size = 0;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(xc9080_dual720p_15fps_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual720p_15fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(xc9080_dual720p_25fps_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual720p_25fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_dbg("xc9080_dual720P_init %dfps setting_size = %d success \n", fps, setting_size);
	return ret;
}
static int sensor_ar0230_1080_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(dual_ar0230_15fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, dual_ar0230_15fps_init_setting[i*2], dual_ar0230_15fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 373 || i == 374) {
					usleep(10*1000);
				}
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(dual_ar0230_25fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, dual_ar0230_25fps_init_setting[i*2], dual_ar0230_25fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 373 || i == 374) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("AR0230_config_1080P %dfps x2dev OK!\n", fps);
	return ret;
}

static int sensor_ar0230_720_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(dual720p_ar0230_15fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, dual720p_ar0230_15fps_init_setting[i*2], dual720p_ar0230_15fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 373 || i == 374) {
					usleep(10*1000);
				}
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(dual720p_ar0230_25fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, dual720p_ar0230_25fps_init_setting[i*2], dual720p_ar0230_25fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 373 || i == 374) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("AR0230_config_720P %dfps x2dev OK!\n", fps);
	return ret;
}
static int sensor_ar0238_1080_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(dual_ar0237_15fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
					dual_ar0237_15fps_init_setting[i*2], dual_ar0237_15fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 0) {
					usleep(50*1000);
				}
				if(i == 1) {
					usleep(200*1000);
				}
				if(i == 250 || i == 251) {
					usleep(10*1000);
				}
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(dual_ar0237_25fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
					dual_ar0237_25fps_init_setting[i*2], dual_ar0237_25fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 0) {
					usleep(50*1000);
				}
				if(i == 1) {
					usleep(200*1000);
				}
				if(i == 250 || i == 251) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("AR0237_config_1080P %dfps x2dev OK!\n", fps);
	return ret;
}

static int sensor_ar0238_720_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_15FPS:
			setting_size = sizeof(dual720p_ar0237_15fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
				dual720p_ar0237_15fps_init_setting[i*2], dual720p_ar0237_15fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 0) {
					usleep(50*1000);
				}
				if(i == 1) {
					usleep(200*1000);
				}
				if(i == 250 || i == 251) {
					usleep(10*1000);
				}
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(dual720p_ar0237_25fps_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
				dual720p_ar0237_25fps_init_setting[i*2], dual720p_ar0237_25fps_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 0) {
					usleep(50*1000);
				}
				if(i == 1) {
					usleep(200*1000);
				}
				if(i == 250 || i == 251) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("AR0237_config_720P %dfps x2dev OK!\n", fps);
	return ret;
}
static int sensor_ar0230_96board_1080_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_25FPS:
			setting_size = sizeof(dual_ar0230_25fps_96board_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				dual_ar0230_25fps_96board_init_setting[i*2],
				dual_ar0230_25fps_96board_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 376 || i == 377) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("xchip9080_AR0230_config 1080P %dfps 96board OK!\n", fps);
	return ret;
}
static int sensor_ar0238_96board_1080_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int fps = sensor_info->fps;

	switch (fps) {
		case DUAL_25FPS:
			setting_size = sizeof(dual_ar0237_25fps_96board_init_setting)/sizeof(uint16_t)/2;
			for(i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
						dual_ar0237_25fps_96board_init_setting[i*2],
						dual_ar0237_25fps_96board_init_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
				if(i == 0) {
					usleep(50*1000);
				}
				if(i == 1) {
					usleep(200*1000);
				}
				if(i == 253 || i == 254) {
					usleep(10*1000);
				}
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("xchip9080_AR0238_config 1080P %dfps 96board OK!\n", fps);
	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	if((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS)
			&& sensor_info->resolution == 1080) {
		ret = sensor_xc9080_1080_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_xc9080_1080_init fps %d fail\n", __LINE__, sensor_info->fps);
			return ret;
		}
	} else if ((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS)
			&& sensor_info->resolution == 720) {
		ret = sensor_xc9080_720_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_xc9080_720_init fps %d fail\n", __LINE__, sensor_info->fps);
			return ret;
		}
	}
	ret = sensor_ar0230_bypass_on(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_0230_bypss_on  %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	if(sensor_info->extra_mode == 0) {/*x2dev config*/
		if((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS) &&
				sensor_info->resolution == 1080) {
			ret = sensor_ar0230_1080_init(sensor_info);
			if (ret < 0) {
				vin_err("%d sensor_ar0230_1080_init fps %d fail\n", __LINE__, sensor_info->fps);
				return ret;
			}
		} else if ((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS) &&
				sensor_info->resolution == 720) {
			ret = sensor_ar0230_720_init(sensor_info);
			if (ret < 0) {
				vin_err("%d sensor_ar0230_720_init fps %d fail\n", __LINE__, sensor_info->fps);
				return ret;
			}
		}
	}
	if(sensor_info->extra_mode == 1) { /*96board config*/
		ret = sensor_ar0230_96board_1080_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_ar0230_96board_1080_init fps %d fail\n", __LINE__,
						sensor_info->fps);
			return ret;
		}
	}
	ret = sensor_xc9080_bypass_off(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_xc9080_bypss_off fail\n", __LINE__);
		return ret;
	}
	ret = sensor_ar0238_bypass_on(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_0230_bypss_on  %s fail\n", __LINE__,
				sensor_info->sensor_name);
		return ret;
	}
	if(sensor_info->extra_mode == 0) { /*x2dev config*/
		if((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS) &&
					sensor_info->resolution == 1080) {
			ret = sensor_ar0238_1080_init(sensor_info);
			if (ret < 0) {
				vin_err("%d sensor_ar0238_1080_init fps %d fail\n", __LINE__,
							sensor_info->fps);
				return ret;
			}
		}
		if((sensor_info->fps == DUAL_15FPS || sensor_info->fps == DUAL_25FPS) &&
					sensor_info->resolution == 720) {
			ret = sensor_ar0238_720_init(sensor_info);
			if (ret < 0) {
				vin_err("%d sensor_ar0238_720_init fps %d fail\n", __LINE__,
							sensor_info->fps);
				return ret;
			}
		}
	}
	if(sensor_info->extra_mode == 1) { /*96board config*/
		ret = sensor_ar0238_96board_1080_init(sensor_info);
		if (ret < 0) {
			vin_err("%d sensor_ar0238_96board_1080_init fps %d fail\n",
						__LINE__, sensor_info->fps);
			return ret;
		}
	}
	ret = sensor_xc9080_bypass_off(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_xc9080_bypss_off fail\n", __LINE__);
		return ret;
	}
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int setting_size = 0;
	int ret = RET_OK;

	setting_size = sizeof(xc9080_dual_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("sensor_start on sensor_name %s, setting_size = %d\n",
		sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
				setting_size, xc9080_dual_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(xc9080_dual_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("sensor_start on sensor_name %s, setting_size = %d\n",
		sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size,
		xc9080_dual_stream_off_setting);
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

static int sensor_xc9080_switch_fps_config(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch (fps) {
		case DUAL_5FPS:
			setting_size = sizeof(xc9080_default_regs_5fps)/sizeof(uint32_t)/2;
			vin_dbg("sensor_xc9080_5fps_config setting_size = %d\n", setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
						setting_size, xc9080_default_regs_5fps);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case DUAL_15FPS:
			setting_size = sizeof(xc9080_default_regs_15fps)/sizeof(uint32_t)/2;
			vin_dbg("sensor_xc9080_15fps_config setting_size = %d\n", setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
					setting_size, xc9080_default_regs_15fps);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case DUAL_25FPS:
			setting_size = sizeof(xc9080_default_regs_25fps)/sizeof(uint32_t)/2;
			vin_dbg("sensor_xc9080_25fps_config setting_size = %d\n", setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2,
					setting_size, xc9080_default_regs_25fps);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	return ret;
}
static int sensor_ar0230_swicth_fps_config(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0, i = 0;

	switch (fps) {
		case DUAL_5FPS:
			setting_size = sizeof(AR0230_default_regs_5fps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
					AR0230_default_regs_5fps[i*2], AR0230_default_regs_5fps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			usleep(100*1000);
			break;
		case DUAL_15FPS:
			setting_size = sizeof(AR0230_default_regs_15fps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
					AR0230_default_regs_15fps[i*2], AR0230_default_regs_15fps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			usleep(100*1000);
			break;
		case DUAL_25FPS:
			setting_size = sizeof(AR0230_default_regs_25fps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
					AR0230_default_regs_25fps[i*2], AR0230_default_regs_25fps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			usleep(2000*1000);
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("ar0230_swicth %dfps success!\n", fps);
	return ret;
}
static int sensor_ar0238_swicth_fps_config(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0, i = 0;

	switch (fps) {
		case DUAL_5FPS:
			setting_size = sizeof(AR0238_default_regs_5ps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
				AR0238_default_regs_5ps[i*2], AR0238_default_regs_5ps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			usleep(100*1000);
			break;
		case DUAL_15FPS:
			setting_size = sizeof(AR0238_default_regs_15ps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
				AR0238_default_regs_15ps[i*2], AR0238_default_regs_15ps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			usleep(100*1000);
			break;
		case DUAL_25FPS:
			setting_size = sizeof(AR0238_default_regs_25ps)/sizeof(uint16_t)/2;
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor1_addr,
					AR0238_default_regs_25ps[i*2], AR0238_default_regs_25ps[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support fps type %d\n", fps);
			break;
	}
	vin_info("ar0238_swicth %dfps success!\n", fps);
	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int resolution = sensor_info->resolution;

	switch (fps) {
		case DUAL_5FPS:
			sensor_xc9080_switch_fps_config(sensor_info, fps);
			sensor_ar0230_bypass_on(sensor_info);
			sensor_ar0230_swicth_fps_config(sensor_info, fps);
			sensor_xc9080_bypass_off(sensor_info);
			sensor_ar0238_bypass_on(sensor_info);
			sensor_ar0238_swicth_fps_config(sensor_info, fps);
			usleep(50*1000);
			break;
		case DUAL_15FPS:
			sensor_xc9080_switch_fps_config(sensor_info, fps);
			sensor_ar0230_bypass_on(sensor_info);
			sensor_ar0230_swicth_fps_config(sensor_info, fps);
			sensor_xc9080_bypass_off(sensor_info);
			sensor_ar0238_bypass_on(sensor_info);
			sensor_ar0238_swicth_fps_config(sensor_info, fps);
			usleep(50*1000);
			break;
		case DUAL_25FPS:
			ret = sensor_poweron(sensor_info);
			if (ret < 0) {
				vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			sensor_info->fps = fps;
			if(resolution == 1080) {
				sensor_xc9080_1080_init(sensor_info);
			} else {
				sensor_xc9080_720_init(sensor_info);
			}
			sensor_ar0230_bypass_on(sensor_info);
			if(resolution == 1080) {
				sensor_ar0230_1080_init(sensor_info);
			} else {
				sensor_ar0230_720_init(sensor_info);
			}
			sensor_xc9080_bypass_off(sensor_info);
			sensor_ar0238_bypass_on(sensor_info);
			if(resolution == 1080) {
				sensor_ar0238_1080_init(sensor_info);
			} else {
				sensor_ar0238_720_init(sensor_info);
			}
			#if 0
			sensor_stop(sensor_info);
			ret = sensor_poweron(sensor_info);
			if (ret < 0) {
				vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			setting_size = sizeof(xc9080_dual_25fps_init_setting)/sizeof(uint32_t)/2;
			vin_dbg("xc9080_dual1080P_init setting_size = %d\n", setting_size);
			vin_dbg("isp_addr 0x%x sensor_addr 0x%x sensor1_addr 0x%x bus_num %d\n", sensor_info->isp_addr, sensor_info->sensor_addr, sensor_info->sensor1_addr, sensor_info->bus_num);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->isp_addr, 2, setting_size, xc9080_dual_25fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			//sensor_xc9080_switch_fps_config(sensor_info, fps);
			sensor_ar0230_bypass_on(sensor_info);
			sensor_ar0230_swicth_fps_config(sensor_info, fps);

			sensor_xc9080_bypass_off(sensor_info);
			sensor_ar0238_bypass_on(sensor_info);
			sensor_ar0238_swicth_fps_config(sensor_info, fps);

			#endif
			break;
		default:
			vin_err("not suport fps type %d\n", fps);
			break;
	}
	ret = sensor_xc9080_bypass_off(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_xc9080_bypss_off fail\n", __LINE__);
		return ret;
	}
	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

sensor_module_t onsemi_dual = {
	.module = "onsemi_dual",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

