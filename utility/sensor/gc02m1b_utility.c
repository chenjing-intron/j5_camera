/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[gc02m1b]:" fmt

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
#include <linux/ioctl.h>

#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/gc02m1b_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static int sensor_common_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[24] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);
	return ret;
}
static void  sensor_common_data_init(sensor_info_t *sensor_info,
	sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
		sizeof(turning_data->sensor_name));
	return;
}

static int sensor_param_init(sensor_info_t *sensor_info,
	sensor_turning_data_t *turning_data, uint32_t fps)
{
	int ret = RET_OK;
	char init_d[3];


	turning_data->sensor_data.conversion = 1;

	turning_data->sensor_data.lane = 4;
	turning_data->sensor_data.clk = GC02M1B_SYSCLKMHZ;  // uncertain
	turning_data->sensor_data.lines_per_second = 30000;
	turning_data->sensor_data.exposure_time_long_max = 2210;
	if (fps == 25) {
		turning_data->sensor_data.exposure_time_max = 2500;
		turning_data->sensor_data.lines_per_second = 30000;
	} else if (fps == 30) {
		turning_data->sensor_data.exposure_time_max = 2500;
		turning_data->sensor_data.lines_per_second = 30000;
	}

	turning_data->sensor_data.fps = fps;  // fps

	//  turning_data->sensor_data.active_height = init_d[0];  // ******self
	//  turning_data->sensor_data.active_height =
	//  (turning_data->sensor_data.active_height << 8) | init_d[1];

	turning_data->sensor_data.turning_type = 6;    // gain calc

	//  sensor info
	turning_data->sensor_data.analog_gain_max = 115 * 8192;
	turning_data->sensor_data.digital_gain_max = 0 * 8192;
	turning_data->sensor_data.exposure_time_min = 4;

	return ret;
}

static int sensor_normal_data_init(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t open_cnt;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data, fps);

	turning_data.normal.s_line = GC02M1B_SHS1;
	turning_data.normal.s_line_length = 2;

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 2500;

	turning_data.normal.again_control_num = 3;
	turning_data.normal.again_control[0] = 0xfe;
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_control[1] = GC02M1B_GAIN;
	turning_data.normal.again_control_length[1] = 1;
	turning_data.normal.again_control[2] = GC02M1B_DGAIN;
	turning_data.normal.again_control_length[2] = 2;

	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = GC02M1B_DGAIN;
	turning_data.normal.dgain_control_length[0] = 0;

	turning_data.normal.again_lut = malloc(256 * 3 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.normal.again_lut, gc02m1b_page_lut,
			sizeof(gc02m1b_page_lut));
		for (open_cnt =0; open_cnt < sizeof(gc02m1b_page_lut)/
			sizeof(uint32_t); open_cnt++) {
			//DOFFSET(&turning_data.normal.again_lut[open_cnt], 1);
		}

		memcpy(turning_data.normal.again_lut + 256, gc02m1b_again_lut,
			sizeof(gc02m1b_again_lut));
		for (open_cnt =0; open_cnt < sizeof(gc02m1b_again_lut)/
			sizeof(uint32_t); open_cnt++) {
			//DOFFSET(&turning_data.normal.again_lut[open_cnt], 1);
		}

		memcpy(turning_data.normal.again_lut + 512, gc02m1b_dgain_lut,
			sizeof(gc02m1b_dgain_lut));
		for (open_cnt =0; open_cnt < sizeof(gc02m1b_dgain_lut)/
			sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt + 512], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", sensor_info->port, ret);
		return -RET_ERROR;
	}
	if (turning_data.normal.again_lut)
		free(turning_data.normal.again_lut);
	if (turning_data.normal.dgain_lut)
		free(turning_data.normal.dgain_lut);

	return ret;
}

static int sensor_update_fps_notify_driver(sensor_info_t *sensor_info,
	uint32_t fps)
{
	int ret = 0;

	sensor_info->fps = fps;

	if (sensor_info->sensor_mode == NORMAL_M) {
		ret = sensor_normal_data_init(sensor_info, fps);
		if(ret < 0) {
			vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
	}
	return ret;
}

static int sensor_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint32_t gc02m1b_fps[8];

	if (sensor_info->sensor_mode == NORMAL_M) {
		if (fps == GC02M1B_25FPS) {
			if (sensor_info->resolution == 1200) {  //  3840*2160
				memcpy(gc02m1b_fps, &gc02m1b_25fps_normal_setting[S2160P],
					sizeof(gc02m1b_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
			}
		} else {
			if (sensor_info->resolution == 1200) {  //  3840*2160
				memcpy(gc02m1b_fps, &gc02m1b_30fps_normal_setting[S2160P],
					sizeof(gc02m1b_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
			}
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
	}

	switch (fps) {
	case GC02M1B_25FPS:
		vin_info("gc02m1b_25FPS\n");
		setting_size = sizeof(gc02m1b_fps)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			1, setting_size, gc02m1b_fps);
		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
		usleep(20*1000);
	break;
	case GC02M1B_30FPS:
		vin_info("gc02m1b_30FPS\n");
		setting_size = sizeof(gc02m1b_fps)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			1, setting_size, gc02m1b_fps);
		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
		usleep(20*1000);
	break;
	default:
		vin_err("not suport fps type %d\n", fps);
		return -1;
	break;
	}

	return ret;
}

static int sensor_init_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			vin_info("%s init normal resolution %d\n", sensor_info->sensor_name,
				sensor_info->resolution);
			if (sensor_info->resolution == 600) {  //  3840*2160
				setting_size = sizeof(gc02m1b_600p_raw8_normal_setting)/
					sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				1, setting_size, gc02m1b_600p_raw8_normal_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->resolution == 1200) {  //  1920*1080
				setting_size = sizeof(gc02m1b_1200p_raw10_normal_setting)/
					sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				1, setting_size, gc02m1b_1200p_raw10_normal_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			}

			break;
		case DOL2_M:  //  DOl2
			break;
		default:
			break;
	}

	ret = sensor_switch_fps(sensor_info, sensor_info->fps);

	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			vin_info("%s init normal\n", sensor_info->sensor_name);
			ret = sensor_common_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_common_open_failed %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_init_setting(sensor_info);
			if (ret < 0) {
				vin_err("sensor_init_setting %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_normal_data_init(sensor_info, sensor_info->fps);
			if(ret < 0) {
				vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				close(sensor_info->sen_devfd);
				sensor_info->sen_devfd = -1;
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			break;
		default:
			break;
	}
	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;

	ret = sensor_switch_fps(sensor_info, fps);
	if (ret < 0)
		return ret;

	ret = sensor_update_fps_notify_driver(sensor_info, fps);
	if (ret < 0) {
		vin_info("update %dfps param failed\n", fps);
		return ret;
	}

	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	setting_size = sizeof(gc02m1b_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	1, setting_size, gc02m1b_stream_on_setting);
	if(ret < 0) {
		vin_err("sensor_start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	setting_size = sizeof(gc02m1b_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	1, setting_size, gc02m1b_stream_off_setting);
	if(ret < 0) {
		vin_err("sensor_stop %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return ret;
}

sensor_module_t gc02m1b = {
	.module = "gc02m1b",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};
