/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0231]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ar0231_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

int write_register(int bus, uint8_t *pdata, int setting_size, int fps)
{
	int ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int i, len;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 5) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = (pdata[i + 4] << 8) | pdata[i + 5];
			if ((reg_addr == AR0231_VTS) && (fps != 0))
				value = DEFAULT_FPS_REGVALUE * DEFAULT_FPS / fps;
			ret = hb_vin_i2c_write_reg16_data16(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write 0231_init_setting error\n");
				return ret;
			}
			usleep(5*1000);
			i = i + len + 1;
			vin_info("init ar0231---addr:0x%x reg:0x%x  value:%x\n",
					i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			usleep(100*1000);
			i = i + len + 1;
			vin_info("init max9296----addr:0x%x  reg:0x%x  value:%x\n",
					i2c_slave, reg_addr, value);
		} else if (len == 3) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = pdata[i + 2];
			value = pdata[i + 3];
			ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write max96705_init_setting error\n");
				return ret;
			}
			usleep(100*1000);
			i = i + len + 1;
			vin_info("init max96705---addr:0x%x reg:0x%x  value:%x\n",
					i2c_slave, reg_addr, value);
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
			i = i + 2;
		}
	}
	return ret;
}
int ar0231_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = NULL;
	int fps = sensor_info->fps;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if->init_state == 1)
		return ret;
	int bus = deserial_if->bus_num;
	if(sensor_info->config_index == 0) {
		pdata = ar0231_linear_30fps_init_setting;
		setting_size = sizeof(ar0231_linear_30fps_init_setting)/sizeof(uint8_t);
		vin_info("linear config mode!\n");
	} else if (sensor_info->config_index == 1) {
		pdata = ar0231_hdr_30fps_init_setting;
		setting_size = sizeof(ar0231_hdr_30fps_init_setting)/sizeof(uint8_t);
		vin_info("hdr config mode!\n");
	} else if (sensor_info->config_index == 2) {
		pdata = ar0231_dual_camera_init_setting;
		setting_size = sizeof(ar0231_dual_camera_init_setting)/sizeof(uint8_t);
		vin_info("hdr dual config mode!\n");
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
	ret = write_register(bus, pdata, setting_size, fps);
	if (ret < 0)
		vin_err("write register error\n");
	deserial_if->init_state = 1;
	return ret;
}

int sensor_reset(sensor_info_t *sensor_info)
{
	int gpio, value, ret = RET_OK;
	int bus = sensor_info->bus_num;
	uint8_t i2c_slave = POC_ADDR;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if->init_state == 1) {
		vin_info("sensor already reset!\n");
		return ret;
	}
	if(sensor_info->power_mode) {
		if (sensor_info->config_index == 2) {
			ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, 0x01, 0);
			if (ret < 0) {
				vin_err("write poc error\n");
				return ret;
			}
			usleep(512*1000);
			ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, 0x01, 0x0f);
			if (ret < 0) {
				vin_err("write poc error\n");
				return ret;
			}
			usleep(512*1000);
			vin_info("dual mode poc power on!\n");
		} else {
			ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, 0x01, 0);
			if (ret < 0) {
				vin_err("write poc error\n");
				return ret;
			}
			usleep(512*1000);
			ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, 0x01, 0x01);
			if (ret < 0) {
				vin_err("write poc error\n");
				return ret;
			}
			usleep(512*1000);
			vin_info("singel mode poc power on!\n");
		}
	}
	return ret;
}
int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

void sensor_common_data_init(sensor_info_t *sensor_info,
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
int sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	char init_d[3];
	uint32_t x0, y0, x1, y1, width, height;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_VTS, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  =
							(turning_data->sensor_data.VMAX  << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_HTS, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX =
							(turning_data->sensor_data.HMAX << 8) | init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_X_START, init_d, 2);
	x0 = init_d[0];
	x0 = (x0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_Y_START, init_d, 2);
	y0 = init_d[0];
	y0 = (y0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_X_END, init_d, 2);
	x1 = init_d[0];
	x1 = (x1 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0231_Y_END, init_d, 2);
	y1 = init_d[0];
	y1 = (y1 << 8) | init_d[1];
	width = x1 - x0 + 1;
	height = y1 - y0 + 1;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;
	turning_data->sensor_data.gain_max = 128 * 8192;
	turning_data->sensor_data.analog_gain_max = 176 * 8192;
	turning_data->sensor_data.digital_gain_max = 0;
	turning_data->sensor_data.exposure_time_min = 222;
	turning_data->sensor_data.exposure_time_max = 4000;
	turning_data->sensor_data.exposure_time_long_max = 4000;
	turning_data->sensor_data.lines_per_second = 44363;   // 87.75M / 1978
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;
	return ret;
}
static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	if(sizeof(turning_data->stream_ctrl.stream_on)
					>= sizeof(ar0231_stream_on_setting)) {
		memcpy(stream_on, ar0231_stream_on_setting,
					sizeof(ar0231_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_off)
					>= sizeof(ar0231_stream_off_setting)) {
		memcpy(stream_off, ar0231_stream_off_setting,
					sizeof(ar0231_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

static int mult_sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int size;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;
	int port = turning_data->port;
	size = sizeof(ar0231_mult_stream_on_setting) / 2;
	if(sizeof(turning_data->stream_ctrl.stream_on) >= size) {
		memcpy(stream_on, ar0231_mult_stream_on_setting[port], size);
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(ar0231_mult_stream_off_setting) / 2;
	if(sizeof(turning_data->stream_ctrl.stream_off) >= size) {
		memcpy(stream_off, ar0231_mult_stream_off_setting[port], size);
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

int sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	char str[24] = {0};
	sensor_turning_data_t turning_data;
	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}
	turning_data.pwl.param_hold = AR0231_PARAM_HOLD;
	turning_data.pwl.param_hold_length = 2;
	turning_data.pwl.line = AR0231_LINE;
	turning_data.pwl.line_length = 2;

	if (sensor_info->config_index == 2) {
		ret = mult_sensor_stream_control_set(&turning_data);
		if (ret < 0) {
			vin_err("sensor_stream_control_set fail %d\n", ret);
			return -RET_ERROR;
		}
	 } else {
		ret = sensor_stream_control_set(&turning_data);
		if (ret < 0) {
			vin_err("sensor_stream_control_set fail %d\n", ret);
			return -RET_ERROR;
		}
	}

#ifdef TUNING_LUT
	turning_data.pwl.line_p.ratio = 1 << 8;
	turning_data.pwl.line_p.offset = 0;
	turning_data.pwl.line_p.max = 4000;

	turning_data.pwl.again_control_num = 3;
	turning_data.pwl.again_control[1] = AR0231_GAIN;
	turning_data.pwl.again_control_length[1] = 2;
	turning_data.pwl.again_control[2] = AR0231_DGAIN;
	turning_data.pwl.again_control_length[2] = 2;
	turning_data.pwl.again_control[0] = AR0231_DC_GAIN;
	turning_data.pwl.again_control_length[0] = 2;
	turning_data.pwl.dgain_control_num = 0;
	turning_data.pwl.dgain_control[0] = 0;
	turning_data.pwl.dgain_control_length[0] = 0;
	turning_data.pwl.again_lut = malloc(256*3*sizeof(uint32_t));
	if (turning_data.pwl.again_lut != NULL) {
		memset(turning_data.pwl.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.pwl.again_lut, ar0231_dcgain,
			sizeof(ar0231_dcgain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_dcgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 256, ar0231_gain,
			sizeof(ar0231_gain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[256 + open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 512, ar0231_dgain,
			sizeof(ar0231_dgain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_dgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);
		turning_data.pwl.again_lut = NULL;
	}

	return ret;
}

int sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	int tmp = 0;
	/*pwl mode*/
	ret = ar0231_init(sensor_info);
	if(ret < 0) {
		vin_err("AR0231_X3_config fail!\n");
		return ret;
	}
	vin_info("AR0231_X3_config OK!\n");
	ret = sensor_pwl_data_init(sensor_info);

	if(ret < 0) {
		vin_err("sensor_pwl_data_init %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

static int sensor_normal_update_notify_driver(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}
	turning_data.pwl.param_hold = AR0231_PARAM_HOLD;
	turning_data.pwl.param_hold_length = 2;
	turning_data.pwl.line = AR0231_LINE;
	turning_data.pwl.line_length = 2;

	if (sensor_info->config_index == 2) {
		ret = mult_sensor_stream_control_set(&turning_data);
		if (ret < 0) {
			vin_err("sensor_stream_control_set fail %d\n", ret);
			return -RET_ERROR;
		}
	 } else {
		ret = sensor_stream_control_set(&turning_data);
		if (ret < 0) {
			vin_err("sensor_stream_control_set fail %d\n", ret);
			return -RET_ERROR;
		}
	}

#ifdef TUNING_LUT
	turning_data.pwl.line_p.ratio = 1 << 8;
	turning_data.pwl.line_p.offset = 0;
	turning_data.pwl.line_p.max = 4000;

	turning_data.pwl.again_control_num = 3;
	turning_data.pwl.again_control[1] = AR0231_GAIN;
	turning_data.pwl.again_control_length[1] = 2;
	turning_data.pwl.again_control[2] = AR0231_DGAIN;
	turning_data.pwl.again_control_length[2] = 2;
	turning_data.pwl.again_control[0] = AR0231_DC_GAIN;
	turning_data.pwl.again_control_length[0] = 2;
	turning_data.pwl.dgain_control_num = 0;
	turning_data.pwl.dgain_control[0] = 0;
	turning_data.pwl.dgain_control_length[0] = 0;
	turning_data.pwl.again_lut = malloc(256*3*sizeof(uint32_t));
	if (turning_data.pwl.again_lut != NULL) {
		memset(turning_data.pwl.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.pwl.again_lut, ar0231_dcgain,
			sizeof(ar0231_dcgain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_dcgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 256, ar0231_gain,
			sizeof(ar0231_gain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[256 + open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 512, ar0231_dgain,
			sizeof(ar0231_dgain));
		for (open_cnt =0; open_cnt <
			sizeof(ar0231_dgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);
		turning_data.pwl.again_lut = NULL;
	}

	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int frame_line = 0;
	int value;
	if (fps > 30 || fps < 5) {
		vin_err("not suport fps type %d\n", fps);
		return -RET_ERROR;
	} else {
		vin_info("AR0231 FPS IS %d\n", fps);
		frame_line = DEFAULT_FPS_REGVALUE * DEFAULT_FPS / fps;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr, AR0231_VTS, frame_line);
		if(ret <0) {
			vin_err("[%s]%s err!\n", __func__, __LINE__);
			ret = -RET_ERROR;
		}
		usleep(20*1000);
	}
	sensor_info->fps = fps;
	sensor_normal_update_notify_driver(sensor_info);
	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	ret = sensor_reset(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	if (sensor_info->fps != DEFAULT_FPS) {
		ret = sensor_dynamic_switch_fps(sensor_info, sensor_info->fps);
		if (ret < 0) {
			vin_err("%d : change fps %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t *pdata = NULL;
	int fps = sensor_info->fps;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	int bus = deserial_if->bus_num;
	int port = sensor_info->port;
	vin_info("0231 sensor start port : %d\n", port);
	if (sensor_info->config_index == 2) {
		pdata = ar0231_mult_stream_on_setting[port];
		setting_size =
			sizeof(ar0231_mult_stream_on_setting)/(2 * sizeof(uint8_t));
	} else {
		pdata = ar0231_stream_on_setting;
		setting_size = sizeof(ar0231_stream_on_setting)/sizeof(uint8_t);
	}

	ret = write_register(bus, pdata, setting_size, fps);
	if (ret < 0)
		vin_err("write register error\n");

	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t *pdata = NULL;
	int fps = sensor_info->fps;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	int bus = deserial_if->bus_num;
	int port = sensor_info->port;
	vin_info("0231 sensor stop port : %d\n", port);
	if (sensor_info->config_index == 2) {
		pdata = ar0231_mult_stream_off_setting[port];
		setting_size =
			sizeof(ar0231_mult_stream_off_setting)/(2 * sizeof(uint8_t));
	} else {
		pdata = ar0231_stream_off_setting;
		setting_size = sizeof(ar0231_stream_off_setting)/sizeof(uint8_t);
	}
	ret = write_register(bus, pdata, setting_size, fps);
	if (ret < 0)
		vin_err("write register error\n");

	return ret;
}
int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}
int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

sensor_module_t ar0231 = {
	.module = "ar0231",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

