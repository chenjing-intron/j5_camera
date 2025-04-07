/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[imx385]:" fmt

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

#include "hb_cam_utility.h"
#include "hb_i2c.h"
#include "inc/imx385_setting.h"
#include "inc/sensor_effect_common.h"

void sensor_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->cs = sensor_info->spi_info.spi_cs;
	turning_data->spi_mode = sensor_info->spi_info.spi_mode;
	turning_data->spi_speed = sensor_info->spi_info.spi_speed;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
			sizeof(turning_data->sensor_name));

	return;
}
int sensor_param_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	char init_d[3];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX385_VAMX, init_d, 3);
	turning_data->sensor_data.VMAX = init_d[2];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[0];
	turning_data->sensor_data.FSC_DOL2 = turning_data->sensor_data.VMAX * 2;
	turning_data->sensor_data.gain_max = 210;
	turning_data->sensor_data.exposure_time_min = 1;
	turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX385_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[1];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[0];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX385_RHS1, init_d, 3);
	turning_data->sensor_data.RHS1 = init_d[2];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[1];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[0];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX385_CSI_LANE_MODE, init_d, 1);
	if (init_d[0] == 3)
		turning_data->sensor_data.lane = 4;
	else
		turning_data->sensor_data.lane = 2;
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX385_INCKFREQ1, init_d, 1);
	if (init_d[0] == 0x40)
		turning_data->sensor_data.clk = 37125000;
	else
		turning_data->sensor_data.clk = 74250000;

	turning_data->sensor_data.active_width = 1936;
	turning_data->sensor_data.active_height = 1097;
	turning_data->sensor_data.turning_type = 1;  // imx385 calc
	return ret;
}

int sensor_normal_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int sen_devfd;
	char str[24] = {0};
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	if (sensor_info->sen_devfd <= 0) {
		snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
		if ((sen_devfd = open(str, O_RDWR)) < 0) {
			printf("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
		sensor_info->sen_devfd = sen_devfd;
	}

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);
	turning_data.normal.s_line = IMX385_SHS1;
	turning_data.normal.s_line_length = 3;

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		printf("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int sen_devfd;
	char str[24] = {0};
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	if (sensor_info->sen_devfd <= 0) {
		snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
		if ((sen_devfd = open(str, O_RDWR)) < 0) {
			printf("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
		sensor_info->sen_devfd = sen_devfd;
	}

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);
	turning_data.dol2.s_line = IMX385_SHS1;
	turning_data.dol2.s_line_length = 3;

	turning_data.dol2.m_line = IMX385_SHS2;
	turning_data.dol2.m_line_length = 3;

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		printf("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			setting_size = sizeof(imx385_raw12_normal_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx385_raw12_normal_setting);
			if (ret < 0) {
				vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			setting_size = sizeof(imx385_raw12_dol2_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx385_raw12_dol2_setting);
			if (ret < 0) {
				vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_dol2_data_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL3_M:  //  DOl3
			break;
		default:
			break;
	}
	return ret;
}
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(imx385_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx385_stream_on_setting);
	if(ret < 0) {
		vin_err("sensor_start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(imx385_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx385_stream_off_setting);
	if(ret < 0) {
		vin_err("sensor_stop %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->dev_port >= 0 && sensor_info->sen_devfd > 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return ret;
}

sensor_module_t imx385 = {
	.module = "imx385",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};

