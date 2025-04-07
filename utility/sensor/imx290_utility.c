/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[imx290]:" fmt

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
#include "inc/imx290_setting.h"
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

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_VAMX, init_d, 3);
	turning_data->sensor_data.VMAX = init_d[2];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[0];
	turning_data->sensor_data.FSC_DOL2 = turning_data->sensor_data.VMAX * 2;
	turning_data->sensor_data.FSC_DOL3 = turning_data->sensor_data.VMAX * 4;
	turning_data->sensor_data.gain_max = 210;
	turning_data->sensor_data.exposure_time_min = 1;
	turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[1];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[0];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_RHS1, init_d, 3);
	turning_data->sensor_data.RHS1 = init_d[2];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[1];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[0];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_RHS2, init_d, 3);
	turning_data->sensor_data.RHS2 = init_d[2];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[1];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[0];

	turning_data->sensor_data.lines_per_second = 10074;
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_X_SIZE, init_d, 2);
	turning_data->sensor_data.active_width = init_d[1];
	turning_data->sensor_data.active_width = (turning_data->sensor_data.active_width << 8) | init_d[0];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX290_Y_SIZE, init_d, 2);
	turning_data->sensor_data.active_height = init_d[1];
	turning_data->sensor_data.active_height = (turning_data->sensor_data.active_height << 8) | init_d[0];
	turning_data->sensor_data.turning_type = 1;  // imx290 calc
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
	turning_data.normal.s_line = IMX290_SHS1;
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
	turning_data.dol2.s_line = IMX290_SHS1;
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX290_SHS2;
	turning_data.dol2.m_line_length = 3;

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		printf("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_dol3_data_init(sensor_info_t *sensor_info)
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
	turning_data.dol3.s_line = IMX290_SHS1;
	turning_data.dol3.s_line_length = 3;

	turning_data.dol3.m_line = IMX290_SHS2;
	turning_data.dol3.m_line_length = 3;

	turning_data.dol3.l_line = IMX290_SHS3;
	turning_data.dol3.l_line_length = 3;

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

	if (sensor_info->extra_mode) {
		int not_support = 0;
		uint16_t vts, hts, vh = 0;

		if (sensor_info->extra_mode != 1) {
			vh = 1;
			vts = ((uint32_t)sensor_info->extra_mode) >> 16;
			hts = ((uint32_t)sensor_info->extra_mode) & 0xffff;
			vin_info("%s hts=%d(0x%04x) vts=%d(0x%04x)\n", sensor_info->sensor_name, hts, hts, vts, vts);
			imx290_extra_hts_vts_setting[1] = hts >> 8;
			imx290_extra_hts_vts_setting[3] = hts & 0xff;
			imx290_extra_hts_vts_setting[5] = vts >> 8;
			imx290_extra_hts_vts_setting[7] = vts & 0xff;
		}

		switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			if (sensor_info->resolution == 720) {  //  1280*720
				if (sensor_info->format == 10) {  //  raw10
					setting_size = sizeof(imx290_1280x720_raw10_25fps_normal_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1280x720_raw10_25fps_normal_setting);
				} else {
					setting_size = sizeof(imx290_1280x720_raw12_25fps_normal_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1280x720_raw12_25fps_normal_setting);
				}
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
				if (sensor_info->format == 10) {  //  raw10
					setting_size = sizeof(imx290_1920x1080_raw10_25fps_normal_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw10_25fps_normal_setting);
				} else {
					setting_size = sizeof(imx290_1920x1080_raw12_25fps_normal_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw12_25fps_normal_setting);
				}
			} else {
				not_support = 1;
			}
			if (ret == 0 && not_support == 0 && vh) {
				setting_size = sizeof(imx290_extra_hts_vts_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, imx290_extra_hts_vts_setting);
			}
			if (ret == 0 && not_support == 0) {
				ret = sensor_normal_data_init(sensor_info);
				if(ret < 0) {
					vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case DOL2_M:  //  DOl2
			if (sensor_info->resolution == 1080) {  //  1920*1080
				if (sensor_info->format == 10) {  //  raw10
					setting_size = sizeof(imx290_1920x1080_raw10_25fps_dol2_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw10_25fps_dol2_setting);
				} else {
					setting_size = sizeof(imx290_1920x1080_raw12_25fps_dol2_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw12_25fps_dol2_setting);
				}
			} else {
				not_support = 1;
			}
			if (ret == 0 && not_support == 0 && vh) {
				setting_size = sizeof(imx290_extra_hts_vts_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, imx290_extra_hts_vts_setting);
			}
			if (ret == 0 && not_support == 0) {
				ret = sensor_dol2_data_init(sensor_info);
				if(ret < 0) {
					vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		case DOL3_M:  //  DOl3
			if (sensor_info->resolution == 1080) {  //  1920*1080
				if (sensor_info->format == 10) {  //  raw12
					setting_size = sizeof(imx290_1920x1080_raw10_25fps_dol3_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw10_25fps_dol3_setting);
				} else {
					setting_size = sizeof(imx290_1920x1080_raw12_25fps_dol3_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx290_1920x1080_raw12_25fps_dol3_setting);
				}
			} else {
				not_support = 1;
			}
			if (ret == 0 && not_support == 0 && vh) {
				setting_size = sizeof(imx290_extra_hts_vts_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, imx290_extra_hts_vts_setting);
			}
			if (ret == 0 && not_support == 0) {
				ret = sensor_dol3_data_init(sensor_info);
				if(ret < 0) {
					vin_err("sensor_dol3_data_init %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			not_support = 1;
			break;
		}

		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
		} else if (not_support) {
			vin_err("%s sensor_mode%d extra_mode%d %dP %dfps not support\n", sensor_info->sensor_name,
				sensor_info->sensor_mode, sensor_info->extra_mode,
				sensor_info->resolution, sensor_info->fps);
			ret = -RET_ERROR;
		}
		return ret;
	}

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			setting_size = sizeof(imx290_raw12_normal_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx290_raw12_normal_setting);
			if (ret < 0) {
				vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			setting_size = sizeof(imx290_raw12_dol2_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx290_raw12_dol2_setting);
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
			setting_size = sizeof(imx290_raw12_dol3_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx290_raw12_dol3_setting);
			if (ret < 0) {
				vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_dol3_data_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_dol3_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
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

	setting_size = sizeof(imx290_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx290_stream_on_setting);
	if(ret < 0) {
		vin_err("sensor_start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(imx290_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx290_stream_off_setting);
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

sensor_module_t imx290 = {
	.module = "imx290",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
