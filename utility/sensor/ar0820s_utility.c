/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0820]:" fmt

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
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ar0820s_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensor_common.h"

#define TUNING_LUT
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x18)
#define DEFAULT_GALAXY_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR_A	(0x62)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR		(0x48)

#define WEISEN_LINES_PER_SECOND             (11764)
#define INCEPTIO_LINES_PER_SECOND           (8784)
#define GALAXY_LINES_PER_SECOND             (8784)
#define GAHDR4_LINES_PER_SECOND             (8832)
#define STOP_DELAY_TIME_US	(1800)
#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"

int32_t write_register(int32_t bus, int32_t deserial_addr, int32_t poc_addr, int32_t serial_addr,
			int32_t sensor_addr, uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int32_t i, len, k;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 5) {
			i2c_slave = pdata[i + 1] >> 1;
			if (sensor_addr != 0 && i2c_slave == DEFAULT_SENSOR_ADDR)
				i2c_slave = sensor_addr;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = (pdata[i + 4] << 8) | pdata[i + 5];
			ret = hb_vin_i2c_write_reg16_data16(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write ar0820 %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// 	usleep(5*1000);
			i = i + len + 1;
			vin_info("write ar0820 %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
			} else if (serial_addr != 0 && i2c_slave == DEFAULT_SERIAL_ADDR_A) {
				i2c_slave = serial_addr;
			}
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			k = 10;
			while (ret < 0 && k--) {
				vin_warn("write serdes %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			}
			if (ret < 0) {
				vin_err("write serdes %d@0x%02x: 0x%04x=0x%02x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// usleep(100*1000);
			i = i + len + 1;
			vin_info("write serdes %d@0x%02x: 0x%04x=0x%02x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 3) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = pdata[i + 2];
			value = pdata[i + 3];
			if (poc_addr != INVALID_POC_ADDR) {
				if (poc_addr != 0 && i2c_slave == DEFAULT_POC_ADDR)
					i2c_slave = poc_addr;
				ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
				if (ret < 0) {
					vin_err("write poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				// usleep(100*1000);
				vin_info("write poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
			} else {
				if (reg_addr == 0x01 && value == 0x00) {
					/* reset all serials replace to poc off */
					for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 2); i2c_slave++) {
						vin_info("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
						hb_vin_i2c_write_reg16_data8(bus, i2c_slave, 0x0010, 0xf1);
					}
				}
			}
			i = i + len + 1;
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
			i = i + 2;
		}
	}
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
		sizeof(turning_data->sensor_name) - 1);
	return;
}

int32_t ar0820_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	uint32_t desport_num = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;
	desport_num = deserial_if->reserved[0];

	/* xj3 / j5 */
	if(sensor_info->sensor_mode == NORMAL_M) {
		pdata = ar0820_linear_30fps_init_setting;
		setting_size = sizeof(ar0820_linear_30fps_init_setting)/sizeof(uint8_t);
		vin_info("linear config mode!\n");
	} else if (sensor_info->sensor_mode == DOL2_M) {
		pdata = ar0820_dol2_15fps_init_setting;
		setting_size = sizeof(ar0820_dol2_15fps_init_setting)/sizeof(uint8_t);
		vin_info("hdr dol2 config mode!\n");
	} else if (sensor_info->sensor_mode == PWL_M) {
		if (sensor_info->config_index & PWL_HDR4_24BIT) {
			pdata = ar0820_hdr_4exp_30fps_init_setting;
			setting_size = sizeof(ar0820_hdr_4exp_30fps_init_setting)/sizeof(uint8_t);
			vin_info("hdr 4exp pwl config mode!\n");
		} else {
			pdata = ar0820_hdr_3exp_30fps_init_setting;
			setting_size = sizeof(ar0820_hdr_3exp_30fps_init_setting)/sizeof(uint8_t);
			vin_info("hdr 3exp pwl config mode!\n");
		}
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}

	// pll setting
	if(sensor_info->config_index & XO_25MHZ) {
		if (deserial_if &&
			(!strcmp(deserial_if->deserial_name, "max96712") ||
			!strcmp(deserial_if->deserial_name, "max96722") ||
			(!strcmp(deserial_if->deserial_name, "max9296") &&
			((sensor_info->extra_mode & 0xff) == GALAXY || desport_num > 1)))) {
			pdata = ar0820_pll_multiplier_hvkeep;
			setting_size = sizeof(ar0820_pll_multiplier_hvkeep)/sizeof(uint8_t);
		} else {
			pdata = ar0820_pll_multiplier;
			setting_size = sizeof(ar0820_pll_multiplier)/sizeof(uint8_t);
		}
		vin_info("25M pll settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// the special setting for weisen ar0820
	if((sensor_info->extra_mode & 0xff) == WEISEN ||
		(sensor_info->extra_mode & 0xff) == GALAXY) {
		/* filp and mirror disable */
		pdata = ar0820_filp_mirror_disable;
		setting_size = sizeof(ar0820_filp_mirror_disable)/sizeof(uint8_t);
		vin_info("disable flip and mirror!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
		/* pwl setting */
		if ((sensor_info->extra_mode & 0xff) == GALAXY) {
			pdata = ar0820_hdr_3exp_galaxy_pwl_setting;
			setting_size = sizeof(ar0820_hdr_3exp_galaxy_pwl_setting)/sizeof(uint8_t);
		}
		/* awb init setting */
		if ((sensor_info->extra_mode & 0xff) == WEISEN ||
			(sensor_info->extra_mode & 0xff) == SENSING) {
			/* xj3 / j5 */
			pdata = ar0820_awb_init_setting;
			setting_size = sizeof(ar0820_awb_init_setting)/sizeof(uint8_t);
		}
		vin_info("awb init setting!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// test pattern enable
	if(sensor_info->config_index & TEST_PATTERN) {
		pdata = ar0820_test_pattern;
		setting_size = sizeof(ar0820_test_pattern)/sizeof(uint8_t);
		vin_info("test pattern init!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}

	// 1080p?
	if(sensor_info->resolution == 1080) {
		if (sensor_info->config_index & SCALER_WEIGHT9331) {
			pdata = ar0820_extra_binning10_setting;
			setting_size = sizeof(ar0820_extra_binning10_setting)/sizeof(uint8_t);
		} else {
			pdata = ar0820_extra_binning_setting;
			setting_size = sizeof(ar0820_extra_binning_setting)/sizeof(uint8_t);
		}
		vin_info("1080p binning %d settint!\n", pdata[5]);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->config_index & SCALER_WEIGHT9331) {
		pdata = ar0820_weight_9331_scaling_setting;
		setting_size = sizeof(ar0820_weight_9331_scaling_setting)/sizeof(uint8_t);
		vin_info("1080p 9:3:3:1 scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->config_index & SCALER_TRUE_BAYER) {
		pdata = ar0820_true_bayer_scaling_setting;
		setting_size = sizeof(ar0820_true_bayer_scaling_setting)/sizeof(uint8_t);
		vin_info("scaler true bayer scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->config_index & SCALER_WEIGHT2110) {
		pdata = ar0820_weight_2110_scaling_setting;
		setting_size = sizeof(ar0820_weight_9331_scaling_setting)/sizeof(uint8_t);
		vin_info("scaler weight 2:1:1:0 scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}

	// fps div.
	if((sensor_info->config_index & FPS_DIV) ||
		(sensor_info->config_index & FORCE_3G)) {
		char init_d[3];
		char xts_c;
		uint32_t xts_r;
		uint32_t xts_v;

		if (deserial_if &&
			(!strcmp(deserial_if->deserial_name, "max9296")) && desport_num > 1) {
			if ((sensor_info->extra_mode & 0xff) == SENSING) {
				pdata = ar0820_pll_multiplier_lowspeed;
				setting_size = sizeof(ar0820_pll_multiplier_lowspeed)/sizeof(uint8_t);
			} else if ((sensor_info->extra_mode & 0xff) == WEISEN ||
				(sensor_info->extra_mode & 0xff) == GALAXY) {
				pdata = ar0820_pll_multiplier_lowspeed_ws;
				setting_size = sizeof(ar0820_pll_multiplier_lowspeed_ws)/sizeof(uint8_t);
			}
			vin_info("pll low speed settint!\n");
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write register error\n");
				return ret;
			}

			xts_c = 'h';
			xts_r = AR0820_HTS;
		} else {
			xts_c = 'v';
			xts_r = AR0820_VTS;
		}
		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				xts_r, init_d, 2);
		xts_v = (init_d[0] << 8) | init_d[1];
		vin_info("%dfps settint, %cts %d to %d!\n", sensor_info->fps / 2, xts_c, xts_v, xts_v * 2);
		xts_v *= 2;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				xts_r, xts_v);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}
	return ret;
}

int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	char init_d[3];
	uint32_t x0, y0, x1, y1, width, height;
	uint32_t desport_num = 0;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_VTS, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  =
							(turning_data->sensor_data.VMAX  << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_HTS, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX =
							(turning_data->sensor_data.HMAX << 8) | init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_X_START, init_d, 2);
	x0 = init_d[0];
	x0 = (x0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_Y_START, init_d, 2);
	y0 = init_d[0];
	y0 = (y0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_X_END, init_d, 2);
	x1 = init_d[0];
	x1 = (x1 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_Y_END, init_d, 2);
	y1 = init_d[0];
	y1 = (y1 << 8) | init_d[1];
	width = x1 - x0 + 1;
	height = y1 - y0 + 1;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;

	// set tuning data for different ar0820
	switch (sensor_info->extra_mode & 0xff) {
		case WEISEN:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = WEISEN_LINES_PER_SECOND;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_GR, BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		case GALAXY:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 1;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_GR, BAYER_PATTERN_RGGB);
			if (sensor_info->config_index & PWL_HDR4_24BIT) {
				turning_data->sensor_data.lines_per_second = GAHDR4_LINES_PER_SECOND;  // 156M / 1104 / 16
				sensor_data_bits_fill(&turning_data->sensor_data, 24);
			} else {
				turning_data->sensor_data.lines_per_second = GALAXY_LINES_PER_SECOND;  // 156M / 1480 / 12
				sensor_data_bits_fill(&turning_data->sensor_data, 20);
			}
			break;
		case SENSING:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 158 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = 8784;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_GR, BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);

			if (deserial_if && (!strcmp(deserial_if->deserial_name, "max9296")) &&
				(deserial_if->reserved[0] > 1) &&
				((sensor_info->extra_mode & 0xff) == SENSING))
				turning_data->sensor_data.lines_per_second = 8784 / 2;
			break;
		default:
			vin_err("don't support extra_mode %d\n", sensor_info->extra_mode);
			return -1;
	}
	return ret;
}
static int32_t sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK, i, cnt;
	uint8_t *stream_src;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	cnt = sizeof(ar0820_stream_on_setting) / 6;
	stream_src = ar0820_stream_on_setting;
	if(sizeof(turning_data->stream_ctrl.stream_on) >= (cnt * 8)) {
		for (i = 0; i < cnt; i++) {
			stream_on[i * 2] = (stream_src[i * 6 + 2] << 8) | (stream_src[i * 6 + 3]);
			stream_on[i * 2 + 1] = (stream_src[i * 6 + 4] << 8) | (stream_src[i * 6 + 5]);
		}
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	cnt = sizeof(ar0820_stream_off_setting) / 6;
	stream_src = ar0820_stream_off_setting;
	if(sizeof(turning_data->stream_ctrl.stream_off) >= (cnt * 8)) {
		for (i = 0; i < cnt; i++) {
			stream_off[i * 2] = (stream_src[i * 6 + 2] << 8) | (stream_src[i * 6 + 3]);
			stream_off[i * 2 + 1] = (stream_src[i * 6 + 4] << 8) | (stream_src[i * 6 + 5]);
		}
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;
	uint32_t open_cnt;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}

	turning_data.normal.param_hold = AR0820_PARAM_HOLD;
	turning_data.normal.param_hold_length = 2;
	turning_data.normal.s_line = AR0820_LINE;
	turning_data.normal.s_line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 4000;
	turning_data.normal.again_control_num = 3;
	turning_data.normal.again_control[0] = AR0820_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.again_control[1] = AR0820_FINE_GAIN;
	turning_data.normal.again_control_length[1] = 2;
	turning_data.normal.again_control[2] = AR0820_DGAIN;
	turning_data.normal.again_control_length[2] = 2;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	// turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	// turning_data.sensor_awb.grgain_length[0] = 2;
	// turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	// turning_data.sensor_awb.grgain_length[1] = 2;
	// turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	// turning_data.sensor_awb.grgain_length[2] = 2;
	// turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	// turning_data.sensor_awb.gbgain_length[0] = 2;
	// turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	// turning_data.sensor_awb.gbgain_length[1] = 2;
	// turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	// turning_data.sensor_awb.gbgain_length[2] = 2;
	turning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->extra_mode & 0xff) {
		case SENSING:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case WEISEN:
		case GALAXY:
			ar0820_gain.pdata = rggb_ar0820_gain;
			ar0820_gain.size = sizeof(rggb_ar0820_gain);

			ar0820_dgain.pdata = rggb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

			ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			break;
		default:
			vin_err("don't support extra_mode %d\n", sensor_info->extra_mode);
			return -1;
	}

	// set lut table
	turning_data.normal.again_lut = malloc(256*3*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.normal.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.normal.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.normal.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[512 + open_cnt], 2);
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

	return ret;
}

static int32_t sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt;
	char str[24] = {0};
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
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
	turning_data.dol2.param_hold = AR0820_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 2;

	turning_data.dol2.s_line = AR0820_LINE_S;
	turning_data.dol2.s_line_length = 2;

	turning_data.dol2.m_line = AR0820_LINE;
	turning_data.dol2.m_line_length = 2;

#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = 1 << 8;
	turning_data.dol2.line_p[0].offset = 0;
	turning_data.dol2.line_p[0].max = 19;
	turning_data.dol2.line_p[1].ratio = 1 << 8;
	turning_data.dol2.line_p[1].offset = 0;
	turning_data.dol2.line_p[1].max = 4000;

	turning_data.dol2.again_control_num = 3;
	turning_data.dol2.again_control[0] = AR0820_GAIN;
	turning_data.dol2.again_control_length[0] = 2;
	turning_data.dol2.again_control[1] = AR0820_FINE_GAIN;
	turning_data.dol2.again_control_length[1] = 2;
	turning_data.dol2.again_control[2] = AR0820_DGAIN;
	turning_data.dol2.again_control_length[2] = 2;
	turning_data.dol2.dgain_control_num = 0;
	turning_data.dol2.dgain_control[0] = 0;
	turning_data.dol2.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	turning_data.sensor_awb.grgain_length[0] = 2;
	turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	turning_data.sensor_awb.grgain_length[1] = 2;
	turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	turning_data.sensor_awb.grgain_length[2] = 2;
	turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	turning_data.sensor_awb.gbgain_length[0] = 2;
	turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	turning_data.sensor_awb.gbgain_length[1] = 2;
	turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	turning_data.sensor_awb.gbgain_length[2] = 2;
	turning_data.sensor_awb.rb_prec = 7;

	turning_data.dol2.again_lut = malloc(256*3*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*2*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut,
			rggb_ar0820_gain, sizeof(rggb_ar0820_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[open_cnt], 2);
		}
		memcpy(turning_data.dol2.again_lut + 256,
			rggb_ar0820_fine_gain, sizeof(rggb_ar0820_fine_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_fine_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.dol2.again_lut + 512,
			rggb_ar0820_dgain, sizeof(rggb_ar0820_dgain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_dgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[512 + open_cnt], 2);
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}
	if (turning_data.dol2.again_lut)
		free(turning_data.dol2.again_lut);
	if (turning_data.dol2.dgain_lut)
		free(turning_data.dol2.dgain_lut);


	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt = 0;
	char str[24] = {0};
	sensor_turning_data_t turning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
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
	turning_data.pwl.param_hold = AR0820_PARAM_HOLD;
	turning_data.pwl.param_hold_length = 2;
	turning_data.pwl.line = AR0820_LINE;
	turning_data.pwl.line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.pwl.line_p.ratio = 1 << 8;
	turning_data.pwl.line_p.offset = 0;
	turning_data.pwl.line_p.max = 4000;

	turning_data.pwl.again_control_num = 3;
	turning_data.pwl.again_control[0] = AR0820_GAIN;
	turning_data.pwl.again_control_length[0] = 2;
	turning_data.pwl.again_control[1] = AR0820_FINE_GAIN;
	turning_data.pwl.again_control_length[1] = 2;
	turning_data.pwl.again_control[2] = AR0820_DGAIN;
	turning_data.pwl.again_control_length[2] = 2;
	turning_data.pwl.dgain_control_num = 0;
	turning_data.pwl.dgain_control[0] = 0;
	turning_data.pwl.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	turning_data.sensor_awb.grgain_length[0] = 2;
	turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	turning_data.sensor_awb.grgain_length[1] = 2;
	turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	turning_data.sensor_awb.grgain_length[2] = 2;
	turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	turning_data.sensor_awb.gbgain_length[0] = 2;
	turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	turning_data.sensor_awb.gbgain_length[1] = 2;
	turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	turning_data.sensor_awb.gbgain_length[2] = 2;
	if (sensor_info->config_index & PWL_HDR4_24BIT) {
		turning_data.sensor_awb.bgain_addr[3] = 0x35b2;
		turning_data.sensor_awb.bgain_length[3] = 2;
		turning_data.sensor_awb.rgain_addr[3] = 0x35b4;
		turning_data.sensor_awb.rgain_length[3] = 2;
		turning_data.sensor_awb.grgain_addr[3] = 0x35b0;
		turning_data.sensor_awb.grgain_length[3] = 2;
		turning_data.sensor_awb.gbgain_addr[3] = 0x35b6;
		turning_data.sensor_awb.gbgain_length[3] = 2;
	}
	turning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->extra_mode & 0xff) {
		case SENSING:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case WEISEN:
			ar0820_gain.pdata = rggb_ar0820_gain;
			ar0820_gain.size = sizeof(rggb_ar0820_gain);

			ar0820_dgain.pdata = rggb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

			ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			break;
		case GALAXY:
			if (sensor_info->config_index & PWL_HDR4_24BIT) {
				ar0820_gain.pdata = rggb_ar0820_hdr4_gain;
				ar0820_gain.size = sizeof(rggb_ar0820_hdr4_gain);

				ar0820_dgain.pdata = rggb_ar0820_dgain;
				ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

				ar0820_fine_gain.pdata = rggb_ar0820_hdr4_fine_gain;
				ar0820_fine_gain.size = sizeof(rggb_ar0820_hdr4_fine_gain);
			} else {
				ar0820_gain.pdata = rggb_ar0820_gain;
				ar0820_gain.size = sizeof(rggb_ar0820_gain);

				ar0820_dgain.pdata = rggb_ar0820_dgain;
				ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

				ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
				ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			}
			break;

		default:
			vin_err("don't support extra_mode %d\n", sensor_info->extra_mode);
			return -1;
	}

	// set lut table
	turning_data.pwl.again_lut = malloc(256*3*sizeof(uint32_t));
	if (turning_data.pwl.again_lut != NULL) {
		memset(turning_data.pwl.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.pwl.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.pwl.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, ret, ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);
		turning_data.pwl.again_lut = NULL;
	}
	return ret;
}

int sensor_ar0820_serializer_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	/* sensing ser is 9295, can not map from ser pipez to des pipez */
	if (sensor_info->extra_mode == SENSING &&
		!strcmp(deserial_if->deserial_name, "max96718") &&
		sensor_info->deserial_port == 1) {
		pdata = serializer_pipey_setting;
		setting_size = sizeof(serializer_pipey_setting) / sizeof(uint8_t);
	} else {
		pdata = serializer_pipez_setting;
		setting_size = sizeof(serializer_pipez_setting) / sizeof(uint8_t);
	}
	ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
	if (ret < 0) {
		vin_err("serializer_setting failed for port%d\n",
			sensor_info->port);
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->serial_addr, REG_ALIAS_ID_SER, DEFAULT_SER_ADDR);
		if (ret < 0) {
			vin_err("set alias id to default failed for port%d\n",
				sensor_info->port);
		}
		return ret;
	}
	if ((!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) &&
		(sensor_info->deserial_port == 1)) {
			vin_info("set patch for max9296 or max96718 second port\n");
			pdata = max9296_dual_setting_patch;
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				pdata[4] = 0x11;
			}
			setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
			ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("max9296_dual_setting_patch failed\n");
				return ret;
			}
	}

	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t);
	ret = write_j5_register(deserial_if->bus_num,
		alias_id_setting[sensor_info->deserial_port], setting_size);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}

	usleep(5000);
	return ret;
}

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	ret = ar0820_init(sensor_info);
	if(ret < 0) {
		vin_err("ar0820_init fail!\n");
		return ret;
	}
	vin_info("ar0820_init OK!\n");

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			ret = sensor_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_linear_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:
			ret = sensor_dol2_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case PWL_M:
			ret = sensor_pwl_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_pwl_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
		    vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}

	if (ret < 0) {
		vin_err("sensor_%s_data_init %s fail\n", sensor_info->sensor_name,
			(sensor_info->sensor_mode != PWL_M) ? "linear" : "pwl");
		return ret;
	}
	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t entry_num = sensor_info->entry_num;
	pthread_t t1;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port, drop\n", __func__);
	} else {
		if(sensor_info->sen_devfd <= 0) {
			char str[24] = {0};

			snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
			if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
				vin_err("port%d: %s open fail\n", sensor_info->port, str);
				return -RET_ERROR;
			}
		}
		vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
				sensor_info->dev_port, sensor_info->sen_devfd);
	}

	req = hb_vin_mipi_pre_request(entry_num, 0, 0);
	if (req == 0) {
		vin_info("Sensor %s des start init \n", sensor_info->sensor_name);
		ret = deserial_setting(sensor_info);
		hb_vin_mipi_pre_result(entry_num, 0, ret);
		if (ret < 0) {
			vin_err("Sensor %s init fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
	ret = common_link_switch(sensor_info, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("link switch to port_%d failed\n", sensor_info->deserial_port);
		return ret;
	}
	if (sensor_info->config_index & FORCE_3G &&
		!strcmp(deserial_if->deserial_name, "max9296")) {
		ret = common_rx_rate_switch(sensor_info, 0);
		if (ret < 0) {
			vin_err("ar0820 set 6g rx rate fail");
			return ret;
		}
		// usleep(100*1000);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, SERIAL_ADDR,
			SER_RATE_REG, SER_3G_VAL);
		if (ret < 0) {
			vin_err("write serializer rate reg failed\n");
			return -1;
		}
		ret = common_rx_rate_switch(sensor_info, 1);
		if (ret < 0) {
			vin_err("ar0820 set 3g rx rate fail");
			return ret;
		}
	}
	usleep(100*1000);
	ret = sensor_ar0820_serializer_init(sensor_info);
	if (ret < 0) {
		vin_err("sensor_ar0820_serializer_init fail\n");
		return ret;
	}
	if ((strcmp(deserial_if->deserial_name, "max9296") &&
		 strcmp(deserial_if->deserial_name, "max96718")) ||
		sensor_info->deserial_port != 0) {
		ret = common_link_switch(sensor_info, LINK_ALL);
		if (ret < 0) {
			vin_err("switch to link all failed for port%d\n",
				sensor_info->port);
		}
		usleep(100*1000);
	}
	/* serial sync config */
	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		setting_size = sizeof(max9295_trigger_setting) / sizeof(uint32_t) / 2;
		vin_dbg("write serial: %d@0x%2x max9295 trig\n",
				sensor_info->bus_num, sensor_info->serial_addr);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
				setting_size, max9295_trigger_setting);
		if (ret < 0) {
			vin_err("write max9295_trig_setting error\n");
		}
	}
	vin_info("0820 serializer init done\n");
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, req;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus = 0, deserial_addr = 0;
	int32_t trigger_gpio = 0;
	int32_t entry_num = sensor_info->entry_num;

	if (deserial_if != NULL) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
	}

	if ((sensor_info->extra_mode & 0xff) == SENSING ||
		(sensor_info->extra_mode & 0xff) == WEISEN) {
		trigger_gpio = 3;
	} else if ((sensor_info->extra_mode & 0xff) == GALAXY) {
		trigger_gpio = 2;
	}
	if (sensor_info->config_index & TRIG_STANDARD) {
		vin_info("standard trigger gpio%d\n", trigger_gpio);
		setting_size = sizeof(ar0820_trigger_standard_setting)/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_standard_setting[i*2],
				ar0820_trigger_standard_setting[i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      sensor_info->sensor_addr, ar0820_trigger_standard_setting[i*2],
				  ar0820_trigger_standard_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : standard trigger %s fail\n", __LINE__,
				       sensor_info->sensor_name);
				return ret;
			}
		}
		setting_size = sizeof(ar0820_trigger_gpio_setting[trigger_gpio])/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				  ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : standard trigger %s gpio%d fail\n", __LINE__,
				       sensor_info->sensor_name, trigger_gpio);
				return ret;
			}
		}
	} else if (sensor_info->config_index & TRIG_SHUTTER_SYNC) {
		vin_info("shutter sync gpio%d\n", trigger_gpio);
		setting_size = sizeof(ar0820_trigger_shutter_sync_setting)/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_shutter_sync_setting[i*2],
				ar0820_trigger_shutter_sync_setting[i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      sensor_info->sensor_addr, ar0820_trigger_shutter_sync_setting[i*2],
				  ar0820_trigger_shutter_sync_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : shutter sync trigger %s fail\n",
				        __LINE__, sensor_info->sensor_name);
				return ret;
			}
		}
		setting_size = sizeof(ar0820_trigger_gpio_setting[trigger_gpio])/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				  ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			if (ret < 0) {
				vin_err("port%d : shutter sync trigger %s gpio%d fail\n",
				       sensor_info->port, sensor_info->sensor_name, trigger_gpio);
				return ret;
			}
		}
	} else {
		pdata = ar0820_stream_on_setting;
		setting_size = sizeof(ar0820_stream_on_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}
	if (deserial_if) {
		req = hb_vin_mipi_pre_request(entry_num, 1, 0);
		if (req == 0) {
			ret = sensor_serdes_stream_on(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
				vin_err("%d : %s sensor_ar0820_serdes_stream_on fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
			hb_vin_mipi_pre_result(entry_num, 1, ret);
		}
	}
	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus = 0, deserial_addr = 0;

	if (deserial_if != NULL) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
	}

	/* xj3 */
	if ((sensor_info->config_index & TRIG_SHUTTER_SYNC) ||
		(sensor_info->config_index & TRIG_STANDARD)) {
		pdata = ar0820_sync_stream_off_setting;
		setting_size = sizeof(ar0820_sync_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else {
		loop_udelay(STOP_DELAY_TIME_US);
		pdata = ar0820_stream_off_setting;
		setting_size = sizeof(ar0820_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}
	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t gpio;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

int get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	int ret = RET_OK;
	uint64_t extclk;
	uint32_t x0, y0, x1, y1;
	int vt_pix_clk_div, vt_sys_clk_div, pre_pll_clk_div, pll_multiplier;

	if (!sp || !si) {
		pr_err("input sp|si is null!\n");
		return -RET_ERROR;
	}

	int i2c_num = si->bus_num;
	int i2c_addr = si->sensor_addr;
	sp->frame_length = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			AR0820_VTS);
	sp->line_length = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			AR0820_HTS);

	x0 = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_X_START);
	y0 = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_Y_START);
	x1 = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_X_END);
	y1 = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_Y_END);

	sp->width = x1 - x0 + 1;
	sp->height = y1 - y0 + 1;

	vt_pix_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_PIX_CLK_DIV);
	vt_sys_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_SYS_CLK_DIV) & 0x1F;
	pre_pll_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PRE_PLL_CLK_DIV);
	pll_multiplier = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PLL_MULTIPLIER);

	if(si->config_index & XO_25MHZ) {
		extclk = 25000000;   // 25M
		sp->lines_per_second = WEISEN_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_WISSEN, sizeof(sp->version));
	} else {
		extclk = 27000000;   // 27M
		sp->lines_per_second = INCEPTIO_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));
	}

	switch (si->extra_mode & 0xff) {
		case GALAXY:
			if (si->config_index & PWL_HDR4_24BIT) {
				sp->lines_per_second = GAHDR4_LINES_PER_SECOND;  // 156M / 1104 / 16
			} else {
				sp->lines_per_second = GALAXY_LINES_PER_SECOND;  // 156M / 1480 / 12
			}
			break;
		default:
			break;
	}

	sp->pclk = (extclk * pll_multiplier) / (pre_pll_clk_div *
			vt_sys_clk_div * vt_pix_clk_div);

	sp->exp_num = ((hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
				REG_EXP_NUM) & 0xC) >> 2) + 1;

	sp->fps = ((float)(2*sp->pclk)) / (sp->line_length *
			sp->exp_num * sp->frame_length);
	return ret;
}

static int32_t e2prom_i2c_addr;
int32_t hb_e2prom_read_data(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint64_t *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val &= 0xff;
		ret <<= 8;
		ret |= val;
	}
	*data = ret;
	return RET_OK;
}

int32_t hb_e2prom_read_double(int32_t i2c_num, int32_t base_addr, double *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 7; i >= 0; i --) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val &= 0xff;
		ret <<= 8;
		ret |= val;
	}
	*data = *((double*)&ret);
	return RET_OK;
}

int32_t hb_e2prom_read_img_info(int32_t i2c_num, int32_t base_addr, uint64_t *data)
{
	int32_t val, ret = 0;

	val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
			base_addr);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr, val);
		return -RET_ERROR;
	}
	val &= 0xff;
	ret = val;

	val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
			base_addr + 1);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr + 1, val);
		return -RET_ERROR;
	}
	ret *= 100;
	ret += val;

	*data = ret;

	return RET_OK;
}

int32_t hb_e2prom_read_array(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint8_t *data)
{
	int32_t i, val, ret = 0;
	for (i = 0; i < i2c_num - 1; i ++) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		data[i] = val;
	}
	return RET_OK;
}

int32_t get_intrinsic_params(sensor_info_t *si,
		sensor_intrinsic_parameter_t *sip)
{
	int32_t i2c_num;
	uint64_t data;
	uint8_t serial_num[40] = {0};
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if;
	uint8_t eeprom_addr_alias_id;

	if (!sip || !si) {
		vin_err("input sip|si is null!\n");
		return -RET_ERROR;
	}
	i2c_num = si->bus_num;
	deserial_if = si->deserial_info;
	/* need set eeprom addr */
	eeprom_addr_alias_id = EEPROM_I2C_ADDR_ALIAS_ID + si->port;
	if (si->eeprom_addr == 0) {
		e2prom_i2c_addr = eeprom_addr_alias_id;
	} else {
		e2prom_i2c_addr = si->eeprom_addr;
	}
	if (e2prom_i2c_addr != eeprom_addr_alias_id)
		vin_warn("The eeprom_addr is not default (0x%x)\n", e2prom_i2c_addr);

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));

	if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_WIDTH_ADDR, &data)) < 0)
		return ret;
	sip->image_width = (uint16_t)data;

	if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_HEIGHT_ADDR, &data)) < 0)
		return ret;
	sip->image_height = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MAJOR_VERSION_ADDR, &data)) < 0)
		return ret;
	sip->major_version = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MINOR_VERSION_ADDR, &data)) < 0)
		return ret;
	sip->minor_version = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, VENDOR_ID_ADDR, &data)) < 0)
		return ret;
	sip->vendor_id = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, MODULE_ID_ADDR, &data)) < 0)
		return ret;
	sip->module_id = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 4, MODULE_SERIAL_ADDR, &data)) < 0)
		return ret;
	sip->module_serial = (uint32_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, YEAR_ADDR, &data)) < 0)
		return ret;
	sip->year = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MONTH_ADDR, &data)) < 0)
		return ret;
	sip->month = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DAY_ADDR, &data)) < 0)
		return ret;
	sip->day = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR, &data)) < 0)
		return ret;
	sip->cam_type = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MODULE_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->module_falg = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, EFL_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->efl_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, COD_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->cod_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, PP_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->pp_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORTION_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->distortion_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_PARAMS_ADDR, &data)) < 0)
		return ret;
	sip->distort_params = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_MODEL_TYPE_ADDR, &data)) < 0)
		return ret;
	sip->distort_model_type = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 4, CRC32_1_ADDR, &data)) < 0)
		return ret;
	sip->crc32_1 = (uint32_t)data;

	if ((ret = hb_e2prom_read_double(i2c_num, COD_X_ADDR, &sip->center_u)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_ADDR, &sip->center_v)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_ADDR, &sip->focal_u)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_ADDR, &sip->focal_v)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, FOV_ADDR, &sip->hfov)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, PP_X_ADDR, &sip->pp_x)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, PP_Y_ADDR, &sip->pp_y)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, CAM_SKEW_ADDR, &sip->cam_skew)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K1_ADDR, &sip->k1)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K2_ADDR, &sip->k2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, P1_ADDR, &sip->p1)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, P2_ADDR, &sip->p2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K3_ADDR, &sip->k3)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K4_ADDR, &sip->k4)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K5_ADDR, &sip->k5)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K6_ADDR, &sip->k6)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K7_ADDR, &sip->k7)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K8_ADDR, &sip->k8)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K9_ADDR, &sip->k9)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K10_ADDR, &sip->k10)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K11_ADDR, &sip->k11)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K12_ADDR, &sip->k12)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K13_ADDR, &sip->k13)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K14_ADDR, &sip->k14)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_2_ADDR, &sip->focal_u_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_2_ADDR, &sip->focal_v_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_X_2_ADDR, &sip->center_u_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_2_ADDR, &sip->center_v_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K1_2_ADDR, &sip->k1_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K2_2_ADDR, &sip->k2_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K3_2_ADDR, &sip->k3_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K4_2_ADDR, &sip->k4_2)) < 0)
		return ret;

	if ((ret = hb_e2prom_read_array(i2c_num, 40, SERIAL_NUM_ADDR, sip->serial_num)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_data(i2c_num, 4, CRC32_GROUP1_ADDR, &data)) < 0)
		return ret;
	sip->crc_group1 = (uint8_t)data;

	return RET_OK;
}

int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
{
	int32_t ret = RET_OK;

	switch (type) {
	case 0:
		ret = get_sensor_info(si, &csp->sns_param);
		break;
	case 1:
		ret = get_intrinsic_params(si, &csp->sns_intrinsic_param);
		break;
	case 3:
		ret = get_sensor_info(si, &csp->sns_param);
		if (ret == RET_OK)
			ret = get_intrinsic_params(si, &csp->sns_intrinsic_param);
		break;
	default:
		vin_err("ar0820 param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

sensor_module_t ar0820s = {
	.module = "ar0820s",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.get_sns_params = get_sns_info,
};

