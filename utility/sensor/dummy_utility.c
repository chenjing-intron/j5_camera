/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[dummy]:" fmt

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

#include "inc/hb_vin.h"
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/ds960_setting.h"
#include "inc/ds954_setting.h"
#include "inc/ds953_setting.h"
#include "inc/max9296_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/hb_vin_emb_cust_info.h"

#define INVALID_DEV_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_9295_ADDR		(0x40)
#define DEFAULT_9296_ADDR		(0x48)

enum ti_config_index {
	TI_CONFIG_CS0,
	TI_CONFIG_CS1,
	TI_CONFIG_CS0P0,
	TI_CONFIG_CS1P0,
	TI_CONFIG_NUM,
};

enum max_config_index {
	MAX_CONFIG_AUTO_AOB_VC0,
	MAX_CONFIG_AUTO_AOB_VC01,
	MAX_CONFIG_AUTO_AXB_VC01,
	MAX_CONFIG_AUTO_A_VC0123,
	MAX_CONFIG_AUTO_B_VC0123,
	MAX_CONFIG_AUTO_AXB_VC0123,
	MAX_CONFIG_SPLT_AXB_VC0101,
	MAX_CONFIG_AUTO_AX4_VC0123,
	MAX_CONFIG_AUTO_BX4_VC0123,
	MAX_CONFIG_9295A_VC0,
	MAX_CONFIG_96718_A_YVC0,
	MAX_CONFIG_96718_A_ZVC0,
	MAX_CONFIG_96718_A_VC01,
	MAX_CONFIG_9295e_96718_A_VC1,
	MAX_CONFIG_9295e_96718_B_VC1,
	MAX_CONFIG_NUM,
};

static int32_t sensor_func(sensor_info_t *sensor_info, const char *func)
{
	vin_dbg("port%d: %s -- %s --\n",
		sensor_info->port, sensor_info->sensor_name, func);
	return RET_OK;
}

static void sensor_common_data_init(sensor_info_t *sensor_info,
		sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
		sizeof(turning_data->sensor_name)-1);
	return;
}

static int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	int32_t param_index = (sensor_info->format >> 16) & 0xFF;
	switch(param_index) {
	case 2: /* 9295e bypass sunny ovx8b */
			turning_data->sensor_data.HMAX = 2134;
			turning_data->sensor_data.VMAX = 1265;

			turning_data->sensor_data.active_width = 3840;
			turning_data->sensor_data.active_height = 2160;
			turning_data->sensor_data.lines_per_second = 37956;
			turning_data->sensor_data.gain_max = 8388608;
			turning_data->sensor_data.analog_gain_max = 8388608;
			turning_data->sensor_data.digital_gain_max = 8388608;
			turning_data->sensor_data.exposure_time_min = 2;
			turning_data->sensor_data.exposure_time_max = 1252;
			turning_data->sensor_data.exposure_time_long_max = 1252;
			turning_data->sensor_data.turning_type = 6;
			turning_data->sensor_data.conversion = 1;
			turning_data->sensor_data.exposure_max_bit_width = 24;
			break;
	case 1: /* ar0820 linear for zu3 */
			turning_data->sensor_data.VMAX = 0x8560;
			turning_data->sensor_data.HMAX = 0xb680;

			turning_data->sensor_data.gain_max = 255 * 8192;
			turning_data->sensor_data.analog_gain_max = 255 * 8192;
			turning_data->sensor_data.digital_gain_max = 255 * 8192;
			turning_data->sensor_data.exposure_time_min = 1;
			turning_data->sensor_data.exposure_time_max = 10000;
			turning_data->sensor_data.exposure_time_long_max = 10000;
			turning_data->sensor_data.lines_per_second = 1141;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			break;
	case 0: /* ar0820 pwl for zu3 */
	default:
			turning_data->sensor_data.VMAX = 0x4752;
			turning_data->sensor_data.HMAX = 0x2c80;

			turning_data->sensor_data.gain_max = 255 * 8192;
			turning_data->sensor_data.analog_gain_max = 255 * 8192;
			turning_data->sensor_data.digital_gain_max = 255 * 8192;
			turning_data->sensor_data.exposure_time_min = 1;
			turning_data->sensor_data.exposure_time_max = 10000;
			turning_data->sensor_data.exposure_time_long_max = 10000;
			turning_data->sensor_data.lines_per_second = 1141;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			break;
	}
	if (sensor_info->sensor_clk) {
		turning_data->sensor_data.lines_per_second = sensor_info->sensor_clk;  // 156M / 4440
	}
	sensor_data_bayer_fill(&turning_data->sensor_data, sensor_info->format & 0xFF,
		(sensor_info->format >> 8) & 0xF, (sensor_info->format >> 12) & 0xF);
	sensor_data_bits_fill(&turning_data->sensor_data, (sensor_info->format >> 24) & 0xFF);

	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;

	vin_dbg("%s %dx%d vmax:%d hmax:%d l:%d x:%d"
#ifndef COMP_XJ3_CAM
		" b:%d o:%d p:%d"
#endif
		"\n", __func__,
		turning_data->sensor_data.active_width, turning_data->sensor_data.active_height,
		turning_data->sensor_data.VMAX, turning_data->sensor_data.HMAX,
		turning_data->sensor_data.lines_per_second, param_index
#ifndef COMP_XJ3_CAM
		, turning_data->sensor_data.data_width,
		turning_data->sensor_data.bayer_start, turning_data->sensor_data.bayer_pattern
#endif
		);

	return ret;
}

static int32_t sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;

	memset(&turning_data->stream_ctrl, 0, sizeof(turning_data->stream_ctrl));

	return ret;
}

static int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);

	turning_data.normal.param_hold = 0;
	turning_data.normal.param_hold_length = 0;
	turning_data.normal.s_line = 0;
	turning_data.normal.s_line_length = 0;

	sensor_stream_control_set(&turning_data);

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, sensor_info->dev_port, ret);
		return -RET_ERROR;
	}
	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);

	turning_data.pwl.param_hold = 0;
	turning_data.pwl.param_hold_length = 0;
	turning_data.pwl.line = 0;
	turning_data.pwl.line_length = 0;

	sensor_stream_control_set(&turning_data);

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, sensor_info->dev_port, ret);
		return -RET_ERROR;
	}

	return ret;
}

static int32_t sensor_calib_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	char str[24];

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}

	sensor_func(sensor_info, __func__);
	if(sensor_info->sen_devfd <= 0) {
		snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}

	switch (sensor_info->sensor_mode) {
		case NORMAL_M:
			ret = sensor_linear_data_init(sensor_info);
			break;
		case PWL_M:
			ret = sensor_pwl_data_init(sensor_info);
			break;
		case DOL2_M:
		case DOL3_M:
		case DOL4_M:
			/* not supported sensor_mode */
			vin_err("dummy not support sensor_mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
		default:
			/* invalid sensor_mode: ignore */
			break;
	}

	return ret;
}

void ti_setting_modify(uint32_t *pdata, int32_t size, uint32_t reg, uint32_t v, uint32_t mask)
{
	int32_t i;

	mask = (mask) ? (mask) : 0xffffffff;
	for (i = 0; i < size; i += 2) {
		if (pdata[i] == reg) {
			pdata[i + 1] &= ~mask;
			pdata[i + 1] |= v;
			vin_dbg("modify ti serdes: 0x%2x=0x%02x\n", pdata[i], pdata[i + 1]);
			break;
		}
	}
}

void max_setting_modify(uint8_t *pdata, int32_t size, uint8_t addr, uint16_t reg, uint16_t v, uint16_t mask)
{
	int32_t i, len;
	uint16_t r;

	mask = (mask) ? (mask) : 0xffff;
	v &= mask;
	for (i = 0; i < size;) {
		len = pdata[i];
		if (len == 0) {
			i += 2;
			continue;
		}
		if (pdata[i + 1] == (addr << 1)) {
			switch (len) {
			case 4:
			case 5:
				r = (pdata[i + 2] << 8) | (pdata[i + 3]);
				break;
			case 3:
				r = pdata[i + 2];
				break;
			case 0:
			default:
				r = 0xffff;
				break;
			}
			if (r == reg) {
				switch (len) {
				case 5:
					pdata[i + 4] &= ~(mask >> 8);
					pdata[i + 4] |= v >> 8;
					pdata[i + 5] &= ~(mask & 0xff);
					pdata[i + 5] |= v & 0xff;
					v = (pdata[i + 4] << 8) | pdata[i + 5];
					break;
				case 4:
					pdata[i + 4] &= ~(mask & 0xff);
					pdata[i + 4] |= v & 0xff;
					v = pdata[i + 4];
					break;
				case 3:
					pdata[i + 3] &= ~(mask & 0xff);
					pdata[i + 3] |= v & 0xff;
					v = pdata[i + 3];
					break;
				case 0:
				default:
					break;
				}
				vin_dbg("modify max serdes(0x%02x) 0x%03x=0x%02x\n", addr, r, v);
				break;
			}
		}
		i += (len + 1);
	}
}

int32_t write_register(int32_t bus, int32_t deserial_addr, int32_t serial_addr, int32_t poc_addr,
				   uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int32_t i, len;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_9296_ADDR) {
				if (deserial_addr == INVALID_DEV_ADDR) {
					vin_dbg("-drop max des %d@0x%02x: 0x%04x=0x%02x\n",
						bus, i2c_slave, reg_addr, value);
					i = i + len + 1;
					continue;
				}
				i2c_slave = deserial_addr;
				vin_dbg("write max des %d@0x%02x: 0x%04x=0x%02x\n",
						bus, i2c_slave, reg_addr, value);
			} else if (serial_addr != 0 && i2c_slave == DEFAULT_9295_ADDR) {
				if (serial_addr == INVALID_DEV_ADDR) {
					vin_dbg("-drop max ser %d@0x%02x: 0x%04x=0x%02x\n",
						bus, i2c_slave, reg_addr, value);
					i = i + len + 1;
					continue;
				}
				i2c_slave = serial_addr;
				vin_dbg("write max ser %d@0x%02x: 0x%04x=0x%02x\n",
						bus, i2c_slave, reg_addr, value);
			} else {
				vin_dbg("write max dev %d@0x%02x: 0x%04x=0x%02x\n",
						bus, i2c_slave, reg_addr, value);
			}
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write max serdes %d@0x%02x: 0x%04x=0x%02x error %d\n",
						bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			i = i + len + 1;
		} else if (len == 3) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = pdata[i + 2];
			value = pdata[i + 3];
			if (poc_addr != INVALID_DEV_ADDR) {
				if ((poc_addr != 0) && (i2c_slave == DEFAULT_POC_ADDR))
					i2c_slave = poc_addr;
				ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
				if (ret < 0) {
					vin_err("write max poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				vin_dbg("write max poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
			} else {
				if ((serial_addr != INVALID_DEV_ADDR) && (reg_addr == 0x01 && value == 0x00)) {
					/* reset all serials replace to poc off */
					for (i2c_slave = DEFAULT_9295_ADDR; i2c_slave < (DEFAULT_9295_ADDR + 1); i2c_slave++) {
						vin_dbg("reset max serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
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

static int32_t sensor_des_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);
	const char *tides_csi_speed[] = { "1.6G", "1.2G", "800M", "400M" };
	const char *des_desp = "";
	int32_t setting_size = 0;
	uint32_t *setting_array;
	uint8_t *pdata;
	uint32_t des_speed, des_ismax = 0;

	if (des->deserial_addr == INVALID_DEV_ADDR)
		return 0;

	vin_dbg("deserial: %s[%d] %s%d 0x%02x\n", des->deserial_name,
			sensor_info->deserial_index, (des->bus_type) ? "spi" : "i2c",
			des->bus_num, des->deserial_addr);
	/* init deserial and serial if need */
	if (!strcmp(des->deserial_name, "s954")) {
		setting_array = ds954_dummy_init_setting;
		setting_size = sizeof(ds954_dummy_init_setting)/sizeof(uint32_t)/2;
	} else if (!strcmp(des->deserial_name, "s960")) {
		switch (sensor_info->config_index) {
			case TI_CONFIG_CS0:
				/* config_index=0: DVB port0~3->csi0 vc0~3 */
				des_desp = "_cs0";
				setting_array = ds960_dummy_init_cs0_setting;
				setting_size = sizeof(ds960_dummy_init_cs0_setting)/sizeof(uint32_t)/2;
				break;
			case TI_CONFIG_CS1:
				/* config_index=1: DVB port0~3->csi1 vc0~3 */
				des_desp = "_cs1";
				setting_array = ds960_dummy_init_cs1_setting;
				setting_size = sizeof(ds960_dummy_init_cs1_setting)/sizeof(uint32_t)/2;
				break;
			case TI_CONFIG_CS0P0:
				/* config_index=2: DVTB port0->csi0 vcx */
				des_desp = "_cs0p0";
				setting_array = ds960_dummy_init_cs0p0_setting;
				setting_size = sizeof(ds960_dummy_init_cs0p0_setting)/sizeof(uint32_t)/2;
				break;
			case TI_CONFIG_CS1P0:
				/* config_index=3: DVTB port0->csi1 vcx */
				des_desp = "_cs1p0";
				setting_array = ds960_dummy_init_cs1p0_setting;
				setting_size = sizeof(ds960_dummy_init_cs1p0_setting)/sizeof(uint32_t)/2;
				break;
			case TI_CONFIG_NUM:
			default:
				vin_err("dummy %s config_index %d not support\n",
					   des->deserial_name, sensor_info->config_index);
				return -RET_ERROR;
		}
	} else if (!strcmp(des->deserial_name, "max9296")) {
		des_ismax = 1;
		des_desp = "_reset";
		pdata = max9296_dummy_reset_preinit_setting;
		setting_size = sizeof(max9296_dummy_reset_preinit_setting)/sizeof(uint8_t);
	} else if (!strcmp(des->deserial_name, "max96718")) {
		des_ismax = 1;
		des_desp = "_reset";
		pdata = max96718_dummy_reset_preinit_setting;
		setting_size = sizeof(max96718_dummy_reset_preinit_setting)/sizeof(uint8_t);
	} else {
		vin_err("deserial %s not support error\n", des->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if (des_ismax) {
		vin_dbg("preinit %s%s size %d\n", des->deserial_name, des_desp, setting_size);
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
	} else {
		/* ti 954/960 csi speed: 0x1f */
		if (sensor_info->extra_mode <= 3)
			ti_setting_modify(setting_array, setting_size, 0x1f, sensor_info->extra_mode, 0);
		des_speed = setting_array[3];
		vin_dbg("init %s%s %s size %d\n", des->deserial_name, des_desp,
				tides_csi_speed[des_speed], setting_size);
		ret = vin_write_array(des->bus_num, des->deserial_addr, 1,
				setting_size, setting_array);
	}

	if (ret < 0) {
		vin_err("write %c%s_dummy_init%s_setting error\n",
			(des_ismax) ? ' ' : 'd', des->deserial_name, des_desp);
		return ret;
	}

	return ret;
}

static void sensor_maxser_fix(sensor_info_t *sensor_info, uint8_t *pdata, int32_t setting_size)
{
	uint8_t fmt, dt1, dt2;
	uint8_t dt_list[10] = { 0, 0x12, 0x18, 0x1A, 0x1C, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F };

	/* frmat fix */
	fmt = sensor_info->format & 0xff;
	if (fmt >= 0x28 && fmt <= 0x2f) {
		/* max9295 data type for raw: 0x314 0x316 0x318 0x31a */
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x314, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x316, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x318, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x31a, 0x40 | fmt, 0);
	} else if (fmt >= 0x18 && fmt <= 0x1f) {
		/* max9295 data type for yuv: 0x315 0x317 0x319 0x31b */
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x315, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x317, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x319, 0x40 | fmt, 0);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x31b, 0x40 | fmt, 0);
	}

	/* extra_mode: dt1 * 1000, dt2 * 10000 */
	dt1 = (sensor_info->extra_mode % 10000) / 1000;
	if (dt1) {
		/* max9295 datatype: 0x316/0x318 */
		dt1 = dt_list[dt1];
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x316, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x318, dt1, 0x3f);
	}
	dt2 = (sensor_info->extra_mode % 100000) / 10000;
	if (dt2) {
		/* max9295 datatype: 0x317/0x319 */
		dt2 = dt_list[dt2];
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x317, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x319, dt2, 0x3f);
	}

	return;
}

static void sensor_maxdes_fix(sensor_info_t *sensor_info, uint8_t *pdata, int32_t setting_size)
{
	uint8_t fmt, speed, dt1, dt2;
	uint8_t dt_list[10] = { 0, 0x12, 0x18, 0x1A, 0x1C, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F };

	/* frmat fix */
	fmt = sensor_info->format & 0xff;
	if (fmt >= 0x28 && fmt <= 0x2f) {
		uint8_t fmt_raw[8] = {6, 7, 8, 10, 12, 14, 16, 20};
		/* max9296 data type for raw: 0x40d/0x40e 0x44d/0x44e 0x48d/0x48e 0x4cd/0x4ce */
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40d, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40e, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44d, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44e, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48d, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48e, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4cd, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4ce, fmt, 0x3f);
		sensor_info->format &= ~0xff;
		sensor_info->format |= fmt_raw[fmt - 0x28];
	} else if (fmt >= 0x18 && fmt <= 0x1f) {
		uint8_t fmt_yuv[8] = {12, 10, 12, 10, 12, 14, 16, 20};
		/* max9296 data type for raw: 0x40f/0x410 0x44f/0x450 0x48f/0x490 0x4cf/0x4d0 */
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40f, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x410, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44f, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x450, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48f, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x490, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4cf, fmt, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4d0, fmt, 0x3f);
		sensor_info->format &= ~0xff;
		sensor_info->format |= fmt_yuv[fmt - 0x18];
	}

	/* extra_mode: speed * 1 */
	speed = (sensor_info->extra_mode) % 100;
	if (speed <= 25 && speed > 0) {
		/* max9296 csi speed: 0x320, 0x323 */
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x320, speed, 0x1f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x323, speed, 0x1f);
	}

	/* extra_mode: dt1 * 1000, dt2 * 10000 */
	dt1 = (sensor_info->extra_mode % 10000) / 1000;
	if (dt1) {
		/* max9296 datatype: 0x40D/0x40E, 0x44D/0x44E, 0x48D/0x48E, 0x4CD/4CE */
		dt1 = dt_list[dt1];
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40D, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40E, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44D, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44E, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48D, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48E, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4CD, dt1, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4CE, dt1, 0x3f);
	}
	dt2 = (sensor_info->extra_mode % 100000) / 10000;
	if (dt2) {
		/* max9296 datatype: 0x40F/0x410, 0x44F/0x450, 0x48F/0x490, 0x4CF/4D0 */
		dt2 = dt_list[dt2];
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x40F, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x410, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x44F, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x450, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x48F, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x490, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4CF, dt2, 0x3f);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x4D0, dt2, 0x3f);
	}

	return;
}


static int32_t sensor_maxlink_manual(sensor_info_t *sensor_info)
{
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);
	uint8_t link;
	int32_t setting_size;
	uint8_t *pdata;
	int32_t ret = RET_OK;

	/* extra_mode: link * 100 */
	link = (sensor_info->extra_mode % 1000) / 100;
	if (link <= 4 && link > 0) {
		/* max9295 max9296 link mode: 0x10 */
		pdata = max9296_max9295e_link_setting;
		setting_size = sizeof(max9296_max9295e_link_setting)/sizeof(uint8_t);
		max_setting_modify(pdata, setting_size, DEFAULT_9295_ADDR, 0x10, link, 0x13);
		max_setting_modify(pdata, setting_size, DEFAULT_9296_ADDR, 0x10, link, 0x13);
		vin_dbg("init %s link size %d\n", des->deserial_name, setting_size);
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
		if (ret < 0) {
			vin_err("write %s_max9295e_link_setting error\n", des->deserial_name);
			return ret;
		}
	}

	return ret;
}

static int32_t disable_maxsredes_heartbeat(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t dt;
	uint8_t *pdata;
	int32_t setting_size;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);

	dt = (sensor_info->extra_mode % 1000000) / 100000;
	if (dt) {
		pdata = max9295_disable_heartbeat;
		setting_size = sizeof(max9295_disable_heartbeat)/sizeof(uint8_t);
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max9295 dis heartbeat error\n");
			return ret;
		}

		pdata = max9296_disable_heartbeat;
		setting_size = sizeof(max9295_disable_heartbeat)/sizeof(uint8_t);
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max9296 dis heartbeat error\n");
			return ret;
		}
	}
	return ret;
}

static int32_t sensor_ser_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size[2];
	uint8_t *pdata[2];
	const char *des_desp[2];
	uint32_t borad_type = 0x00;

	if (!strcmp(des->deserial_name, "s954") ||
		!strcmp(des->deserial_name, "s960")) {
		setting_size[0] = sizeof(ds953_dummy_init_setting)/sizeof(uint32_t)/2;

		vin_dbg("init s953 size %d\n", setting_size[0]);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1,
				setting_size[0], ds953_dummy_init_setting);
		if (ret < 0) {
			vin_err("write ds953_dummy_init_setting error\n");
			return ret;
		}
	} else if (!strcmp(des->deserial_name, "max9296") ||
				!strcmp(des->deserial_name, "max96718")) {
		switch (sensor_info->config_index) {
			case MAX_CONFIG_AUTO_AOB_VC0:
				des_desp[0] = "_A2Y_B2Z";
				pdata[0] = max9295e_dummy_A2Y_B2Z_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_B2Z_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_YZ2VC0";
				pdata[1] = max9296_dummy_YZ2VC0_init_setting;
				setting_size[1] = sizeof(max9296_dummy_YZ2VC0_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_AOB_VC01:
				des_desp[0] = "_A2YX_B2ZU";
				pdata[0] = max9295e_dummy_A2YX_B2ZU_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2YX_B2ZU_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_YZ2VC0_XU2VC1";
				pdata[1] = max9296_dummy_YZ2VC0_XU2VC1_init_setting;
				setting_size[1] = sizeof(max9296_dummy_YZ2VC0_XU2VC1_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_AXB_VC01:
				des_desp[0] = "_A2Y_B2Z";
				pdata[0] = max9295e_dummy_A2Y_B2Z_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_B2Z_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC0_Z2VC1";
				pdata[1] = max9296_dummy_Y2VC0_Z2VC1_init_setting;
				setting_size[1] = sizeof(max9296_dummy_Y2VC0_Z2VC1_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_A_VC0123:
				des_desp[0] = "_A2XYZU";
				pdata[0] = max9295e_dummy_A2XYZU_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2XYZU_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_XYZU2VC0123";
				pdata[1] = max9296_dummy_XYZU2VC0123_init_setting;
				setting_size[1] = sizeof(max9296_dummy_XYZU2VC0123_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_B_VC0123:
				des_desp[0] = "_B2XYZU";
				pdata[0] = max9295e_dummy_B2XYZU_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_B2XYZU_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_XYZU2VC0123";
				pdata[1] = max9296_dummy_XYZU2VC0123_init_setting;
				setting_size[1] = sizeof(max9296_dummy_XYZU2VC0123_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_AXB_VC0123:
				des_desp[0] = "_A2YX_B2ZU";
				pdata[0] = max9295e_dummy_A2YX_B2ZU_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2YX_B2ZU_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_XYxZU2VC0123";
				pdata[1] = max9296_dummy_XYxZU2VC0123_init_setting;
				setting_size[1] = sizeof(max9296_dummy_XYxZU2VC0123_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_SPLT_AXB_VC0101:
				des_desp[0] = "A2YXA_B2ZUB";
				pdata[0] = max9295e_dummy_A2YXA_B2ZUB_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2YXA_B2ZUB_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_YZ2VC0_XU2VC1";
				pdata[1] = max9296_dummy_YZ2VC0_XU2VC1_init_setting;
				setting_size[1] = sizeof(max9296_dummy_YZ2VC0_XU2VC1_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_AX4_VC0123:
				des_desp[0] = "_A2Y_B2Z";
				pdata[0] = max9295e_dummy_A2Y_B2Z_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_B2Z_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC0123";
				pdata[1] = max9296_dummy_Y2VC0123_init_setting;
				setting_size[1] = sizeof(max9296_dummy_Y2VC0123_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_AUTO_BX4_VC0123:
				des_desp[0] = "_A2Y_B2Z";
				pdata[0] = max9295e_dummy_A2Y_B2Z_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_B2Z_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Z2VC0123";
				pdata[1] = max9296_dummy_Z2VC0123_init_setting;
				setting_size[1] = sizeof(max9296_dummy_Z2VC0123_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_9295A_VC0:
				des_desp[0] = "_9295A";
				pdata[0] = max9295a_dummy_init_setting;
				setting_size[0] = sizeof(max9295a_dummy_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_YZ2VC0";
				pdata[1] = max9296_dummy_YZ2VC0_init_setting;
				setting_size[1] = sizeof(max9296_dummy_YZ2VC0_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_96718_A_YVC0:
				des_desp[0] = "_A2Y";
				pdata[0] = max9295e_dummy_A2Y_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC0";
				pdata[1] = max96718_dummy_Y2VC0_init_setting;
				setting_size[1] = sizeof(max96718_dummy_Y2VC0_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_96718_A_ZVC0:
				des_desp[0] = "_A2Y";
				pdata[0] = max9295e_dummy_A2Y_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Z2VC0";
				pdata[1] = max96718_dummy_Z2VC0_init_setting;
				setting_size[1] = sizeof(max96718_dummy_Z2VC0_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_96718_A_VC01:
				des_desp[0] = "_A2Y_";
				pdata[0] = max9295e_dummy_A2Y_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_A2Y_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC0";
				pdata[1] = max96718_dummy_Y2VC0_single_init_setting;
				setting_size[1] = sizeof(max96718_dummy_Y2VC0_single_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_9295e_96718_A_VC1:
				// des_desp[0] = "_A2YZ_";
				// pdata[0] = max9295e_dummy_bypass_A2YZ_init_setting;
				// setting_size[0] = sizeof(max9295e_dummy_bypass_A2YZ_init_setting)/sizeof(uint8_t);
				// des_desp[1] = "_Y2VC0_Z2VC1";
				// pdata[1] = max96718_dummy_bypass_Y2VC0_Z2VC1_init_setting;
				// setting_size[1] = sizeof(max96718_dummy_bypass_Y2VC0_Z2VC1_init_setting)/sizeof(uint8_t);
				des_desp[0] = "_A2Y_";
				pdata[0] = max9295e_dummy_bypass_A2Y_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_bypass_A2Y_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC1";
				pdata[1] = max96718_dummy_bypass_portA_Y2VC1_init_setting;
				setting_size[1] = sizeof(max96718_dummy_bypass_portA_Y2VC1_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_9295e_96718_B_VC1:
				des_desp[0] = "_A2Y_";
				pdata[0] = max9295e_dummy_bypass_A2Y_init_setting;
				setting_size[0] = sizeof(max9295e_dummy_bypass_A2Y_init_setting)/sizeof(uint8_t);
				des_desp[1] = "_Y2VC1";
				pdata[1] = max96718_dummy_bypass_portB_Y2VC1_init_setting;
				setting_size[1] = sizeof(max96718_dummy_bypass_portB_Y2VC1_init_setting)/sizeof(uint8_t);
				break;
			case MAX_CONFIG_NUM:
			default:
				vin_err("dummy %s config_index %d not support\n",
					   des->deserial_name, sensor_info->config_index);
				return -RET_ERROR;
		}

		if (sensor_info->serial_addr != INVALID_DEV_ADDR) {
			sensor_maxser_fix(sensor_info, pdata[0], setting_size[0]);
			vin_dbg("init max9295e_dummy%s_init_setting size %d\n", des_desp[0], setting_size[0]);
			ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
					sensor_info->serial_addr1, pdata[0], setting_size[0]);
			if (ret < 0) {
				vin_err("write max9295e_dummy%s_init_setting error\n", des_desp[0]);
				return ret;
			}
		}

		ret = sensor_maxlink_manual(sensor_info);
		if (ret < 0) {
			return ret;
		}

		if (des->deserial_addr != INVALID_DEV_ADDR) {
			sensor_maxdes_fix(sensor_info, pdata[1], setting_size[1]);
			vin_dbg("init %s%s size %d\n", des->deserial_name, des_desp[1], setting_size[1]);
			ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
					sensor_info->serial_addr1, pdata[1], setting_size[1]);
			if (ret < 0) {
				vin_err("write %s%s_init_setting error\n", des->deserial_name, des_desp[1]);
				return ret;
			}
		}

		ret = disable_maxsredes_heartbeat(sensor_info);
		if (ret < 0) {
			return ret;
		}

		if (!strcmp(des->deserial_name, "max96718")) {
			if ((sensor_info->config_index & 0xff) == MAX_CONFIG_96718_A_YVC0 ||
						(sensor_info->config_index & 0xff) == MAX_CONFIG_96718_A_ZVC0 ||
						(sensor_info->config_index & 0xff) == MAX_CONFIG_96718_A_VC01) {
				borad_type = vin_get_board_id();
				if (borad_type == BOARD_ID_MATRIXDUO_A ||
						borad_type == BOARD_ID_MATRIXDUO_A_V2 ||
						borad_type == BOARD_ID_MATRIXDUO_A_V3) {
					pdata[0] = max96718_portb_out_setting;
					setting_size[0] = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
					ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
							sensor_info->serial_addr1, pdata[0], setting_size[0]);
					if (ret < 0) {
						vin_err("write max96718_portb_out_setting error\n");
						return ret;
					}
				} else if (borad_type == BOARD_ID_MATRIXDUO_B ||
								borad_type == BOARD_ID_MATRIXDUO_B_V2 ||
								borad_type == BOARD_ID_MATRIXDSOLO_V2 ||
								borad_type == BOARD_ID_MATRIXDUO_B_V3 ||
								borad_type == BOARD_ID_MATRIXSOLO_V3) {
					pdata[0] = max96718_porta_out_setting;
					setting_size[0] = sizeof(max96718_porta_out_setting)/sizeof(uint8_t);
					ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
							sensor_info->serial_addr1, pdata[0], setting_size[0]);
					if (ret < 0) {
						vin_err("write max96718_porta_out_setting error\n");
						return ret;
					}
				}
			}
		}
	}
	return ret;
}

static int32_t sensor_des_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size = 0;
	int32_t des_ismax = 0;
	uint32_t *setting_array;
	uint8_t *pdata;
	const char *des_desp = "";

	if (des->deserial_addr == INVALID_DEV_ADDR)
		return 0;

	if (!strcmp(des->deserial_name, "s954")) {
		setting_array = ds954_dummy_start_setting;
		setting_size = sizeof(ds954_dummy_start_setting)/sizeof(uint32_t)/2;
	} else if (!strcmp(des->deserial_name, "s960")) {
		if (sensor_info->config_index == 3) {
			/* config_index=3: DVTB port0->csi1 vcx */
			des_desp = "_cs1p0";
			setting_array = ds960_dummy_start_cs1p0_setting;
			setting_size = sizeof(ds960_dummy_start_cs1p0_setting)/sizeof(uint32_t)/2;
		} else if (sensor_info->config_index == 2) {
			/* config_index=2: DVTB port0->csi0 vcx */
			des_desp = "_cs0p0";
			setting_array = ds960_dummy_start_cs0p0_setting;
			setting_size = sizeof(ds960_dummy_start_cs0p0_setting)/sizeof(uint32_t)/2;
		} else if (sensor_info->config_index == 1) {
			/* config_index=1: DVB port0~3->csi1 vc0~3 */
			des_desp = "_cs1";
			setting_array = ds960_dummy_start_cs1_setting;
			setting_size = sizeof(ds960_dummy_start_cs1_setting)/sizeof(uint32_t)/2;
		} else {
			/* config_index=0: DVB port0~3->csi0 vc0~3 */
			des_desp = "_cs0";
			setting_array = ds960_dummy_start_cs0_setting;
			setting_size = sizeof(ds960_dummy_start_cs0_setting)/sizeof(uint32_t)/2;
		}
	} else if (!strcmp(des->deserial_name, "max9296") ||
				!strcmp(des->deserial_name, "max96718")) {
			des_ismax = 1;
			des_desp = "_cs0";
			pdata = max9296_dummy_start_setting;
			setting_size = sizeof(max9296_dummy_start_setting)/sizeof(uint8_t);
	} else {
		vin_err("deserial %s not support error\n", des->deserial_name);
		return -HB_CAM_SERDES_STREAM_ON_FAIL;
	}

	vin_dbg("start %s%s size %d\n", des->deserial_name, des_desp,
			setting_size);
	if (des_ismax == 0) { /* TI des */
		ret = vin_write_array(des->bus_num, des->deserial_addr, 1,
				setting_size, setting_array);
	} else { /* MAXIN des */
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
	}

	if (ret < 0) {
		vin_err("write %c%s_dummy_start%s_setting error\n",
				(des_ismax) ? ' ' : 'd', des->deserial_name, des_desp);
		return -HB_CAM_SERDES_STREAM_ON_FAIL;
	}
	// set bypass 96718 sync flag
	if ((sensor_info->config_index == MAX_CONFIG_9295e_96718_A_VC1 ||
			sensor_info->config_index == MAX_CONFIG_9295e_96718_B_VC1) &&
			(sensor_info->serial_addr == INVALID_DEV_ADDR)) {
		ret = hb_vin_i2c_write_reg16_data8(des->bus_num, des->deserial_addr,
			BYPASS_96718_SYNC_REG, BYPASS_96178_SYNC_VAL);
		if (ret < 0) {
			vin_err("write max serdes %d@0x%02x: 0x%04x=0x%02x error %d\n",
					des->bus_num, des->deserial_addr, BYPASS_96718_SYNC_REG,
						BYPASS_96178_SYNC_VAL, ret);
			return ret;
		}
	}
	return ret;
}

static int32_t sensor_des_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size = 0;
	int32_t des_ismax = 0;
	uint32_t *setting_array;
	uint8_t *pdata;

	if (!strcmp(des->deserial_name, "s954")) {
		setting_array = ds954_dummy_stop_setting;
		setting_size = sizeof(ds954_dummy_stop_setting)/sizeof(uint32_t)/2;
	} else if (!strcmp(des->deserial_name, "s960")) {
		setting_array = ds960_dummy_stop_setting;
		setting_size = sizeof(ds960_dummy_stop_setting)/sizeof(uint32_t)/2;
	} else if (!strcmp(des->deserial_name, "max9296") ||
				!strcmp(des->deserial_name, "max96718")) {
		des_ismax = 1;
		pdata = max9296_dummy_stop_setting;
		setting_size = sizeof(max9296_dummy_stop_setting)/sizeof(uint8_t);
	} else {
		return 0;
	}

	vin_dbg("stop %s size %d\n", des->deserial_name, setting_size);
	if (des_ismax == 0) { /* TI des */
		ret = vin_write_array(des->bus_num, des->deserial_addr, 1,
				setting_size, setting_array);
	} else { /* MAXIN des */
		ret = write_register(des->bus_num, des->deserial_addr, sensor_info->serial_addr,
				sensor_info->serial_addr1, pdata, setting_size);
	}

	if (ret < 0) {
		vin_err("write %c%s_dummy_stop_setting error\n",
				(des_ismax) ? ' ' : 'd', des->deserial_name);
		return -HB_CAM_SERDES_STREAM_OFF_FAIL;
	}

	// clear bypass 96718 sync flag
	if ((sensor_info->config_index == MAX_CONFIG_9295e_96718_A_VC1 ||
			sensor_info->config_index == MAX_CONFIG_9295e_96718_B_VC1) &&
			(sensor_info->serial_addr == INVALID_DEV_ADDR)) {
		ret = hb_vin_i2c_write_reg16_data8(des->bus_num, des->deserial_addr,
			BYPASS_96718_SYNC_REG, BYPASS_96178_SYNC_VAL & 0x00);
		if (ret < 0) {
			vin_err("write max serdes %d@0x%02x: 0x%04x=0x%02x error %d\n",
					des->bus_num, des->deserial_addr, BYPASS_96718_SYNC_REG,
						BYPASS_96178_SYNC_VAL, ret);
			return ret;
		}
	}
	return ret;
}

static int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i, req;
	deserial_info_t *des;
	char mipi_cfg[128], *gpio;

	sensor_func(sensor_info, __func__);
	vin_dbg("%s: %dx%d@%dfps 0x%02x\n",
		sensor_info->sensor_name, sensor_info->height,
		sensor_info->width, sensor_info->fps,
		sensor_info->format);
	snprintf(mipi_cfg, sizeof(mipi_cfg), sensor_info->config_path,
		sensor_info->fps, sensor_info->resolution);
	vin_dbg("mipi%d: %s\n", sensor_info->entry_num, mipi_cfg);
	if (sensor_info->bus_type) {
		vin_dbg("spi%d: cs%d mode %d speed %d\n",
			sensor_info->bus_num, sensor_info->spi_info.spi_cs,
			sensor_info->spi_info.spi_mode, sensor_info->spi_info.spi_speed);
	} else {
		vin_dbg("i2c%d: 0x%02x 0x%02x width %d\n",
			sensor_info->bus_num, sensor_info->sensor_addr,
			sensor_info->sensor1_addr, sensor_info->reg_width);
	}
	if (sensor_info->serial_addr || sensor_info->serial_addr1)
		vin_dbg("serial: 0x%02x 0x%02x\n", sensor_info->serial_addr,
			sensor_info->serial_addr1);
	des = (deserial_info_t *)(sensor_info->deserial_info);
	if (des) {
		if (sensor_info->dev_port < 0)
			req = 0;
		else
			req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		if (req == 0) {
			ret = sensor_des_init(sensor_info);
			if (sensor_info->dev_port >= 0)
				hb_vin_mipi_pre_result(sensor_info->entry_num, 0, ret);
			if (ret < 0) {
				vin_err("sensor_des_init_error %d\n", ret);
				return ret;
			}
			if (!strcmp(des->deserial_name, "max9296") ||
				!strcmp(des->deserial_name, "max96718")) {
				ret = sensor_ser_init(sensor_info);
				if (ret < 0) {
					vin_err("dummy sensor_ser_init %d\n", ret);
					return ret;
				}
			}
		}
		if (!strcmp(des->deserial_name, "s954") ||
			!strcmp(des->deserial_name, "s960")) {
			ret = sensor_ser_init(sensor_info);
			if (ret < 0) {
				vin_err("dummy sensor_ser_init %d\n", ret);
				return ret;
			}
		}
	}
	vin_dbg("isp: 0x%02x, imu: 0x%02x, eep: 0x%02x\n",
		sensor_info->isp_addr, sensor_info->imu_addr,
		sensor_info->eeprom_addr);
	vin_dbg("sensor_mode: %d, extra_mode: %d, config_index: %d\n",
		sensor_info->sensor_mode, sensor_info->extra_mode,
		sensor_info->config_index);
	vin_dbg("power_mode: %d, power_delay: %d\n",
		sensor_info->power_mode, sensor_info->power_delay);
	if (sensor_info->gpio_num) {
		gpio = mipi_cfg;
		for (i = 0; i < sensor_info->gpio_num; i++)
			gpio += sprintf(gpio, "%d-%d, ", sensor_info->gpio_pin[i],
					sensor_info->gpio_level[i]);
		vin_dbg("gpio %d: %s\n", sensor_info->gpio_num, gpio);
	}
	if (sensor_info->power_delay && !sensor_info->power_mode) {
		vin_dbg("sleep %dms\n", sensor_info->power_delay);
		usleep(sensor_info->power_delay * 1000);
	}

	ret = sensor_calib_data_init(sensor_info);

	return ret;
}

static int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	sensor_func(sensor_info, __func__);

	if (sensor_info->dev_port >= 0 && sensor_info->sen_devfd > 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return RET_OK;
}

static int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);

	sensor_func(sensor_info, __func__);

	if (des) {
		ret = sensor_des_stop(sensor_info);
		if (ret < 0) {
			vin_err("dummy sensor_des_stop %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, req;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);

	sensor_func(sensor_info, __func__);

	if (des) {
		if (sensor_info->dev_port < 0)
			req = 0;
		else
			req = hb_vin_mipi_pre_request(sensor_info->entry_num, 1, 0);
		if (req == 0) {
			ret = sensor_des_start(sensor_info);
			if (sensor_info->dev_port >= 0)
				hb_vin_mipi_pre_result(sensor_info->entry_num, 1, ret);
			if (ret < 0) {
				vin_err("dummy sensor_des_start %d\n", ret);
				return ret;
			}
		}
	}
	if (sensor_info->power_delay && sensor_info->power_mode) {
		vin_dbg("sleep %dms\n", sensor_info->power_delay);
		usleep(sensor_info->power_delay * 1000);
	}

	return RET_OK;
}

static int32_t sensor_power_on(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_power_off(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_power_reset(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_extern_isp_poweron(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_extern_isp_poweroff(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_extern_isp_reset(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int32_t sensor_spi_read(sensor_info_t *sensor_info,  uint32_t reg_addr, char *buffer, uint32_t sizee)
{
	sensor_func(sensor_info, __func__);
	vin_dbg("reg_addr: 0x%02x, buffser: %p, sizee: %d\n",
			reg_addr, buffer, sizee);
	return RET_OK;
}

static int32_t sensor_spi_write(sensor_info_t *sensor_info,  uint32_t reg_addr, char  *buffer, uint32_t sizee)
{
	sensor_func(sensor_info, __func__);
	vin_dbg("reg_addr: 0x%02x, buffser: %p, sizee: %d\n",
			reg_addr, buffer, sizee);
	return RET_OK;
}

static int32_t sensor_set_awb(int32_t i2c_bus, int32_t sensor_addr, float rg_gain, float b_gain)
{
	vin_dbg("dummy -- %s --", __func__);
	vin_dbg("i2c_bus: %d, sensor_addr: 0x%02x, rg_gain: %.2f, b_gain: %.2f\n",
			i2c_bus, sensor_addr, rg_gain, b_gain);
	return RET_OK;

}

static int32_t sensor_set_ex_gain( int32_t i2c_bus, int32_t sensor_addr, uint32_t exposure_setting,
			uint32_t gain_setting_0, uint16_t gain_setting_1)
{
	vin_dbg("dummy -- %s --", __func__);
	vin_dbg("i2c_bus: %d, sensor_addr: 0x%02x, exposure_setting: %d, gain_setting_0: %d, gain_setting_1: %d\n",
			i2c_bus, sensor_addr, exposure_setting, gain_setting_0, gain_setting_1);
	return RET_OK;
}

static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	sensor_func(sensor_info, __func__);
	vin_dbg("fps: %d\n", fps);
	return RET_OK;
}

static int32_t get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	sensor_func(si, __func__);

	sp->frame_length = si->width;
	sp->line_length = si->height;
	sp->width = si->width;
	sp->height = si->height;
	sp->fps = si->fps;
	sp->pclk = si->sensor_clk;
	sp->exp_num = 1;
	sp->lines_per_second = sp->line_length * sp->fps;
	strncpy(sp->version, "1.0.0", sizeof(sp->version)-1);

	return RET_OK;
}

static int32_t get_intrinsic_params(sensor_info_t *si,
		sensor_intrinsic_parameter_t *sip)
{
	sensor_func(si, __func__);

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));
	sip->major_version = 1;
	sip->minor_version = 0;
	sip->vendor_id = 0x01;
	sip->module_id = 0x01;
	sip->module_serial = 0xABCD1234;
	sip->year = 2022;
	sip->month = 7;
	sip->day = 6;
	sip->cam_type = 1;
	sip->module_falg = 1;
	sip->efl_flag = 2;
	sip->cod_flag = 3;
	sip->pp_flag = 4;
	sip->distortion_flag = 5;
	sip->image_width = si->width;
	sip->image_height = si->height;
	strncpy(sip->serial_num, "dummy_intrinstic_test", sizeof(sip->serial_num));
	sip->pp_x = 10.0;
	sip->pp_y = 20.0;
	sip->hfov = 120.0;
	sip->k1 = 1.0;
	sip->k2 = 2.0;
	sip->k3 = 3.0;
	sip->k4 = 4.0;
	sip->k5 = 5.0;
	sip->k6 = 6.0;
	sip->k7 = 7.0;
	sip->k8 = 8.0;
	sip->k9 = 9.0;
	sip->k10 = 10.0;
	sip->k11 = 11.0;
	sip->k12 = 12.0;
	sip->k13 = 13.0;
	sip->k14 = 14.0;
	sip->p1 = 1.0;
	sip->p2 = 2.0;
	sip->k1_2 = 21.0;
	sip->k2_2 = 22.0;
	sip->k3_2 = 23.0;
	sip->k4_2 = 24.0;

	return RET_OK;
}

static int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
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
		vin_err("dummy param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

typedef struct dummy_emb_s {
	const char *name;
	uint32_t raw_size;
	uint32_t info_size;
	int32_t (*parse_embed_data)(sensor_info_t *sensor_info, char* embed_raw, embed_data_info_t* embed_info, const struct dummy_emb_s *emb);
} dummy_emb_t;

enum ISX021SyncMode
{
	ISX021_INTERNAL_SYNCHRONIZATION = 0,
	ISX021_EXTERNAL_PULSE_SYNCHRONIZATION,
	ISX021_SHUTTER_TRIGGER_SYNCHRONIZATION
};
#define FPS_30		(30u)
#define ISX_VMAX	(1400u)
#define ISX021_CALIB_TEMPERATURE (50u)
#define ISX021_DIVID_TEMPERATURE (16u)
#define ISX021_EMB_VALID_SIZE    (256u)

static int32_t isx021_parse_embed_data(sensor_info_t *sensor_info, char* embed_raw, embed_data_info_t* embed_info, const struct dummy_emb_s *emb)
{
	int32_t i;
	const uint8_t* praw = (const uint8_t* )embed_raw;
	embed_isx021_info_t isx021_info = {0};
	embed_isx021_info_t *cust = &isx021_info;

#if 0
	vin_dbg("%x ", praw[12]);
	vin_dbg("%x ", praw[13]);
	vin_dbg("%x ", praw[16]);
	vin_dbg("%x ", praw[17]);
	vin_dbg("%x ", praw[18]);
	vin_dbg("%x ", praw[19]);
	vin_dbg("%x ", praw[19]);
	vin_dbg("%x \n", praw[225] & 0xF0 );
	vin_dbg("SP1ExpTLineExtPulse %x %x %x\n", praw[30],praw[31],praw[32]);
	vin_dbg("SP2ExpTLineExtPulse %x %x %x\n", praw[33],praw[34],praw[35]);
	vin_dbg("SP1ExpTLineShutterTrigger %x %x %x\n", praw[100],praw[101],praw[102]);
	vin_dbg("SP2ExpTLineShutterTrigger %x %x %x\n", praw[103],praw[104],praw[105]);
	vin_dbg("TemperatureSensor0 %x %x %x %x\n", praw[52],praw[53],praw[54],praw[55]);
	vin_dbg("DGain %x %x %x\n", praw[140],praw[141],praw[142]);
#endif

	/* cust info parse */
	cust->DataCountInfo = praw[0] + praw[1] * 256;
	cust->TypeOfSensor = praw[224] + (praw[225] & 0xF) * 256;//225 226
	for (i = 0; i < 10; i++)
	{
		cust->HostSeletedRegs[i] =  praw[20 + i];
	}
	cust->SP1ExpTLineExtPulse = praw[30] + praw[31] * 256 + (praw[32] & 0x3) * 256 * 256;//31 33
	cust->SP2ExpTLineExtPulse = praw[33] + praw[34]* 256 + (praw[35] & 0x3) * 256* 256;//34 36
	cust->frameCount = praw[36] + praw[37] * 256 + praw[38] * 256 * 256 + praw[39] * 256 * 256 * 256; //37 -40
	cust->frameCountEnable = praw[40]; //41
	cust->FrameIDIndicate = praw[41]; // 42
	cust->FrameMaskEnable  = praw[42] & 0x01;//43
	cust->TRCTypeReplacementEnable = praw[40] & 0x02;//43
	cust->HsyncSingleMaskEnable = praw[40] & 0x04;//43
	cust->VsyncSingleMaskEnable = praw[40] & 0x08;//43
	cust->AdjLevelVsyncSignalExtFsync = praw[48] + praw[50] * 256;//49 -50 CompensationLevelInExternalPulseBasedSync
	cust->TemperatureSensor0 = praw[52] + (praw[53] & 0x0F) * 256;//53 54
	cust->TemperatureSensor1 = praw[54] + (praw[55] & 0x0F) * 256;//55 56
	cust->ExternalFsyncDetector = praw[98];//99
	cust->SP1ExpTLineShutterTrigger = praw[100] + praw[101] * 256 + (praw[102] & 0x3) * 256 * 256;//101 103
	cust->SP2ExpTLineShutterTrigger = praw[103] + praw[104] * 256 + (praw[105] & 0x3) * 256 * 256;//104 106
	cust->AGainSP1H = praw[120] + praw[121] * 256;//121 122
	cust->AGainSP1L = praw[122] + praw[123] * 256;//123 124
	cust->AGainSP2H = praw[124] + praw[125] * 256;//125 126
	cust->AGainSP2L = praw[126] + praw[127] * 256;//126 128
	cust->DGain = praw[140] + praw[141] * 256 + praw[142] * 256 * 256;//141 143
	cust->HCompenstationLevelOpticalCenter = praw[164] + praw[165] * 256; //165 166
	cust->VCompenstationLevelOpticalCenter = praw[166] + praw[167] * 256;  //168 167
	cust->HFlippingEnable = praw[168]; //169
	cust->VFlippingEnable = praw[169]; //170
	cust->HCropOffset = praw[180] + praw[181] * 256; //181 182
	cust->VCropOffset = praw[182] + praw[183] * 256; //183 184
	cust->HCropSize = praw[184] + praw[185] * 256; // 185 186
	cust->VCropSize = praw[186] + praw[187] * 256; // 187 188
	cust->DriveMode = praw[196]; //197
	cust->SyncMethod = praw[200]; //201
	cust->AutoModeEnabel = praw[201]; //202
	cust->DriveClkHorizontal = praw[204]+ praw[205] * 256 ;//205 206
	cust->DriveClkVertical = praw[208] + praw[209] * 256 + praw[210] * 256 * 256; // 209 211
	cust->NumExtFrames = praw[212]; //213
	cust->NumExtLines = praw[213] + praw[214] * 256 + praw[215] * 256 * 256;//214 216
	cust->CropEnable = praw[216];//217
	cust->PGModeEnable = praw[217];//218
	cust->HostCommunicationMaskEnable = praw[218] & 0x1;//219
	cust->ReplacementDataTypeHostCommunicationMaskEnable = praw[218] & 0x2;//219
	cust->HsyncSignalHostCommunicationMaskEnable = praw[218] & 0x4;//219
	cust->VsyncSignalHostCommunicationMaskEnable = praw[218] & 0x8;//219

	/* cust to common info */
	embed_info->port = sensor_info->port;
	embed_info->dev_port = sensor_info->dev_port;
	embed_info->frame_count = cust->frameCount;
#if 0
	embed_info->again[0] = cust->AGainSP1H;
	embed_info->again[1] = cust->AGainSP1L;
	embed_info->again[2] = cust->AGainSP2H;
	embed_info->again[3] = cust->AGainSP2L;
	embed_info->dgain[0] = cust->DGain;
#endif
	embed_info->height = sensor_info->height;
	embed_info->width = sensor_info->width;

	if (cust->SyncMethod < ISX021_SHUTTER_TRIGGER_SYNCHRONIZATION)
	{
		embed_info->line[0] = cust->SP1ExpTLineExtPulse;
		embed_info->line[1] = cust->SP2ExpTLineExtPulse;
		embed_info->exposures[0] = (uint32_t)(cust->SP1ExpTLineExtPulse * 1000000UL / FPS_30 / ISX_VMAX);
		embed_info->exposures[1] = (uint32_t)(cust->SP2ExpTLineExtPulse * 1000000UL / FPS_30 / ISX_VMAX);
	} else {
		embed_info->line[0] = cust->SP1ExpTLineShutterTrigger;
		embed_info->line[1] = cust->SP2ExpTLineShutterTrigger;
		embed_info->exposures[0] = (uint32_t)(cust->SP1ExpTLineShutterTrigger * 1000000UL / FPS_30 / ISX_VMAX);
		embed_info->exposures[1] = (uint32_t)(cust->SP2ExpTLineShutterTrigger * 1000000UL / FPS_30 / ISX_VMAX);
	}
	embed_info->num_exp = 2;
	embed_info->exposure = embed_info->exposures[0] + embed_info->exposures[1];

	embed_info->num_temp = 2;
	embed_info->temperatures[0] = (cust->TemperatureSensor0 / ISX021_DIVID_TEMPERATURE) - ISX021_CALIB_TEMPERATURE;
	embed_info->temperatures[1] = (cust->TemperatureSensor1 / ISX021_DIVID_TEMPERATURE) - ISX021_CALIB_TEMPERATURE;
	embed_info->temperature = (embed_info->temperatures[0] + embed_info->temperatures[1]) / 2;

	/* cust info */
	embed_info->cust.name = emb->name;
	if ((embed_info->cust.size >= emb->info_size) && (embed_info->cust.info != NULL)) {
		memcpy(embed_info->cust.info, cust, emb->info_size);
	}

	return 0;
}

const dummy_emb_t dummy_emb_list[] = {
	{ "isx021", ISX021_EMB_VALID_SIZE, sizeof(embed_isx021_info_t), isx021_parse_embed_data },
};
#define DUMMY_EMB_NUM	(sizeof(dummy_emb_list)/sizeof(dummy_emb_list[0]))

static int32_t sensor_parse_embed_data(sensor_info_t *sensor_info, char* embed_raw, embed_data_info_t* embed_info)
{
	int32_t i, ret;
	int32_t param_index = (sensor_info->format >> 16) & 0xFF;
	const dummy_emb_t *emb;

	if (param_index >= DUMMY_EMB_NUM) {
		vin_err("index %d emb not support\n", param_index);
		return -HB_CAM_EMB_INFO_FAIL;
	}
	emb = &dummy_emb_list[param_index];
	if ((emb->name == NULL) || (emb->parse_embed_data == NULL)) {
		vin_err("index %d emb invalid\n", param_index);
		return -HB_CAM_EMB_INFO_FAIL;
	}

	for (i = 0; i < emb->raw_size; i+=8) {
		vin_dbg("p%ds%d %s E%03d: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				sensor_info->port, sensor_info->dev_port, emb->name, i + 1,
				embed_raw[i], embed_raw[i+1], embed_raw[i+2], embed_raw[i+3],
				embed_raw[i+4], embed_raw[i+5], embed_raw[i+6], embed_raw[i+7]);
	}

	ret = emb->parse_embed_data(sensor_info, embed_raw, embed_info, emb);
	return ret;
}

sensor_module_t dummy = {
	.module = "dummy",
	.init = sensor_init,
	.deinit = sensor_deinit,
	.start = sensor_start,
	.stop = sensor_stop,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
	.power_reset = sensor_power_reset,
	.extern_isp_poweron = sensor_extern_isp_poweron,
	.extern_isp_poweroff = sensor_extern_isp_poweroff,
	.extern_isp_reset = sensor_extern_isp_reset,
	.spi_read = sensor_spi_read,
	.spi_write = sensor_spi_write,
	.set_awb = sensor_set_awb,
	.set_ex_gain = sensor_set_ex_gain,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.get_sns_params = get_sns_info,
	.parse_embed_data = sensor_parse_embed_data,
};

