/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[virtual]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
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
#include <linux/ioctl.h>
#include <sys/shm.h>

#include "inc/hb_vin.h"
#include "hb_cam_utility.h"
#include "hb_i2c.h"
#include "inc/sensor_effect_common.h"
#include "inc/virtual_setting.h"
#include "inc/sensor_common.h"

#include "cJSON.h"

#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)

#define DES_REG_ADDR_WID     (0x1067)
#define DES_REG_ADDR_HEI     (0x106B)
#define REG_ADDR_VS_LOW_1    (0x1058)
#define REG_ADDR_VS_LOW_2    (0x1059)
#define REG_ADDR_VS_LOW_3    (0x105A)
#define REG_ADDR_DT    		 (0x090E)
#define REG_DST_PORT         (0x092D)

#define DES_REG_ADDR_WID_1   (0x1097)
#define DES_REG_ADDR_HEI_1   (0x109B)
#define REG_ADDR_VS_LOW_1_1  (0x1088)
#define REG_ADDR_VS_LOW_2_1  (0x1089)
#define REG_ADDR_VS_LOW_3_1  (0x108A)
#define REG_ADDR_DT_1 		 (0x0A0E)
#define REG_DST_PORT_1       (0x0A2D)

#define DT_MASK              (0x3)
#define PORT_MASK            (0xC)
#define DT_RAW               (0)
#define DT_YUV               (1)
#define DT_RGB               (2)
#define PORT_A_EN            (BIT(2))
#define PORT_B_EN            (BIT(3))
#define VIRTUAL_CAM_MASK     (0xF << 4)
#define VIRTUAL_MAX96718_MAX9295_YUV		 ((1) << 4)

#define VS_HIGH_NUM          (0xABE0)

#define DES718_REG_ADDR_WID  (0x0257)
#define DES718_REG_ADDR_HEI  (0x025B)
#define DES718_DATA_ADDR     (0x044E)
#define DES718_PORT_ADDR     (0x046D)
#define REG718_ADDR_VS_LOW_1 (0x0248)
#define REG718_ADDR_VS_LOW_2 (0x0249)
#define REG718_ADDR_VS_LOW_3 (0x024A)

uint32_t reg_patgen_clk_src_96712[8] =
	{0x01DC, 0x01FC, 0x021C, 0x023C, 0x025C, 0x027C, 0x029C, 0x02BC};

uint32_t reg_patgen_clk_src_96718[2] =
	{0x01FC, 0x021C};

struct sif_pattern_cfg {
	uint32_t instance;
	uint32_t framerate;
};

int32_t poc_linked_first(int32_t bus, int32_t poc_addr)
{
	int32_t ret = 0, i, val;
	for (i = 0; i < 4; i++) {
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, 0x6 + i);
		usleep(2000);
		/* read again to get current state */
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, 0x6 + i);
		if (val < 0)
			break;
		/* linked on if > 150ma */
		if (val > 0x5) {
			ret = i + 1;
			break;
		}
	}
	return ret;
}

int32_t sensor_setting_array(int32_t bus, uint32_t i2c_addr, int32_t reg_width,
                         int32_t setting_size, uint16_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int32_t ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = reg_width;

	for (i = 0; i < setting_size; i++) {
		i2c_cfg.reg = cam_setting[2 * i];
		i2c_cfg.data = cam_setting[2 * i + 1];
		if (i2c_cfg.reg_size == 2)
			ret = hb_vin_i2c_write_reg16_data16(bus, i2c_cfg.i2c_addr,
				i2c_cfg.reg, i2c_cfg.data);
		else
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_cfg.i2c_addr,
				i2c_cfg.reg, i2c_cfg.data);
		k = 10;
		while (ret < 0 && k--) {
			if (k % 10 == 9)
				usleep(200 * 1000);
			if (i2c_cfg.reg_size == 2)
				ret = hb_vin_i2c_write_reg16_data16(bus, i2c_cfg.i2c_addr,
					i2c_cfg.reg, i2c_cfg.data);
			else
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_cfg.i2c_addr,
					i2c_cfg.reg, i2c_cfg.data);
		}
		if (ret < 0) {
			vin_err("camera write 0x%2x fail \n", i2c_cfg.reg);
			break;
		}
	}
	return ret;
}

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
				vin_err("write virtual %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// 	usleep(5*1000);
			i = i + len + 1;
			vin_info("write virtual %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
			// } else if (serial_addr != 0 && i2c_slave == DEFAULT_SERIAL_ADDR) {
				// i2c_slave = serial_addr;
			}
			k = 10;
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
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
			if (poc_addr != INVALID_POC_ADDR) {
				i2c_slave = pdata[i + 1] >> 1;
				reg_addr = pdata[i + 2];
				value = pdata[i + 3];
				if (poc_addr != 0 && i2c_slave == DEFAULT_POC_ADDR)
					i2c_slave = poc_addr;
				ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
				if (ret < 0) {
					vin_err("write poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				// usleep(100*1000);
				vin_info("write poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
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

/**
 * @brief sensor_virtual_des_init : write deserial init setting,
 *                                 including 96718, 9296, 96712, etc.
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_virtual_des_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	sensor_info_t **sensor_if = (sensor_info_t **)deserial_if->sensor_info;
	int32_t bus, deserial_addr;
	uint32_t base_cfg[2] = {0, 0};  // vpg only support two diff settings
	uint32_t vpg_sel[4] = {0, 0, 0, 0};
	uint32_t pclk_sel[4] = {150, 150, 150, 150};

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;

	base_cfg[0] = (sensor_if[0]->width << 20) |
		(sensor_if[0]->resolution << 8) |
		(sensor_if[0]->fps);

	if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		pdata = max96712_base_init;
		setting_size = sizeof(max96712_base_init)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max96712_base_init register error\n");
			return ret;
		}

		for (int i = 0; i < 4; ++i) {
			if (!sensor_if[i]) {
				break;
			}
			if (sensor_info->config_index & TEST_PATTERN_SERDES) {
				vin_info("%s testpattern extra_mode=%d %dx%d init\n",
						deserial_if->deserial_name, sensor_info->extra_mode,
						(sensor_info->width) ? sensor_info->width : 2048,
						(sensor_info->resolution) ? sensor_info->resolution : 1280);
				uint32_t base_cfg_tmp = 0;
				uint32_t base_addr = 0;
				base_cfg_tmp = (sensor_if[i]->width << 20) |
					(sensor_if[i]->resolution << 8) |
					(sensor_if[i]->fps);
				if (base_cfg_tmp == base_cfg[0]) {
					if (sensor_if[i]->resolution == 2160) {
						vpg_sel[i] = 2;
					} else {
						vpg_sel[i] = 0;
					}
				} else if (base_cfg_tmp == base_cfg[1]) {
					if (sensor_if[i]->resolution == 2160) {
						vpg_sel[i] = 3;
					} else {
						vpg_sel[i] = 1;
					}
				} else if (base_cfg[1] == 0) {
					base_cfg[1] = base_cfg_tmp;
					if (sensor_if[i]->resolution == 2160) {
						vpg_sel[i] = 3;
					} else {
						vpg_sel[i] = 1;
					}
				} else {
					vin_err("not support three cfgs as %x:%x:%x\n", base_cfg[0], base_cfg[1], base_cfg_tmp);
					return -1;
				}

				pdata = max96712_vpg_base_init[vpg_sel[i]];

				setting_size = sizeof(max96712_vpg_base_init[vpg_sel[i]])/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_vpg_base_init register error\n");
					return ret;
				}

				if (vpg_sel[i] & 0x1) {
					ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES_REG_ADDR_WID_1, sensor_if[i]->width);
					if (ret < 0) {
						vin_err("write %s vpg width error\n", deserial_if->deserial_name);
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES_REG_ADDR_HEI_1, sensor_if[i]->resolution);
					if (ret < 0) {
						vin_err("write %s vpg height error\n", deserial_if->deserial_name);
						return ret;
					}
				} else {
					ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES_REG_ADDR_WID, sensor_if[i]->width);
					if (ret < 0) {
						vin_err("write %s vpg width error\n", deserial_if->deserial_name);
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES_REG_ADDR_HEI, sensor_if[i]->resolution);
					if (ret < 0) {
						vin_err("write %s vpg width error\n", deserial_if->deserial_name);
						return ret;
					}
				}
				uint32_t vs_low_set = 0;
				if ((vpg_sel[i] > 1) && (sensor_if[i]->fps > 15)) {
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr,
						reg_patgen_clk_src_96712[i+ (4 * (vpg_sel[i] & 0x1))], 0x80);
					if (ret < 0) {
						vin_err("write %s vpg pclk src error\n", deserial_if->deserial_name);
						return ret;
					}
					pclk_sel[i] = 375;
				}
				vs_low_set = pclk_sel[i] * 1000000 / sensor_if[i]->fps - VS_HIGH_NUM;
				if (vpg_sel[i] & 0x1) {
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_1_1, vs_low_set >> 16);
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_2_1, (vs_low_set & 0xFF00) >> 8);
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_3_1, (vs_low_set & 0xFF));
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
				} else {
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_1, vs_low_set >> 16);
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_2, (vs_low_set & 0xFF00) >> 8);
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
					ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG_ADDR_VS_LOW_3, (vs_low_set & 0xFF));
					if (ret < 0) {
						vin_err("write vs_low error\n");
						return ret;
					}
				}

				uint32_t data_type = 0;
				if ((sensor_if[i]->extra_mode & DT_MASK) == DT_RAW) {
					data_type = 0x2C;
				} else if ((sensor_if[i]->extra_mode & DT_MASK) == DT_YUV) {
					data_type = 0x1E;
				} else if ((sensor_if[i]->extra_mode & DT_MASK) == DT_RGB) {
					data_type = 0x24;
				}
				base_addr = (vpg_sel[i] & 0x1)?REG_ADDR_DT_1:REG_ADDR_DT;
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, base_addr + 0x40*i, data_type + 0x40*i);
				if (ret < 0) {
					vin_err("write %s vpg data_type error\n", deserial_if->deserial_name);
					return ret;
				}
				uint32_t controller_val = 0;
				if ((sensor_if[i]->extra_mode & PORT_MASK) == PORT_B_EN) {
					controller_val = 0x2A;
				} else {
					controller_val = 0x15;
				}
				base_addr = (vpg_sel[i] & 0x1)?REG_DST_PORT_1:REG_DST_PORT;
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, base_addr + 0x40*i, controller_val);
				if (ret < 0) {
					vin_err("write %s vpg dst_port error\n", deserial_if->deserial_name);
					return ret;
				}
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			pdata = max96718_vpg_init_setting;
			setting_size = sizeof(max96718_vpg_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_vpg register error\n");
				return ret;
			}
			ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES718_REG_ADDR_WID, sensor_info->width);
			if (ret < 0) {
				vin_err("write %s vpg width error\n", deserial_if->deserial_name);
				return ret;
			}
			ret = hb_vin_i2c_write_reg16_data16(bus, deserial_addr, DES718_REG_ADDR_HEI, sensor_info->resolution);
			if (ret < 0) {
				vin_err("write %s vpg height error\n", deserial_if->deserial_name);
				return ret;
			}
			uint32_t vs_low_set = 0;
			for (i = 0; i < 2; ++i) {
				if (!sensor_if[i]) {
					break;
				}
				uint32_t base_cfg_tmp = 0;
				base_cfg_tmp = (sensor_if[i]->width << 20) |
					(sensor_if[i]->resolution << 8) |
					(sensor_if[i]->fps);
				if (base_cfg_tmp != base_cfg[0]) {
					vin_err("cfg is error \n");
					vin_err("not support two cfgs\n");
					return -1;
				}

				if ((sensor_if[i]->resolution == 2160) && (sensor_if[i]->fps > 15)) {
					/*ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr,
						reg_patgen_clk_src_96718[i], 0x80);
					if (ret < 0) {
						vin_err("write %s vpg pclk src error\n", deserial_if->deserial_name);
						return ret;
					}*/
					vin_err("not support 4k resolution with fps higher than 15fps\n");
					return -1;
					// pclk_sel[i] = 600;
				}
				uint32_t data_type = 0;
				if ((sensor_if[i]->extra_mode & DT_MASK) == DT_RAW) {
					data_type = 0x2C;
				} else if ((sensor_if[i]->extra_mode & DT_MASK) == DT_YUV) {
					data_type = 0x1E;
				} else if ((sensor_if[i]->extra_mode & DT_MASK) == DT_RGB) {
					data_type = 0x24;
				}
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, DES718_DATA_ADDR + 0x40*i, data_type + 0x40*i);
				if (ret < 0) {
					vin_err("write %s vpg data_type error\n", deserial_if->deserial_name);
					return ret;
				}
				uint32_t controller_val = 0;
				if ((sensor_if[i]->extra_mode & PORT_MASK) == PORT_B_EN) {
					controller_val = 0x2A;
				} else {
					controller_val = 0x15;
				}
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, DES718_PORT_ADDR + 0x40*i, controller_val);
				if (ret < 0) {
					vin_err("write %s vpg dst_port error\n", deserial_if->deserial_name);
					return ret;
				}
				vs_low_set = pclk_sel[i] * 1000000 / sensor_if[i]->fps - VS_HIGH_NUM;
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG718_ADDR_VS_LOW_1, vs_low_set >> 16);
				if (ret < 0) {
					vin_err("write vs_low error\n");
					return ret;
				}
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG718_ADDR_VS_LOW_2, (vs_low_set & 0xFF00) >> 8);
				if (ret < 0) {
					vin_err("write vs_low error\n");
					return ret;
				}
				ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, REG718_ADDR_VS_LOW_3, (vs_low_set & 0xFF));
				if (ret < 0) {
					vin_err("write vs_low error\n");
					return ret;
				}
			}
		} else {
			for (int i = 0; i < 2; ++i) {
				if (!sensor_if[i]) {
					break;
				}
				/* virtual cam */
				if ((sensor_if[i]->extra_mode & VIRTUAL_CAM_MASK) == VIRTUAL_MAX96718_MAX9295_YUV) {
					pdata = max96718_max9295_yuv_init_setting;
					setting_size = sizeof(max96718_max9295_yuv_init_setting)/sizeof(uint8_t);
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
					if (ret < 0) {
						vin_err("write max96718_max9295_yuv_init_setting register error\n");
						return ret;
					}
				} else {
					vin_err("virtual cam(%d)%s mode 0x%x not support error\n", \
						i, sensor_if[i]->sensor_name, (sensor_if[i]->extra_mode));
					return -HB_CAM_SERDES_CONFIG_FAIL;
				}
				// set out port
				uint32_t controller_val = 0;
				if ((sensor_if[i]->extra_mode & PORT_MASK) == PORT_B_EN) {
					controller_val = 0xAA;
				} else {
					controller_val = 0x55;
				}
				max96718_max9295_out_port_setting[4] = controller_val;
				max96718_max9295_out_port_setting[9] = controller_val;
				pdata = max96718_max9295_out_port_setting;
				setting_size = sizeof(max96718_max9295_out_port_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_max9295_out_port_setting register error\n");
					return ret;
				}
			}
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	deserial_if->init_state = 1;
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}

void virtual_common_data_init(sensor_info_t *sensor_info,
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

int32_t virtual_tuning_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	virtual_common_data_init(sensor_info, &turning_data);

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("SENSOR_TURNING_PARAM ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);
		turning_data.pwl.again_lut = NULL;
	}
	if (turning_data.pwl.dgain_lut) {
		free(turning_data.pwl.dgain_lut);
		turning_data.pwl.dgain_lut = NULL;
	}

	return ret;
}

int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if (sensor_info->power_mode) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
										1-sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);
			}
		}
	}
	return ret;
}

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if (sensor_info->power_mode) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}

	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t entry_num = sensor_info->entry_num;


	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port, drop\n", __func__);
	} else {
		if (sensor_info->sen_devfd <= 0) {
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

	if (deserial_if) {
		req = hb_vin_mipi_pre_request(entry_num, 0, 0);
		if (req == 0) {
			vin_info("virtual serdes start init \n");
			ret = sensor_virtual_des_init(sensor_info);
			hb_vin_mipi_pre_result(entry_num, 0, ret);
			if (ret < 0) {
				vin_err("sensor_virtual_des_init fail\n");
				return ret;
			}
		}
	}
	ret = virtual_tuning_data_init(sensor_info);
	if (ret < 0) {
		vin_err("virtual_tuning_data_init fail\n");
		return ret;
	}

	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t req;
	int32_t ret = RET_OK;
	int32_t entry_num = sensor_info->entry_num;

	req = hb_vin_mipi_pre_request(entry_num, 1, 0);
	if (req == 0) {
		ret = sensor_serdes_stream_on(sensor_info);
		if (ret < 0) {
			ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
			vin_err("%d : %s sensor_virtual_serdes_stream_on fail\n",
					__LINE__, sensor_info->sensor_name);
		}
		hb_vin_mipi_pre_result(entry_num, 1, ret);
	}

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t req;
	int32_t ret = RET_OK;
	int32_t entry_num = sensor_info->entry_num;

	req = hb_vin_mipi_pre_request(entry_num, 1, 0);
	if (req == 0) {
		ret = sensor_serdes_stream_off(sensor_info);
		if (ret < 0) {
			ret = -HB_CAM_SERDES_STREAM_OFF_FAIL;
			vin_err("%d : %s sensor_virtual_serdes_stream_off fail\n",
					__LINE__, sensor_info->sensor_name);
		}
		hb_vin_mipi_pre_result(entry_num, 1, ret);
	}

	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweroff fail\n");
		return -1;
	}

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

sensor_module_t virtual = {
	.module = "virtual",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};

