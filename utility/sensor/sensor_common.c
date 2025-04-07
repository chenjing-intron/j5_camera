/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
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
#include <byteswap.h>
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/sensor_common.h"
#include "inc/sensor_effect_common.h"

#define RETRY_POC_TIMES 3
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x48)

extern int32_t write_register(int32_t bus, int32_t deserial_addr, int32_t poc_addr, int32_t serial_addr,
			int32_t sensor_addr, uint8_t *pdata, int32_t setting_size);
int32_t max9296_max96718_reset(uint32_t bus, uint8_t slave_addr)
{
	int32_t ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
			MAX9296_RESET_REG, MAX9296_RESET_VAL);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x\n", bus, slave_addr,
				MAX9296_RESET_REG, MAX9296_RESET_VAL);
		return ret;
	}
	usleep(100*1000);
	return ret;
}

int32_t max96712_reset(uint32_t bus, uint8_t slave_addr)
{
	int32_t ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
			MAX96712_RESET_REG, MAX96712_RESET_VAL);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x\n", bus, slave_addr,
				MAX96712_RESET_REG, MAX96712_RESET_VAL);
		return ret;
	}
	usleep(100*1000);
	return 0;
}

int32_t max9296_gmsl_speed_init(uint32_t bus, uint8_t slave_addr,
	uint8_t *gmsl_speed, uint32_t desport_num)
{
	int32_t ret = RET_OK;
	int8_t val;

	if (gmsl_speed == NULL) {
		vin_err("no gmsl_speed here error\n");
		return -1;
	}

    for (int i = 0; i < desport_num; i++) {
		if (gmsl_speed[i] == 3) {
			val = TXRATE_3G;
			break;
		} else {
			val = TXRATE_6G;
		}
    }
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, REG_TXRATE_9296, val);
	if (ret < 0)
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
				slave_addr, REG_TXRATE_9296, val);

	return ret;
}

int32_t max96718_gmsl_speed_init(uint32_t bus, uint8_t slave_addr,
	uint8_t *gmsl_speed, uint32_t desport_num)
{
	int32_t ret = RET_OK;
	int8_t val_linka, val_linkb;

	if (gmsl_speed == NULL) {
		vin_err("no gmsl_speed here error\n");
		return -1;
	}

	if (desport_num == 1) {
		if (gmsl_speed[0] == 3) {
			val_linka = TXRATE_3G;
			val_linkb = TXRATE_3G;
		} else {
			val_linka = TXRATE_6G;
			val_linkb = TXRATE_6G;
		}
	} else {
		if (gmsl_speed[0] == 3) {
			val_linka = TXRATE_3G;
		} else {
			val_linka = TXRATE_6G;
		}
		if (gmsl_speed[1] == 3) {
			val_linkb = TXRATE_3G;
		} else {
			val_linkb = TXRATE_6G;
		}
	}
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, REG_TXRATE_96718_A, val_linka);
	if (ret < 0)
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
				slave_addr, REG_TXRATE_96718_A, val_linka);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, REG_TXRATE_96718_B, val_linkb);
	if (ret < 0)
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
				slave_addr, REG_TXRATE_96718_B, val_linkb);

	return ret;
}

int32_t max96712_gmsl_speed_init(uint32_t bus, uint8_t slave_addr,
		uint8_t *gmsl_speed, uint32_t desport_num)
{
	int32_t ret = RET_OK;
	int16_t value = 0;
	uint8_t val;

	if (gmsl_speed == NULL) {
		vin_err("no gmsl_speed here error\n");
		return -1;
	}

	if (desport_num == 1) {
		if (gmsl_speed[0] == 3) {
			val =  TXRATE_3G;
		} else {
			val =  TXRATE_6G;
		}
		value |= (val | (val << 4) |
			(val << 8) | (val << 12)); 	   // all link swich same rate
	} else {
	    for (int link_index = 0; link_index < desport_num; link_index++) {
			if (gmsl_speed[link_index] == 3) {
				val = TXRATE_3G;
			} else {
				val = TXRATE_6G;
			}
			value |= (val << (link_index * MAX96712_GMSL_LINK_SHIFT));
	    }
	}
	value = bswap_16(value);
	ret = hb_vin_i2c_write_reg16_data16(bus, slave_addr, REG_TXRATE_96712, value);
	if (ret < 0)
		vin_err("write 0x%x reg 0x%x val 0x%x fail\n",
				slave_addr, REG_TXRATE_96712, value);
	return ret;
}

int32_t data_type_map(deserial_info_t *deserial_if, uint32_t desport_num)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;

	if (deserial_if->data_type == NULL) {
		vin_err("no data_type here error\n");
		return -1;
	}

	if ((!strcmp(deserial_if->deserial_name, "max96718") ||
		!strcmp(deserial_if->deserial_name, "max96712")) && desport_num == 1) {
		if (!strcmp(deserial_if->deserial_name, "max96718")) {
			setting_size = sizeof(max96718_all_dt_vc_setting) / sizeof(uint8_t);
			pdata = max96718_all_dt_vc_setting;
		} else {
			setting_size = sizeof(max96712_all_dt_vc_setting) / sizeof(uint8_t);
			pdata = max96712_all_dt_vc_setting;
		}
		if (deserial_if->data_type[0] == DATATYPE_YUV) {
			for (int i = 4; i < setting_size; i += 5) {
				pdata[i] = DATATYPE_YUV;
			}
		}
		ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
		if (ret < 0) {
		  vin_err("dt_vc_setting failed\n");
		  return -RET_ERROR;
		}
	} else {
		for (int i = 0; i < desport_num; i++) {
			if (!strcmp(deserial_if->deserial_name, "max9296") ||
				!strcmp(deserial_if->deserial_name, "max96718")) {
				if (!strcmp(deserial_if->deserial_name, "max96718")) {
					setting_size = sizeof(max96718_dt_vc_setting[i]) / sizeof(uint8_t);
					pdata = max96718_dt_vc_setting[i];
				} else {
					setting_size = sizeof(max9296_dt_vc_setting[i]) / sizeof(uint8_t);
					pdata = max9296_dt_vc_setting[i];
				}
			} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
				setting_size = sizeof(max96712_dt_vc_setting[i]) / sizeof(uint8_t);
				pdata = max96712_dt_vc_setting[i];
			}
			if (deserial_if->data_type[i] == DATATYPE_YUV) {
				pdata[4] = DATATYPE_YUV;
				pdata[9] = (DATATYPE_YUV | (i << 6));
			}
			ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
			  vin_err("dt_vc_setting failed\n");
			  return -RET_ERROR;
			}
		}
	}
	return ret;
}

int32_t deserial_source_map(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	uint32_t desport_num = deserial_if->reserved[0];

	if (!strcmp(deserial_if->deserial_name, "max96718") && desport_num == 1) {
		setting_size = sizeof(max96718_all_maplink_setting) / sizeof(uint8_t);
		pdata = max96718_all_maplink_setting;
		ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
		if (ret < 0) {
			vin_err("write maplink_setting failed\n");
			return -RET_ERROR;
		}
	} else {
		for (int i = 0; i < desport_num; i++) {
			if (!strcmp(deserial_if->deserial_name, "max9296") ||
				!strcmp(deserial_if->deserial_name, "max96718")) {
				if (!strcmp(deserial_if->deserial_name, "max96718")) {
					setting_size = sizeof(max96718_maplink_setting[i]) / sizeof(uint8_t);
					pdata = max96718_maplink_setting[i];
				} else {
					setting_size = sizeof(max9296_maplink_setting[i]) / sizeof(uint8_t);
					pdata = max9296_maplink_setting[i];
				}
			} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
				if (desport_num == 1) {
					setting_size = sizeof(max96712_all_maplink_setting) / sizeof(uint8_t);
					pdata = max96712_all_maplink_setting;
				} else {
					setting_size = sizeof(max96712_maplink_setting[i]) / sizeof(uint8_t);
					pdata = max96712_maplink_setting[i];
				}
			}
			ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("write maplink_setting[%d] failed\n", i);
				return -RET_ERROR;
			}
		}
	}

	ret = data_type_map(deserial_if, desport_num);
	if (ret < 0) {
		vin_err("data_type_map failed\n");
		return -RET_ERROR;
	}
	return ret;
}

int bpp_datetype_init(deserial_info_t *deserial_if) {
	int32_t ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0;

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		pdata = max9296_datatype_bpp_init_setting;
		setting_size = sizeof(max9296_datatype_bpp_init_setting) / sizeof(uint8_t);
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		pdata = max96718_datatype_bpp_init_setting;
		setting_size = sizeof(max96718_datatype_bpp_init_setting) / sizeof(uint8_t);
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		pdata = max96712_datatype_bpp_init_setting;
		setting_size = sizeof(max96712_datatype_bpp_init_setting) / sizeof(uint8_t);
	}
	ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
	if (ret < 0) {
	  vin_err("datatype_bpp_init_setting failed\n");
	  return ret;
	}

	return ret;
}

int override_and_mux_mode_set(deserial_info_t *deserial_if, uint32_t deserial_port) {
	int32_t ret = RET_OK;
	uint16_t reg1 = 0;
	uint16_t reg2 = 0;
	int val_read1 = 0;
	int val_read2 = 0;

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg1 = OVERRIDE_96718_9296_ZU;
		reg2 = MUXED_96718_9296;
		val_read1 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				  deserial_if->deserial_addr, reg1);
		val_read2 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				  deserial_if->deserial_addr, reg2);
		val_read1 |= (0x40 << deserial_port);
		val_read2 |= (0x40 << deserial_port);
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (deserial_port == 0) {
			reg1 = OVERRIDE_96718_9296_ZU;
			val_read1 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
					  deserial_if->deserial_addr, reg1);
			val_read1 |= (0x40 << deserial_port);
		} else {
			reg1 = OVERRIDE_96718_Y;
			val_read1 = 0xB4;
		}
		reg2 = MUXED_96718_9296;
		val_read2 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				  deserial_if->deserial_addr, reg2);
		if (deserial_port == 0) {
			val_read2 |= 0x40;
		} else {
			val_read2 |= 0x20;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
	  reg1 = MUXED_96712;
	  if (deserial_port < 2) {
		  reg2 = OVERRIDE_96712_PIPE01;
	  } else {
		  reg2 = OVERRIDE_96712_PIPE23;
	  }
	  val_read1 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
		  deserial_if->deserial_addr, reg1);
	  val_read2 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
		  deserial_if->deserial_addr, reg2);
	  val_read1 |= (0x10 << deserial_port);
	  if (deserial_port < 2) {
		  val_read2 |= (0x40 << deserial_port);
	  } else {
		  val_read2 |= (0x40 << (deserial_port - 2));
	  }
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
		reg1, val_read1);
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
		reg2, val_read2);
	if (ret < 0) {
	  vin_err("muxed mode set failed\n");
	  return ret;
	}

	return ret;
}

int32_t dvp_yuv_mode_init(deserial_info_t *deserial_if, uint32_t desport_num, uint32_t deserial_port)
{
	int32_t ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0;

	if (desport_num == 1) {
		if (!strcmp(deserial_if->deserial_name, "max9296")) {
			pdata = max9296_alllink_yuv_mode_setting;
			setting_size = sizeof(max9296_alllink_yuv_mode_setting) / sizeof(uint8_t);
		} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
			pdata = max96718_alllink_yuv_mode_setting;
			setting_size = sizeof(max96718_alllink_yuv_mode_setting) / sizeof(uint8_t);
		} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
			pdata = max96712_alllink_yuv_mode_setting;
			setting_size = sizeof(max96712_alllink_yuv_mode_setting) / sizeof(uint8_t);
		}
		ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
		if (ret < 0) {
		  vin_err("alllink_datatype_setting failed\n");
		  return ret;
		}
	} else {
		ret = bpp_datetype_init(deserial_if);
		if (ret < 0) {
			vin_err("bpp_datetype_init failed\n");
			return ret;
		}
		ret = override_and_mux_mode_set(deserial_if, deserial_port);
		if (ret < 0) {
		  vin_err("muxed mode set failed\n");
		  return ret;
		}
	}
	return ret;
}

static struct timespec diff_timespec(const struct timespec *time1,
		const struct timespec *time0)
{
	struct timespec diff = {
		.tv_sec = time1->tv_sec - time0->tv_sec,
		.tv_nsec = time1->tv_nsec - time0->tv_nsec};

	if (diff.tv_nsec < 0) {
		diff.tv_nsec += 1000000000UL;
		diff.tv_sec--;
	}

	return diff;
}

/*
 * loop_udelay: udelay not sleep
 * @x: delay time, uint is us, must be less than 1000000
 * */
void __attribute__((weak)) loop_udelay(const uint64_t x)
{
	struct timespec t1, t2;
	struct timespec diff;
	uint64_t nsec = x * 1000;

	clock_gettime(CLOCK_MONOTONIC, &t1);

	do {
		clock_gettime(CLOCK_MONOTONIC, &t2);
		diff = diff_timespec(&t2, &t1);
	} while (diff.tv_nsec < nsec);
}

int32_t __attribute__((weak)) poc_linked_first(int32_t bus, int32_t poc_addr)
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

void __attribute__((weak)) setting_modify(uint8_t *pdata, int32_t size, uint8_t addr, uint16_t reg, uint16_t v)
{
	int32_t i, len;
	uint16_t r;

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
				default:
					r = 0xffff;
					break;
			}
			if (r == reg) {
				switch (len) {
					case 5:
						pdata[i + 4] = v >> 8;
						pdata[i + 5] = v & 0xff;
						break;
					case 4:
						pdata[i + 4] = v & 0xff;
						break;
					case 3:
						pdata[i + 3] = v & 0xff;
						break;
					default:
						break;
				}
				break;
			}
		}
		i += (len + 1);
	}
}

int32_t __attribute__((weak)) sensor_setting_array(int32_t bus, uint32_t i2c_addr, int32_t reg_width,
			int32_t setting_size, uint16_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int32_t ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = reg_width;

	for(i = 0; i < setting_size; i++) {
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

int32_t __attribute__((weak)) sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
										1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);
			}
		}
	}
	return ret;
}

uint8_t __attribute__((weak)) common_one_shot_reset(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	uint16_t reg = 0;
	uint16_t val = 0;
	uint16_t rega = 0;
	uint16_t vala = 0;
	uint16_t regb = 0;
	uint16_t valb = 0;

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = MAX9296_ONE_SHOT_RESET_REG;
		val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= 0x20;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		rega = MAX96718_ONE_SHOT_RESETA_REG;
		regb = MAX96718_ONE_SHOT_RESETB_REG;
		vala = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, rega);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, rega);
			return -1;
		}
		valb = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, regb);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, regb);
			return -1;
		}
		vala |= 0x20;
		valb |= 0x20;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			rega, vala);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, rega);
			return -1;
		}
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			regb, valb);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, regb);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		reg = MAX96712_ONE_SHOT_RESET_REG;
		val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= 0x0F;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
	}

	return 0;
}

/* default date_rate 0 means 6g, 1 means 3g*/
uint8_t __attribute__((weak)) common_rx_rate_switch(sensor_info_t *sensor_info, uint8_t data_rate)
{
	int ret = RET_OK;
	uint16_t reg = 0, value = 0;
	uint8_t  val = 0;
	int      val_read = 0;
	char tx_rate_s[10];
	static int rx_rate_init_once = 0;
	uint32_t desport_num = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	desport_num = deserial_if->reserved[0];

	if (data_rate == 1) {
		val = TXRATE_3G;
		memcpy(tx_rate_s, "3G", 3);
	} else if (data_rate == 0) {
		val = TXRATE_6G;
		memcpy(tx_rate_s, "6G", 3);
	} else {
		vin_err("%s data_rate %d not supported for des-%s!\n",
			__func__, data_rate, deserial_if->deserial_name);
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_TXRATE_9296;
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
			reg = REG_TXRATE_96712;
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (sensor_info->deserial_port == 0)
			reg = REG_TXRATE_96718_A;
		else
			reg = REG_TXRATE_96718_B;
	}
	val_read = hb_vin_i2c_read_reg16_data16(deserial_if->bus_num,
		deserial_if->deserial_addr, reg);
	val_read = bswap_16(val_read);
	if (val_read < 0) {
		vin_err("%s read reg 0x%x failed!\n", __func__, reg);
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max9296") ||
			!strcmp(deserial_if->deserial_name, "max96718")) {
		if (desport_num > 1) {
			val |= (val_read & 0xFC);
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg, val);
		} else {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96718_A, val);
			if (ret < 0) {
				vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
				return -1;
			}
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96718_B, val);
		}
		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		value = val << (sensor_info->deserial_port * 4);
		value |= (val_read & ~(0x3 << (sensor_info->deserial_port * 4)));
		value = bswap_16(value);
		if (desport_num > 1) {
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
				deserial_if->deserial_addr, reg, value);
		} else {
			value &= 0xCCCC;
			value |= (val | (val << 4) | (val << 8) | (val << 12));        // all link swich same rate
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96712, value);
		}

		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
	}

	vin_info("%s switch to %s successfully for des-%s!\n", __func__, tx_rate_s,
		deserial_if->deserial_name);
	ret = common_one_shot_reset(deserial_if);
	if (ret < 0) {
		vin_err("common_one_shot_reset failed!\n");
		return -1;
	}
	usleep(160 * 1000);

	return 0;
}

int __attribute__((weak)) write_j5_register(int bus, uint8_t *pdata, int setting_size)
{
	int ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int i, len, k = 10;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			vin_info("write reg:0x%x  value:%x\n", reg_addr, value);
			while (ret < 0 && k--) {
				vin_info("init serdes reg:0x%x  value:%x  k:%d\n", reg_addr, value, k);
				if (k % 10 == 9) {
					usleep(20 * 1000);
					ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
					vin_err("write reg:0x%x  value:%x\n", reg_addr, value);
				}
			}
			if (ret < 0) {
				vin_err("init serdes bus %x i2c_slave = %x reg:0x%x value:%x error\n",
				       bus, i2c_slave, reg_addr, value);
				return ret;
			}
			i = i + len + 1;
			vin_info("init serdes bus %x i2c_slave = %x reg:0x%x value:%x\n",
				       bus, i2c_slave, reg_addr, value);
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
			i = i + 2;
		}
	}
	return ret;
}

uint8_t __attribute__((weak)) common_link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint16_t reg1 = 0;
	uint16_t reg2 = 0;
	uint8_t  val = 0;
	int      val_read1 = 0;
	int      val_read2 = 0;
	uint32_t desport_num = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	desport_num = deserial_if->reserved[0];

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_LINK_SET_9296;
		if (desport_num > 1) {
			if (link_port < DES_PORT_NUM_MAX - 2) {
				val = LINK_NONE_9296 | (1 << link_port);
			} else if (link_port == LINK_ALL) {
				val = LINK_ALL_9296;
			} else {
				vin_err("%s link_port 0x%x not supported for des-%s!\n",
					__func__, link_port, deserial_if->deserial_name);
				return -1;
			}
		} else {
			val = LINK_ALL_9296_ANY;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
			val = LINK_ALL_9296;
			reg = REG_LINK_SET_9296;
			reg1 = REG_LINKA_SET_96718;
			reg2 = REG_LINKB_SET_96718;
			if (desport_num == 1) {
				link_port = LINK_ALL;
			}
			val_read1 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg1);
			if (val_read1 < 0) {
				vin_err("%s read reg 0x%x failed!\n", __func__, reg1);
				return -1;
			}
			val_read2 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg2);
			if (val_read2 < 0) {
				vin_err("%s read reg 0x%x failed!\n", __func__, reg2);
				return -1;
			}
			if (link_port == 0) {
				val_read1 &= 0xEF;                // open linkA
				val_read2 |= 0x04;                // close linkB
			} else if (link_port == 1) {
				val_read1 |= 0x10;                // close linkA
				val_read2 &= 0xFB;                // open linkB
			} else if (link_port == LINK_ALL) {
				val_read1 &= 0xEF;                // open linkA
				val_read2 &= 0xFB;                // open linkB
			}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		reg = REG_LINK_SET_96712;
		if (desport_num > 1) {
			if (link_port < DES_PORT_NUM_MAX) {
				val = LINK_NONE_96712 & (~(1 << (2 * link_port)));
				// val = LINK_NONE_96712 | (1 << link_port);
			} else if (link_port == LINK_ALL) {
				val = LINK_ALL_96712;
			} else {
				vin_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
					deserial_if->deserial_name);
				return -1;
			}
		} else {
			val = LINK_ALL_96712;
		}
	} else {
		vin_info("%s not supported des-%s, drop\n", __func__, deserial_if->deserial_name);
		return 0;
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
		reg, val);
	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg1, val_read1);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg2, val_read2);
	}
	if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
			deserial_if->deserial_name);
		return -1;
	}
	vin_info("%s switch to port 0x%x successfully for des-%s!\n",
		__func__, link_port, deserial_if->deserial_name);
	usleep(20 * 1000);
	return 0;
}

static int32_t set_gpio(deserial_info_t *deserial_info)
{
	int32_t gpio, ret = -HB_CAM_SENSOR_POWERON_FAIL;

	for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
		vin_info("Set gpio level is %d for gpio%d\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
		if(deserial_info->gpio_pin[gpio] != -1) {
			ret = vin_power_ctrl(deserial_info->gpio_pin[gpio],
					deserial_info->gpio_level[gpio]);
			usleep(100 *1000);
			ret |= vin_power_ctrl(deserial_info->gpio_pin[gpio],
					1-deserial_info->gpio_level[gpio]);
			if(ret < 0) {
				vin_err("Set gpio level is %d for gpio%d failed\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
				return -HB_CAM_SENSOR_POWERON_FAIL;
			}
			usleep(100*1000);
		}
	}
	return ret;
}

int32_t poc_power_reset(sensor_info_t *sensor_info)
{
	int32_t retry_poc_times = 0;
	int32_t setting_size = 0;
	int32_t ret = RET_OK;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	setting_size = 1;
	poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
	for (retry_poc_times = 0; retry_poc_times < RETRY_POC_TIMES; retry_poc_times++) {
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, poc_addr, 0x01, 0x00);
		if (ret < 0) {
			vin_warn("write %d@0x%02x=0x00 failed, try to reset the poc, %d times\n", \
				deserial_if->bus_num, poc_addr, retry_poc_times);
			ret = set_gpio(deserial_if);
			if (ret < 0) {
				vin_err("set_gpio fail\n");
				return ret;
			}
			continue;
		}
		usleep(10*1000);
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, poc_addr, 0x01, 0x1f);
		if (ret < 0) {
			vin_warn("write %d@0x%02x=0x1f failed, try to reset the poc, %d times\n", \
				deserial_if->bus_num, poc_addr, retry_poc_times);
			ret = set_gpio(deserial_if);
			if (ret < 0) {
				vin_err("set_gpio fail\n");
				return ret;
			}
			continue;
		}
		usleep(50*1000);
		return ret;
	}
	return -RET_ERROR;
}

int32_t __attribute__((weak)) sensor_serdes_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	do {
		if (!strcmp(deserial_if->deserial_name, "max96712") ||
				   !strcmp(deserial_if->deserial_name, "max96722")) {
			uint32_t setting_size = sizeof(max96712_stop_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_stop_setting[2*i], (uint8_t)(max96712_stop_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					break;
				}
			}
		} else if (!strcmp(deserial_if->deserial_name, "max96718") ||
						!strcmp(deserial_if->deserial_name, "max9296")) {
			uint32_t setting_size = sizeof(max9296_max96718_stop_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max9296_max96718_stop_setting[2*i], (uint8_t)(max9296_max96718_stop_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					break;
				}
			}
		}
	    vin_info("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} while (0);
	return ret;
}

int32_t __attribute__((weak)) sensor_serdes_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}

	do {
		if (!strcmp(deserial_if->deserial_name, "max96712") ||
				   !strcmp(deserial_if->deserial_name, "max96722")) {
			uint32_t setting_size = sizeof(max96712_start_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_start_setting[2*i], (uint8_t)(max96712_start_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					break;
				}
			}
		} else if (!strcmp(deserial_if->deserial_name, "max96718") ||
						!strcmp(deserial_if->deserial_name, "max9296")) {
			uint32_t setting_size = sizeof(max9296_max96718_start_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max9296_max96718_start_setting[2*i], (uint8_t)(max9296_max96718_start_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					break;
				}
			}
		}
	    vin_info("sensor_start write %s successfully\n", deserial_if->deserial_name);
	} while (0);
	return ret;
}

/*
 * deserial_errchpwrup_config() - enable ErrChPwrUp for maxim deseiral on 6G mode
 */
int32_t deserial_errchpwrup_config(uint32_t bus, uint8_t slave_addr,
				   uint8_t *gmsl_speed, uint32_t desport_num)
{
	int32_t ret = RET_OK;
	int32_t i;
	for (i = 0; i < desport_num; i++) {
		if (gmsl_speed[i] != 3) {
			ret = vin_i2c_bit_write8(bus, slave_addr, maxdes_errchpwrup_en[i],
						 REG_WIDTH_16bit, ERRCHPWRUP_EN, ERRCHPWRUP_EN);
			if (ret < 0) {
				vin_err("deserial port %d errchprwup enable fail\n", i);
				return ret;
			}
		}
	}
	return ret;
}

int32_t deserial_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	uint8_t i2c_slave;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;
	uint32_t desport_num = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;
	desport_num =  deserial_if->reserved[0];

	if (deserial_if->init_state == 1)
		return ret;

#ifdef POC_RETRY_POLICY
		if (poc_addr != INVALID_POC_ADDR) {
			ret = poc_power_reset(sensor_info);
			if (ret < 0) {
				vin_err("poc_power_reset fail\n");
				return ret;
			}
		} else {
			for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
				vin_info("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
				hb_vin_i2c_write_reg16_data8(bus, i2c_slave, 0x0010, 0xf1);
			}
		}
#endif

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		ret = max9296_max96718_reset(bus, deserial_addr);
		if (ret < 0) {
			vin_err("max9296 max96718 reset fail!\n");
			return ret;
		}
		if (!strcmp(deserial_if->deserial_name, "max9296")) {
			ret = max9296_gmsl_speed_init(bus, deserial_addr, deserial_if->gmsl_speed, desport_num);
			if (ret < 0) {
				vin_err("max9296 gmsl init fail!\n");
				return ret;
			}
		} else {
			ret = max96718_gmsl_speed_init(bus, deserial_addr, deserial_if->gmsl_speed, desport_num);
			if (ret < 0) {
				vin_err("max96718 gmsl init fail!\n");
				return ret;
			}
		}
		pdata = max9296_max96718_init_setting_base;
		setting_size = sizeof(max9296_max96718_init_setting_base) / sizeof(uint8_t);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		ret = max96712_reset(bus, deserial_addr);
		if (ret < 0) {
			vin_err("max96712 reset fail!\n");
			return ret;
		}
		ret = max96712_gmsl_speed_init(bus, deserial_addr, deserial_if->gmsl_speed, desport_num);
		if (ret < 0) {
			vin_err("max96712 gmsl init fail!\n");
			return ret;
		}
		pdata = max96712_init_setting_base;
		setting_size = sizeof(max96712_init_setting_base)/sizeof(uint8_t);
	} else {
		vin_err("des %s not support err\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}

	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}
	ret = deserial_errchpwrup_config(bus, deserial_addr,
					 deserial_if->gmsl_speed, desport_num);
	if (ret < 0) {
		vin_err("deserial_errchpwrup_config fail\n");
		return ret;
	}

	ret = deserial_source_map(deserial_if);
	if (ret < 0) {
		vin_err("write deserial_source_map error\n");
		return ret;
	}

	if (deserial_if->serial_type == NULL) {
		vin_err("no serial_type here error\n");
		return -1;
	}

	for (uint32_t deserial_port = 0; deserial_port < desport_num; deserial_port++) {
		if (!strcmp(deserial_if->serial_type[deserial_port], "dvp")) {
			ret = dvp_yuv_mode_init(deserial_if, desport_num, deserial_port);
			if (ret < 0) {
				vin_err("write dvp_yuv_mode_init error\n");
				return ret;
			}
		}
	}
	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (desport_num == 1) {
			pipe_96718 = 0x16;
		} else {
			pipe_96718 = 0x15;
		}
		pdata = max9296_add_max96718_init_setting;
		setting_size = sizeof(max9296_add_max96718_init_setting)/sizeof(uint8_t);
		if (pipe_96718 != 0) {
			pdata[4] = pipe_96718;
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max9296_add_max96718_init_setting error\n");
			return ret;
		}
		if (sensor_info->config_index & DPHY_PORTB) {
			pdata = max96718_portb_out_setting;
			setting_size = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_portb_out_setting error\n");
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		if (sensor_info->config_index & DPHY_COPY) {
			setting_size = sizeof(max9296_phy_portall_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_phy_portall_init_setting);
			if (ret < 0) {
				vin_err("write max9296_phy_portall_init_setting error\n");
				return ret;
			}
		} else if (sensor_info->config_index & DPHY_PORTB) {
			setting_size = sizeof(max9296_phy_portb_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_phy_portb_init_setting);
			if (ret < 0) {
				vin_err("write max9296_phy_portall_init_setting error\n");
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			!strcmp(deserial_if->deserial_name, "max96722")) {
		if ((sensor_info->config_index & DPHY_PORTB) &&
				(sensor_info->config_index & DPHY_COPY)) {
			pdata = max96712_phy_cpB2A_init_setting;
			setting_size = sizeof(max96712_phy_cpB2A_init_setting)/
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_phy_cpB2A register error\n");
				return ret;
			}
		} else {
			if (sensor_info->config_index & DPHY_PORTB) {
				pdata = max96712_phy_portb_init_setting;
				setting_size = sizeof(max96712_phy_portb_init_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_phy_portb_init_setting error\n");
					return ret;
				}
			}
			if (sensor_info->config_index & DPHY_COPY) {
				pdata = max96712_phy_cpA2B_init_setting;
				setting_size = sizeof(max96712_phy_cpA2B_init_setting)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_phy_cpA2B register error\n");
					return ret;
				}
			}
		}
	}

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->config_index & TRIG_STANDARD) ||
			(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
			if (deserial_if->mfp_index > MAX9296_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max9296_trig_setting MFP index error\n");
				return ret;
			}
			uint32_t setting_size = 0;
			uint16_t regaddr = 0, offset = 0;
			uint16_t *trigger_reg;

			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				trigger_reg = max96718_trigger_setting_mfp;
				setting_size = sizeof(max96718_trigger_setting_mfp) / sizeof(uint16_t) / 2;
				offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
			} else {
				if (deserial_if->mfp_index == 0xffff) {
					trigger_reg = max9296_trigger_mfp5;
					setting_size = sizeof(max9296_trigger_mfp5) / sizeof(uint16_t) / 2;
					offset = 0;
				} else {
					trigger_reg = max9296_trigger_mfp;
					setting_size = sizeof(max9296_trigger_mfp) / sizeof(uint16_t) / 2;
					offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
				}
			}
			for (int32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i] + offset;
				vin_info("write mfp: w%d@0x%02x 0x%04x=0x%02x\n", deserial_if->bus_num,
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i+1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i+1] & 0xFF));
				if(ret < 0) {
					vin_err("write max9296_trig_setting error\n");
				}
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			!strcmp(deserial_if->deserial_name, "max96722")) {
		if ((sensor_info->config_index & TRIG_STANDARD) ||
		    (sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		    if (deserial_if->mfp_index > MAX96712_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max96712_trig_setting MFP index error\n");
				return ret;
			}
			uint32_t setting_size = sizeof(max96712_trigger_setting_mfp) /
			                        sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
			uint16_t offset = 0, size = 0, regaddr = 0;
			uint16_t *trigger_reg;

			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max96712_trigger_setting_mfp14;
				setting_size = sizeof(max96712_trigger_setting_mfp14) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
			} else {
				trigger_reg = max96712_trigger_setting_mfp;
				setting_size = sizeof(max96712_trigger_setting_mfp) / sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
				offset = deserial_if->mfp_index % MAX96712_MFP_LOOP * setting_size;
				size = deserial_if->mfp_index / MAX96712_MFP_LOOP * MAX96712_MFP_OFFSET;
			}

			for (int i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i + 2*offset] + size;
				vin_info("write mfp: w%d@0x%02x 0x%04x=0x%02x\n", deserial_if->bus_num,
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				if (ret < 0) {
					vin_err("write max96712_trig_setting error\n");
				}
			}
		}
	}
	deserial_if->init_state = 1;
	return ret;
}


