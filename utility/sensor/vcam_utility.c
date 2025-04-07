/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[vcam]:" fmt

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
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensor_common.h"
#include "inc/vcam_setting.h"

#define RETRY_TIME_MAX  5

#define BUF_LEN  128
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SERIAL_ADDR		(0x40)
// #define DEFAULT_DESERIAL_ADDR	(0x29)
#define DEFAULT_DESERIAL_ADDR	(0x48)


#define REG_LINK_SET_96712      0x0003
#define LINK_ALL_96712          0xAA
#define LINK_NONE_96712         0xFF

#define LINK_ALL                0xFF
#define LINK_NONE               0x00

#define MAX96712_PORT_NUM_MAX   (4)

enum MODE_TYPE {
	VCAM_D4_S0_YUV_QUAD,
	VCAM_D4_S0_YUV_SINGLE,
	VCAM_D4_S0_YUV_DUO_INIT,
};


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
				vin_err("write vcam %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			i = i + len + 1;
			vin_info("write vcam %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
			}

			k = RETRY_TIME_MAX;
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

static int vcam_link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	uint16_t reg = 0, reg1 = 0, reg2 = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0, val_read1 = 0, val_read2 = 0;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		val = LINK_ALL_9296;
		reg = REG_LINK_SET_9296;
		reg1 = REG_LINKA_SET_96718;
		reg2 = REG_LINKB_SET_96718;
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

		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
		ret += hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg1, val_read1);
		ret += hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg2, val_read2);
		if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
			deserial_if->deserial_name);
		return -1;
	}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		reg = REG_LINK_SET_96712;
		if (link_port < DES_PORT_NUM_MAX) {
			val = LINK_NONE_96712 & (~(1 << (2 * link_port)));
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_96712;
		} else {
			vin_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
				deserial_if->deserial_name);
			return -1;
		}
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
	}

	usleep(300 * 1000);
	return ret;
}

static int vcam_serial_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0, i;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		if (sensor_info->extra_mode == VCAM_D4_S0_YUV_SINGLE) {
			// set serial
			pdata = max96718_single_yuv_setting_base;
			setting_size = sizeof(max96718_single_yuv_setting_base)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_single_yuv_setting_base quad error\n");
				return ret;
			}
		} else if (sensor_info->extra_mode == VCAM_D4_S0_YUV_DUO_INIT) {
			// DO NOTHING
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		if (sensor_info->extra_mode == VCAM_D4_S0_YUV_QUAD) {
			// try to reset serial
			ret = hb_vin_i2c_write_reg16_data8(bus, serial_addr, \
										0x0000, DEFAULT_SERIAL_ADDR << 1);
			if (ret < 0) {
				vin_err("reset sensor quad error, no need care\n");
			}
			usleep(200 * 1000);
			// set serial
			pdata = max96712_quad_yuv_setting_base[sensor_info->deserial_port];
			setting_size = sizeof(max96712_quad_yuv_setting_base[sensor_info->deserial_port])/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_quad_yuv_setting_base quad error\n");
				return ret;
			}
		}
	} else {
		vin_err("no support virtual cam extra_mode = 0x%x\n", sensor_info->extra_mode);
		return ret;
	}


	return ret;
}

static int vcam_deserial_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	uint8_t controller_val = 0x00;
	int setting_size = 0;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr, reg_addr;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if (deserial_if->init_state == 1) {
		// no need init deserial
		return ret;
	}

	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		if (sensor_info->extra_mode == VCAM_D4_S0_YUV_SINGLE) {
			// yuv_sensor 1 vc
			pdata = max96718_max9295_yuv_init_setting;
			setting_size = sizeof(max96718_max9295_yuv_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_max9295_yuv_init_setting quad error\n");
				return ret;
			}
			// set out port
			if (sensor_info->config_index & DPHY_PORTB) {
				controller_val = 0xAA;
			} else {
				controller_val = 0x55;
			}
			max96718_out_port_setting[4] = controller_val;
			max96718_out_port_setting[9] = controller_val;
			pdata = max96718_out_port_setting;
			setting_size = sizeof(max96718_out_port_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_out_port_setting register error\n");
				return ret;
			}
		} else if (sensor_info->extra_mode == VCAM_D4_S0_YUV_DUO_INIT) {
			if(sensor_info->deserial_port == 0) {
				// try to reset serial
				vcam_link_switch(sensor_info, 0xff);  // link all
				ret = hb_vin_i2c_write_reg16_data8(bus, 0x41, \
											0x0000, DEFAULT_SERIAL_ADDR << 1);
				ret = hb_vin_i2c_write_reg16_data8(bus, 0x42, \
											0x0000, DEFAULT_SERIAL_ADDR << 1);
				if (ret < 0) {
					vin_err("reset sensor quad error, no need care\n");
				}
				usleep(200 * 1000);

				// linkA to portA 6G ; linkB to portB 3G
				pdata = max96718_max9295_yuv_duo_init_setting;
				setting_size = sizeof(max96718_max9295_yuv_duo_init_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_max9295_yuv_duo_init_setting quad error\n");
					return ret;
				}

			} else if (sensor_info->deserial_port == 1) {
				// DO NOTHING
				vin_info("int do nothing\r\n");
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		if (sensor_info->extra_mode == VCAM_D4_S0_YUV_QUAD) {
			// yuv_sensor 4 vc
			pdata = max96712_max9295_yuv_init_setting;
			setting_size = sizeof(max96712_max9295_yuv_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_max9295_yuv_init_setting quad error\n");
				return ret;
			}
			// set out port
			if (sensor_info->config_index & DPHY_PORTB) {
				controller_val = 0xAA;
			} else {
				controller_val = 0x55;
			}
			max96712_out_port_setting[4] = controller_val;
			max96712_out_port_setting[9] = controller_val;
			max96712_out_port_setting[14] = controller_val;
			max96712_out_port_setting[19] = controller_val;
			pdata = max96712_out_port_setting;
			setting_size = sizeof(max96712_out_port_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_out_port_setting register error\n");
				return ret;
			}
		}
	}

	deserial_if->init_state = 1;
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}


int32_t sensor_init(sensor_info_t *sensor_info)
{
    int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	int32_t entry_num = sensor_info->entry_num;

    if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}

	// set deserial
	req = hb_vin_mipi_pre_request(entry_num, 0, 0);
	if (req == 0) {
		vin_info("vcam des start init \n");
		ret = vcam_deserial_setting(sensor_info);
		hb_vin_mipi_pre_result(entry_num, 0, ret);
		if (ret < 0) {
			vin_err("sensor_vcam_des_init fail\n");
			return ret;
		}
	}

	// link switch
	vcam_link_switch(sensor_info, sensor_info->deserial_port);

	// set serial
	ret = vcam_serial_setting(sensor_info);
	if (ret < 0) {
		vin_err("sensor_vcam_serial_init fail\n");
		return ret;
	}

	// link all
	vcam_link_switch(sensor_info, LINK_ALL);

    return ret;
}



int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

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


	return ret;
}

int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;
	// NULL
	return ret;
}

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;
	ret = sensor_deinit(sensor_info);

	return ret;
}

int32_t sensor_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size, i;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		pdata = max96712_stream_off_setting;
		setting_size = sizeof(max96712_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max96712_stream_off_setting quad error\n");
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		pdata = max96718_stream_off_setting;
		setting_size = sizeof(max96718_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max96718_stream_off_setting quad error\n");
			return ret;
		}
	} else {
		vin_err("%s stream on failed\n", deserial_if->deserial_name);
		return ret;
	}
	return ret;
}

int32_t sensor_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size, i;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		pdata = max96712_stream_on_setting;
		setting_size = sizeof(max96712_stream_on_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max96712_stream_on_setting quad error\n");
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (sensor_info->extra_mode == VCAM_D4_S0_YUV_DUO_INIT && sensor_info->deserial_port == 1) {
			vin_info("start do nothing\r\n");
			return 0;
		}
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		pdata = max96718_stream_on_setting;
		setting_size = sizeof(max96718_stream_on_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max96718_stream_on_setting quad error\n");
			return ret;
		}
	} else {
		vin_err("%s stream on failed\n", deserial_if->deserial_name);
		return ret;
	}
	return ret;
}


int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, tmp = 0;

	ret = sensor_stream_on(sensor_info);
	if (ret < 0) {
		vin_err("sensor_vcam_sensor_stream_on fail\n");
		return ret;
	}

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, tmp = 0;

	ret = sensor_stream_off(sensor_info);
	if (ret < 0) {
		vin_err("sensor_vcam_sensor_stream_on fail\n");
		return ret;
	}

	return ret;
}


int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
{
	int32_t ret = RET_OK;
    // NULL
	return ret;
}



sensor_module_t vcam = {
	.module = "vcam",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
    .get_sns_params = get_sns_info,
	.stream_off = sensor_stream_off,
	.stream_on = sensor_stream_on,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};

