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
#include <sys/shm.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ovx1f_setting.h"
#include "inc/sensor_effect_common.h"

uint8_t link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0;

	if (!strcmp(deserial_if->deserial_name, "max96712")) {
	    if (link_port < DES_PORT_NUM_MAX) {
			val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, LINK_ENABLE_REG);
			val = val | (1 << link_port);
		} else {
            pr_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
                deserial_if->deserial_name);
            return -1;
	    }

		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, LINK_RESET_REG, val);
		if (ret < 0) {
			pr_err("%s reset to port 0x%x for des-%s failed!\n", __func__, link_port,
				deserial_if->deserial_name);
			return -1;
	    }
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, LINK_ENABLE_REG, val);
		if (ret < 0) {
			pr_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
				deserial_if->deserial_name);
			return -1;
	    }
	} else {
		// NULL
	}

	vin_info("%s switch to port 0x%x successfully for des-%s!\n",
		__func__, link_port, deserial_if->deserial_name);
	usleep(20 * 1000);
	return 0;
}

/**
 * @brief sensor_ovx1f_deserializer_init : write deserial init setting,
 *                                 including 965, 9296, 96712, etc.
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_ovx1f_deserializer_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;

	if (poc_addr != INVALID_POC_ADDR) {
		// close all power
		setting_size = 1;
		poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
		ret = vin_write_array(deserial_if->bus_num, poc_addr, REG8_VAL8,
									setting_size, poc_init_setting);
		if (ret < 0) {
			vin_err("write poc 0x%02x init setting error\n", poc_addr);
			return ret;
		}
	}

	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		// set deserial
		setting_size = 1;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, REG16_VAL8,
				setting_size, max96712_quad_init_setting_base);
		usleep(300 * 1000);
		setting_size = sizeof(max96712_quad_init_setting_base) / sizeof(uint32_t) / 2 - 1;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, REG16_VAL8,
				setting_size, max96712_quad_init_setting_base + 2);
		if (ret < 0) {
			vin_err("write deserial register error\n");
			return ret;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	deserial_if->init_state = 1;
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}


int sensor_ovx1f_serializer_init(sensor_info_t *sensor_info)
{
    int addr, reg;
    int ret = RET_OK;
    uint8_t *pdata = NULL;
    int setting_size = 0;

    deserial_info_t *deserial_if = NULL;
    deserial_if = (deserial_info_t *)sensor_info->deserial_info;
    if (deserial_if == NULL) {
		pr_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
    }

    setting_size = sizeof(serializer_gmsl1_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, DEFAULT_SERIAL_ADDR,
		REG8_VAL8, setting_size, serializer_gmsl1_setting);
    if (ret < 0) {
		pr_err("serializer_gmsl1_setting failed for port%d\n",
			sensor_info->deserial_port);
	    return ret;
    }

	pr_info("set alias id!\n");
	ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num,
		DEFAULT_SERIAL_ADDR, alias_id_setting[sensor_info->deserial_port][0],
		alias_id_setting[sensor_info->deserial_port][1]);
	if (ret < 0) {
		pr_err("set alias id failed\n");
		return ret;
	}
	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t) / 2;
	for (int i = 1; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num,
			sensor_info->serial_addr, alias_id_setting[sensor_info->deserial_port][2 * i],
			alias_id_setting[sensor_info->deserial_port][2 * i + 1]);
		if (ret < 0) {
			pr_err("alias_id_setting failed\n");
			return ret;
		}
	}
    usleep(100 * 1000);
    pr_info("sensor %s serializer init done\n", sensor_info->sensor_name);

    return ret;
}

int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;
	int32_t poc_addr = sensor_info->serial_addr1;

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

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK, val = 0;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t poc_addr = sensor_info->serial_addr1;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	static int reg_val = 0x10;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port, drop\n", __func__);
	} else {
		if(sensor_info->sen_devfd <= 0) {
			char str[25] = {0};

			snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
			if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
				vin_err("port%d: %s open fail\n", sensor_info->deserial_port, str);
				return -RET_ERROR;
			}
		}
		vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
				sensor_info->dev_port, sensor_info->sen_devfd);
	}

	if (NULL != sensor_info->deserial_info) {
		req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		if (deserial_if && (deserial_if->init_state == 0)) {
			ret = sensor_ovx1f_deserializer_init(sensor_info);
			hb_vin_mipi_pre_result(sensor_info->entry_num, 0, ret);
			if (ret < 0) {
				pr_err("ovx1f_deserializer_init fail\n");
				return -RET_ERROR;
			}
	    }

		if (poc_addr != INVALID_POC_ADDR) {
			setting_size = 1;
			val = reg_val | (1 << (sensor_info->deserial_port));
			ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num,
					poc_addr, 0x01, val);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
			reg_val = val;
			usleep(100*1000);
		}

	    ret = link_switch(sensor_info, sensor_info->deserial_port);
	    if (ret < 0) {
			pr_err("link switch to port_%d failed\n", sensor_info->deserial_port);
			return ret;
	    }
		usleep(30 * 1000);

	    ret = sensor_ovx1f_serializer_init(sensor_info);
	    if (ret < 0) {
			pr_err("sensor_ovx1f_serializer_init fail\n");
			return ret;
	    }
	}

	return ret;
}

int32_t sensor_ovx1f_serdes_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		uint32_t setting_size = sizeof(max96712_start_setting) / sizeof(uint16_t) / 2;
		for (int32_t i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
					max96712_start_setting[2*i], (uint8_t)(max96712_start_setting[2*i+1] & 0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_info("sensor_start write %s successfully\n", deserial_if->deserial_name);
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}
unlock:
	return ret;
}

int32_t sensor_ovx1f_serdes_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		uint32_t setting_size = sizeof(max96712_stop_setting) / sizeof(uint16_t) / 2;
		for (int32_t i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
					max96712_stop_setting[2*i], (uint8_t)(max96712_stop_setting[2*i+1] & 0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_info("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}
unlock:
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t setting_size = 0, i, req;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t ret = RET_OK, tmp = 0;
	if ( !(strcmp(deserial_if->deserial_name, "max96712")) ) {
		ret = sensor_ovx1f_serdes_stream_on(sensor_info);
		if (ret < 0) {
			pr_err("%d : %s sensor_ovx1f_serdes_stream_on fail\n",
				__LINE__, sensor_info->sensor_name);
		}
	}

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size = 0, i;
	uint8_t value;

	if ( !(strcmp(deserial_if->deserial_name, "max96712")) ) {
		ret = sensor_ovx1f_serdes_stream_off(sensor_info);
		if (ret < 0) {
			pr_err("%d : %s sensor_ovx1f_serdes_stream_on fail\n",
				__LINE__, sensor_info->sensor_name);
		}
	}
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

sensor_module_t ovx1f = {
	.module = "ovx1f",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};

