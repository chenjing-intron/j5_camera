/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ov9284]:" fmt

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

#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/ov9284_setting.h"
#include "inc/sensor_effect_common.h"

static int write_register(int bus, uint8_t *pdata, int setting_size)
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

int sensor_poweron(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;

  vin_info("sensor_poweron \n");
  ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
                                  ov9284_power_on_setting[1] >> 1,
                                  ov9284_power_on_setting[3],
                                  ov9284_power_on_setting[4]);
  if (ret < 0) {
    vin_err("max20087 camera module 12v output power up fail! !, ret = %d\n",
           ret);
    return -1;
  }
  usleep(500 * 1000);
  sleep(1);
  return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
  ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
                                  ov9284_power_off_setting[1] >> 1,
                                  ov9284_power_off_setting[3],
                                  ov9284_power_off_setting[4]);

  vin_info("sensor_poweroff \n");
  if (ret < 0) {
    vin_err("max20087 camera module 12v output power down fail! !, ret = %d\n",
           ret);
    return -1;
  }
  usleep(500 * 1000);
  return ret;
}

int sensor_init(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = NULL;

  if (sensor_info->port != 0)
	return ret;
  /*======sensor_poweron======*/
  ret = sensor_poweron(sensor_info);
  if (ret < 0) {
    vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
    return ret;
  }

  /*======max9296a_max9295a======*/
  pdata = ov9284_init_setting;
  setting_size = sizeof(ov9284_init_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);
  if (ret < 0) {
    sensor_poweroff(sensor_info);
    vin_err("write register error\n");
  }
  return ret;
}

int sensor_start(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = NULL;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = NULL;

  if (sensor_info->port != 0)
	return ret;

  deserial_if = (deserial_info_t *)sensor_info->deserial_info;
  ret = ((deserial_module_t *)(deserial_if->deserial_ops))
            ->stream_on(deserial_if, sensor_info->deserial_port);

  pdata = ov9284_stream_on_setting;
  setting_size = sizeof(ov9284_stream_on_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);

  if (ret < 0) {
    vin_err("ov9284 max9296a stream_on fail\n");
    return -1;
  }
  vin_err("ov9284 max9296a stream_on success\n");
  return ret;
}

int sensor_stop(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = NULL;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = NULL;

  if (sensor_info->port != 0)
    return ret;

  deserial_if = (deserial_info_t *)sensor_info->deserial_info;
  ret = ((deserial_module_t *)(deserial_if->deserial_ops))
            ->stream_off(deserial_if, sensor_info->deserial_port);

  pdata = ov9284_stream_off_setting;
  setting_size = sizeof(ov9284_stream_off_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);

  if (ret < 0) {
    vin_err("ov9284 max9296a stream_off fail\n");
    return -2;
  }
  vin_err("ov9284 max9296a stream_off success\n");
  return ret;
}

int sensor_deinit(sensor_info_t *sensor_info) {
  return sensor_poweroff(sensor_info);
}

sensor_module_t ov9284 = {
    .module = "ov9284",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,
};
