/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ov2311]:" fmt

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
#include "./hb_i2c.h"
#include "./hb_cam_utility.h"
#include "inc/sensor_common.h"
#include "inc/ov2311s_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/hb_vin.h"

#define DEFAULT_SERIAL_ADDR		(0x40)
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_DESERIAL_ADDR	(0x48)

int32_t write_register(int32_t bus, int32_t deserial_addr, int32_t poc_addr, int32_t serial_addr,
			int32_t sensor_addr, uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int32_t i, len, k;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
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


int sensor_poweron(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;

  vin_info("sensor_poweron \n");
  ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
                                  ov2311_power_on_setting[1] >> 1,
                                  ov2311_power_on_setting[3],
                                  ov2311_power_on_setting[4]);
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
                                  ov2311_power_off_setting[1] >> 1,
                                  ov2311_power_off_setting[3],
                                  ov2311_power_off_setting[4]);

  vin_info("sensor_poweroff \n");
  if (ret < 0) {
    vin_err("max20087 camera module 12v output power down fail! !, ret = %d\n",
           ret);
    return -1;
  }
  usleep(500 * 1000);
  return ret;
}

int sensor_ov2311_serializer_init(sensor_info_t *sensor_info)
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

	setting_size = sizeof(serializer_pipex_setting) / sizeof(uint8_t);
	ret = write_j5_register(deserial_if->bus_num, serializer_pipex_setting,
		setting_size);
	if (ret < 0) {
		vin_err("serializer_pipez_setting failed for port%d\n",
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
			vin_info("set patch for max9296's second port\n");
			pdata = max9296_dual_setting_patch;
			pdata[3] = 0x53;
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

int sensor_init(sensor_info_t *sensor_info) {
  int32_t ret = RET_OK;
  int32_t req;
  int32_t entry_num = sensor_info->entry_num;
  uint8_t *pdata = NULL;
  int setting_size = 0;
  deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
  if (deserial_if == NULL) {
	  vin_err("no deserial here error\n");
	  return -1;
  }

  req = hb_vin_mipi_pre_request(entry_num, 0, 0);
  if (req == 0) {
    vin_info("ov2311 request init begin req = %d\n", req);
    ret = deserial_setting(sensor_info);
    hb_vin_mipi_pre_result(entry_num, 0, ret);
    if (ret < 0) {
      vin_err("sensor_ov2311_des_init fail\n");
      return ret;
    }
    vin_info("ov2311 request init end\n");
  }

  ret = common_link_switch(sensor_info, sensor_info->deserial_port);
  if (ret < 0) {
	  vin_err("link switch to des port_%d failed\n", sensor_info->deserial_port);
	  return ret;
  }
  /* ret = common_rx_rate_switch(sensor_info, 1);
  if (ret < 0) {
	  vin_err("rx rate switch to 3g failed\n");
	  return ret;
  } */
  usleep(100*1000);
  ret = sensor_ov2311_serializer_init(sensor_info);
  if (ret < 0) {
	  vin_err("sensor_ovx3c_serializer_init fail\n");
	  return ret;
  }
  if (((strcmp(deserial_if->deserial_name, "max9296") &&
	  strcmp(deserial_if->deserial_name, "max96718"))) ||
	  sensor_info->deserial_port == 1) {
	  ret = common_link_switch(sensor_info, LINK_ALL);
	  if (ret < 0) {
		  vin_err("switch to link all failed for port%d\n",
			  sensor_info->port);
	  }
  }

  return ret;
}

int sensor_start(sensor_info_t *sensor_info) {
  int ret = RET_OK, req;
  deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
  int32_t entry_num = sensor_info->entry_num;

  if (deserial_if) {
	  req = hb_vin_mipi_pre_request(entry_num, 1, 0);
	  if (req == 0) {
		  ret = sensor_serdes_stream_on(sensor_info);
		  if (ret < 0) {
			  ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
			  vin_err("%d : %s sensor_ovx3c_serdes_stream_on fail\n",
					 __LINE__, sensor_info->sensor_name);
		  }
		  hb_vin_mipi_pre_result(entry_num, 1, ret);
	  }
  }

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
  if (deserial_if) {
	  if (sensor_info->config_index & DES_STREAMOFF) {
		  ret = sensor_serdes_stream_off(sensor_info);
		  if (ret < 0) {
			  ret = -HB_CAM_SERDES_STREAM_OFF_FAIL;
			  vin_err("%d : %s sensor_ov2311_serdes_stream_off fail\n",
					 __LINE__, sensor_info->sensor_name);
		  }
	  }
  }
  return ret;
}

int sensor_deinit(sensor_info_t *sensor_info) {
  int32_t poc_addr = sensor_info->serial_addr1;
  int32_t ret = RET_OK;
  if (poc_addr != INVALID_POC_ADDR)
    return sensor_poweroff(sensor_info);
  return ret;
}

sensor_module_t ov2311s = {
    .module = "ov2311s",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,
};
