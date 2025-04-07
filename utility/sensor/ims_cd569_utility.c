/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ims_cd569]:" fmt

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/i2c-dev.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
// #include "inc/max20084_setting.h"
#include "inc/ims_cd569_setting.h"
// #include "inc/x2_camera.h"
// #include "inc/x2_camera_common.h"

int sensor_deinit(sensor_info_t *sensor_info);
int write_register(int bus, uint8_t *pdata, int setting_size) {
  int ret = RET_OK;
  uint8_t i2c_slave;
  uint16_t reg_addr, value;
  uint32_t delay;
  int i, len;

  for (i = 0; i < setting_size;) {
    len = pdata[i];
    if (len == 4) {
      i2c_slave = pdata[i + 1] >> 1;
      reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
      value = pdata[i + 4];
      // usleep(2000*1000);
      printf("init i2c_addr:%0x, reg:0x%x, value:%x\n", i2c_slave, reg_addr,value);
      ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
      if (ret < 0) {
        vin_err("write i2c_addr:%0x, reg:%0x, error\n", i2c_slave, reg_addr);
        return ret;
      }
      i = i + len + 1;
      // sleep(2);

    } else if (len == 3) {
      i2c_slave = pdata[i + 1] >> 1;
      reg_addr = pdata[i + 2];
      value = pdata[i + 3];
      ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
      if (ret < 0) {
        vin_err("write i2c_addr:%0x, reg:%0x, error\n", i2c_slave, reg_addr);
        return ret;
      }
      i = i + len + 1;
      printf("init max96701 reg:0x%x  value:%x\n", reg_addr, value);
    } else if (len == 0) {
      delay = pdata[i + 1];
      usleep(delay * 1000);
      i = i + 2;
    }
  }
  return ret;
}

int sensor_poweron(sensor_info_t *sensor_info) {
  int gpio, ret = RET_OK;

  return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info) {
  int gpio, ret = RET_OK;

  ret = sensor_deinit(sensor_info);
  return ret;
}
int sensor_init(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = NULL;

  /*======max9296a_max96717f======*/
  pdata = ims_cd569_init_setting;
  printf("szz cd569 ims \n");
  setting_size = sizeof(ims_cd569_init_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);
  if (ret < 0) {
    sensor_poweroff(sensor_info);
    vin_err("write register error\n");
  }
  return ret;
}

int sensor_start(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = ims_cd569_stream_on_setting;
  setting_size = sizeof(ims_cd569_stream_on_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);
  if (ret < 0) {
    vin_err("ims290 max9296a stream_on fail\n");
    return -1;
  }
  return ret;
}

int sensor_stop(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  deserial_info_t *deserial_if = NULL;

  deserial_if = (deserial_info_t *)sensor_info->deserial_info;
  ret = ((deserial_module_t *)(deserial_if->deserial_ops))
            ->stream_off(deserial_if, sensor_info->deserial_port);
  if (ret < 0) {
    vin_err("ims290 max9296a stream_off fail\n");
    return -1;
  }
  return ret;
}
int sensor_deinit(sensor_info_t *sensor_info) {
  int ret = RET_OK;

  return ret;
}

sensor_module_t ims_cd569 = {
    .module = "ims_cd569",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,
};
