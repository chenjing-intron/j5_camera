/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
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
#include <logging.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/irs2877a_setting.h"

#define FIRST_MODE_CONFIG  (1)
#define SECOND_MODE_CONFIG  (3)
#define THIRD_MODE_CONFIG  (2)
#define FOURTH_MODE_CONFIG  (4)
#define FIFTH_MODE_CONFIG  (5)
#define SIXTH_MODE_CONFIG  (0)
#define SEVENTH_MODE_CONFIG  (6)
//#define PRINT_LOG
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x3d)
#define DEFAULT_SERIAL_ADDR_A	(0x62)
#define DEFAULT_SERIAL_ADDR		(0x42)
#define DEFAULT_DESERIAL_ADDR		(0x2a)
#define INVALID_POC_ADDR		(0xFF)

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
			i2c_slave = pdata[i + 1];
			if (sensor_addr != 0 && i2c_slave == DEFAULT_SENSOR_ADDR)
				i2c_slave = sensor_addr;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = (pdata[i + 4] << 8) | pdata[i + 5];
			ret = hb_vin_i2c_write_reg16_data16(bus, i2c_slave, reg_addr, value);
			k = 10;
			while (ret < 0 && k--) {
				vin_warn("write sensor %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			}
			if (ret < 0) {
				vin_err("write x8b %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			i = i + len + 1;
			vin_info("write x8b %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1];
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
			} else if (serial_addr != 0 && i2c_slave == DEFAULT_SERIAL_ADDR) {
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


int sensor_poweron(sensor_info_t *sensor_info) {
    int gpio, ret = RET_OK;
    int power_gpio_num;
    int fd;
    int len;
    char name[64];

    pr_err(" entry sensor_poweron!\n");

    for(gpio = 0; gpio < sensor_info->gpio_num; gpio++)
    {
        memset(name, 0, sizeof(name));
        snprintf(name, sizeof(name), "/sys/class/gpio/gpio%d", sensor_info->gpio_pin[gpio]);
        if(access(name, F_OK) == 0)
        {
            pr_err("gpio export exist!\n");
            fd = open("/sys/class/gpio/unexport", O_WRONLY);
            if(fd < 0)
            {
                pr_err("open file path fail, occur error!\n");
                return -HB_CAM_SENSOR_POWERON_FAIL;
            }
            memset(name, 0, sizeof(name));
            len = snprintf(name, sizeof(name), "%d", sensor_info->gpio_pin[gpio]);
            if(write(fd, name, len) < 0)
            {
                pr_err("Failed to unexport gpio!");
                close(fd);
                return -HB_CAM_SENSOR_POWERON_FAIL;
            }
            close(fd);
        }
    }

    for(gpio = 0; gpio < sensor_info->gpio_num; gpio++)
    {
        if (sensor_info->gpio_pin[gpio] >= 0) {
	    pr_err("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
			 sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio],
			    1 - sensor_info->gpio_level[gpio]);
	    ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],sensor_info->gpio_level[gpio]);
        }
    }

    pr_err(" power off delay 150ms!\n");
    usleep(150 * 1000);
		
    for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) 
    {
	if(sensor_info->gpio_pin[gpio] >= 0)
        {	
            pr_err("power up num: %d\n", sensor_info->gpio_pin[gpio]);
  	    ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio], 1 - sensor_info->gpio_level[gpio]);
  	    if(ret < 0) 
            {
  		pr_err("vin_power_ctrl fail\n");
  		return -HB_CAM_SENSOR_POWERON_FAIL;
  	    }		
	}
    }
    pr_err(" power on delay 150ms!\n");
    usleep(150 * 1000);
    return ret;
}

int sensor_init(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  uint8_t *pdata = NULL;
  uint8_t try_count = 3;
  deserial_info_t *deserial_if = sensor_info->deserial_info;
  int32_t serial_addr = sensor_info->serial_addr;
  int32_t poc_addr = sensor_info->serial_addr1;
  int32_t sensor_addr = sensor_info->sensor_addr;
  int32_t bus, deserial_addr;

  if (deserial_if == NULL) {
	vin_err("no deserial here\n");
	return -1;
  }
  bus = deserial_if->bus_num;
  deserial_addr = deserial_if->deserial_addr;
  if (deserial_if->init_state == 1) {
    pr_err("irs2877a_des_init once !!\n");
    return ret;
  }

  while(try_count){
#if 0	  
    ret = sensor_poweron(sensor_info);
    if (ret < 0) {
      pr_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
      return ret;
    }
    pr_err("sensor_poweron %s OK!\n", sensor_info->sensor_name);
#endif
    pdata = irs2877a_init_setting;
    setting_size = sizeof(irs2877a_init_setting) / sizeof(uint8_t);
    pr_info("sensor_init enter setting_size %d \n", setting_size);
    ret =write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);

    switch(sensor_info->extra_mode)
    {
        case FIRST_MODE_CONFIG:
             pdata = Mode5_60Mhz_30fps_setting;
             setting_size = sizeof(Mode5_60Mhz_30fps_setting) / sizeof(uint8_t);
             break;
        case SECOND_MODE_CONFIG:
             pdata = Mode5_80Mhz_30fps_setting;
             setting_size = sizeof(Mode5_80Mhz_30fps_setting) / sizeof(uint8_t);
             break;
        case THIRD_MODE_CONFIG:
             pdata = Mode5_80Mhz_24fps_setting;
             setting_size = sizeof(Mode5_80Mhz_24fps_setting) / sizeof(uint8_t);
             break;
        case FOURTH_MODE_CONFIG:
             pdata = Mode9_60to80Mhz_30fps_setting;
             setting_size = sizeof(Mode9_60to80Mhz_30fps_setting) / sizeof(uint8_t);
             break;
        case FIFTH_MODE_CONFIG:
             pdata = face_id_30fps_setting;
             setting_size = sizeof(face_id_30fps_setting) / sizeof(uint8_t);
             break;
        case SIXTH_MODE_CONFIG:
             pdata = Mode5_60Mhz_24fps_setting;
             setting_size = sizeof(Mode5_60Mhz_24fps_setting) / sizeof(uint8_t);
             break;
        case SEVENTH_MODE_CONFIG:
             pdata = Mode5_60Mhz_30fps_trigger_setting;
             setting_size = sizeof(Mode5_60Mhz_30fps_trigger_setting) / sizeof(uint8_t);
             break;
        default:
             pr_err("config mode not exist\n");
             return -HB_CAM_SERDES_CONFIG_FAIL;
    }
    pr_info("sensor_init enter sensor_register_size %d\n", setting_size);
    ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
    if (ret < 0) {
        pr_err("sensor_reg write register error\n");
        return -HB_CAM_SERDES_CONFIG_FAIL;
    }

    pdata = max96712_setting;
    setting_size = sizeof(max96712_setting) / sizeof(uint8_t);
    pr_info("sensor_init enter max96712_register_size %d\n", setting_size);
    ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
    if (ret < 0) {
      //sensor_poweroff(sensor_info);
      pr_err("sensor_init write register error\n");
       --try_count;
      if(!try_count){
        pr_err("sensor_init write register error, try 3 times!\n");
        return -HB_CAM_SERDES_CONFIG_FAIL;
      } 
      else
      {
        continue;
      }
    }
    else
    {
      deserial_if->init_state = 1;
      pr_info("sensor_init Done !\n");
      return RET_OK;
    }
  }
  return ret;
}

int sensor_start(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  deserial_info_t *deserial_if = sensor_info->deserial_info;
  int32_t serial_addr = sensor_info->serial_addr;
  int32_t poc_addr = sensor_info->serial_addr1;
  int32_t sensor_addr = sensor_info->sensor_addr;
  int32_t bus, deserial_addr;

  if (deserial_if == NULL) {
	vin_err("no deserial here\n");
	return -1;
  }
  bus = deserial_if->bus_num;
  deserial_addr = deserial_if->deserial_addr;

  uint8_t *pdata = irs2877a_stream_on_setting;
  setting_size = sizeof(irs2877a_stream_on_setting) / sizeof(uint8_t);
  ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
  if (ret < 0) {
    pr_err("irs2877a stream_on fail\n");
    return -1;
  }
  return ret;
}

int sensor_stop(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  
  int setting_size = 0;
  deserial_info_t *deserial_if = sensor_info->deserial_info;
  int32_t serial_addr = sensor_info->serial_addr;
  int32_t poc_addr = sensor_info->serial_addr1;
  int32_t sensor_addr = sensor_info->sensor_addr;
  int32_t bus, deserial_addr;

  if (deserial_if == NULL) {
	vin_err("no deserial here\n");
	return -1;
  }
  bus = deserial_if->bus_num;
  deserial_addr = deserial_if->deserial_addr;

  uint8_t *pdata = irs2877a_stream_off_setting;
  setting_size = sizeof(irs2877a_stream_off_setting) / sizeof(uint8_t);
  ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
  if (ret < 0) {
    pr_err("irs2877a stream_off fail\n");
    return -1;
  } 
  return ret;
}
int sensor_deinit(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int gpio;

		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
		  if (sensor_info->gpio_pin[gpio] != -1) {
  			ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
  									sensor_info->gpio_level[gpio]);
  			if (ret < 0) {
  			  pr_err("vin_power_ctrl fail\n");
  			  return -HB_CAM_SENSOR_POWERON_FAIL;
  			}
		  }
		}

  return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info) {
  int gpio, ret = RET_OK;

  ret = sensor_deinit(sensor_info);
  return ret;
}

sensor_module_t irs2877a = {
    .module = "irs2877a",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,

};
