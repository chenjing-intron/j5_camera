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
#include "inc/ov2311_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/hb_vin.h"

#define DEFAULT_SERIAL_ADDR		(0x40)
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define REG_ALIAS_ID_SER        0x0000
#define DEFAULT_SER_ADDR        0x80

enum MODE_TYPE {
	DMS_WITH_SUNNY_X3C_TRIP,
	DMS,
};

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

uint8_t link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint16_t reg1 = 0;
	uint16_t reg2 = 0;
	uint8_t  val = 0;
	int 	 val_read1 = 0;
	int 	 val_read2 = 0;
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
				val_read1 &= 0xEF;				  // open linkA
				val_read2 |= 0x04;				  // close linkB
			} else if (link_port == 1) {
				val_read1 |= 0x10;				  // close linkA
				val_read2 &= 0xFB;				  // open linkB
			} else if (link_port == LINK_ALL) {
				val_read1 &= 0xEF;				  // open linkA
				val_read2 &= 0xFB;				  // open linkB
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

static int32_t set_gpio(deserial_info_t *deserial_info)
{
	int32_t gpio, ret = -HB_CAM_SENSOR_POWERON_FAIL;

	for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
		vin_dbg("Set gpio level is %d for gpio%d\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
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

static int32_t poc_power_reset(sensor_info_t *sensor_info)
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
		usleep(200*1000);
		return ret;
	}
	return -RET_ERROR;
}

int32_t sensor_ov2311_des_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	int32_t bus, deserial_addr, board_type;
  if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;

  if (!strcmp(deserial_if->deserial_name, "max96712")) {
	 if (poc_addr != INVALID_POC_ADDR) {
#ifdef POC_RETRY_POLICY
				ret = poc_power_reset(sensor_info);
				if (ret < 0) {
					vin_err("poc_power_reset fail\n");
					return ret;
				}
#else
			setting_size = 1;
			poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
			ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
									 setting_size, poc_init_setting);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
			usleep(10 * 1000);
			ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
								 setting_size, poc_init_setting + 2);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
#endif
  } else if (!sensor_info->power_mode) {
			/* reset all serials replace to poc off */
			int32_t i2c_slave;
			for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
				vin_info("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
				hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, i2c_slave, 0x0010, 0xf1);
			}
		}
    board_type = vin_get_board_id();
    if (board_type == BOARD_ID_MATRIXDUO_A ||
        board_type == BOARD_ID_MATRIXDUO_A_V2 ||
        board_type == BOARD_ID_MATRIXDUO_A_V3) {
        pdata = max96712_dms_init_setting_4lane;
        setting_size = sizeof(max96712_dms_init_setting_4lane) / sizeof(uint8_t);
        ret = write_register(bus, pdata, setting_size);
        if (ret < 0) {
          vin_err("write max96712 dms register error\n");
          return ret;
        }
    }
  } else if (!strcmp(deserial_if->deserial_name, "max9296")) {
    if (sensor_info->port != 0)
      return ret;
    /*======sensor_poweron======*/
    ret = sensor_poweron(sensor_info);
    if (ret < 0) {
      vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
      return ret;
    }
    /*======max9296a_max9295a======*/
    pdata = ov2311_init_setting;
    setting_size = sizeof(ov2311_init_setting) / sizeof(uint8_t);
    ret = write_register(bus, pdata, setting_size);
    if (ret < 0) {
      sensor_poweroff(sensor_info);
      vin_err("write register error\n");
      return ret;
    }
  }
  deserial_if->init_state = 1;
	vin_dbg("deserial %s init done\n", deserial_if->deserial_name);
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
	ret = write_register(deserial_if->bus_num, serializer_pipex_setting,
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
		(sensor_info->deserial_port == 0)) {
			vin_info("set patch for max9296's second port\n");
			pdata = max9296_dual_setting_patch;
			pdata[3] = 0x53;
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				pdata[4] = 0x11;
			}
			setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
			ret = write_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("max9296_dual_setting_patch failed\n");
				return ret;
			}
	}

	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t);
	ret = write_register(deserial_if->bus_num,
		alias_id_setting[sensor_info->deserial_port], setting_size);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}

	usleep(5000);
	return ret;
}

int hotplug_init(sensor_info_t *sensor_info) {
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

  ret = link_switch(sensor_info, sensor_info->deserial_port);
  if (ret < 0) {
	  vin_err("link switch to des port_%d failed\n", sensor_info->deserial_port);
	  return ret;
  }
  /* ret = common_rx_rate_switch(sensor_info, 1);
  if (ret < 0) {
	  vin_err("rx rate switch to 3g failed\n");
	  return ret;
  } */
  // usleep(100*1000);
  ret = sensor_ov2311_serializer_init(sensor_info);
  if (ret < 0) {
	  vin_err("sensor_ovx3c_serializer_init fail\n");
	  return ret;
  }
  if (((strcmp(deserial_if->deserial_name, "max9296") &&
	  strcmp(deserial_if->deserial_name, "max96718"))) ||
	  sensor_info->deserial_port == 1) {
	  ret = link_switch(sensor_info, LINK_ALL);
	  if (ret < 0) {
		  vin_err("switch to link all failed for port%d\n",
			  sensor_info->port);
	  }
  }

  return ret;
}

int sensor_init(sensor_info_t *sensor_info) {
  int32_t ret = RET_OK;
  int32_t req;
  int32_t entry_num = sensor_info->entry_num;

  if (sensor_info->extra_mode == DMS_WITH_SUNNY_X3C_TRIP) return ret;

  req = hb_vin_mipi_pre_request(entry_num, 0, 0);
  if (req == 0) {
    vin_info("ov2311 request init begin req = %d\n", req);
    ret = sensor_ov2311_des_init(sensor_info);
    hb_vin_mipi_pre_result(entry_num, 0, ret);
    if (ret < 0) {
      vin_err("sensor_ov2311_des_init fail\n");
      return ret;
    }
    vin_info("ov2311 request init end\n");
  }
  return ret;
}

int sensor_start(sensor_info_t *sensor_info) {
  int ret = RET_OK;
  int setting_size = 0;
  int bus = sensor_info->bus_num;
  uint8_t *pdata = NULL;
  deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
  int32_t board_type = 0x00;

  // J5A add 3*ovx3c+1dms,enable dms max9295 stream on in J5A,J5B enable max96712 mipi stream on
  if ((deserial_if != NULL) && (deserial_if->deserial_name != NULL) &&
		(!strcmp(deserial_if->deserial_name, "max96712"))) {
    board_type = vin_get_board_id();
    if (board_type == BOARD_ID_MATRIXDUO_A ||
        board_type == BOARD_ID_MATRIXDUO_A_V2 ||
        board_type == BOARD_ID_MATRIXDUO_A_V3) {
      pdata = max96712_dms_parellel_enable;
      setting_size = sizeof(max96712_dms_parellel_enable)/sizeof(uint8_t);
      ret = write_register(bus, pdata, setting_size);
      if (ret < 0) {
        vin_err("write max96718_portb_out_setting error\n");
        return ret;
      }
    } else if (board_type == BOARD_ID_MATRIXDUO_B ||
            board_type == BOARD_ID_MATRIXDUO_B_V2 ||
            board_type == BOARD_ID_MATRIXDUO_B_V3) {
      pdata = max96712_j5b_dms_out_setting;
      setting_size = sizeof(max96712_j5b_dms_out_setting)/sizeof(uint8_t);
      ret = write_register(bus, pdata, setting_size);
      if (ret < 0) {
        vin_err("write max96718_porta_out_setting error\n");
        return ret;
      }
    }
	} else {
    if (sensor_info->port != 0)
    return ret;

    deserial_if = (deserial_info_t *)sensor_info->deserial_info;
    ret = ((deserial_module_t *)(deserial_if->deserial_ops))
              ->stream_on(deserial_if, sensor_info->deserial_port);

    pdata = ov2311_stream_on_setting;
    setting_size = sizeof(ov2311_stream_on_setting) / sizeof(uint8_t);
    ret = write_register(bus, pdata, setting_size);

    if (ret < 0) {
      vin_err("ov2311 max9296a stream_on fail\n");
      return -1;
    }
    vin_err("ov2311 max9296a stream_on success\n");
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
  ret = ((deserial_module_t *)(deserial_if->deserial_ops))
            ->stream_off(deserial_if, sensor_info->deserial_port);

  pdata = ov2311_stream_off_setting;
  setting_size = sizeof(ov2311_stream_off_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);

  if (ret < 0) {
    vin_err("ov2311 max9296a stream_off fail\n");
    return -2;
  }
  vin_err("ov2311 max9296a stream_off success\n");
  return ret;
}

int sensor_deinit(sensor_info_t *sensor_info) {
  int32_t poc_addr = sensor_info->serial_addr1;
  int32_t ret = RET_OK;
  if (poc_addr != INVALID_POC_ADDR)
    return sensor_poweroff(sensor_info);
  return ret;
}

sensor_module_t ov2311 = {
    .module = "ov2311",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,
    .hotplug_init = hotplug_init,
};
