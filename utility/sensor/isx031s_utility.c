/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[isx031]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/shm.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "inc/sensor_effect_common.h"
#include "./hb_i2c.h"
#include "inc/sensor_common.h"
#include "inc/isx031s_setting.h"

#define VERSION_SENSING "0.1.1"

#define RETRY_TIME_MAX  5

#define PILOT_A1        0
#define PILOT_A2        1
#define BUF_LEN         128

#define SENSOR_REG_WIDTH    REG16_VAL16
#define SERDES_REG_WIDTH    REG16_VAL8
#define POC_REG_WIDTH       REG8_VAL8
#define DEFAULT_POC_ADDR    (0x28)
#define DEFAULT_SERIAL_ADDR (0x40)
#define INVALID_POC_ADDR	(0xFF)
#define DEFAULT_SENSOR_I2C_ADDR  (0x34)
#define HKISX031_SERIAL_I2C_ADDR (0x84)
#define DEFAULT_DESERIAL_ADDR	(0x48)

#define COMMON_ADDR_SENSOR  0x1A

#ifndef I2C_SMBUS_BLOCK_MAX
#define I2C_SMBUS_BLOCK_MAX 32
#endif

#define EFL_X_ADDR_031     (0x00)
#define EFL_Y_ADDR_031     (0x04)
#define COD_X_ADDR_031     (0x08)
#define COD_Y_ADDR_031     (0x12)
#define K1_ADDR_031        (0x1C)
#define K2_ADDR_031        (0x20)
#define P1_ADDR_031        (0xD0)
#define P2_ADDR_031        (0xD8)
#define K3_ADDR_031        (0x2C)
#define K4_ADDR_031        (0x30)
#define K5_ADDR_031        (0xF0)
#define K6_ADDR_031        (0xF8)

#define INTRIN_DATA_SIZE    58

enum MODE_TYPE {
	SENSING_24M_H190S,
	HKISX031_2,
	HKISX031,
	SENSING_24M_H190X,
	SENSING_24M_H190XA,
};

typedef union _eepromtrans
{
	float dEeepromDouble;
	unsigned char lEeepromLong[4];
} eepromtrans_u;

static uint32_t dphy_data_rate_reg_96712[] = {
	REG_DPHY_0_DATA_RATE_96712, REG_DPHY_1_DATA_RATE_96712,
	REG_DPHY_2_DATA_RATE_96712, REG_DPHY_3_DATA_RATE_96712,
};

/**
 * @brief write_register : write sensor and serdes reg
 *
 * @param [in] bus : i2c num
 * @param [in] pdata : setting need to write
 * @param [in] setting_size : setting num
 *
 * @return ret
 */
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


int serializer_init_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	int retry_time = RETRY_TIME_MAX;
	int data_rate = 0;
	uint8_t* p_serializer_pipez_setting = NULL;
	int setting_size = 0;
	int alias_setting_size = 0;
	uint8_t ser_addr = 0x80;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	switch(sensor_info->extra_mode) {
		case SENSING_24M_H190S:
			data_rate = 0;
			p_serializer_pipez_setting = serializer_pipez_setting;
			setting_size = sizeof(serializer_pipez_setting)/ sizeof(uint8_t);
			ser_addr = DEFAULT_SER_ADDR;
			break;
		case HKISX031:
		case HKISX031_2:
			data_rate = 1;
			p_serializer_pipez_setting = serializer_pipez_setting_hk;
			setting_size = sizeof(serializer_pipez_setting_hk) / sizeof(uint8_t);
			ser_addr = HAOMO_SER_ADDR;
			break;
		case SENSING_24M_H190X:
			data_rate = 0;
			p_serializer_pipez_setting = serializer_pipez_setting;
			setting_size = sizeof(serializer_pipez_setting)/ sizeof(uint8_t);
			ser_addr = DEFAULT_SER_ADDR;
			break;
		case SENSING_24M_H190XA:
			data_rate = 1;
			p_serializer_pipez_setting = serializer_pipez_setting;
			setting_size = sizeof(serializer_pipez_setting)/ sizeof(uint8_t);
			ser_addr = DEFAULT_SER_ADDR;
			break;
		default: vin_err("no serializer here error\n"); return -1;
	}

	while(retry_time > 0) {
		ret = common_rx_rate_switch(sensor_info, data_rate);
		if (ret < 0) {
			vin_err("rx rate switch to 6g failed\n");
			return ret;
		}
		/*if (sensor_info->extra_mode == SENSING_24M_H190X) {
			ret = common_rx_rate_switch(sensor_info, 1);
			if (ret < 0) {
				vin_err("rx rate switch to 6g failed\n");
				return ret;
			}
		}*/
		ret = write_j5_register(deserial_if->bus_num, p_serializer_pipez_setting,
				setting_size);
		if(ret < 0) {
			vin_err("serializer_pipez_setting failed for port%d\n",
			sensor_info->port);
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
					sensor_info->serial_addr, REG_ALIAS_ID_SER, ser_addr);
			if(ret < 0) {
				vin_err("set alias id to default failed for port%d\n",
				sensor_info->port);
				retry_time--;
				vin_info("retry for %d times\n", RETRY_TIME_MAX - retry_time);
				continue;
			}
		}
		break;
	}
	if(retry_time <= 0) {
		vin_info("retry for %d times\n", RETRY_TIME_MAX - retry_time);
		return ret;
	}
	if ((!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) &&
		(sensor_info->deserial_port == 1)) {
			vin_info("set patch for max9296's second port\n");
			pdata = max9296_dual_setting_patch;
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				pdata[4] = 0x11;
			}
			if (data_rate == 0) {
				pdata[9] = 0x12;
			}
			setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
			ret = write_j5_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("max9296_dual_setting_patch failed\n");
				return ret;
			}
	}

	vin_info("set alias id!\n");
	if ((sensor_info->extra_mode & 0xff) == HKISX031 ||
		(sensor_info->extra_mode & 0xff) == HKISX031_2) {
		alias_id_setting[sensor_info->deserial_port][1] = HKISX031_SERIAL_I2C_ADDR;
	}
	alias_id_setting[sensor_info->deserial_port][24] = DEFAULT_SENSOR_I2C_ADDR;
	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t);
	ret = write_j5_register(deserial_if->bus_num,
		alias_id_setting[sensor_info->deserial_port], setting_size);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}

	if (sensor_info->config_index & TRIG_MODE) {
		if (sensor_info->extra_mode == SENSING_24M_H190X ||
			sensor_info->extra_mode == HKISX031 ||
			sensor_info->extra_mode == HKISX031_2) {
			setting_size = sizeof(max96717_max9295_trigger_mfp7) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr,
					SERDES_REG_WIDTH, setting_size, max96717_max9295_trigger_mfp7);
			if (ret < 0) {
				vin_err("write serializer_trig_setting error\n");
				return ret;
			}
		} else if (sensor_info->extra_mode == SENSING_24M_H190XA) {
			setting_size = sizeof(max96717f_trigger_mfp7) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr,
					SERDES_REG_WIDTH, setting_size, max96717f_trigger_mfp7);
			if (ret < 0) {
				vin_err("write serializer_trig_setting error\n");
				return ret;
			}
		} else {
		    setting_size = sizeof(max96717_max9295_trigger_mfp8) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr,
					SERDES_REG_WIDTH, setting_size, max96717_max9295_trigger_mfp8);
			if (ret < 0) {
				vin_err("write serializer_trig_setting error\n");
				return ret;
			}
		}
	}

	usleep(5000);
	vin_info("sensor %s serializer init done\n", sensor_info->sensor_name);
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int i, req, req1, ret = RET_OK;
	int setting_size = 0;
	char str[24] = {0};
	deserial_info_t *deserial_if = NULL;
	uint8_t *pdata = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	int32_t entry_num = sensor_info->entry_num;
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	int desport_num = deserial_if->reserved[0];

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if (sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port_%d dev_port %d open fail\n", sensor_info->port,
				sensor_info->dev_port);
			return -RET_ERROR;
		}
	}

	req = hb_vin_mipi_pre_request(entry_num, 0, 0);
	if (req == 0) {
	  vin_info("031 request init begin req = %d\n", req);
	  ret = deserial_setting(sensor_info);
	  hb_vin_mipi_pre_result(entry_num, 0, ret);
	  if (ret < 0) {
		vin_err("sensor_031_des_init fail\n");
		return ret;
	  }
	  vin_info("031 request init end\n");
	}

	ret = common_link_switch(sensor_info, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("link switch to port_%d failed\n", sensor_info->deserial_port);
		return ret;
	}
	usleep(100*1000);
	ret = serializer_init_setting(sensor_info);
	if (ret < 0) {
		vin_err("serializer_init_setting fail\n");
		return ret;
	}

	/* set isx031 trigger mode */
	if (sensor_info->config_index & TRIG_MODE) {
		usleep(100*1000);
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_off_setting[i * 2],
				isx031_stream_off_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s set trig stream off failed\n", sensor_info->sensor_name);
				return ret;
			}
		}
		usleep(10 * 1000);
		if(sensor_info->config_index & TRIG_SHUTTER) {
			setting_size = sizeof(isx031_trigger_shutter_mode_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_trigger_shutter_mode_setting);
		} else if (sensor_info->config_index & TRIG_EXTERNAL) {
			setting_size = sizeof(isx031_trigger_external_mode_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_trigger_external_mode_setting);
		}
		if (ret < 0) {
			vin_err("senor %s write trigger shutter mode setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_on_setting[i * 2],
				isx031_stream_on_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s set trig stream on failed\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}

	if (sensor_info->config_index & TEST_PATTERN) {
		pr_info("set test pattern\n");
		usleep(40 * 1000);
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
						sensor_info->sensor_addr, isx031_stream_off_setting[i * 2],
						isx031_stream_off_setting[i * 2 + 1]);
				if (ret < 0) {
						pr_err("%s set stream off failed\n", sensor_info->sensor_name);
						return ret;
				}
		}
		usleep(40 * 1000);
		setting_size = sizeof(isx031_pattern_mode_setting) /
				sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_pattern_mode_setting);
		if (ret < 0) {
				pr_err("senor %s write isx031_pattern_mode_setting\n",
						sensor_info->sensor_name);
				return ret;
		}
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
				ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
						sensor_info->sensor_addr, isx031_stream_on_setting[i * 2],
						isx031_stream_on_setting[i * 2 + 1]);
				if (ret < 0) {
						pr_err("%s set stream on failed\n", sensor_info->sensor_name);
						return ret;
				}
		}
		usleep(40 * 1000);
    }

	if (((25 == sensor_info->fps) || (sensor_info->extra_mode == HKISX031)) &&
		(sensor_info->extra_mode != HKISX031_2)) {
		usleep(10 * 1000);
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size,	isx031_stream_off_setting);
		if (ret < 0) {
			vin_err("sensor %s stream off failed!\n",
				sensor_info->sensor_name);
			return ret;
		}
		// usleep(100 * 1000);
		// ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
		// 	sensor_info->sensor_addr, isx031_vmax_setting[0],
		// 	isx031_vmax_setting[1]);
		// if (ret < 0) {
		// 	vin_err("%s set vmax stream failed\n", sensor_info->sensor_name);
		// 	return ret;
		// }
		usleep(200 * 1000);
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, isx031_stream_on_setting[0],
			isx031_stream_on_setting[1]);
		if (ret < 0) {
			vin_err("%s set stream on failed\n", sensor_info->sensor_name);
			return ret;
		}
	}

	if (sensor_info->config_index & DUAL_LANE) {
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(deserial_if->bus_num, COMMON_ADDR_SENSOR,
			SERDES_REG_WIDTH, setting_size,	isx031_stream_off_setting);
		if (ret < 0) {
			vin_err("sensor %s stream off failed!\n",
				sensor_info->sensor_name);
			return ret;
		}
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

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

int sensor_stream_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size, i;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		setting_size = sizeof(max96712_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max96712_stream_off_setting[2 * i],
				(uint8_t)(max96712_stream_off_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream off failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size = sizeof(max9296_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max9296_stream_off_setting[2 * i],
				(uint8_t)(max9296_stream_off_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream off failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else {
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		vin_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_off_setting[i * 2],
				isx031_stream_off_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s stream off failed\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	return ret;
}

int sensor_stream_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size, i;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 2;
		for (int i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max96712_stream_on_setting[2 * i],
				(uint8_t)(max96712_stream_on_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s strema on failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size = sizeof(max9296_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max9296_stream_on_setting[2 * i],
				(uint8_t)(max9296_stream_on_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream on failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else {
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_on_setting[i * 2],
				isx031_stream_on_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s : stream on failed\n", sensor_info->sensor_name);
			}
		}
	}
	return ret;
}

static float float_trans_from_char(uint8_t *raw_data, uint8_t base)
{
	eepromtrans_u temp;

	memset(temp.lEeepromLong, 0, sizeof(float));
	memcpy(temp.lEeepromLong, raw_data + base, sizeof(float));

	return temp.dEeepromDouble;
}

static int float_trans_to_char(uint8_t *trans_data, float raw_data, uint8_t base)
{
	eepromtrans_u temp;

	temp.dEeepromDouble = raw_data;
	memcpy(trans_data + base, temp.lEeepromLong, sizeof(float));

	return 0;
}

static int flash_size_set(sensor_info_t *sensor_info)
{
	int ret = 0;
	uint8_t reg_addr[4] = {};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0, };
	uint8_t buf_size = 0;

	// Register Access request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%s : Register Access request failed\n",
			sensor_info->sensor_name);
	}

	// FLASH_SIZE
	reg_addr[0] = 0x8a;
	reg_addr[1] = 0x58;
	reg_size = 2;

	/*
	4 Mbits: 0x080000
	8 Mbits: 0x100000
	16 Mbits: 0x200000
	32 Mbits: 0x400000
	*/
	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x10;
	buf_size = 3;

	// Serial NOR Flash size set (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set to 0x%x failed\n",
			sensor_info->sensor_name, buf[2] << 16 | buf[1] << 8 | buf[0]);
	}
	return ret;
}

static int flash_read(sensor_info_t *sensor_info, uint8_t addr,
	uint8_t* data, uint32_t bytes)
{
	int ret = 0;
	uint8_t reg_addr[4] = {0, };
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0, };
	uint8_t buf_size = 0;
	uint8_t offset = 0;

	ret = flash_size_set(sensor_info);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash access unlock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF4);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access unlock request failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash all read request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF7);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access request failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash Read Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x01;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash read request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Read request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1500 * 1000);

	// Buffer read request (Host -> Sensor)
	while (bytes > 0) {
		memset(buf, 0, I2C_SMBUS_BLOCK_MAX);
		reg_size = 2;
		memset(reg_addr, 0, reg_size);
		reg_addr[0] = 0x00;
		reg_addr[1] = addr + offset;

		if (bytes > I2C_SMBUS_BLOCK_MAX - reg_size) {
			buf_size = I2C_SMBUS_BLOCK_MAX - reg_size;
			ret = hb_i2c_read(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : read reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, reg_addr[1], buf_size);
			}
		} else {
			buf_size = bytes;
			ret = hb_i2c_read(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : read reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, reg_addr[1], buf_size);
			}
		}
		memcpy(data + offset, buf, buf_size);
		bytes -= buf_size;
		offset += buf_size;
	}

	// Serial NOR Flash access lock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF5);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access lock request failed\n",
			sensor_info->sensor_name);
	}
	return ret;
}

static int flash_write(sensor_info_t *sensor_info, uint32_t addr,
	uint8_t *data, uint32_t bytes)
{
	int ret = 0;
	uint8_t reg_addr[4] = {};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0, };
	uint8_t buf_size = 0;
	uint8_t offset = 0;

	ret = flash_size_set(sensor_info);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash access unlock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF4);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access unlock request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash all read request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF7);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash Erase Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x03;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash sector erase request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash erase request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000 * 1000);

	// Buffer read request (Host -> Sensor)
	while (bytes > 0) {
		memset(buf, 0, I2C_SMBUS_BLOCK_MAX);
		reg_size = 2;
		memset(reg_addr, 0, reg_size);
		reg_addr[0] = 0x00;
		reg_addr[1] = addr + offset;

		if (bytes > I2C_SMBUS_BLOCK_MAX - reg_size) {
			buf_size = I2C_SMBUS_BLOCK_MAX - reg_size;
			memcpy(buf, data + offset, buf_size);
			ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : write reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, addr, buf_size);
			}
		} else {
			buf_size = bytes;
			memcpy(buf, data + offset, buf_size);
			ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : write reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, addr, buf_size);
			}
		}
		offset += buf_size;
		bytes -= buf_size;
	}
	usleep(1000);

	// Serial NOR Flash Write Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x02;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash sector write request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : sSerial NOR FLash write request failed\n",
			sensor_info->sensor_name);
	}
	usleep(100000);

	// Serial NOR Flash all write (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xFF);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash All Write failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash access lock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF5);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access lock request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);
	return ret;
}

int get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	return 0;
}

#if 0
// for debug
void data_trans(uint8_t *raw_data, uint8_t size)
{
	uint64_t data;

	float intrinsic_val[9][8] = {
		{433.965759, 433.792175, 961.870239, 718.525574, 0.184173, -0.040078, 0.012576, -0.002737},
		{435.231445, 434.846069, 954.734192, 718.173096, 0.179404, -0.031997, 0.008313, -0.001971},
		{441.790314, 441.508453, 959.902954, 718.690857, 0.224429, -0.064115, 0.010673, -0.001307},
		{436.090973, 435.730499, 970.060242, 718.738525, 0.182133, -0.038199, 0.011377, -0.002514},
		{434.681976, 434.515808, 971.333008, 713.064148, 0.181375, -0.036394, 0.010672, -0.002420},
		{440.628845, 440.290710, 960.465759, 723.735596, 0.22079, -0.058774, 0.006814, -0.000351},
		{442.504578, 442.167877, 950.125061, 725.225403, 0.225708, -0.065291, 0.011578, -0.001479},
		{439.687469, 439.457977, 952.696716, 719.186035, 0.222076, -0.060151, 0.008003, -0.000678},
		{442.774628, 442.333954, 962.581482, 723.569214, 0.223152, -0.063500, 0.010194, -0.001171},
	};
	int base_addr[8] = {EFL_X_ADDR_031, EFL_Y_ADDR_031, COD_X_ADDR_031, COD_Y_ADDR_031,
		K1_ADDR_031, K2_ADDR_031, K3_ADDR_031, K4_ADDR_031};

	for (int i = 0; i < 1; ++i) {
		memset(raw_data, 0, size);
		for (int j = 0; j < 8; ++j) {
			float_trans_to_char(raw_data,
				intrinsic_val[i][j], base_addr[j]);
			vin_info("raw_data[%d]=0x%x, 0x%x, 0x%x, 0x%x\n",
				base_addr[j], raw_data[base_addr[j]], raw_data[base_addr[j]+1],
				raw_data[base_addr[j]+2], raw_data[base_addr[j]+3]);
		}
		// for (int k = 0; k < sizeof(raw_data); ++k) {
		// 	vin_info("0x%x\t", raw_data[k]);
		// }
		vin_info("\nend of the %dth group data\n\n", i);
	}
}
#endif

int get_intrinsic_params(sensor_info_t *si,
		sensor_intrinsic_parameter_t *sip)
{
	uint64_t data;
	int ret = RET_OK;
	uint8_t intrin_raw_data[INTRIN_DATA_SIZE] = {0};

	if (!sip || !si) {
		vin_err("input sip|si is null!\n");
		return -RET_ERROR;
	}

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));

#if 0
	// for debug
	memset(intrin_raw_data, 0, INTRIN_DATA_SIZE);
	data_trans(intrin_raw_data, INTRIN_DATA_SIZE);
	ret = flash_write(si, EFL_X_ADDR_031, intrin_raw_data, INTRIN_DATA_SIZE);
	if (ret < 0) {
		vin_err("%s : flash_write failed\n",	si->sensor_name);
	}
#endif

	memset(intrin_raw_data, 0, sizeof(intrin_raw_data));

	ret = flash_read(si, EFL_X_ADDR_031, intrin_raw_data,
		sizeof(intrin_raw_data));
	if (ret < 0) {
		vin_err("%s : flash_read failed\n", si->sensor_name);
	}

	sip->focal_u = float_trans_from_char(intrin_raw_data, EFL_X_ADDR_031);
	sip->focal_v = float_trans_from_char(intrin_raw_data, EFL_Y_ADDR_031);
	sip->center_u = float_trans_from_char(intrin_raw_data, COD_X_ADDR_031);
	sip->center_v = float_trans_from_char(intrin_raw_data, COD_Y_ADDR_031);
	sip->k1 = float_trans_from_char(intrin_raw_data, K1_ADDR_031);
	sip->k2 = float_trans_from_char(intrin_raw_data, K2_ADDR_031);
	sip->k3 = float_trans_from_char(intrin_raw_data, K3_ADDR_031);
	sip->k4 = float_trans_from_char(intrin_raw_data, K4_ADDR_031);

	vin_info("focal_u:%0.12f focal_v:%0.12f center_u:%0.12f center_v:%0.12f\n",
			sip->focal_u, sip->focal_v, sip->center_u, sip->center_v);
	vin_info("k1:%0.12f k2:%0.12f k3:%0.12f K4:%0.12f\n",
			sip->k1, sip->k2, sip->k3, sip->k4);

// Register Access request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(si->bus_num,
		si->sensor_addr, 0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%s : Register Access request failed\n",
			si->sensor_name);
	}

	return RET_OK;
}

int get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
{
	int ret = RET_OK;

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
		vin_err("%s: get_sns_info param error: type %d !!\n",
			si->sensor_name, type);
		ret = -RET_ERROR;
	}
	return ret;
}

sensor_module_t isx031s = {
	.module = "isx031s",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	// .stream_off = sensor_stream_off,
	// .stream_on = sensor_stream_on,
	.get_sns_params = get_sns_info,
};

