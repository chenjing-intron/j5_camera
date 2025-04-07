/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ov10635]:" fmt

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

#include "inc/hb_vin.h"
#include "hb_cam_utility.h"
#include "inc/ov10635_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ds960_setting.h"
#include "inc/ds913_setting.h"
#include "inc/ds954_setting.h"
#include "hb_i2c.h"

int sensor_deserial_line_concatenated_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (!strcmp(deserial_if->deserial_name, "s960")) {
		setting_size = sizeof(ds960_ov10635_line_concatenated_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
				setting_size, ds960_ov10635_line_concatenated_setting);
		if (ret < 0) {
			vin_err("write ds960_ov10635_init_setting error\n");
			return ret;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	usleep(5000);
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}

int sensor_ov10635_954_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (!strcmp(deserial_if->deserial_name, "s954")) {
		setting_size = 1;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1, setting_size, ds954_ov10635_init_setting);
		usleep(500);
		ret |= vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1, setting_size, &(ds954_ov10635_init_setting[2]));
		usleep(300);
		setting_size = sizeof(ds954_ov10635_init_setting)/sizeof(uint32_t)/2 - 2;
		ret |= vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1, setting_size, &(ds954_ov10635_init_setting[4]));
		if (ret < 0) {
			vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "s960")) {
		setting_size = sizeof(ds960_ov10635_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
				setting_size, ds960_ov10635_init_setting);
		if (ret < 0) {
			vin_err("write ds960_ov10635_init_setting error\n");
			return ret;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	usleep(5000);
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}

int sensor_ov10635_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 1;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, ds913_ov10635_init_setting);
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds913_ov10635_init_setting[2]));
	usleep(30);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds913_ov10635_init_setting[4]));
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return ret;
	}
	usleep(4000);
	vin_dbg("913init DONE, begin 913\n");
	setting_size = sizeof(ov10635_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
		2, setting_size, ov10635_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	setting_size = sizeof(ov10635_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("in ov10635 sensor_init stream off sensor: %s setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	int i;
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, ov10635_stream_off_setting[i*2],
			ov10635_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	vin_dbg("ov10635:sensor_init DONE\n");
	return ret;
}

int sensor_ov10635_line_concate_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 1;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, ds913_ov10635_init_setting);
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds913_ov10635_init_setting[2]));
	usleep(30);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds913_ov10635_init_setting[4]));
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return ret;
	}
	usleep(4000);
	vin_dbg("913init DONE, begin 913\n");
	setting_size = sizeof(ov10635_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
		2, setting_size, ov10635_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	setting_size = sizeof(ov10635_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("in ov10635 sensor_init stream off sensor: %s setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, ov10635_stream_off_setting[i*2],
			ov10635_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	vin_dbg("ov10635:sensor_init serial_addr 0x%x sensor_addr 0x%x DONE\n", sensor_info->serial_addr, sensor_info->sensor_addr);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 1, 1, 1, ds913_ov10635_init_setting);
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 1, 1, 1, &(ds913_ov10635_init_setting[2]));
	usleep(30);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 1, 1, 1, &(ds913_ov10635_init_setting[4]));
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return ret;
	}
	usleep(4000);
	vin_dbg("913init DONE, begin 913\n");
	setting_size = sizeof(ov10635_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr + 1,
		2, setting_size, ov10635_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	setting_size = sizeof(ov10635_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("in ov10635 sensor_init stream off sensor: %s setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 1, ov10635_stream_off_setting[i*2],
			ov10635_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	vin_dbg("ov10635:sensor_init serial_addr 0x%x sensor_addr 0x%x DONE\n", sensor_info->serial_addr + 1, sensor_info->sensor_addr + 1);

	#if 0
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 2, 1, setting_size, ds913_ov10635_init_setting);
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 2, 1, setting_size, &(ds913_ov10635_init_setting[2]));
	usleep(30);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 2, 1, setting_size, &(ds913_ov10635_init_setting[4]));
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return ret;
	}
	usleep(4000);
	vin_dbg("913init DONE, begin 913\n");
	setting_size = sizeof(ov10635_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr + 2,
		2, setting_size, ov10635_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	setting_size = sizeof(ov10635_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("in ov10635 sensor_init stream off sensor: %s setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 2, ov10635_stream_off_setting[i*2],
			ov10635_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	vin_dbg("ov10635:sensor_init serial_addr 0x%x sensor_addr 0x%x DONE\n", sensor_info->serial_addr + 2, sensor_info->sensor_addr + 2);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 3, 1, setting_size, ds913_ov10635_init_setting);
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 3, 1, setting_size, &(ds913_ov10635_init_setting[2]));
	usleep(30);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr + 3, 1, setting_size, &(ds913_ov10635_init_setting[4]));
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return ret;
	}
	usleep(4000);
	vin_dbg("913init DONE, begin 913\n");
	setting_size = sizeof(ov10635_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr + 3,
		2, setting_size, ov10635_init_setting);
	if (ret < 0) {
		vin_dbg("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	setting_size = sizeof(ov10635_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("in ov10635 sensor_init stream off sensor: %s setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 3, ov10635_stream_off_setting[i*2],
			ov10635_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	vin_dbg("ov10635:sensor_init serial_addr 0x%x sensor_addr 0x%x DONE\n", sensor_info->serial_addr + 3, sensor_info->sensor_addr + 3);
	#endif
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int req, ret = RET_OK;
	char str[24] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	if(sensor_info->extra_mode == 0) {
		req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		if (req == 0) {
			ret = sensor_ov10635_954_init(sensor_info);
			hb_vin_mipi_pre_result(sensor_info->entry_num, 0, ret);
			if (ret < 0) {
				vin_err("ov10635_954_init fail\n");
				return ret;
			}
		}
		ret = sensor_ov10635_init(sensor_info);
		if(ret < 0) {
			vin_err("sensor_ov10635_init fail\n");
			return ret;
		}

	} else if(sensor_info->extra_mode == 1) {
		req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		if (req == 0) {
			ret = sensor_deserial_line_concatenated_init(sensor_info);
			hb_vin_mipi_pre_result(sensor_info->entry_num, 0, ret);
			if (ret < 0) {
				vin_err("sensor_deserial_line_concatenated_init fail\n");
				return ret;
			}
		}
		ret = sensor_ov10635_line_concate_init(sensor_info);
		if(ret < 0) {
			vin_err("sensor_line_concate_init fail\n");
			return ret;
		}
	} else {
		vin_err("sesnor not support extra_mode %d \n", sensor_info->extra_mode);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_ov10635_stream_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint8_t value;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	setting_size = sizeof(ov10635_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s sensor_start setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, ov10635_stream_on_setting[i*2],
			ov10635_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -RET_ERROR;
		}
	}
	usleep(5000);
	ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20);
	if(ret < 0) {
		vin_err("ov10635 start read %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
	value = ret;
	if (!strcmp(deserial_if->deserial_name, "s954")) {
		vin_info("sensor_start read ds954 0x20 value:%02x\n", value);
		switch(sensor_info->deserial_port) {
			case 0:
				value = value & 0x20;   //  enable port0
				break;
			case 1:
				value = value & 0x10;   //  enable port1
				break;
		}
	} else if (!strcmp(deserial_if->deserial_name, "s960")) {
		vin_info("sensor_start read ds960 0x20 value:%02x\n", value);
		switch(sensor_info->deserial_port) {
			case 0:
				value = value & 0xE0;   //  enable port0
				break;
			case 1:
				value = value & 0xD0;   //  enable port1
				break;
			case 2:
				value = value & 0xB0;   //  enable port2
				break;
			case 3:
				value = value & 0x70;    //  enable port3
				break;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_STREAM_ON_FAIL;
	}
	ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20, value);
	if(ret < 0) {
		vin_err("write %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}

	usleep(5000);
	vin_info("sensor_start write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	return ret;
}

int sensor_ov10635_line_concate_stream_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint8_t value;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	setting_size = sizeof(ov10635_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s sensor_start setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, ov10635_stream_on_setting[i*2],
			ov10635_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -RET_ERROR;
		}
	}
	usleep(5000);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 1, ov10635_stream_on_setting[i*2],
			ov10635_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -RET_ERROR;
		}
	}
	usleep(5000);
	#if 0
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 2, ov10635_stream_on_setting[i*2],
			ov10635_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -RET_ERROR;
		}
	}
	usleep(5000);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr + 3, ov10635_stream_on_setting[i*2],
			ov10635_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -RET_ERROR;
		}
	}
	usleep(5000);
	#endif
	ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20);
	if(ret < 0) {
		vin_err("ov10635 start read %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
	value = ret;
	if (!strcmp(deserial_if->deserial_name, "s960")) {
		vin_info("sensor_start read ds960 0x20 value:%02x\n", value);
		value = value & 0xCF;   //  enable port0 por1
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20, value);
	if(ret < 0) {
		vin_err("write %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}

	usleep(5000);
	vin_info("sensor_start write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if(sensor_info->extra_mode == 0) {
		ret = sensor_ov10635_stream_on(sensor_info);
		if(ret < 0) {
			vin_err("%d : %s sensor_ov10635_stream_on fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if(sensor_info->extra_mode == 1) {
		ret = sensor_ov10635_line_concate_stream_on(sensor_info);
		if(ret < 0) {
			vin_err("%d : %s sensor_ov10635_line_concate_stream_on fail\n",
				__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("sesnor not support extra_mode %d \n", sensor_info->extra_mode);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_ov10635_stream_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20);
	if(ret < 0) {
		vin_err("ov10635 start read ds960 failed\n");
		return -RET_ERROR;
	}
	value = ret;
	if (!strcmp(deserial_if->deserial_name, "s954")) {
		vin_info("sensor_stop read s954 0x20 value:%02x\n", value);
		switch(sensor_info->deserial_port) {
			case 0:
				value = value | 0x10;	//	disable port0
				break;
			case 1:
				value = value | 0x20;	//	disable port1
				break;
		}
	}  else if (!strcmp(deserial_if->deserial_name, "s960")) {
		vin_info("sensor_stop read s960 0x20 value:%02x\n", value);
		switch(sensor_info->deserial_port) {
			case 0:
				value = value | 0x10;	//	disable port0
				break;
			case 1:
				value = value | 0x20;	//	disable port1
				break;
			case 2:
				value = value | 0x40;	 //  disable port2
				break;
			case 3:
				value = value | 0x80;	 //  disable port3
				break;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_STREAM_OFF_FAIL;
	}
	ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20, value);
	if(ret < 0) {
		vin_err("write %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
	vin_info("sensor_stop write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	return ret;
}

int sensor_ov10635_line_concate_stream_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20);
	if(ret < 0) {
		vin_err("ov10635 read ds960 failed\n");
		return -RET_ERROR;
	}
	value = ret;
	if (!strcmp(deserial_if->deserial_name, "s960")) {
		vin_info("sensor_stop read s960 0x20 value:%02x\n", value);
		value = 0xf0;
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_STREAM_OFF_FAIL;
	}
	ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, deserial_if->deserial_addr, 0x20, value);
	if(ret < 0) {
		vin_err("write %s failed\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
	vin_info("sensor_stop write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if(sensor_info->extra_mode == 0) {
		ret = sensor_ov10635_stream_off(sensor_info);
		if(ret < 0) {
			vin_err("%d : %s sensor_ov10635_stream_off fail\n",
				__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if(sensor_info->extra_mode == 1) {
		ret = sensor_ov10635_line_concate_stream_off(sensor_info);
		if(ret < 0) {
			vin_err("%d : %s sensor_ov10635_line_concate_stream_off fail\n",
				__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("sesnor not support extra_mode %d \n", sensor_info->extra_mode);
		return -RET_ERROR;
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

sensor_module_t ov10635 = {
	.module = "ov10635",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
};


