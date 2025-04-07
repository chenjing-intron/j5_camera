/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[s960]:[%s][%d]" fmt, __func__, __LINE__

#include "cJSON.h"
#include "hb_cam_utility.h"
#include "hb_i2c.h"
#define STREAM_CONTROL_REG  0x20
#define INIT_STATE 1
#define DEINIT_STATE 2
#define DES_RESET_DELAY 2000
#define DES_PORT0_BIT 4
#define DES_PORT1_BIT 5
#define DES_PORT_NUM  2u
#define DES_BOTH_PORT_OP 3

// PRQA S 3206++

static int32_t deserializer_deinit(deserial_info_t *s960_info)
{
	#if 0
	 if (s960_info->init_state != INIT_STATE) {
		 vin_info("%s 960 must be init\n", TAG);
		 return -RET_ERROR;
	 }
	s960_info->init_state = DEINIT_STATE;
	#endif
	return RET_OK;
}
static int32_t deserializer_init(deserial_info_t *s960_info)
{
#if 0
	char *filebuf = NULL, *filename = NULL;
	FILE *fp = NULL;
	struct stat statbuf;
	cJSON *deserial_root = NULL;
	cJSON *sensor_node = NULL, *config = NULL, *arrayitem = NULL;
	int32_t  array_size, i, array_index;
	int32_t  setting_size;
	char *sensor_name = NULL;
	sensor_info_t **sensor_if = (sensor_info_t **)s960_info->sensor_info;

	if(s960_info->init_state == INIT_STATE) {
			vin_info("S960 has been inited\n");
			return -RET_ERROR;
	}
	filename = s960_info->deserial_config_path;
	if(filename == NULL) {
		vin_err("serial config file is null !!\n");
		return -RET_ERROR;
	}
	stat(filename, &statbuf);
	vin_dbg("%s filename = %s\n", TAG, filename);
	if(0 == statbuf.st_size) {
		vin_err("sreial config file size is zero !!\n");
		return -RET_ERROR;
	}
	fp = fopen(filename, "r");
	if(fp == NULL) {
		vin_dbg("filename is NULL, needn't config 960\n", filename);
		return RET_OK;
	}
	filebuf = (char *)malloc(statbuf.st_size);
	if(NULL == filebuf) {
		vin_err("%s malloc buff fail !!\n", TAG);
		goto err;
	}
	memset(filebuf, 0, statbuf.st_size);
	fread(filebuf, statbuf.st_size, 1, fp);
	deserial_root = cJSON_Parse((const char *)filebuf);
	if(NULL == deserial_root) {
		vin_err("%s parse json fail\n", TAG);
		goto err;
	}
	vin_info("s960_info->deserial_addr 0x%x s960_info->bus_num %d \n", s960_info->deserial_addr, s960_info->bus_num);
	for(i = 0; i < CAM_MAX_NUM; i++) {
		if(sensor_if[i]) {
			if(sensor_if[i]->alias_name == NULL) {
				sensor_name = sensor_if[i]->sensor_name;
			} else {
				sensor_name = sensor_if[i]->alias_name;
			}
			vin_info("sensor_name %s sensor_if[i]->sensor_name %s\n", sensor_name, sensor_if[i]->sensor_name);
			sensor_node = cJSON_GetObjectItem(deserial_root, sensor_name);
				config = cJSON_GetObjectItem(sensor_node, "init_config");
				if(NULL != config) {
					array_size = cJSON_GetArraySize(config);
					vin_dbg("array_size %d\n", array_size);
					uint32_t s960_init[array_size];
					memset(s960_init, 0, sizeof(s960_init));
					for(array_index = 0; array_index < array_size; array_index++) {
						arrayitem = cJSON_GetArrayItem(config, array_index);
						s960_init[array_index] = hb_cam_htoi(arrayitem->valuestring);
					}
					setting_size = sizeof(s960_init)/sizeof(uint32_t)/2;
					vin_info("setting_size %d\n", setting_size);
					ret = vin_write_array(s960_info->bus_num, s960_info->deserial_addr, 1, setting_size, s960_init);
					if (ret < 0) {
						vin_err("S960 init error\n");
						goto err;
					}
			}
		}
	}

	vin_info("s960 init success\n");
	free(filebuf);
	fclose(fp);
	s960_info->init_state = INIT_STATE;
	if(deserial_root) {
		cJSON_Delete(deserial_root);
		deserial_root = NULL;
	}
	return RET_OK;
err:
	free(filebuf);
	fclose(fp);
	if(deserial_root) {
		cJSON_Delete(deserial_root);
		deserial_root = NULL;
	}
	deserializer_deinit(s960_info);
	return -RET_ERROR;
#endif
	return RET_OK;
}

static int32_t deserializer_stream_on(deserial_info_t *s960_info, uint32_t port) /*PRQA S 3673*/
{
	int32_t ret;
	int32_t ret_value;
	int32_t value;
	sensor_info_t **sensor_if = (sensor_info_t **)&s960_info->sensor_info[0];/* PRQA S 0310 */

	if((port >= CAM_MAX_NUM) || (sensor_if[port] == NULL))
			return -RET_ERROR;
	if(sensor_if[port]->stream_control == 0u) {
		vin_dbg("don't need 960 stream on\n");/* PRQA S ALL */
		return RET_OK;
	}
	if (sensor_if[port]->stream_control == 1u) {
		ret_value = hb_vin_i2c_read_reg8_data8(s960_info->bus_num,
				(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG);
		vin_dbg("value 0x%x\n", ret_value);/* PRQA S ALL */
		if(port == 0u) {
			value = ret_value & (~(1 << DES_PORT0_BIT));/* PRQA S ALL */
		} else {
			value = ret_value & (~(1 << DES_PORT1_BIT));/* PRQA S ALL */
		}
	} else if (sensor_if[port]->stream_control == DES_PORT_NUM) {
		ret_value = hb_vin_i2c_read_reg8_data8(s960_info->bus_num,
					(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG);
		value = ret_value & (~(DES_BOTH_PORT_OP << DES_PORT0_BIT));/* PRQA S ALL */
	} else {
		vin_err("s960 stream_control %d error\n", sensor_if[port]->stream_control);/* PRQA S ALL */
		return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg8_data8(s960_info->bus_num, (uint8_t)s960_info->deserial_addr,
			STREAM_CONTROL_REG, (uint8_t)value);
	if(ret < 0) {
		 vin_err("s960_stream_on i2c write8 error\n");/* PRQA S ALL */
	}
	return ret;
}

static int32_t deserializer_stream_off(deserial_info_t *s960_info, uint32_t port) /*PRQA S 3673*/
{
	int32_t ret;
	int32_t ret_value;
	int32_t value;
	sensor_info_t **sensor_if = (sensor_info_t **)&s960_info->sensor_info[0];/* PRQA S 0310 */

	if((port >= CAM_MAX_NUM) || (sensor_if[port] == NULL))
			return -RET_ERROR;
	if(sensor_if[port]->stream_control == 0u) {
			vin_dbg("don't need 960 stream off\n");/* PRQA S ALL */
			return RET_OK;
	}
	if(sensor_if[port]->stream_control == 1u) {
		ret_value = hb_vin_i2c_read_reg8_data8(s960_info->bus_num,
				(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG);
		vin_dbg("value 0x%x\n", ret_value);/* PRQA S ALL */
		if(port == 0u) {
			value = (ret_value | (1 << DES_PORT0_BIT));/* PRQA S ALL */
		} else {
			value = (ret_value | (1 << DES_PORT1_BIT));/* PRQA S ALL */
		}
	} else if (sensor_if[port]->stream_control == DES_PORT_NUM) {
			ret_value = hb_vin_i2c_read_reg8_data8(s960_info->bus_num,
					(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG);
			value = (ret_value | (DES_BOTH_PORT_OP << DES_PORT0_BIT));/* PRQA S ALL */
	} else {
		vin_err("s960 stream_control %d error\n", sensor_if[port]->stream_control);/* PRQA S ALL */
		return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg8_data8(s960_info->bus_num,
			(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG, (uint8_t)value);
	if(ret < 0) {
		 vin_err("s960_stream_off i2c write8 error\n");/* PRQA S ALL */
	}
	return ret;
}

static int32_t deserializer_start_physical(const deserial_info_t *s960_info)
{
	int32_t ret;
	int32_t ret_value;
	int32_t value;

	if(s960_info->physical_entry == 0u) {
		vin_dbg("don't need 960 start phsical entry\n");/* PRQA S ALL */
		return RET_OK;
	}
	if (s960_info->physical_entry == 1u) {
		ret_value = hb_vin_i2c_read_reg8_data8(s960_info->bus_num,
						(uint8_t)s960_info->deserial_addr, STREAM_CONTROL_REG);
		value = ret_value & (~(DES_BOTH_PORT_OP << DES_PORT0_BIT));/* PRQA S ALL */
		ret = hb_vin_i2c_write_reg8_data8(s960_info->bus_num, (uint8_t)s960_info->deserial_addr,
				STREAM_CONTROL_REG, (uint8_t)value);
		if(ret < 0) {
			vin_err("s960_stream_on i2c write8 error\n");/* PRQA S ALL */
		}
		return ret;
	} else {
		vin_err("s960 physical_entry %d error\n", s960_info->physical_entry); /* PRQA S ALL */
		ret = -RET_ERROR;
	}
	return ret;
}

static int32_t deserializer_reset(const deserial_info_t *s960_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	if(s960_info->power_mode == 1u) {
		for(gpio = 0; gpio < s960_info->gpio_num; gpio++) {
			if(s960_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)s960_info->gpio_pin[gpio],
							s960_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n"); /* PRQA S ALL */
					return -RET_ERROR;
				}
			}
		}
		(void)usleep(DES_RESET_DELAY);
		for(gpio = 0; gpio < s960_info->gpio_num; gpio++) {
			if(s960_info->gpio_pin[gpio] >= 0) {
				ret = vin_power_ctrl((uint32_t)s960_info->gpio_pin[gpio],
							1-s960_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n"); /* PRQA S ALL */
					return -RET_ERROR;
				}
			}
		}
	}
	return ret;
}

deserial_module_t s960 = {
	.module = "s960",
	.init = deserializer_init,
	.stream_on = deserializer_stream_on,
	.stream_off = deserializer_stream_off,
	.start_physical = deserializer_start_physical,
	.deinit = deserializer_deinit,
	.reset = deserializer_reset,
};

// PRQA S --

