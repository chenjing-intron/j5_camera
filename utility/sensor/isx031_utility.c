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
#include "hb_cam_utility.h"
#include "inc/isx031_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_i2c.h"

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
	WHETRON_24M,
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
int write_register(int bus, uint8_t *pdata, int setting_size, uint8_t i2c_slave_external)
{
	int ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int i, len, k = 10;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			if (i2c_slave_external != 0)
			{
				i2c_slave = i2c_slave_external;
			}
			else
			{
				i2c_slave = pdata[i + 1] >> 1;
			}
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];

			k = 10;
			do {
				vin_info("init serdes reg:0x%x  value:%x  k:%d\n", reg_addr, value, k);
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
				if (ret == RET_OK) {
					break;
				}
				usleep(20 * 1000);
			} while (k--);

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

int poc_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 1;
	int default_poc_addr_g_pulse, default_poc_i2c_g_pulse;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	ret = vin_write_array(deserial_if->bus_num, DEFAULT_POC_ADDR,
		POC_REG_WIDTH, setting_size, poc_init_setting);
	if (ret < 0) {
		vin_err("Poc for des-%s connected cameras off failed!\n",
			deserial_if->deserial_name);
		return ret;
	}
	usleep(10 * 1000);
	ret = vin_write_array(deserial_if->bus_num, DEFAULT_POC_ADDR,
		POC_REG_WIDTH, setting_size, poc_init_setting + 2);
	if (ret < 0) {
		vin_err("Poc for des-%s connected cameras on failed!\n",
			deserial_if->deserial_name);
		return ret;
	}
	return ret;
}

uint8_t link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int ret = RET_OK;
	uint16_t reg = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_LINK_SET_9296;
		if (link_port < DES_PORT_NUM_MAX - 2) {
			val = LINK_NONE_9296 | (1 << link_port);
		} else if (link_port == LINK_NONE) {
			val = LINK_NONE_9296;
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_9296;
		} else {
			vin_err("%s link_port 0x%x not supported for des-%s!\n",
				__func__, link_port, deserial_if->deserial_name);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		reg = REG_LINK_SET_96712;
		if (link_port < DES_PORT_NUM_MAX) {
			val = LINK_NONE_96712 & (~(1 << (2 * link_port)));
		} else if (link_port == LINK_NONE) {
			val = LINK_NONE_96712;
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_96712;
		} else {
			vin_err("%s link_port 0x%x not supported for des-%s!\n",
				__func__, link_port, deserial_if->deserial_name);
			return -1;
		}
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
		deserial_if->deserial_addr, reg, val);
	if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n",
			__func__, link_port, deserial_if->deserial_name);
		return -1;
	}
	vin_info("%s switch to port 0x%x successfully 111 for des-%s!\n",
		__func__, link_port, deserial_if->deserial_name);
	usleep(800 * 1000);
	return 0;
}

/* default date_rate 0 means 6g, 1 means 3g*/
uint8_t rx_rate_switch(sensor_info_t *sensor_info, uint8_t data_rate)
{
	int ret = RET_OK;
	uint16_t reg = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0;
	char tx_rate_s[10];
	static int rx_rate_init_once = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	if (data_rate == 1) {
		val = TXRATE_3G;
		memcpy(tx_rate_s, "3G", 3);
	} else if (data_rate == 0) {
		val = TXRATE_6G;
		memcpy(tx_rate_s, "6G", 3);
	} else {
		vin_err("%s data_rate %d not supported for des-%s!\n",
			__func__, data_rate, deserial_if->deserial_name);
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_TXRATE_9296;
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		if (rx_rate_init_once)
			return 0;
		if (sensor_info->deserial_port < 2)
			reg = REG_TXRATE_96712_AB;
		else
			reg = REG_TXRATE_96712_CD;
	}
	val_read = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
		deserial_if->deserial_addr, reg);
	if (val_read < 0) {
		vin_err("%s read reg 0x%x failed!\n", __func__, reg);
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		val |= (val_read & 0xFC);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg, val);
		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		// if (sensor_info->deserial_port < 2) {
		// 	val = val << (sensor_info->deserial_port * 4);
		// 	val |= (val_read & ~(0x3 << (sensor_info->deserial_port * 4)));
		// } else {
		// 	val = val << ((sensor_info->deserial_port - 2) * 4);
		// 	val |= (val_read & ~(0x3 << ((sensor_info->deserial_port - 2) * 4)));
		// }
		reg = REG_TXRATE_96712_AB;
		if (data_rate == 0) {
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
			deserial_if->deserial_addr, reg, 0x2222);
		} else {
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
			deserial_if->deserial_addr, reg, 0x1111);
		}

		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
		rx_rate_init_once = 1;
	}

	vin_info("%s switch to %s successfully for des-%s!\n", __func__, tx_rate_s,
		deserial_if->deserial_name);
	usleep(800 * 1000);

	return 0;
}


int des_data_rate_setting(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	for (int i = 0; i < DES_OUT_NUM_MAX; ++i) {
		pdata = max96712_config_soft_rst_n_0;
		setting_size = sizeof(max96712_config_soft_rst_n_0) /
			sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, deserial_if->deserial_addr);
		if (ret < 0) {
			vin_err("write max96712 soft rst n error\n");
			return ret;
		}


		pdata = max96712_config_soft_rst_n_1;
		setting_size = sizeof(max96712_config_soft_rst_n_1) / sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, deserial_if->deserial_addr);
		if (ret < 0) {
			vin_err("write max96712 release soft rst n error\n");
			return ret;
		}
	}
	return ret;
}

int des_data_lane_setting(sensor_info_t *sensor_info, uint8_t data_lane)
{
	int ret = RET_OK;
	uint16_t reg = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0;
	int setting_size = 0;
	int borad_type = 0x00;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_DATALANE_PORTA_9296;
		val = (data_lane & 0x3) << 6;
		val_read = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val_read < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= (val_read & 0x3F);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg, val);
		if (ret < 0) {
			vin_err("%s set to data_lane %d failed!\n", __func__, data_lane);
			return -1;
		}
		reg = REG_DATALANE_PORTB_9296;
		val = ((data_lane & 0xC) >> 2) << 6;
		val_read = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val_read < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= (val_read & 0x3F);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg, val);
		if (ret < 0) {
			vin_err("%s set to data_lane %d failed!\n", __func__, data_lane);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") &&
			(data_lane < sizeof(max96712_lane_setting)/sizeof(max96712_lane_setting[0]))) {
		setting_size = sizeof(max96712_lane_setting[data_lane]) / sizeof(uint8_t);
		pdata = max96712_lane_setting[data_lane];
		ret = write_register(deserial_if->bus_num, pdata, setting_size, deserial_if->deserial_addr);
		if (ret < 0) {
			vin_err("write max96712_lane_setting[%d] failed\n", data_lane);
			return -1;
		}

#if 0
		//borad_type = vin_get_board_id();
		//if (borad_type == BOARD_ID_MATRIXDUO_B) 
		{
			// Modify clock of reverse P&N
			vin_info("borad id: 0x%x; reverse deserial clock polarity\n", 0);
			setting_size = sizeof(max96712_lane_colck_reverse) / sizeof(uint8_t);
			pdata = max96712_lane_colck_reverse;
			ret = write_register(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_lane_colck_reverse[%d] failed\n", data_lane);
				return -1;
			}
		}
	        #if 0	
		else {
			vin_info("borad id: 0x%x\n", borad_type);
		}
		#endif
#endif
	}
	vin_info("%s set data_lane to %d successfully for des-%s!\n",
		__func__, data_lane, deserial_if->deserial_name);
	// usleep(20 * 1000);
	return 0;
}

int deserializer_init_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t *pdata = NULL;
	uint8_t data_lane = 0;
	int board_ver = 0;
	int des_port_num = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	if (deserial_if->init_state == 1)
		return ret;

	for (int i = 0; i < CAM_MAX_NUM; ++i) {
		if (deserial_if->sensor_info[i]) {
			des_port_num++;
			vin_info("port[%d] is attached to des_%s\n", i,
				deserial_if->deserial_name);
		}
	}

	if (!(sensor_info->config_index & POC_DISABLE)) {
		ret = poc_init(sensor_info);
		if (ret < 0) {
			vin_err("poc_init error\n");
			return ret;
		}
	} else if (!sensor_info->power_mode) {
		/* reset all serials replace to poc off */
#if 0
		int32_t i2c_slave;
		for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
			vin_info("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
			hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, i2c_slave, 0x0010, 0xf1);
		}
#endif
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		pdata = max9296_dual_init_setting_base;
		setting_size = sizeof(max9296_dual_init_setting_base) /
				sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, 0);
		if (ret < 0) {
			vin_err("write max9296_dual_init_setting_base error\n");
			return ret;
		}
		if (sensor_info->config_index & DUAL_LANE) {
			data_lane = ((2-1) | ((4-1) << 2));
		} else {
			data_lane = ((4-1) | ((4-1) << 2));
		}
		ret = des_data_lane_setting(sensor_info, data_lane);
		if (ret < 0) {
			vin_err("write des_data_lane_setting error\n");
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		pdata = max96712_quad_init_setting_base[0];
		setting_size = sizeof(max96712_quad_init_setting_base[0]) /
			sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, deserial_if->deserial_addr);
		if (ret < 0) {
			vin_err("write max96712_quad_init_setting_base port A error\n");
			return ret;
		}
		ret = des_data_lane_setting(sensor_info, 3);
		if (ret < 0) {
			vin_err("set des_data_lane_setting failed\n");
			return ret;
		}
		ret = des_data_rate_setting(sensor_info);
		if (ret < 0) {
			vin_err("write des_data_rate_setting error\n");
			return ret;
		}
#if 1
		// for 20fps or 3+2 30fps stable version
		pdata = max96712_mipi_rate_setting_2g;
		setting_size = sizeof(max96712_mipi_rate_setting_2g) /
			sizeof(uint8_t);

		ret = write_register(deserial_if->bus_num, pdata, setting_size, deserial_if->deserial_addr);
		if (ret < 0) {
			vin_err("write max96712 mipi rate register error\n");
			return ret;
		}
#endif
#if 0
		if (sensor_info->extra_mode == SENSING_24M_H190X) {
			pdata = max96712_errb_mfp_mapping_setting;
			setting_size = sizeof(max96712_errb_mfp_mapping_setting) / sizeof(uint8_t);

			ret = write_register(deserial_if->bus_num, pdata, setting_size, 0);
			if (ret < 0) {
				vin_err("write max96712_errb_mfp_mapping_setting error\n");
				return ret;
			}
		}
#endif
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -1;
	}
	// trigger setting
	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		if (sensor_info->config_index & TRIG_MODE) {
			if (deserial_if->mfp_index > MAX9296_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max9296_trig_setting MFP index error\n");
				return ret;
			}
			uint32_t setting_size = 0;
			uint16_t regaddr = 0, offset = 0;
			uint16_t *trigger_reg;

			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max9296_trigger_mfp5;
				setting_size = sizeof(max9296_trigger_mfp5) / sizeof(uint16_t) / 2;
				offset = 0;
			} else {
				trigger_reg = max9296_trigger_mfp;
				setting_size = sizeof(max9296_trigger_mfp) / sizeof(uint16_t) / 2;
				offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
			}

			for (int32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i] + offset;
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write max9296_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		if (sensor_info->config_index & TRIG_MODE) {
			if (deserial_if->mfp_index > MAX96712_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max96712_trig_setting MFP index error\n");
				return ret;
			}
			uint32_t setting_size = sizeof(max96712_trigger_setting_mfp) /
			                        sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
			uint16_t offset = 0, size = 0, regaddr = 0;
			uint16_t *trigger_reg;
			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max96712_trigger_mfp4;
				setting_size = sizeof(max96712_trigger_mfp4) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
			} else if (deserial_if->mfp_index == 14) {
				trigger_reg = max96712_trigger_mfp14;
				setting_size = sizeof(max96712_trigger_mfp14) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
			} else if (deserial_if->mfp_index == 9) {
				trigger_reg = max96712_trigger_mfp9;
				setting_size = sizeof(max96712_trigger_mfp9) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
				vin_info("---chj---:mfp_index == 9\n");
			} else {
				trigger_reg = max96712_trigger_setting_mfp;
				setting_size = sizeof(max96712_trigger_setting_mfp) / sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
				offset = deserial_if->mfp_index % MAX96712_MFP_LOOP * setting_size;
				size = deserial_if->mfp_index / MAX96712_MFP_LOOP * MAX96712_MFP_OFFSET;
			}

			for (int32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i + 2*offset] + size;
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				if (ret < 0) {
					vin_err("write max96712_trig_setting error\n", deserial_if->deserial_name);
				}
			}
			if (ret < 0)
				vin_err("write max96712_trig_setting error\n", deserial_if->deserial_name);
		}
	}
	usleep(5000);
	deserial_if->init_state = 1;
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
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
	uint8_t* p_alias_id_setting = NULL;
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
			p_alias_id_setting = alias_id_setting[sensor_info->deserial_port];
			alias_setting_size = sizeof(alias_id_setting[sensor_info->deserial_port]) / sizeof(uint8_t);
			break;
		case HKISX031:
		case HKISX031_2:
			data_rate = 1;
			p_serializer_pipez_setting = serializer_pipez_setting_hk;
			setting_size = sizeof(serializer_pipez_setting_hk) / sizeof(uint8_t);
			ser_addr = HAOMO_SER_ADDR;
			p_alias_id_setting = alias_id_setting_hmisx031[sensor_info->deserial_port];
			alias_setting_size = sizeof(alias_id_setting_hmisx031[sensor_info->deserial_port]) / sizeof(uint8_t);
			break;
		case SENSING_24M_H190X:
			data_rate = 0;
			p_serializer_pipez_setting = serializer_pipez_setting;
			setting_size = sizeof(serializer_pipez_setting)/ sizeof(uint8_t);
			ser_addr = DEFAULT_SER_ADDR;
			p_alias_id_setting = alias_id_setting[sensor_info->deserial_port];
			alias_setting_size = sizeof(alias_id_setting[sensor_info->deserial_port]) / sizeof(uint8_t);
			break;
		case SENSING_24M_H190XA:
		case WHETRON_24M:
			data_rate = 1;
			p_serializer_pipez_setting = serializer_pipez_setting;
			setting_size = sizeof(serializer_pipez_setting)/ sizeof(uint8_t);
			ser_addr = DEFAULT_SER_ADDR;
			p_alias_id_setting = alias_id_setting[sensor_info->deserial_port];
			alias_setting_size = sizeof(alias_id_setting[sensor_info->deserial_port]) / sizeof(uint8_t);
			break;
		default: vin_err("no serializer here error\n"); return -1;
	}

	while(retry_time > 0) {
		ret = rx_rate_switch(sensor_info, data_rate);
		if (ret < 0) {
			vin_err("rx rate switch to 6g failed\n");
			return ret;
		}
		ret = write_register(deserial_if->bus_num, p_serializer_pipez_setting,
				setting_size, 0);
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
	if ((!strcmp(deserial_if->deserial_name, "max9296")) &&
		(sensor_info->deserial_port == 1)) {
		vin_info("set patch for max9296's second port\n");
		pdata = max9296_dual_setting_patch;
		setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, 0);
		if (ret < 0) {
			vin_err("max9296_dual_setting_patch failed\n");
			return ret;
		}
	}
	vin_info("set alias id!\n");
	ret = write_register(deserial_if->bus_num,
		p_alias_id_setting, alias_setting_size, 0);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}
	if (sensor_info->config_index & TRIG_MODE) {
		if (sensor_info->extra_mode == SENSING_24M_H190X ||
			sensor_info->extra_mode == SENSING_24M_H190XA) {
			setting_size = sizeof(max96717_max9295_trigger_mfp7_mfp8) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr,
					SERDES_REG_WIDTH, setting_size, max96717_max9295_trigger_mfp7_mfp8);
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

#if 0
	if (sensor_info->extra_mode == SENSING_24M_H190X) {
		pdata = serializer_errb_mfp5_setting[sensor_info->deserial_port];
		setting_size = sizeof(serializer_errb_mfp5_setting[sensor_info->deserial_port]) / sizeof(uint8_t);
		ret = write_register(deserial_if->bus_num, pdata, setting_size, 0);
		if (ret < 0) {
			vin_err("serializer_errb_mfp5_setting failed\n");
			return ret;
		}
	}
#endif
	usleep(5000);
	vin_info("sensor %s serializer init done\n", sensor_info->sensor_name);
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int i, req, req1, ret = RET_OK;
	int setting_size;
	char str[24] = {0};
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if (sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port_%d dev_port %d open fail\n", sensor_info->port,
				sensor_info->dev_port);
			return -RET_ERROR;
		}
	}

	if (sensor_info->extra_mode <= WHETRON_24M) {
		ret = deserializer_init_setting(sensor_info);
		if (ret < 0) {
			vin_err("deserializer_init_setting fail\n");
			return ret;
		}
	}

	ret = link_switch(sensor_info, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("link switch to port_%d failed\n", sensor_info->deserial_port);
		return ret;
	}

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

	// sensor fps set
	if (sensor_info->fps != 30 &&
	( (sensor_info->extra_mode == SENSING_24M_H190S) ||
	(sensor_info->extra_mode == SENSING_24M_H190X) ||
	(sensor_info->extra_mode == SENSING_24M_H190XA) )) {
		usleep(100 * 1000);
		uint16_t vmax_offset = (ISX031_DEF_VMAX * 30 / sensor_info->fps) - ISX031_DEF_VMAX;
		vmax_offset = ((vmax_offset >> 8) & 0xff) | ((vmax_offset << 8) & 0xff00);  // LSB
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, \
					sensor_info->sensor_addr, ISX031_VMAX_OFFSET, vmax_offset);
		if (ret < 0) {
			vin_err("sensor set vmax_offset err!\r\n");
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, \
					sensor_info->sensor_addr, ISX031_VMAX, ISX031_DEF_VMAX);
		if (ret < 0) {
			vin_err("sensor set vmax err!\r\n");
			return ret;
		}
	}

	//hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, 0x8AF0, 0x00);
	//hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, 0xBF14, 0x00);

	if (strcmp(deserial_if->deserial_name, "max9296") ||
		sensor_info->deserial_port != 0) {
		ret = link_switch(sensor_info, LINK_ALL);
		if (ret < 0) {
			vin_err("switch to link all failed for port%d\n",
				sensor_info->dev_port);
		}
	}

	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t value;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size =
			sizeof(max9296_stream_on_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(deserial_if->bus_num,
			deserial_if->deserial_addr, SERDES_REG_WIDTH, setting_size,
			max9296_stream_on_setting);
		if (ret < 0) {
			vin_err("deserial %s stream on failed!\n",
				deserial_if->deserial_name);
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		// setting_size =
		// 	sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 2;
		// ret = vin_write_array(deserial_if->bus_num,
		// 	deserial_if->deserial_addr, SERDES_REG_WIDTH, setting_size,
		// 	max96712_stream_on_setting);
		// if (ret < 0) {
		// 	vin_err("deserial %s stream on failed!\n",
		// 		deserial_if->deserial_name);
		// 	return ret;
		// }
		value = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, REG_STREAM_ON);
		if (value < 0) {
			vin_err("read %s reg %x failed\n", deserial_if->deserial_name, REG_STREAM_ON);
			return ret;
		}
		value |= FORCE_CSI_OUT_96712;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, REG_STREAM_ON, value);
		if (ret < 0) {
			vin_err("write %s reg %x failed\n", deserial_if->deserial_name,
				REG_STREAM_ON);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, 0x04, 0x0f);
		if (ret < 0) {
			vin_err("write max96712 video transmit Channel 0/1/2/3 enable error\n");
			return ret;
		}

		/* Video transmit enable for Port Z */
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, sensor_info->serial_addr, 0x02, 0x43);
		if (ret < 0) {
			vin_err("write 0x%x of serial failed!\n", 0x02);
			return ret;
		}

	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -1;
	}

	if (sensor_info->config_index & DUAL_LANE) {
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(deserial_if->bus_num, COMMON_ADDR_SENSOR,
			SERDES_REG_WIDTH, setting_size,	isx031_stream_on_setting);
		if (ret < 0) {
			vin_err("sensor %s stream on failed!\n",
				sensor_info->sensor_name);
			return ret;
		}
	}

	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size =
			sizeof(max9296_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(deserial_if->bus_num,
			deserial_if->deserial_addr, SERDES_REG_WIDTH, setting_size,
			max9296_stream_off_setting);
		if (ret < 0) {
			vin_err("deserial %s stream off failed!\n",
				deserial_if->deserial_name);
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		#if 0
		setting_size =
			sizeof(max96712_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(deserial_if->bus_num,
			deserial_if->deserial_addr, SERDES_REG_WIDTH, setting_size,
			max96712_stream_off_setting);
		if (ret < 0) {
			vin_err("deserial %s stream on failed!\n",
				deserial_if->deserial_name);
			return ret;
		}
		#endif

		/* Video transmit disable for Port Z */
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, sensor_info->serial_addr, 0x02, 0x03);
		if (ret < 0) {
			vin_err("write 0x%x of serial failed!\n", 0x02);
			return ret;
		}
	}  else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -1;
	}
	usleep(50*1000);
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
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0,};
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
	uint8_t reg_addr[4] = {0,};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0,};
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
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0,};
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

sensor_module_t isx031 = {
	.module = "isx031",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.stream_off = sensor_stream_off,
	.stream_on = sensor_stream_on,
	.get_sns_params = get_sns_info,
};

