/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[cam utility]:" fmt

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <linux/spi/spidev.h>
#include <pthread.h>

#include "cJSON.h"
#include "hb_cam_utility.h"
#include "hb_cam_gpio.h"
#include "hb_i2c.h"
#include "hb_spi.h"
#include "hb_vin_interface.h"
#include "cam_common.h"
#include "../inc/hb_vin.h"

#define BOARD_VERSION   "/sys/class/socinfo/board_id"
#define BUF_LEN         128
/**
 * Purpose: store spi fd
 * Range: hb_cam_utility.c
 * Attention: NA
 */
static int32_t cam_spi_fd[CAM_FD_NUMBER];
/**
 * Purpose: descript ae gain
 * Range: hb_cam_utility.c
 * Attention: NA
 */
static uint32_t ae_share_flag;	//src port:0xA0 + bit[16~31], dst port:bit[0~15]
/**
 * Purpose: store sensor device fd
 * Range: hb_cam_utility.c
 * Attention: NA
 */
static int32_t snsfds[CAM_MAX_NUM] = {-1};
#define CAM_I2C_RETRY_MAX	10

static int32_t sensor_ctrl_start_i2cbus(uint32_t port, uint32_t ops_flag, void *ops);
static void sensor_ctrl_stop_i2cbus(uint32_t port);

void VIN_DOFFSET(uint32_t *x, uint32_t n)
{
	switch (n) {
	case 2:
	*x = ((((*x) & 0xff00) >> 8) | (((*x) & 0xff) << 8));
	break;
	case 3:
	*x = (((*x) & 0x0000ff00) + (((*x) & 0xff0000) >> 16) + (((*x) & 0xff) << 16));
	break;
	case 4:
	*x = ((((((*x) & 0xff000000) >> 8) + (((*x) & 0xff0000) << 8)) >> 16) |
		(((((*x) & 0xff00) >> 8) + (((*x) & 0xff) << 8)) << 16));
	break;
	}
}

int vin_power_ctrl(unsigned int gpio, int on_off)
{
	int ret = RET_OK;

	ret = vin_gpio_export(gpio);
	if(ret < 0) {
		vin_err("gpio export fail\n");
		return ret;
	}
	ret = vin_gpio_set_dir(gpio, 1);
	if(ret < 0) {
		vin_err("gpio export fail\n");
		return ret;
	}
	if(GPIO_HIGH == on_off) {
		vin_gpio_set_value(gpio, 1);
	} else {
		vin_gpio_set_value(gpio, 0);
	}
	usleep(5 * 1000);
	vin_gpio_unexport(gpio);
	return ret;
}

int vin_control_isp(int *gpio_pin, int *gpio_level, int enable)
{
	int ret = RET_OK;

	if(enable == 0) {
		ret = vin_power_ctrl((uint32_t)gpio_pin[ISP_RESET], gpio_level[ISP_RESET]);
		usleep(5*1000);
	}
	if(enable == 1) {
		ret = vin_power_ctrl((uint32_t)gpio_pin[ISP_RESET], 1-gpio_level[ISP_RESET]);
		usleep(5*1000);
	}
	return ret;
}

int vin_write_array(int bus, uint32_t i2c_addr, int reg_width, int setting_size, uint32_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = (uint32_t)reg_width;

	for(i = 0; i < setting_size; i++) {
		i2c_cfg.reg = cam_setting[2 * i];
		i2c_cfg.data = cam_setting[2 * i + 1];

		k = CAM_I2C_RETRY_MAX;
		do {
			if (i2c_cfg.reg_size == REG16_VAL16) {
				ret = hb_vin_i2c_write_reg16_data16(
					(uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint16_t)i2c_cfg.data);
			} else if (i2c_cfg.reg_size == REG16_VAL8) {
				ret = hb_vin_i2c_write_reg16_data8(
					(uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			} else {
				ret = hb_vin_i2c_write_reg8_data8(
					(uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			}

			if (ret == RET_OK) {
				vin_dbg("camera write reg 0x%2x, val 0x%x\n", i2c_cfg.reg, i2c_cfg.data);
				break;
			} else {
				vin_warn("camera write reg 0x%2x, val 0x%x fail for the %d time\n",
					i2c_cfg.reg, i2c_cfg.data, CAM_I2C_RETRY_MAX + 1 - k);
			}

			usleep(20 * 1000);
		} while (k--); /* bad case: write 10 time in case of fail */

		if (ret < 0) {
			vin_err("camera write 0x%2x fail \n", i2c_cfg.reg);
			break;
		}
	}
	return ret;
}

int vin_i2c_write_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_vin_i2c_write_block_reg16((uint32_t)bus, (uint8_t)device_addr, (uint16_t)reg_addr, buffer, size);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_vin_i2c_write_block_reg8((uint32_t)bus, (uint8_t)device_addr, (uint16_t)reg_addr, buffer, size);
	}
	return ret;
}

int vin_i2c_read_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_vin_i2c_read_block_reg16((uint32_t)bus, (uint8_t)device_addr, (uint8_t)reg_addr, buffer, size);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_vin_i2c_read_block_reg8((uint32_t)bus, (uint8_t)device_addr, (uint16_t)reg_addr, buffer, size);
	}
	return ret;
}

int vin_i2c_write16(int bus, int reg_width, int sensor_addr, uint32_t reg_addr, uint16_t value)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_vin_i2c_write_reg16_data16((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr, value);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_vin_i2c_write_reg8_data16((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr, value);
	}
	return ret;
}

int vin_i2c_write8(int bus, int reg_width, int sensor_addr, uint32_t reg_addr, uint8_t value)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr, value);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_vin_i2c_write_reg8_data8((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr, value);
	}
	return ret;
}

int vin_i2c_read16(int bus, int reg_width, int sensor_addr, uint32_t reg_addr)
{
	int val = 0;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		val = hb_vin_i2c_read_reg16_data16((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		val = hb_vin_i2c_read_reg8_data16((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr);
	}
	vin_info(" reg_addr 0x%x val 0x%x\n", reg_addr, val);
	return val;
}

int vin_i2c_read8(int bus, int reg_width, int sensor_addr, uint32_t reg_addr)
{
	int val = 0;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		val = hb_vin_i2c_read_reg16_data8((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		val = hb_vin_i2c_read_reg8_data8((uint32_t)bus, (uint8_t)sensor_addr, (uint16_t)reg_addr);
	}
	return val;
}

int32_t vin_i2c_bit_write8(uint32_t bus, uint32_t i2c_addr, uint32_t reg_addr,
			   uint32_t reg_width, int32_t bit_mask, uint32_t value)
{
	int32_t ret = RET_OK;
	uint8_t val = 0;

	vin_info("bus %d@0x%x, reg 0x%x, bit_mask 0x%x value %d\n",
		 bus, i2c_addr, reg_addr, bit_mask, value);
	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
			return -RET_ERROR;
		/* read reg */
		ret = hb_vin_i2c_read_reg16_data8(bus, i2c_addr, reg_addr);
		if (ret < 0) {
			vin_err("%d@0x%x read reg addr 0x%x fail!!! ret %d\n",
				bus, i2c_addr, reg_addr, ret);
			return ret;
		}

		/* set value */
		ret &= (~bit_mask);	 // clear bit_mask to 0 for read value
		value &= bit_mask;	 // except bit_mask other bit cleraed to 0 for value
		val = ret | value;

		/* write reg */
		ret = hb_vin_i2c_write_reg16_data8(bus, i2c_addr, reg_addr, val);
		if (ret != 0) {
			vin_err("%d@0x%x write reg addr 0x%x fail!!! ret %d\n",
				bus, i2c_addr, reg_addr, ret);
			ret = -1;
		}
	} else if (reg_width == REG_WIDTH_8bit) {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		/* read reg value */
		ret = hb_vin_i2c_read_reg8_data8(bus, i2c_addr, reg_addr);
		if (ret < 0) {
			vin_err("%d@0x%x read reg addr 0x%x fail!!! ret %d\n",
				bus, i2c_addr, reg_addr, ret);
			return ret;
		}

		/* set value */
		ret &= (~bit_mask);	 // clear bit_mask to 0 for read value
		value &= bit_mask;	 // except bit_mask other bit cleraed to 0 for value
		val = ret | value;

		/* write reg */
		ret = hb_vin_i2c_write_reg8_data8(bus, i2c_addr, reg_addr, val);
		if (ret != 0) {
			vin_err("%d@0x%x write reg addr 0x%x fail!!! ret %d\n",
				bus, i2c_addr, reg_addr, ret);
			ret = -1;
		}
	} else {
		vin_err("invalid value, don't support reg_width %d\n", reg_width);
		return -1;
	}
	return ret;
}

int32_t vin_i2c_bit_array_write8(uint32_t bus, uint32_t i2c_addr,  uint32_t reg_width,
				 int setting_size, uint32_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	uint32_t bitmask;
	int ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = (uint32_t)reg_width;

	for(i = 0; i < setting_size; i++) {
		i2c_cfg.reg = cam_setting[3 * i];
		bitmask = cam_setting[3 * i + 1];
		i2c_cfg.data = cam_setting[3 * i + 2];
		if (DELAY_FLAG == i2c_cfg.reg) {
			continue;
		}
		k = CAM_I2C_RETRY_MAX;
		do {
			ret = vin_i2c_bit_write8(bus, i2c_addr, i2c_cfg.reg, reg_width,
						 bitmask, i2c_cfg.data);
			if (ret == RET_OK) {
				vin_dbg("camera write reg 0x%2x, val 0x%x\n", i2c_cfg.reg, i2c_cfg.data);
				break;
			} else if (ret < 0) {
				vin_err("i2c bit write fail!!!\n");
				return ret;
			}
			usleep(20 * 1000);
		} while (k--);
	}

	return ret;
}

int vin_spi_write_block(int bus, char *buffer, int size)
{
	int ret = RET_OK;

	ret = hb_vin_spi_write_block(cam_spi_fd[bus], (const int8_t *)buffer, size);
	if(ret < 0) {
		vin_err("camera: spi write 0x%x block fail\n", buffer[0]);
		return -HB_CAM_SPI_WRITE_BLOCK_FAIL;
	}
	return ret;
}

int vin_spi_read_block(int bus, char *buffer, int size)
{
	int ret = RET_OK;

	ret = hb_vin_spi_read_block(cam_spi_fd[bus], (const int8_t *)buffer, size);
	if(ret < 0) {
		vin_err("camera: spi read block fail\n");
		return -HB_CAM_SPI_READ_BLOCK_FAIL;
	}
	return ret;
}
int32_t cam_spi_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int8_t str[24] = {0};
	uint8_t mode, bits;
	uint32_t speed;

	if(cam_spi_fd[sensor_info->bus_num] <= 0) {
		snprintf((char *)str, sizeof(str), "/dev/spidev%d.%d",
				sensor_info->bus_num, sensor_info->spi_info.spi_cs);
		cam_spi_fd[sensor_info->bus_num]= open((const char *)str, O_RDWR);
		if(cam_spi_fd[sensor_info->bus_num] < 0) {
			vin_err("open spidev%d.%d fail\n", sensor_info->bus_num, sensor_info->spi_info.spi_cs);
			return -RET_ERROR;
		}
	}
	ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_MODE, &mode);/* PRQA S 4513 */
    if (ret < 0) {
        vin_err("can't set spi mode");
		return -RET_ERROR;
    }
	ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_WR_MODE, &sensor_info->spi_info.spi_mode);/* PRQA S 4513 */
    if (ret < 0) {
        vin_err("can't set spi mode");
		return -RET_ERROR;
    }
    ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_BITS_PER_WORD, &bits);	/* PRQA S 4513 */
    if (ret < 0) {
        vin_err("can't set spi mode");
		return -RET_ERROR;
    }
	ret = ioctl(cam_spi_fd[sensor_info->bus_num],
		SPI_IOC_WR_MAX_SPEED_HZ, &sensor_info->spi_info.spi_speed);/* PRQA S 4513 */
    if (ret < 0) {
        vin_err("can't set spi mode");
		return -RET_ERROR;
    }
    ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_MAX_SPEED_HZ, &speed);/* PRQA S 4513 */
    if (ret < 0) {
        vin_err("can't set spi mode");
		return -RET_ERROR;
    }
	return ret;
}

static int32_t camera_open_get_cnt(sensor_info_t *sensor_info, int32_t *open_cnt)
{
	int32_t ret = RET_OK;
	int8_t str[24] = {0};

	if (sensor_info->dev_port < 0) {
		*open_cnt = 0;
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	snprintf((char *)str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open((const char *)str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
		snsfds[sensor_info->port] = sensor_info->sen_devfd;
	}
	vin_dbg("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);
	ret = ioctl(sensor_info->sen_devfd, SENSOR_OPEN_CNT, open_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("%s port%d:%s ioctl fail %d\n", __func__,
			sensor_info->port, str, ret);
	} else {
		if (*open_cnt > 1) {
			vin_info("%s %s already open %d,return ok", __func__, str, *open_cnt);
			return RET_OK;
		}
	}

	return ret;
}
/**
 * @brief camera_start_stop_lock : get satart_cnt
 *
 * @param [in] sensor_info : sensor_info parsed in cam json
 * @param [in] start_cnt : store start_cnt
 *
 * @return ret
 */
static int32_t camera_start_stop_lock(sensor_info_t *sensor_info, int32_t *start_cnt)
{
	int32_t ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		*start_cnt = 0;
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_LOCK, 0);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("sensor_%d sen_devfd %d ioctl SENSOR_USER_LOCK fail %d\n",
			ret, sensor_info->sen_devfd);
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, start_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
		vin_err("sensor_%d sen_devfd %d ioctl SENSOR_SET_START_CNT fail %d\n",
			ret, sensor_info->sen_devfd);
		return -RET_ERROR;
	}
	return ret;
}
/**
 * @brief camera_start_stop_get_unlock : unlock sensor_devfd,
 *                                       modify start_cnt
 *
 * @param [in] sensor_info : sensor_info parsed in cam json
 *
 * @return ret
 */
static int32_t camera_start_stop_get_unlock(sensor_info_t *sensor_info)
{
	int32_t start_cnt;
	int32_t ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, &start_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
		vin_err("sensor_%d sen_devfd %d ioctl SENSOR_SET_START_CNT fail %d\n",
			ret, sensor_info->sen_devfd);
		return -RET_ERROR;
	}
	start_cnt += 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_START_CNT, &start_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
		vin_err("sensor_%d sen_devfd %d ioctl SENSOR_SET_START_CNT fail %d\n",
			ret, sensor_info->sen_devfd);
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("sensor_%d ioctl SENSOR_USER_UNLOCK fail %d\n", ret);
		return -RET_ERROR;
	}
	return ret;
}
/**
 * @brief camera_start_stop_put_unlock : unlock sensor_devfd,
 *                                       modify start_cnt
 *
 * @param [in] sensor_info : sensor_info parsed in cam json
 *
 * @return ret
 */
static int32_t camera_start_stop_put_unlock(sensor_info_t *sensor_info)
{
	int32_t start_cnt;
	int32_t ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, &start_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
		vin_err("sensor_%d ioctl SENSOR_SET_START_CNT fail %d\n", ret);
		return -RET_ERROR;
	}
	start_cnt -= 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_START_CNT, &start_cnt);	/* PRQA S 4513 */
	if (ret < 0) {
		ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
		vin_err("sensor_%d ioctl SENSOR_SET_START_CNT fail %d\n", ret);
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("sensor_%d ioctl SENSOR_USER_UNLOCK fail %d\n", ret);
		return -RET_ERROR;
	}
	return ret;
}
/**
 * @brief hb_deserial_init : init i2c, poc, load .so
 *
 * @param [in] deserial_info : deserial_info parsed from cam json
 *
 * @return ret
 */
int32_t hb_vin_deserial_init(deserial_info_t *deserial_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;
	int8_t deserial_buff[128] = {0};

	hb_vin_i2c_init(deserial_info->bus_num);
	ret = hb_vin_i2c_timeout_set(deserial_info->bus_num, deserial_info->bus_timeout);
	if (ret < 0) {
		return ret;
	}
	if (deserial_info->power_mode) {
		for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
			if(deserial_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)deserial_info->gpio_pin[gpio], deserial_info->gpio_level[gpio]);
			}
		}
		usleep(20*1000);
		for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
			if(deserial_info->gpio_pin[gpio] >= 0) {
				ret = (int32_t)((uint32_t)ret |
					(uint32_t)vin_power_ctrl((uint32_t)deserial_info->gpio_pin[gpio], 1-deserial_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("deserial_power_ctrl fail\n");
					ret = -HB_CAM_SERDES_POWERON_FAIL;
					goto free_i2cinit;
				}
			}
		}
	}
	if (deserial_info->deserial_name) {
		snprintf((char *)deserial_buff, sizeof(deserial_buff), "lib%s.so", deserial_info->deserial_name);
		if(NULL == deserial_info->deserial_fd) {
			deserial_info->deserial_fd =  dlopen((const char *)deserial_buff, RTLD_LAZY);
			if (deserial_info->deserial_fd == NULL) {
					vin_err("dlopen get error %s\n", dlerror());
					ret = -HB_CAM_SERDES_LIB_OPEN_FAIL;
					goto free_i2cinit;
				}
		}
		dlerror();
		deserial_info->deserial_ops = dlsym(deserial_info->deserial_fd, deserial_info->deserial_name);
		if (deserial_info->deserial_ops == NULL) {
			vin_err("dlsym get error %s\n", dlerror());
			ret = -HB_CAM_SERDES_LIB_OPS_FAIL;
			goto free_deserial;
		}
	} else {
		vin_err("there's no deserial\n");
		ret = -HB_CAM_SERDES_INFO_FAIL;
		goto free_deserial;
	}
	ret = ((deserial_module_t *)(deserial_info->deserial_ops))->init(deserial_info);
	if (ret < 0) {
		vin_err("hb_cam_s954_init_fail\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	return ret;

free_deserial:
		if(deserial_info->deserial_fd) {
			dlclose(deserial_info->deserial_fd);
			deserial_info->deserial_fd = NULL;
		}
free_i2cinit:
		hb_vin_i2c_deinit(deserial_info->bus_num);
		return ret;
}

/**
 * @brief hb_vin_cam_utility : deal with CAM_INIT, CAM_DEINIT,
                               CAM_START and CAM_STOP
 *
 * @param [in] cam_ctl : CAMERA_STATUS
 * @param [in] sensor_info : sensor_info parsed in cam json
 *
 * @return ret
 */
int32_t hb_vin_cam_utility(int32_t cam_ctl, sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int8_t name_buff[128] = {0};
	int32_t open_cnt = 0;
	int32_t start_cnt = 0;
	uint32_t userspace_enable = 0;

	if(cam_ctl == (int32_t)CAM_INIT) {
		if(sensor_info->bus_type == I2C_BUS) {
			hb_vin_i2c_init(sensor_info->bus_num);
			ret = hb_vin_i2c_timeout_set(sensor_info->bus_num, sensor_info->bus_timeout);
			if (ret < 0) {
				return ret;
			}
		} else if (sensor_info->bus_type == SPI_BUS) {
			cam_spi_init(sensor_info);
		}
		snprintf((char *)name_buff, sizeof(name_buff),  "lib%s.so", sensor_info->sensor_name);
		if(NULL == sensor_info->sensor_fd) {
			sensor_info->sensor_fd =  dlopen((const char *)name_buff, RTLD_LAZY);
			if (sensor_info->sensor_fd == NULL) {
				vin_err("dlopen %s  error %s\n", name_buff, dlerror());
				ret = -HB_VIN_SENSOR_LIB_OPEN_FAIL;
				goto err3;
			}
		}
		dlerror();
		sensor_info->sensor_ops = dlsym(sensor_info->sensor_fd, sensor_info->sensor_name);
		if (sensor_info->sensor_ops == NULL) {
			vin_err("dlsym get error %s\n", dlerror());
			ret = -HB_VIN_SENSOR_LIB_OPS_FAIL;
			goto err2;
		}
		ret = camera_open_get_cnt(sensor_info, &open_cnt);
		if (ret < 0) {
			vin_err("camera_open_get_cnt fail\n");
			ret = -HB_VIN_SENSOR_IOCTL_CNT_FAIL;
			goto err1;
		}
		if (open_cnt <= 1) {
			uint32_t src_port = 0;
			uint32_t port = sensor_info->port;

			src_port = (ae_share_flag >> 16) & 0xff;

			if (port < CAM_MAX_NUM && snsfds[port] > 0 && src_port - 0xA0 == port) {
				ret = ioctl(snsfds[port], SENSOR_AE_SHARE, &ae_share_flag);	/* PRQA S 4513 */
				if (ret < 0) {
					vin_err("%s port_%d ioctl fail %d\n", __func__, port, ret);
					return -HB_VIN_SENSOR_IOCTL_AE_FAIL;
				}
			}
			if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->ae_share_init) {
				ret = ((sensor_module_t *)(sensor_info->sensor_ops))->ae_share_init(ae_share_flag);
				if (ret < 0) {
					vin_err("ae_share_init fail\n");
					ret = -HB_VIN_SENSOR_LIB_AE_INIT_FAIL;
					goto err;
				}
				ae_share_flag = 0;
			}
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_init fail\n");
				ret = -HB_VIN_SENSOR_LIB_INIT_FAIL;
				goto err;
			}

			// start a new pthread write i2c in userspace
			if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control) {
				ret = ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control(port, &userspace_enable);
				if ((ret == 0) && (userspace_enable)) {
					ret = sensor_ctrl_start_i2cbus(port, userspace_enable, sensor_info);
					if (ret < 0) {
						vin_err("sensor_start pthread fail\n");
						ret = -HB_VIN_SENSOR_LIB_CTRL_FAIL;
						goto err;
					}
				}
			}
			// start a new pthread end
		}
	}
	if(cam_ctl == (int32_t)CAM_START) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->start) {
			return -HB_VIN_SENSOR_LIB_OPS_FAIL;
		}
		ret = camera_start_stop_lock(sensor_info, &start_cnt);
		if (ret < 0) {
			vin_err("camera_start_stop_lock fail\n");
			return -HB_VIN_SENSOR_IOCTL_LOCK_FAIL;
		}
		if (start_cnt == 0) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->start(sensor_info);
			if (ret < 0) {
				return -HB_VIN_SENSOR_LIB_START_FAIL;
			}
		}
		ret = camera_start_stop_get_unlock(sensor_info);
		if (ret < 0) {
			vin_err("camera_start_stop_get_unlock fail\n");
			return -HB_VIN_SENSOR_IOCTL_UNLOCK_FAIL;
		}
	}
	if(cam_ctl == (int32_t)CAM_STOP) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->stop) {
			return -HB_VIN_SENSOR_LIB_OPS_FAIL;
		}
		ret = camera_start_stop_lock(sensor_info, &start_cnt);
		if (ret < 0) {
			vin_err("camera_start_stop_lock fail\n");
			return -HB_VIN_SENSOR_IOCTL_LOCK_FAIL;
		}
		ae_share_flag = 0;
		if(start_cnt == 1) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->stop(sensor_info);
			if (ret < 0) {
				return -HB_VIN_SENSOR_LIB_STOP_FAIL;
			}
		}
		ret = camera_start_stop_put_unlock(sensor_info);
		if (ret < 0) {
			vin_info("camera_start_stop_put_unlock fail\n");
			return -HB_VIN_SENSOR_IOCTL_UNLOCK_FAIL;
		}
	}
	if(cam_ctl == (int32_t)CAM_DEINIT) {
		uint32_t port = sensor_info->port;
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->deinit) {
			ret = -HB_VIN_SENSOR_LIB_OPS_FAIL;
			goto err;
		}
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->deinit(sensor_info);
		if (ret < 0) {
			ret = -HB_VIN_SENSOR_LIB_DEINIT_FAIL;
			goto err;
		}
		// stop pthred
		if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control(port, &userspace_enable);
			if ((ret == 0) && (userspace_enable)) {
				sensor_ctrl_stop_i2cbus(port);
			}
		}
		// -----------
		hb_vin_i2c_deinit(sensor_info->bus_num);
		if(cam_spi_fd[sensor_info->bus_num] != 0) {
			close(cam_spi_fd[sensor_info->bus_num]);
			cam_spi_fd[sensor_info->bus_num] = -1;
		}
		if(sensor_info->sensor_fd) {
		   dlclose(sensor_info->sensor_fd);
		   sensor_info->sensor_fd = NULL;
		   sensor_info->sensor_ops = NULL;
		}
	}
	return ret;

err:
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
err1:
	if(sensor_info->sensor_fd) {
		sensor_info->sensor_ops = NULL;
	}
err2:
	if(sensor_info->sensor_fd) {
		dlclose(sensor_info->sensor_fd);
		sensor_info->sensor_fd = NULL;
	}
err3:
	hb_vin_i2c_deinit(sensor_info->bus_num);
	if(cam_spi_fd[sensor_info->bus_num] != 0) {
		close(cam_spi_fd[sensor_info->bus_num]);
		cam_spi_fd[sensor_info->bus_num] = -1;
	}

	return ret;
}
/**
 * @brief hb_vin_cam_ae_share_init : copy flag into cdev
 *
 * @param [in] port : user_dev_port
 * @param [in] flag : ae_share_flag
 *
 * @return ret
 */
int32_t hb_vin_cam_ae_share_init(uint32_t port, uint32_t flag)
{
	int32_t ret = 0;
	uint32_t src_port = 0;

	ae_share_flag = flag;
	src_port = (ae_share_flag >> 16) & 0xff;

	if (port < CAM_MAX_NUM && snsfds[port] > 0 && src_port - 0xA0 == port) {
		ret = ioctl(snsfds[port], SENSOR_AE_SHARE, &ae_share_flag);	/* PRQA S 4513 */
		if (ret < 0) {
			vin_err("%s port_%d ioctl fail %d\n", __func__, port, ret);
			return ret;
		}
	}
	return ret;
}
/**
 * @brief hb_deserial_deinit : power on, unload .so
 *
 * @param [in] deserial_info : deserial_info parsed form cam json
 *
 * @return ret
 */
int32_t hb_vin_deserial_deinit(deserial_info_t *deserial_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	// ret = ((deserial_module_t *)(deserial_info->deserial_ops))->deinit(deserial_info);
	if(deserial_info->power_mode) {
		for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
			if(deserial_info->gpio_pin[gpio] >=0) {
			    vin_dbg("gpio_num %d  %d %d %d \n", deserial_info->gpio_num, deserial_info->gpio_pin[gpio], deserial_info->gpio_level[gpio], 1-deserial_info->gpio_level[gpio]);
				ret = vin_power_ctrl((uint32_t)deserial_info->gpio_pin[gpio], deserial_info->gpio_level[gpio]);
			}
		}
		usleep(20*1000);
	}
	if(deserial_info->deserial_fd) {
	   dlclose(deserial_info->deserial_fd);
	   deserial_info->deserial_fd = NULL;
	   deserial_info->deserial_ops = NULL;
	}

	hb_vin_i2c_deinit(deserial_info->bus_num);
	vin_info("hb deserial deinit done.\n");
	return ret;
}

int32_t hb_vin_deserial_start_physical(deserial_info_t *deserial_info)
{
	int32_t ret = RET_OK;

	ret = ((deserial_module_t *)(deserial_info->deserial_ops))->start_physical(deserial_info);
	if (ret < 0) {
		vin_err("hb_deserial_start_physical error\n");
		return -HB_CAM_START_PHYSICAL_FAIL;
	}

	return ret;
}

/**
 * @brief get board id
 *
 * @return ret
 *
 * 	MATRIXDUO_A	<---> 0x631
 *	MATRIXDUO_B	<---> 0x632
 *	MATRIXDSOLO	<---> 0x641
 */
int vin_get_board_id(void)
{
	int32_t ret = RET_OK;
	char board_id[BUF_LEN] = {0};
	int board_ver_fd;

	board_ver_fd = open(BOARD_VERSION, O_RDONLY);
	if (board_ver_fd < 0) {
		vin_err("open pilot board version fail!\n");
		return -1;
	}
	do {
		ret = read(board_ver_fd, board_id, BUF_LEN);
		if (ret < 0) {
			vin_err("read board version is error! ret = %d\n", ret);
			ret = -1;
			break;
		}
		ret = atoi(board_id);
	} while (0);
	close(board_ver_fd);
	return ret;
}

// -- ctrl sensor in user-space
typedef struct sensor_ctrl_model_s {
	uint32_t port;
	pthread_t dev_ptr;
	int32_t dev_fd;
	uint8_t ctrl_runing;
	void *sensor_ops;
	// pthread_mutex_t mutex;
	uint32_t func_flag;
	hal_control_info_t info;
} sensor_ctrl_model_t;

sensor_ctrl_model_t model[CAM_MAX_NUM];

typedef struct sensor_ctrl_info_s {
	uint32_t port;
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint32_t grgain;
	uint32_t gbgain;
	uint32_t af_pos;
	uint32_t zoom_pos;
	uint32_t mode;
	uint32_t color_temper;
	uint32_t reserverd[8];
} sensor_ctrl_info_t;

#define CAMERA_CTRL_IOC_MAGIC	'x'
#define SENSOR_CTRL_INFO_SYNC	_IOWR(CAMERA_CTRL_IOC_MAGIC, 20, sensor_ctrl_info_t)

sensor_ctrl_info_t ctrl_data[CAM_MAX_NUM];

static int32_t sensor_ctrl_init(uint32_t port)
{
	if ((model[port].dev_fd = open("/dev/sensor_ctrl", O_RDWR)) < 0) {
		vin_err("sensor_ctrl port %d open fail, err \n", port, strerror(errno));
		return -RET_ERROR;
	}
	return 0;
}

static void sensor_ctrl_deinit(uint32_t port)
{
	close(model[port].dev_fd);
	model[port].dev_fd = 0;
}

static void *sensor_ctrl_pthread(void *input)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	int8_t send[4] = {0};
	sensor_ctrl_model_t *arg = (sensor_ctrl_model_t *)(input);

	vin_info(" start fe pthread %d\n", arg->port);
	ret = sensor_ctrl_init(arg->port);
	if (ret) {
		model[arg->port].ctrl_runing = 0;
		vin_err(" start fe pthread %d failed!\n", arg->port);
	} else {
		ctrl_data[arg->port].port = arg->port;
		while (arg->ctrl_runing) {
			// clock_gettime(CLOCK_REALTIME, &time);
			// vin_err("time---%d %d\n", time.tv_sec, time.tv_nsec);
			ret = ioctl(arg->dev_fd, SENSOR_CTRL_INFO_SYNC, &ctrl_data[arg->port]);	/* PRQA S 4513 */
			if ((!ret) && (arg->sensor_ops)) {
				// start control
				if((NULL != ((sensor_module_t *)(arg->sensor_ops))->start_control)) {
					((sensor_module_t *)(arg->sensor_ops))->start_control(&arg->info);
				}
				// write gain
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->aexp_gain_control) && (arg->func_flag & HAL_GAIN_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->aexp_gain_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].gain_buf, ctrl_data[arg->port].dgain_buf, ctrl_data[arg->port].gain_num);
				}
				// write line
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->aexp_line_control) && (arg->func_flag & HAL_LINE_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->aexp_line_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].line_buf, ctrl_data[arg->port].line_num);
				}
				// write line/gain
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->aexp_line_gain_control) && (arg->func_flag & HAL_AE_LINE_GAIN_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->aexp_line_gain_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].line_buf, ctrl_data[arg->port].line_num,
						ctrl_data[arg->port].gain_buf, ctrl_data[arg->port].dgain_buf, ctrl_data[arg->port].gain_num);
				}
				// write awb
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->awb_control) && (arg->func_flag & HAL_AWB_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->awb_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].rgain, ctrl_data[arg->port].bgain,
						ctrl_data[arg->port].grgain, ctrl_data[arg->port].gbgain);
				}
				// write awb cct
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->awb_cct_control) && (arg->func_flag & HAL_AWB_CCT_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->awb_cct_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].rgain, ctrl_data[arg->port].bgain,
						ctrl_data[arg->port].grgain, ctrl_data[arg->port].gbgain,
						ctrl_data[arg->port].color_temper);
				}
				// af control
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->af_control) && (arg->func_flag & HAL_AF_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->af_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].af_pos);
				}
				// zoom control
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->zoom_control) && (arg->func_flag & HAL_ZOOM_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->zoom_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].zoom_pos);
				}
				// start control
				if((NULL != ((sensor_module_t *)(arg->sensor_ops))->end_control)) {
					((sensor_module_t *)(arg->sensor_ops))->end_control(&arg->info);
				}
			}
		}
	}

	sensor_ctrl_deinit(arg->port);
	return NULL;
}

static int32_t sensor_ctrl_start_i2cbus(uint32_t port, uint32_t ops_flag, void *ops)
{
	int32_t ret = 0;

	if (port >= CAM_MAX_NUM || !ops) {
		vin_err("port %d or ops %p is err!\n", port, ops);
		return -1;
	}

	if (model[port].dev_ptr) {
		vin_info("port is init success!\n");
		return 0;
	}
	sensor_info_t *info = (sensor_info_t *)ops;

	// init info
	memset(&model[port].info, 0, sizeof(hal_control_info_t));
	model[port].func_flag = ops_flag;
	model[port].port = port;
	model[port].info.port = port;
	model[port].info.sensor_mode = info->sensor_mode;
	model[port].info.bus_type = info->bus_type;
	model[port].info.bus_num = info->bus_num;
	model[port].info.sensor_addr = info->sensor_addr;
	model[port].info.sensor1_addr = info->sensor1_addr;
	model[port].info.serial_addr = info->serial_addr;
	model[port].info.serial_addr1 = info->serial_addr1;
	model[port].info.eeprom_addr = info->eeprom_addr;
	memcpy(&model[port].info.sensor_spi_info, &info->spi_info, sizeof(spi_data_t));
	// init info end
	model[port].ctrl_runing = 1;
	ret = pthread_create(&model[port].dev_ptr, NULL, sensor_ctrl_pthread, (void *)(&model[port]));
	if(ret != 0){
		model[port].ctrl_runing = 0;
		model[port].dev_ptr = 0;
		vin_err("can't create thread: %s\n",strerror(ret));
		ret = -1;
	} else {
		vin_info("start ptf %ld \n", model[port].dev_ptr);
		model[port].sensor_ops = info->sensor_ops;
	}

	return ret;
}

static void sensor_ctrl_stop_i2cbus(uint32_t port)
{
	model[port].ctrl_runing = 0;
	if (model[port].dev_ptr) {
		vin_info("start stop ptf %d \n", model[port].dev_ptr);
		pthread_join(model[port].dev_ptr, NULL);	/* PRQA S 2810*/
	}
	memset(&model[port], 0, sizeof(sensor_ctrl_model_t));
	vin_info(" stop fe pthread %d\n", port);
}
