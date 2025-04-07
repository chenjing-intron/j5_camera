/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[imx390c]:" fmt

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/i2c-dev.h>
#include <malloc.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "../hb_cam_gpio.h"
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "inc/hb_vin.h"
#include "inc/imx390c_setting.h"
#include "inc/sensor_effect_common.h"

#define DEFAULT_POC_ADDR (0x28)
#define POC_RESET_ADDR   (0x01)
#define DEF_SERIAL_ADDR  (0x40)

enum {
    EXTRA_MODE_SINGLE_9296 = 0,
    EXTRA_MODE_DULE_9296,
    EXTRA_MODE_SINGLE_96712,
    EXTRA_MODE_QUAD_96712,
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
static int write_register(int bus, uint8_t *pdata, int setting_size)
{
    int ret = RET_OK;
    uint8_t i2c_slave;
    uint16_t reg_addr, value, delay;
    int i, len, k = 10;

    if (!pdata) {
        vin_err("Invalid param\n");
        return -1;
    }

    for (i = 0; i < setting_size;) {
        len = pdata[i];
        if (len == 4) {
            i2c_slave = pdata[i + 1] >> 1;
            reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
            value = pdata[i + 4];
            ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
            while (ret < 0 && k--) {
                if (k % 10 <= 9) {
					vin_info("write des %d@0x%02x 0x%04x=0x%02x ret %d retry %d\n",
						bus, i2c_slave, reg_addr, value, ret, k);
                    usleep(20 * 1000);
                    ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
                }
            }
            if (ret < 0) {
				vin_info("write des %d@0x%02x 0x%04x=0x%02x error %d\n",
					bus, i2c_slave, reg_addr, value, ret);
                return ret;
            }
            i = i + len + 1;
			vin_info("write des %d@0x%02x 0x%04x=0x%02x\n",
				bus, i2c_slave, reg_addr, value);
        } else if (len == 0) {
            delay = pdata[i + 1];
            usleep(delay * 1000);
            i = i + 2;
        }
    }

    return ret;
}

static int sensor_serial_config_i2c_write(sensor_info_t *sensor_info,
                                          uint16_t *pbuf, size_t size)
{
    int ret = RET_OK;
    int setting_size = 0, i, retry = 10;

    setting_size = size / sizeof(uint16_t) / 2;
    vin_info("max9295 setting_size %d\n", setting_size);
    for (i = 0; i < setting_size; i++) {
        if (DELAY_FLAG == pbuf[i * 2]) {
            usleep(pbuf[i * 2 + 1] * 1000U);
            continue;
        }
		vin_info("write ser %d@0x%02x 0x%04x=0x%02x\n", sensor_info->bus_num,
				sensor_info->serial_addr, pbuf[i * 2], (uint8_t)pbuf[i * 2 + 1]);
        ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
                                       sensor_info->serial_addr,
                                       pbuf[i * 2],
                                       (uint8_t)pbuf[i * 2 + 1]);
        if (ret < 0) {
            if (retry-- > 0) {
                (0 == i) ? (i = 0) : i--;
                continue;
            }
            vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
                   sensor_info->sensor_name, sensor_info->bus_num,
                   sensor_info->serial_addr, i,
                   pbuf[i * 2], pbuf[i * 2 + 1]);
            return ret;
        }
    }
    return ret;
}

static int deserial_mode_config_i2c_write(deserial_info_t *deserial_info,
                                          uint16_t *pbuf, size_t size)
{
    int ret = RET_OK;
    int setting_size = 0, i, retry = 10;

    setting_size = size / sizeof(uint16_t) / 2;
    vin_info("deserial setting_size %d\n", setting_size);
    for (i = 0; i < setting_size; i++) {
        if (DELAY_FLAG == pbuf[i * 2]) {
            usleep(pbuf[i * 2 + 1] * 1000U);
            continue;
        }
		vin_info("write des %d@0x%02x 0x%04x=0x%02x\n", deserial_info->bus_num,
				deserial_info->deserial_addr, pbuf[i * 2], (uint8_t)pbuf[i * 2 + 1]);
        ret = hb_vin_i2c_write_reg16_data8(deserial_info->bus_num,
                                       deserial_info->deserial_addr,
                                       pbuf[i * 2],
                                       (uint8_t)pbuf[i * 2 + 1]);
        if (ret < 0) {
            if (retry-- > 0) {
                (0 == i) ? (i = 0) : i--;
                continue;
            }
            vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
                   deserial_info->deserial_name, deserial_info->bus_num,
                   deserial_info->deserial_addr, i,
                   pbuf[i * 2], pbuf[i * 2 + 1]);
            return ret;
        }
    }
    return ret;
}

static int sensor_stream_control_by_port(sensor_info_t *sensor_info, bool on)
{
    int ret = -RET_ERROR;
    uint8_t val = 0, port;
    deserial_info_t *deserial_if = NULL;
    deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

    if (!strcmp(deserial_if->deserial_name, "max9296") ||
        !strcmp(deserial_if->deserial_name, "max96712")) {
        if (sensor_info->deserial_port > 3) {
            vin_err("Max support max deserial_port is 3.\n");
            return -RET_ERROR;
        }
		/* fix index for dvb poc */
        if (sensor_info->deserial_port >= 2) {
			port = 5 - sensor_info->deserial_port;
		} else {
			port = sensor_info->deserial_port;
		}
        ret = hb_vin_i2c_read_reg8_data8(sensor_info->bus_num, DEFAULT_POC_ADDR, POC_RESET_ADDR);
        if (ret < 0) {
            vin_err("fail to read poc(0x28--0x%x) current val\n", POC_RESET_ADDR);
            return -RET_ERROR;
        }
        val = (uint8_t)ret;
        if (on)
            val |= 1 << port;
        else
            val &= ~(1 << port);
		vin_info("write poc %d@0x%02x 0x%02x=0x%02x\n", sensor_info->bus_num,
				DEFAULT_POC_ADDR, POC_RESET_ADDR, val);
        ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num, DEFAULT_POC_ADDR, POC_RESET_ADDR, val);
        if (ret < 0) {
            vin_err("fail to write poc(0x28--0x%x) val(0x%x)\n", POC_RESET_ADDR, val);
            return -RET_ERROR;
        }
    } else {
        vin_err("serdes %s not support error\n", deserial_if->deserial_name);
        return -RET_ERROR;
    }

    return ret;
}

static int sensor_imx390c_deserial_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int setting_size = 0;
    uint8_t *pdata = NULL;
    deserial_info_t *deserial_if = NULL;
    deserial_if = (deserial_info_t *)sensor_info->deserial_info;

    if (deserial_if == NULL) {
        vin_err("no deserial here error\n");
        return -HB_CAM_SERDES_CONFIG_FAIL;
    }

    if (deserial_if->init_state == 1)
        return ret;

    if (!strcmp(deserial_if->deserial_name, "max9296") ||
        !strcmp(deserial_if->deserial_name, "max96712")) {
        setting_size = 1;
		vin_info("write poc %d@0x%02x off\n", deserial_if->bus_num, DEFAULT_POC_ADDR);
        ret = vin_write_array(deserial_if->bus_num, DEFAULT_POC_ADDR, REG8_VAL8,
                                 setting_size, poc_init_setting);
        if (ret < 0) {
            vin_err("write poc_init_setting error\n");
            return ret;
        }

        usleep(10 * 1000);
		vin_info("write poc %d@0x%02x on\n", deserial_if->bus_num, DEFAULT_POC_ADDR);
        ret = vin_write_array(deserial_if->bus_num, DEFAULT_POC_ADDR, REG8_VAL8,
                                 setting_size, poc_init_setting + 2);
        if (ret < 0) {
            vin_err("write poc_init_setting error\n");
            return ret;
        }
        usleep(50 * 1000);
    }

    if (!strcmp(deserial_if->deserial_name, "max9296")) {
        if (sensor_info->extra_mode == EXTRA_MODE_SINGLE_9296) {
            /* write max9296 init setting of 6Gbps mode */
            ret = deserial_mode_config_i2c_write(deserial_if, single_max9296_imx390c_init_setting,
                                                 sizeof(single_max9296_imx390c_init_setting));
            if (ret < 0) {
                vin_err("write single_max9296_imx390c_init_setting error\n");
                return ret;
            }
        } else if (sensor_info->extra_mode == EXTRA_MODE_DULE_9296) {
            pdata = imx390c_max9295a_9296_dual_init_setting;
            setting_size = sizeof(imx390c_max9295a_9296_dual_init_setting) / sizeof(uint8_t);
            if ((ret = write_register(deserial_if->bus_num, pdata, setting_size)) < 0)
                return ret;
        } else {
        	vin_err("Invalid extra_mode, should be %d <= extra_mode <= %d\n", \
				EXTRA_MODE_SINGLE_9296, EXTRA_MODE_QUAD_96712);
            return -RET_ERROR;
		}
    } else if (!strcmp(deserial_if->deserial_name, "max96712")) {
        if (EXTRA_MODE_SINGLE_96712 == sensor_info->extra_mode ||
            EXTRA_MODE_QUAD_96712 == sensor_info->extra_mode) {
            pdata = imx390c_max96712_init_setting;
            setting_size = sizeof(imx390c_max96712_init_setting) / sizeof(uint8_t);
            ret = write_register(deserial_if->bus_num, pdata, setting_size);
            if (ret < 0) {
                vin_err("write imx390c_max96712_init_setting error\n");
                return ret;
            }
        } else {
			vin_err("des %s not support extra_mode=%d error\n", deserial_if->deserial_name,
				sensor_info->extra_mode);
            return -RET_ERROR;
		}

        if (EXTRA_MODE_QUAD_96712 == sensor_info->extra_mode) {
            pdata = imx390c_max9295_port_a_init_setting;
            setting_size = sizeof(imx390c_max9295_port_a_init_setting) / sizeof(uint8_t);
            if ((ret = write_register(deserial_if->bus_num, pdata, setting_size)) < 0)
                return ret;

            pdata = imx390c_max9295_port_b_init_setting;
            setting_size = sizeof(imx390c_max9295_port_b_init_setting) / sizeof(uint8_t);
            if ((ret = write_register(deserial_if->bus_num, pdata, setting_size)) < 0)
                return ret;

            pdata = imx390c_max9295_port_c_init_setting;
            setting_size = sizeof(imx390c_max9295_port_c_init_setting) / sizeof(uint8_t);
            if ((ret = write_register(deserial_if->bus_num, pdata, setting_size)) < 0)
                return ret;

            pdata = imx390c_max9295_port_d_init_setting;
            setting_size = sizeof(imx390c_max9295_port_d_init_setting) / sizeof(uint8_t);
            if ((ret = write_register(deserial_if->bus_num, pdata, setting_size)) < 0)
                return ret;
        }
    } else {
        vin_err("des %s not support error\n", deserial_if->deserial_name);
        return -RET_ERROR;
	}

    deserial_if->init_state = 1;
    vin_info("deserial %s init done\n", deserial_if->deserial_name);
    return ret;
}

int sensor_imx390c_deserial_stream_on(sensor_info_t *sensor_info)
{
    int ret = -RET_ERROR;
    deserial_info_t *deserial_if = NULL;
    deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

    if (!strcmp(deserial_if->deserial_name, "max9296")) {
        ret = deserial_mode_config_i2c_write(deserial_if, max9296_imx390c_stream_on,
                                             sizeof(max9296_imx390c_stream_on));
        if (ret < 0) {
            vin_err("write %s failed\n", deserial_if->deserial_name);
            return -RET_ERROR;
        }
        vin_info("stream_on write %s successfully\n", deserial_if->deserial_name);
    } else if (!strcmp(deserial_if->deserial_name, "max96712")) {
        ret = deserial_mode_config_i2c_write(deserial_if, max96712_imx390c_stream_on,
                                             sizeof(max96712_imx390c_stream_on));
        if (ret < 0) {
            vin_err("write %s failed\n", deserial_if->deserial_name);
            return -RET_ERROR;
        }
        vin_info("stream_on write %s successfully\n", deserial_if->deserial_name);
    } else {
        vin_err("serdes %s not support error\n", deserial_if->deserial_name);
        return -RET_ERROR;
    }

    return ret;
}

static int sensor_serial_config_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    deserial_info_t *deserial_if = NULL;
    deserial_if = (deserial_info_t *)sensor_info->deserial_info;
    uint8_t *p_serial_info = NULL;
    int setting_size = 0;

    if (!strcmp(deserial_if->deserial_name, "max9296")) {
        if (EXTRA_MODE_SINGLE_9296 == sensor_info->extra_mode) {
            if ((ret = sensor_serial_config_i2c_write(sensor_info, single_imx390c_9295_init_setting,
                                                      sizeof(single_imx390c_9295_init_setting))) < 0)
                return ret;

			usleep(10*1000);
			ret = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, 0x40, 0x112);
			vin_info("read %d@0x40 0x112=0x%x\n", sensor_info->bus_num, ret);
        }
    } else if (!strcmp(deserial_if->deserial_name, "max96712")) {
        if (EXTRA_MODE_SINGLE_96712 == sensor_info->extra_mode) {
            switch (sensor_info->dev_port) {
            case 0:
                p_serial_info = imx390c_max9295_port_a_init_setting;
                setting_size = sizeof(imx390c_max9295_port_a_init_setting) / sizeof(uint8_t);
                break;
            case 1:
                p_serial_info = imx390c_max9295_port_b_init_setting;
                setting_size = sizeof(imx390c_max9295_port_b_init_setting) / sizeof(uint8_t);
                break;
            case 2:
                p_serial_info = imx390c_max9295_port_c_init_setting;
                setting_size = sizeof(imx390c_max9295_port_c_init_setting) / sizeof(uint8_t);
                break;
            case 3:
                p_serial_info = imx390c_max9295_port_d_init_setting;
                setting_size = sizeof(imx390c_max9295_port_d_init_setting) / sizeof(uint8_t);
                break;
            default:
                vin_err("Invalid dev port:%d\n", sensor_info->dev_port);
                return -RET_ERROR;
            }
            ret = write_register(deserial_if->bus_num, p_serial_info, setting_size);
            if (ret < 0)
                return ret;
        }
    } else {
    	vin_err("Invalid deserial_name: %s\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}

    /* max9295 need enable LDO */
	if (((sensor_info->extra_mode & 0xff) == EXTRA_MODE_DULE_9296) ||
	    ((sensor_info->extra_mode & 0xff) == EXTRA_MODE_QUAD_96712) ||
	    ((sensor_info->extra_mode & 0xff) == EXTRA_MODE_SINGLE_9296) ||
	    ((sensor_info->extra_mode & 0xff) == EXTRA_MODE_SINGLE_96712)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}
    vin_info("IMX390C_serial_config OK!\n");
    return ret;
}

static int imx390c_sensor_poweron(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int gpio = 0;

    if (sensor_info->power_mode) {
        for (; gpio < sensor_info->gpio_num; gpio++) {
            if (sensor_info->gpio_pin[gpio] >= 0) {
                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], 0);
                usleep(sensor_info->power_delay * 1000);
                ret += vin_power_ctrl(sensor_info->gpio_pin[gpio], 1);
                if (ret < 0) {
                    vin_err("vin_power_ctrl fail\n");
                    return -HB_CAM_SENSOR_POWERON_FAIL;
                }
            }
        }
    }

    return ret;
}

static int imx390c_sensor_poweroff(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int gpio = 0;

    if (sensor_info->power_mode) {
        for (; gpio < sensor_info->gpio_num; gpio++) {
            if (sensor_info->gpio_pin[gpio] >= 0) {
                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], 0);
                if (ret < 0) {
                    vin_err("vin_power_ctrl fail\n");
                    return -HB_CAM_SENSOR_POWEROFF_FAIL;
                }
            }
        }
    }

    return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
    int req, ret = RET_OK;
    char str[12] = {0};

    snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
    if (sensor_info->sen_devfd <= 0) {
        if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
            vin_err("port_%d open fail\n", sensor_info->port);
            return -RET_ERROR;
        }
    }
    vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
            sensor_info->dev_port, sensor_info->sen_devfd);

    if ((ret = imx390c_sensor_poweron(sensor_info)) < 0) {
	    close(sensor_info->sen_devfd);
	    return ret;
	}

	req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
	if (req == 0) {
		vin_info("imx390c serdes start init \n");
		ret = sensor_imx390c_deserial_init(sensor_info);
		hb_vin_mipi_pre_result(sensor_info->entry_num, 0, ret);
		if (ret < 0) {
			vin_err("sensor_imx390c_deserial_init fail\n");
			close(sensor_info->sen_devfd);
			return ret;
		}
	}

    if ((ret = sensor_serial_config_init(sensor_info)) < 0) {
        vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		close(sensor_info->sen_devfd);
		return ret;
    }
    return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
    int req, ret = RET_OK;

	req = hb_vin_mipi_pre_request(sensor_info->entry_num, 1, 0);
	if (req == 0) {
		ret = sensor_imx390c_deserial_stream_on(sensor_info);
		hb_vin_mipi_pre_result(sensor_info->entry_num, 1, ret);
		if (ret < 0)
			vin_err("sensor_imx390c_deserial_stream_on fail\n");
	}
    return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
    int ret = RET_OK;

    if ((ret = sensor_stream_control_by_port(sensor_info, false)) < 0)
        vin_err("sensor_stream_control_by_port off fail\n");

    return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
    return imx390c_sensor_poweroff(sensor_info);
}

sensor_module_t imx390c = {
    .module = "imx390c",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
};
