/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
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

#include "../utility/hb_cam_gpio.h"
#include "../utility/hb_cam_utility.h"
#include "../utility/hb_i2c.h"
#include "inc/hb_vin.h"
#include "inc/cc261c_setting.h"
#include "inc/sensor_effect_common.h"

/**
 * @brief write_register : write sensor and serdes reg
 *
 * @param [in] bus : i2c num
 * @param [in] pdata : setting need to write
 * @param [in] setting_size : setting num
 *
 * @return ret
 */
static int write_register(int bus,uint8_t i2c_slave_, uint8_t *pdata, int setting_size)
{
    int ret = RET_OK;
    uint8_t i2c_slave=i2c_slave_;
    uint16_t reg_addr, value;
    uint32_t delay;
    int i, len, k = 10;

    if (!pdata) {
        vin_info("Invalid param\n");
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
                    vin_info("init serdes reg:0x%x value:0x%x k:%d\n", reg_addr, value, k);
                    usleep(20 * 1000);
                    ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
                }
            }
            if (ret < 0) {
                vin_info("init serdes bus 0x%x i2c_slave = 0x%x reg:0x%x value:0x%x error\n",
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

/**
 * @brief sensor_init : sensor init
 *
 * @param [in] sensor_info : sensor info
 *
 * @return ret
 */
static int sensor_init(sensor_info_t *sensor_info)
{

	if(sensor_info->extra_mode == 0) {
		return RET_OK;
	}
	int ret = RET_OK;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	int bus = deserial_if->bus_num;
	if (deserial_if->init_state == 1) {
		vin_info("sensor already reset!\n");
		return ret;
	}

	int setting_size = 0;
	uint8_t *pdata = NULL;


	if (sensor_info->extra_mode==1) {
		pdata = cc261c_max96712_max9295_init_setting;
		setting_size = sizeof(cc261c_max96712_max9295_init_setting)/sizeof(uint8_t);
	}else if (sensor_info->extra_mode==2) {
		pdata = cc261c_max96712_max9295_init_setting_4lane;
		setting_size = sizeof(cc261c_max96712_max9295_init_setting_4lane)/sizeof(uint8_t);
	}else if(sensor_info->extra_mode==3){
		pdata = cc261c_max9296_max9295_init_setting;
		setting_size = sizeof(cc261c_max9296_max9295_init_setting)/sizeof(uint8_t);	
	}else{
		vin_err("please reset extra_mode value\n");
	}
	ret = write_register(deserial_if->bus_num,deserial_if->deserial_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write static uint8_t cc261c_max96712_max9295_init_setting error\n");
		return ret;
	}
	deserial_if->init_state = 1;
	return ret;	
}

/**
 * @brief sensor_start : sensor starts working
 *
 * @param [in] sensor_info : sensor info
 *
 * @return ret
 */
static int sensor_start(sensor_info_t *sensor_info)
{
   int ret = RET_OK;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;
	int setting_size = 0;
	uint8_t *pdata = NULL;

	if(sensor_info->extra_mode==2){
		pdata = cc261c_max96712_max9295_streamon;
		setting_size = sizeof(cc261c_max96712_max9295_streamon)/sizeof(uint8_t);
	}else if(sensor_info->extra_mode==3){
		pdata = cc261c_max9296_max9295_streamon;
		setting_size = sizeof(cc261c_max9296_max9295_streamon)/sizeof(uint8_t);	
	}else{
		vin_err("please reset extra_mode value\n");
	}

	ret = write_register(deserial_if->bus_num, deserial_if->deserial_addr,pdata, setting_size);
	if (ret < 0) {
		vin_err("write static uint8_t cc261c_max96712_max9295_streamon error\n");
		return ret;
	}

	 
	return ret;
}

/**
 * @brief sensor_stop : sensor stopped working
 *
 * @param [in] sensor_info : sensor info
 *
 * @return ret
 */
static int sensor_stop(sensor_info_t *sensor_info)
{
   	int ret = RET_OK;
         deserial_info_t *deserial_if = NULL;
         deserial_if = sensor_info->deserial_info;
         int setting_size = 0;
         uint8_t *pdata = NULL;

		
		if(sensor_info->extra_mode==2){
        	pdata = cc261c_max96712_max9295_streamoff;
        	setting_size = sizeof(cc261c_max96712_max9295_streamoff)/sizeof(uint8_t);
		}else if(sensor_info->extra_mode==3){
			pdata = cc261c_max9296_max9295_streamoff;
			setting_size = sizeof(cc261c_max9296_max9295_streamoff)/sizeof(uint8_t);	

		}else{
			vin_err("please reset extra_mode value\n");
		}
         ret = write_register(deserial_if->bus_num, deserial_if->deserial_addr,pdata, setting_size);
         if (ret < 0) {
                 vin_err("write static uint8_t cc261c_max96712_max9295_streamon error\n");
                 return ret;
         }


         return ret;

    return ret;
}

/**
 * @brief sensor_deinit : sensor deinit
 *
 * @param [in] sensor_info : sensor info
 *
 * @return ret
 */
static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

sensor_module_t cc261c = {
    .module = "cc261c",
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
};
