/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[imx390rcm]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "hb_cam_utility.h"
#include "inc/imx390_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ds953_setting.h"
#include "inc/ds954_setting.h"
#include "hb_i2c.h"

uint32_t sp1_lcg_line_save = 80;
uint32_t sp1_gain_save = 255;

enum mode_e mode_save;
struct GAIN_DATA_s gain_save;
struct GAIN_LIMIT_s gain_limit_save;
uint32_t Global_SP1_HCG_ex;
uint32_t Global_SP1_LCG_ex;
uint32_t Global_SP2_ex;
static uint8_t mode_pt_sw_save;
struct STA_exposure_s exposure_data;

int sensor_set_390rcm_reg_setting_unvalid(int bus, int sensor_addr)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x0008, 1);
	if(ret <0) {
		vin_err("390rcm_reg_setting_unvalid i2c write 0x0008 error ret %d\n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_set_390rcm_reg_setting_valid(int bus, int sensor_addr)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x0008, 0);
	if(ret <0) {
		vin_err("390rcm_reg_setting_valid i2c write 0x0008 error ret %d\n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_imx390rcm_954_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	setting_size = sizeof(ds954_imx390_rcm_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("deserial_if->bus_num= %d, deserial_if->deserial_addr = 0x%x, setting_size = %d, \n", deserial_if->bus_num, deserial_if->deserial_addr, setting_size);
	ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1, setting_size, ds954_imx390_rcm_init_setting);
	if (ret < 0) {
		vin_err("write ds954_imx390_init_setting error\n", sensor_info->serial_addr);
		return ret;
	}
	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	int retry = 0;
	int command_ret = -1;

	command_ret = system("api_cmd_camquery");
	if(command_ret != 0) {
		printf("reading fps failed\n");
		printf("try to reset ISP/camera\n");
		system("mcu_util --cas 3 0");
		if(sensor_info->power_mode) {
			for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
				if(sensor_info->gpio_pin[gpio] >= 0) {
					vin_dbg("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio], 1-sensor_info->gpio_level[gpio]);
					ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
					usleep(sensor_info->power_delay *1000);
					ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio], 1-sensor_info->gpio_level[gpio]);
					if(ret < 0) {
						vin_err("vin_power_ctrl fail\n");
						return -HB_CAM_SENSOR_POWERON_FAIL;
					}
				}
			}
		}
		sleep(1);
		system("mcu_util --cas 0 33.333 80 1");
		if(ret < 0) {
			printf("retry power on fail\n");
			return -1;
		}
		printf("after reset, checking fps....\n");
		for(retry = 0; retry < 5; retry++) {
			sleep(1);
			command_ret = system("api_cmd_camquery");
			if(command_ret == 0) {
				printf("ISP working stall normally, stop isp\n");
				system("api_cmd -I 0x14 max >> /tmp/isp_check 2>&1");
				sleep(1);
				break;
			} else {
				printf("retry %d, ISP not working\n", retry);
			}
		}
		if(retry == 5) {
			printf("reset ISP is still not ok\n");
			return -RET_ERROR;
		}
	} else {
			printf("gw5200-390 stop isp\n");
			system("api_cmd -I 0x14 max");
			sleep(1);
	}
	return ret;
}

int sensor_set_gain_sp1_h_part(int bus, int sensor_addr, struct SP1_HCG_P_gain_s sp1_hcg)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1H, sp1_hcg.AGAIN_SP1H_num & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H + 1, (sp1_hcg.PGA_GAIN_SP1H_num >> 8) & 0x03);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H, sp1_hcg.PGA_GAIN_SP1H_num & 0xff);
	if(ret <0) {
		vin_err("set_gain_sp1_h_part  i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}

int sensor_set_gain_sp1_l_part(int bus, int sensor_addr, struct SP1_LCG_P_gain_s sp1_lcg)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1H, sp1_lcg.AGAIN_SP1L_num & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H + 1, (sp1_lcg.PGA_GAIN_SP1L_num >> 8) & 0x03);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H, sp1_lcg.PGA_GAIN_SP1L_num & 0xff);
	if(ret <0) {
		vin_err("set_gain_sp1_l_part  i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}

int sensor_set_gain_sp1_h_toal(int bus, int sensor_addr, struct SP1_HCG_T_gain_s sp1_hcg)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, GAIN_SP1H, sp1_hcg.GAIN_SP1H_num & 0xff);
	if(ret <0) {
		vin_err("set_gain_sp1_h_toal  i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}

int sensor_set_gain_sp1_l_toal(int bus, int sensor_addr, struct SP1_LCG_T_gain_s sp1_lcg)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, GAIN_SP1L, sp1_lcg.GAIN_SP1L_num & 0xff);
	if(ret <0) {
		vin_err("set_gain_sp1_l_toal  i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_set_awb(int bus, int sensor_addr, float rg_gain, float b_gain)
{
	int ret = RET_OK;
	AWB_DATA_s awb_data;
	awb_data.WBG_R = (uint16_t)((rg_gain*256) + 0.5);
	awb_data.WBG_B = (uint16_t)b_gain;
	awb_data.WBG_GR = 256;
	awb_data.WBG_GB = 256;
	sensor_set_390rcm_reg_setting_unvalid(bus, sensor_addr);
	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x31, ((awb_data.WBG_R >> 8) & 0x0f));
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x30, awb_data.WBG_R & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x37, ((awb_data.WBG_GB >> 8) & 0x0f));
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x36, awb_data.WBG_GB & 0xff);
	if (ret < 0) {
		vin_err("sensor_set_awb fail\n");
		return ret;
	}
	sensor_set_390rcm_reg_setting_valid(bus, sensor_addr);
	return ret;
}
int sensor_set_gain_sp2_part(int bus, int sensor_addr, struct SP2_P_gain_s sp2)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1L, sp2.AGAIN_SP1L_num & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H + 1, (sp2.PGA_GAIN_SP2_num >> 8) & 0x03);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H, sp2.PGA_GAIN_SP2_num & 0xff);
	if(ret <0) {
		vin_err("set_gain_sp2_part i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}

int sensor_set_ex_gain_pwl(int bus, int sensor_addr, struct GAIN_PWL_s pwl_data)
{
	int ret = RET_OK;

	ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1L, pwl_data.AGAIN_SP1L_num & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1H, pwl_data.AGAIN_SP1H_num & 0xff);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H + 1, ((pwl_data.PGA_GAIN_SP1H_num >> 8) & 0x03));
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, PGA_GAIN_SP1H, pwl_data.PGA_GAIN_SP1H_num & 0xff);
	if(ret <0) {
		vin_err("set_ex_gain_pwl i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_set_390_exposure_init(int bus, int sensor_addr)
{
	int ret = RET_OK;
	exposure_data.MODE_VMAX_num = 1125;
	exposure_data.MODE_HMAX_num = 4400;
	exposure_data.FMAX_num = 0;
	exposure_data.OFFSET_CLK_SP1H_num = 0;
	exposure_data.OFFSET_CLK_SP1L_num = 0;

	Global_SP1_HCG_ex = exposure_data.MODE_VMAX_num * (exposure_data.FMAX_num + 1);
	Global_SP1_LCG_ex = exposure_data.MODE_VMAX_num * (exposure_data.FMAX_num + 1);
	Global_SP2_ex = exposure_data.MODE_VMAX_num * (exposure_data.FMAX_num + 1);

	ret = hb_vin_i2c_write_block(bus, sensor_addr, OFFSET_CLK_SP1H, exposure_data.OFFSET_CLK_SP1H_num, 2);
	ret |= hb_vin_i2c_write_block(bus, sensor_addr, AGAIN_SP1H, exposure_data.OFFSET_CLK_SP1L_num, 2);
	ret |= hb_vin_i2c_write_block(bus, sensor_addr, MODE_VMAX, exposure_data.MODE_VMAX_num, 3);
	ret |= hb_vin_i2c_write_block(bus, sensor_addr, MODE_HMAX, exposure_data.MODE_HMAX_num, 2);
	ret |= hb_vin_i2c_write_reg16_data8(bus, sensor_addr, FMAX, exposure_data.FMAX_num & 0xff);
	if(ret <0) {
		vin_err("set_390_exposure_init i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_set_sp1_exposure(int bus, int sensor_addr, uint32_t input_data)
{
	int ret = RET_OK;
	uint32_t shs1_data = 0;

	shs1_data = Global_SP1_HCG_ex - input_data;

	if(shs1_data < 2 || shs1_data > Global_SP2_ex)
		return -1;
	// camera_write_reg(SHS1 + 2, (shs1_data >> 16) & 0xff);
	// camera_write_reg(SHS1 + 1, (shs1_data >> 8) & 0xff);
	// camera_write_reg(SHS1, shs1_data & 0xff);

	ret = hb_vin_i2c_write_block(bus, sensor_addr, SHS1, shs1_data, 3);
	if(ret <0) {
		vin_err("set_sp1_exposure i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}

int sensor_set_sp2_exposure(int bus, int sensor_addr, uint32_t input_data)
{
	int ret = RET_OK;
	uint32_t shs2_data = 0;

	shs2_data = Global_SP2_ex - input_data;

	if(shs2_data < 2 || shs2_data > Global_SP2_ex)
		return -1;
	// camera_write_reg(SHS2 + 2, (shs2_data >> 16) & 0xff);
	// camera_write_reg(SHS2 + 1, (shs2_data >> 8) & 0xff);
	// camera_write_reg(SHS2, shs2_data & 0xff);
	ret = hb_vin_i2c_write_block(bus, sensor_addr, SHS2, shs2_data, 3);
	if(ret <0) {
		vin_err("set_sp2_exposure i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return ret;
}
int sensor_awb_control(int bus, int sensor_addr, uint8_t awb_sw)
{
	int ret = RET_OK;
    char temp_data;

	temp_data = hb_vin_i2c_read_reg16_data8(bus, sensor_addr, 0x36a8);
    if (awb_sw == 1)
            temp_data |= 0x01;
    else
            temp_data &= 0xfe;

	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x36a8, awb_sw & 0x01);
	if(ret <0) {
		vin_err("sensor_awb_control i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return RET_OK;
}

int sensor_set_awb_init(int bus, int sensor_addr, AWB_DATA_s awb_data)
{
	int ret = RET_OK;

	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x31, ((awb_data.WBG_R >> 8) & 0x0f));
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x30, awb_data.WBG_R & 0xff);
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x33, ((awb_data.WBG_GR >> 8) & 0x0f));
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x32, awb_data.WBG_GR & 0xff);
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x35, ((awb_data.WBG_GB >> 8) & 0x0f));
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x34, awb_data.WBG_GB & 0xff);
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x37, ((awb_data.WBG_B >> 8) & 0x0f));
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x36, awb_data.WBG_B & 0xff);
	if(ret <0) {
		vin_err("sensor_set_awb_init i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return RET_OK;
}
int sensor_set_limit_gain(int bus, int sensor_addr, struct GAIN_LIMIT_s gain_limit)
{
	int ret = RET_OK;
	gain_limit_save = gain_limit;

	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1H_LIMIT, gain_limit.AG_SP1H_LIMIT_num & 0xff);
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, AGAIN_SP1L_LIMIT, gain_limit.AG_SP1L_LIMIT_num & 0xff);
	if(ret <0) {
		vin_err("sensor_set_limit_gain i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return RET_OK;
}
int sensor_set_390_mode_sw(int bus, int sensor_addr, uint16_t mode_sw)
{
	int ret = RET_OK;

	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, WDC_OUTSEL, (mode_sw >> 8) & 0xff);
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, WDC_THR_FRM_SEL, mode_sw & 0xff);
	if(ret <0) {
		vin_err("sensor_set_390_mode_sw i2c write error ret %d \n", ret);
		ret = -RET_ERROR;
	}
	return RET_OK;
}

int sensor_set_ex_gain(int bus, int sensor_addr, uint32_t exposure_setting, uint32_t gain_setting_0, uint16_t gain_setting_1)
{
	int ret = RET_OK;
	float val = 0;
	uint16_t line_setting = exposure_setting;
	double temp_gain = 1.0;
	double temp_setting_0 = 1.0;

	if (gain_setting_1 < 256)
		gain_setting_1 = 256;
	temp_gain = gain_setting_1 / 256.0;
	val = 20 * log10(temp_gain) / 0.3;
	gain_setting_1 = (uint16_t)(val + 0.5);
	temp_gain = gain_setting_0 / 256.0;

	switch(mode_save) {
		case SP1_HCG_P:
		case SP1_HCG_T:
			temp_setting_0 = (exposure_setting * temp_gain * SP1_LCG_MULTIPLE)/(SP1_HCG_MULTIPLE * SP1_HCG_LINE);
			line_setting = SP1_HCG_LINE;
			break;
		case SP1_LCG_P:
		case SP1_LCG_T:
			temp_setting_0 = temp_gain;
			line_setting = exposure_setting;
			break;
		case SP2_P:
			if ((exposure_setting * gain_setting_0) > (SP2_MULTIPLE * 255 )) {
				temp_setting_0 = (exposure_setting * temp_gain * SP1_LCG_MULTIPLE)/(SP2_MULTIPLE * SP2_LINE_H);
				line_setting = SP2_LINE_H;
			} else {
				temp_setting_0 = (exposure_setting * temp_gain * SP1_LCG_MULTIPLE)/(SP2_MULTIPLE * SP2_LINE_L);
				line_setting = SP2_LINE_L;
			}
			break;
		default:
			line_setting = exposure_setting;
			temp_setting_0 = temp_gain;
			break;
	}
	if(temp_setting_0 < 1)
		temp_setting_0 = 1.0;

	val = 20 * log10(temp_setting_0) / 0.3;
	gain_setting_0 = (uint16_t)(val + 0.5);

	if (gain_setting_0 > 0xf0)
		gain_setting_0 = 0xf0;
	if (gain_setting_1 > 0xf0)
		gain_setting_1 = 0xf0;

	switch(mode_save) {
		case SP1_HCG_P: {
			struct SP1_HCG_P_gain_s gain_p_h;
			if(gain_setting_0 <= gain_limit_save.AG_SP1H_LIMIT_num) {
				gain_p_h.AGAIN_SP1H_num = gain_setting_0;
				gain_p_h.PGA_GAIN_SP1H_num = 0;
			} else {
				gain_p_h.AGAIN_SP1H_num = gain_limit_save.AG_SP1H_LIMIT_num;
				gain_p_h.PGA_GAIN_SP1H_num = (gain_setting_0 - gain_limit_save.AG_SP1H_LIMIT_num);
			}
			gain_save.gain_p_d.sp1_p_hcg_num = gain_p_h;
			sensor_set_gain_sp1_h_part(bus, sensor_addr, gain_p_h);
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			break;
		}
		case SP1_HCG_T: {
			struct SP1_HCG_T_gain_s gain_t_h;
			gain_t_h.GAIN_SP1H_num = gain_setting_0;
			gain_save.gain_t_d.sp1_t_hcg_num = gain_t_h;
			sensor_set_gain_sp1_h_toal(bus, sensor_addr, gain_t_h);
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			break;
		}
		case SP1_LCG_P: {
			struct SP1_LCG_P_gain_s gain_p_l;
			if(gain_setting_0 <= gain_limit_save.AG_SP1L_LIMIT_num) {
				gain_p_l.AGAIN_SP1L_num = gain_setting_0;
				gain_p_l.PGA_GAIN_SP1L_num = 0;
			} else {
				gain_p_l.AGAIN_SP1L_num = gain_limit_save.AG_SP1L_LIMIT_num;
				gain_p_l.PGA_GAIN_SP1L_num = (gain_setting_0 - gain_limit_save.AG_SP1L_LIMIT_num);
			}
			gain_save.gain_p_d.sp1_p_lcg_num = gain_p_l;
			sensor_set_gain_sp1_l_part(bus, sensor_addr, gain_p_l);
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			break;
		}
		case SP1_LCG_T: {
			struct SP1_LCG_T_gain_s gain_t_l;
			gain_t_l.GAIN_SP1L_num = gain_setting_0;
			gain_save.gain_t_d.sp1_t_lcg_num = gain_t_l;
			sensor_set_gain_sp1_l_toal(bus, sensor_addr, gain_t_l);
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			break;
		}
		case SP2_P: {
			struct SP2_P_gain_s gain_sp2;
			if(gain_setting_0 <= gain_limit_save.AG_SP1L_LIMIT_num) {
				gain_sp2.AGAIN_SP1L_num = gain_setting_0;
				gain_sp2.PGA_GAIN_SP2_num = 0;
			} else {
				gain_sp2.AGAIN_SP1L_num = gain_limit_save.AG_SP1L_LIMIT_num;
				gain_sp2.PGA_GAIN_SP2_num = (gain_setting_0 - gain_limit_save.AG_SP1L_LIMIT_num);
			}
			gain_save.gain_p_d.sp2_p_g_num = gain_sp2;
			sensor_set_gain_sp2_part(bus, sensor_addr, gain_sp2);
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			break;
		}
		case HDR: {
			struct GAIN_PWL_s gain_pwl;
			uint16_t gain_temp = gain_setting_0 > gain_setting_1 ? gain_setting_0 : gain_setting_1;

			if(gain_setting_0 > 0x64 || gain_setting_1 > 0x64) {
				gain_pwl.PGA_GAIN_SP1H_num = gain_temp - 0x64;
				if(gain_setting_0 > gain_setting_1) {
					gain_pwl.AGAIN_SP1H_num = 0x64;
					gain_pwl.AGAIN_SP1L_num = gain_setting_1;
				} else {
					if ( gain_setting_0 > gain_pwl.PGA_GAIN_SP1H_num)
						gain_pwl.AGAIN_SP1H_num = gain_setting_0 - gain_pwl.PGA_GAIN_SP1H_num;
					else
						gain_pwl.AGAIN_SP1H_num = 0;
					gain_pwl.AGAIN_SP1L_num = 0x64;
				}
			} else {
				gain_pwl.AGAIN_SP1H_num = gain_setting_0;
				gain_pwl.AGAIN_SP1L_num = gain_setting_1;
				gain_pwl.PGA_GAIN_SP1H_num = 0;
			}

			gain_save.pwl_d = gain_pwl;
			sensor_set_sp1_exposure(bus, sensor_addr, line_setting);
			sensor_set_ex_gain_pwl(bus, sensor_addr, gain_pwl);
			break;
		}
		default:
			break;
		}

	return ret;
}

int sensor_moder_sw(int bus, int sensor_addr, enum mode_e mode_input)
{
	uint16_t mode_data = 0;
	uint8_t  mode_pt_sw = 0;
	uint8_t  alter_data = 0x68;

	mode_data = (uint16_t)(mode_input & 0xffff);
	// vin_camera_read_reg(ADGAIN_SP1X_SEP_MODE, &mode_pt_sw);
	mode_pt_sw = mode_pt_sw_save;

	mode_pt_sw &= 0xf1;
	switch(mode_input) {
	case SP1_HCG_P:
		alter_data = 0x68;
		mode_pt_sw &= (~(1 << 1));
		mode_pt_sw &= (~(1 << 0));
		break;
	case SP1_LCG_P:
		alter_data = 0x6a;
		mode_pt_sw &= (~(1 << 2));
		mode_pt_sw &= (~(1 << 0));
		break;
	case SP2_P:
		alter_data = 0x6c;
		mode_pt_sw &= (~(1 << 3));
		mode_pt_sw &= (~(1 << 0));
		break;
	case SP1_HCG_T:
		alter_data = 0x68;
		mode_pt_sw |= (1 << 1);
		mode_pt_sw &= (~(1 << 0));
		break;
	case SP1_LCG_T:
		mode_pt_sw |= (1 << 2);
		mode_pt_sw &= (~(1 << 0));
		alter_data = 0x6a;
		break;
	case SP2_T:
		alter_data = 0x6c;
		mode_pt_sw |= (1 << 3);
		mode_pt_sw &= (~(1 << 0));
		break;
	case HDR:
		mode_pt_sw &= (~(0x07 << 1));
		mode_pt_sw &= (~(1 << 0));
		alter_data = 0x95;
		break;
	default:
		break;
	}

	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, 0x0332, alter_data);
	mode_pt_sw_save = mode_pt_sw;
	hb_vin_i2c_write_reg16_data8(bus, sensor_addr, ADGAIN_SP1X_SEP_MODE, mode_pt_sw);

	mode_save = mode_input;
	sensor_set_ex_gain(bus, sensor_addr, sp1_lcg_line_save, sp1_gain_save, 385);
	sensor_set_390_mode_sw(bus, sensor_addr, mode_data);

	return 0;
}

int sensor_global_data_init(int bus, int sensor_addr)
{
	char temp_data = 0;

	gain_limit_save.AG_SP1H_LIMIT_num = 0x64;
	gain_limit_save.AG_SP1L_LIMIT_num = 0x64;

	memset(&gain_save, 0, sizeof(struct GAIN_DATA_s));

	sensor_set_gain_sp1_h_part(bus, sensor_addr, gain_save.gain_p_d.sp1_p_hcg_num);
	sensor_set_gain_sp1_h_toal(bus, sensor_addr, gain_save.gain_t_d.sp1_t_hcg_num);
	sensor_set_gain_sp1_l_part(bus, sensor_addr, gain_save.gain_p_d.sp1_p_lcg_num);
	sensor_set_gain_sp1_l_toal(bus, sensor_addr, gain_save.gain_t_d.sp1_t_lcg_num);
	sensor_set_gain_sp2_part(bus, sensor_addr, gain_save.gain_p_d.sp2_p_g_num);
	sensor_set_ex_gain_pwl(bus, sensor_addr, gain_save.pwl_d);

	mode_save = HDR;
	sensor_set_390_exposure_init(bus, sensor_addr);
	sensor_set_sp1_exposure(bus, sensor_addr, 337);  // 151
	sensor_set_sp2_exposure(bus, sensor_addr, 371);  // 11ms

	temp_data = hb_vin_i2c_read_reg16_data8(bus, sensor_addr, SHS2_SET_MODE);
	mode_pt_sw_save = temp_data;
	AWB_DATA_s awb_start_data;
	awb_start_data.WBG_R = 0x200;
	awb_start_data.WBG_GR = 0x100;
	awb_start_data.WBG_GB = 0x100;
	awb_start_data.WBG_B = 0x180;
	sensor_awb_control(bus, sensor_addr, 0);

	sensor_set_awb_init(bus, sensor_addr, awb_start_data);
	sensor_set_limit_gain(bus, sensor_addr, gain_limit_save);
	sensor_set_ex_gain(bus, sensor_addr, sp1_lcg_line_save, sp1_gain_save, sp1_gain_save);

	sensor_moder_sw(bus, sensor_addr, mode_save);
	return RET_OK;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	ret = sensor_imx390rcm_954_init(sensor_info);
	if (ret < 0) {
		vin_err("sensor_imx390rcm_954_init fail\n");
		return ret;
	}
	setting_size = 1;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, ds953_imx390_init_setting);
	usleep(100*1000);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds953_imx390_init_setting[2]));
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds953_imx390_init_setting[4]));
	usleep(800);
	ret |= vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, setting_size, &(ds953_imx390_init_setting[6]));
	if (ret < 0) {
		vin_err("ub953 write error\n");
		deserial_if = (deserial_info_t *)sensor_info->deserial_info;
		ret = ((deserial_module_t *)(deserial_if->deserial_ops))->reset(deserial_if);
		return -RET_ERROR;
	}
	vin_info("ub953 init done\n");
	setting_size = sizeof(imx390_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx390_init_setting);
	if (ret < 0) {
		vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	char temp_data = 0;
	temp_data = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3053);
	temp_data = temp_data | 0x80;
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3053, temp_data);
	if (ret < 0) {
		vin_err("imx390rcm write 0x3053 error\n");
		return ret;
	}
	setting_size = sizeof(imx390_calibration_setting_SHD_DIFF)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx390_calibration_setting_SHD_DIFF);
	if (ret < 0) {
		vin_err("imx390rcm vin_write_array error\n");
		return ret;
	}
	temp_data = 0;
	temp_data = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x1d0);
	temp_data = temp_data & 0xfe;
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x1d0, temp_data);
	if (ret < 0) {
		vin_err("imx390rcm write 0x1d0 error\n");
		return ret;
	}
	temp_data = 0;
	temp_data = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3AF6);
	temp_data = temp_data & 0xf3;
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3AF6, temp_data);
	if (ret < 0) {
		vin_err("imx390rcm write 0x3AF6 error\n");
		return ret;
	}
	setting_size = sizeof(imx390_calibration_setting_SGAIN)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx390_calibration_setting_SGAIN);
	if (ret < 0)
		return ret;

	temp_data = 0;
	temp_data = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x1d0);
	temp_data = temp_data | 0x01;
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x1d0, temp_data);
	if (ret < 0) {
		vin_err("imx390rcm write 0x1d0 error\n");
		return ret;
	}
	sensor_global_data_init(sensor_info->bus_num, sensor_info->sensor_addr);
	return ret;
}
int sensor_start(sensor_info_t *sensor_info)
{
	int setting_size = 0;
	int ret = RET_OK;

	setting_size = sizeof(imx390_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx390_stream_on_setting);
	if(ret < 0) {
		vin_err("sensor_start %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(imx390_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx390_stream_off_setting);
	if(ret < 0) {
		vin_err("sensor_stop %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}
int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}
	return ret;
}
int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

sensor_module_t imx390rcm = {
	.module = "imx390rcm",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.set_ex_gain = sensor_set_ex_gain,
	.set_awb = sensor_set_awb,
};

