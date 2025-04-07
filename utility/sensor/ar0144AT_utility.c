/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0144AT]:" fmt

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

#include "../hb_cam_utility.h"
#include "inc/ds933_setting.h"
#include "inc/ds954_setting.h"
#include "inc/ds960_setting.h"
#include "inc/ar0144AT_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_i2c.h"

#define MAX_EXPOSUNRE_REG       (0x311C)
#define MIN_EXPOSUNRE_REG       (0x311E)
#define FRAME_LENGTH_LINES      (0x300A)
#define X_ADDR_START            (0x3004)
#define X_ADDR_END              (0x3008)
#define Y_ADDR_START            (0x3002)
#define Y_ADDR_END              (0x3006)
#define COARSE_INTEGRATION_TIME (0x3012)
#define ANALOG_GAIN             (0x3060)
#define DIGITAL_GAIN            (0x305E)

#define TTF  9
#define T_FFV  14
#define MARGIN 5
#define DEFAULT_FPS 30
#define DEFAULT_FRAMELINE 0x640

#define TUNING_LUT

static uint32_t ae_share_flag;

int sensor_ar0144AT_954_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = sensor_info->deserial_info;

	if (!strcmp(deserial_if->deserial_name, "s954")) {
		setting_size = sizeof(ds954_ar0144AT_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("deserial_if->bus_num= %d\n", deserial_if->bus_num);
		vin_dbg("deserial_if->deserial_addr = 0x%x\n", deserial_if->deserial_addr);
		vin_dbg("setting_size = %d\n", setting_size);
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
								setting_size, ds954_ar0144AT_init_setting);
		if (ret < 0) {
			vin_err("write ds954_ar0144AT_init_setting error\n",
					sensor_info->serial_addr);
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "s960")) {
		setting_size = sizeof(ds960_ar0144AT_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("deserial_if->bus_num= %d\n", deserial_if->bus_num);
		vin_dbg("deserial_if->deserial_addr = 0x%x\n", deserial_if->deserial_addr);
		vin_dbg("setting_size = %d\n", setting_size);
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
								setting_size, ds960_ar0144AT_init_setting);
		if (ret < 0) {
			vin_err("write ds960_ar0144AT_init_setting error\n",
					sensor_info->serial_addr);
			return ret;
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				vin_dbg("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
                        sensor_info->gpio_pin[gpio],
                        sensor_info->gpio_level[gpio],
                        sensor_info->gpio_level[gpio]);
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}

void sensor_common_data_init(sensor_info_t *sensor_info,
								sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
		sizeof(turning_data->sensor_name));

	return;
}

int sensor_param_init(sensor_info_t *sensor_info,
							sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int tmp, tmp1;
	tmp = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr, X_ADDR_START);
	tmp1 = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr, X_ADDR_END);
	turning_data->sensor_data.active_width = tmp1 - tmp + 1;

	tmp = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr, Y_ADDR_START);
	tmp1 = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr, Y_ADDR_END);
	turning_data->sensor_data.active_height = tmp1 - tmp + 1;

	tmp = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
                                sensor_info->sensor_addr, FRAME_LENGTH_LINES);
	turning_data->sensor_data.exposure_time_max = tmp + MARGIN + TTF + T_FFV;
	turning_data->sensor_data.exposure_time_long_max =
                                tmp + MARGIN + TTF + T_FFV;
	turning_data->sensor_data.exposure_time_min = 1;
	turning_data->sensor_data.analog_gain_max = 128*8192;
	turning_data->sensor_data.digital_gain_max = 127*8192;
	turning_data->sensor_data.lines_per_second = 49900;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.fps = sensor_info->fps;  // fps
	turning_data->sensor_data.conversion = 1;
	return ret;
}

static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
    int ret = RET_OK;
    uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
    uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

    turning_data->stream_ctrl.data_length = 2;
    if (sizeof(turning_data->stream_ctrl.stream_on) >=
                sizeof(ar0144AT_stream_on_setting)) {
        memcpy(stream_on, ar0144AT_stream_on_setting,
                sizeof(ar0144AT_stream_on_setting));
    } else {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    if (sizeof(turning_data->stream_ctrl.stream_off) >=
                sizeof(ar0144AT_stream_off_setting)) {
        memcpy(stream_off, ar0144AT_stream_off_setting,
                sizeof(ar0144AT_stream_off_setting));
    } else {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    return ret;
}

static int sensor_normal_update_notify_driver(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);

	turning_data.normal.s_line = COARSE_INTEGRATION_TIME;
	turning_data.normal.s_line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}
	vin_info("sensor_normal_update_notify_driver\n");
	vin_info("sensor_info fps %d\n\n", sensor_info->fps);

#ifdef TUNING_LUT
	if (ae_share_flag > 0 && sensor_info->port == ae_share_flag & 0xf) {
		turning_data.normal.line_p.ratio = 410;
		turning_data.normal.line_p.offset = 22;
		turning_data.normal.line_p.max = 985;
	} else {
		turning_data.normal.line_p.ratio = 1 << 8;
		turning_data.normal.line_p.offset = 0;
		turning_data.normal.line_p.max = turning_data.sensor_data.exposure_time_max;
	}

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = ANALOG_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		if (ae_share_flag > 0 && sensor_info->port == ae_share_flag & 0xf) {
			memcpy(turning_data.normal.again_lut, ar0144at_ae_share_again_lut,
				sizeof(ar0144at_ae_share_again_lut));
		} else {
			memcpy(turning_data.normal.again_lut, ar0144at_again_lut,
				sizeof(ar0144at_again_lut));
		}
		for (open_cnt =0; open_cnt <
			sizeof(ar0144at_again_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, ar0144at_dgain_lut,
			sizeof(ar0144at_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ar0144at_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut) {
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}

	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int frame_line = 0;
	int value;
	if (fps > 30 || fps < 5) {
		vin_err("not suport fps type %d\n", fps);
		return -RET_ERROR;
	} else {
		vin_info("AR0144 FPS IS %d\n", fps);
		frame_line = DEFAULT_FRAMELINE * DEFAULT_FPS / fps;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr, FRAME_LENGTH_LINES, frame_line);

		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, FRAME_LENGTH_LINES, frame_line);
		if(ret <0) {
			vin_err("[%s]%s err!\n", __func__, __LINE__);
			ret = -RET_ERROR;
		}
		usleep(20*1000);
	}
	sensor_info->fps = fps;
	sensor_normal_update_notify_driver(sensor_info);
	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

int sensor_normal_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[24] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data);
	turning_data.normal.s_line = COARSE_INTEGRATION_TIME;
	turning_data.normal.s_line_length = 2;

    ret = sensor_stream_control_set(&turning_data);
    if (ret < 0) {
        vin_err("sensor_stream_control_set fail %d\n", ret);
        return -RET_ERROR;
    }

#ifdef TUNING_LUT
	if (ae_share_flag > 0 && sensor_info->port == (ae_share_flag & 0xf)) {
		turning_data.normal.line_p.ratio = 410;
		turning_data.normal.line_p.offset = 22;
		turning_data.normal.line_p.max = 985;
	} else {
		turning_data.normal.line_p.ratio = 1 << 8;
		turning_data.normal.line_p.offset = 0;
		turning_data.normal.line_p.max = turning_data.sensor_data.exposure_time_max;
	}

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = ANALOG_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		if (ae_share_flag > 0 && sensor_info->port == ae_share_flag & 0xf) {
			memcpy(turning_data.normal.again_lut, ar0144at_ae_share_again_lut,
				sizeof(ar0144at_ae_share_again_lut));
		} else {
			memcpy(turning_data.normal.again_lut, ar0144at_again_lut,
				sizeof(ar0144at_again_lut));
		}
		for (open_cnt =0; open_cnt <
			sizeof(ar0144at_again_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, ar0144at_dgain_lut,
			sizeof(ar0144at_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ar0144at_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut) {
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}

	return ret;
}

int sensor_ae_share_init(uint32_t flag)
{
	ae_share_flag = flag;

	return 0;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int i;
	int ret = RET_OK;
	int setting_size = 0;
	int size;
	int tmp = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	ret = sensor_ar0144AT_954_init(sensor_info);
	if (ret < 0) {
		vin_err("sensor_poweron %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	usleep(200*1000);
	size = sizeof(ds933_ar0144AT_init_setting);
	setting_size = size/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1,
                            setting_size, ds933_ar0144AT_init_setting);
	if (ret < 0) {
		vin_err("ub933 ar0144AT write 0x%x error\n", sensor_info->serial_addr);
		return ret;
	}
	usleep(10*1000);
	size = sizeof(ar0144AT_raw12_normal_setting_step1);
	setting_size = size/sizeof(uint16_t)/2;
	vin_info("setting_size %d\n", setting_size);
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr,
				ar0144AT_raw12_normal_setting_step1[i*2],
 				ar0144AT_raw12_normal_setting_step1[i*2 + 1]);

                if (ret < 0) {
                        tmp++;
                        if (tmp < 10) {
                                i--;
                                usleep(10*1000);
                                continue;
                        }
                        vin_err("%d : init %s fail\n", __LINE__,
				sensor_info->sensor_name);
                        return ret;
                }
		tmp = 0;
	}
	usleep(200*1000);
	tmp = 0;
	size = sizeof(ar0144AT_raw12_normal_setting_step2);
	setting_size = size/sizeof(uint16_t)/2;
	vin_info("setting_size %d\n", setting_size);
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr,
				ar0144AT_raw12_normal_setting_step2[i*2],
				ar0144AT_raw12_normal_setting_step2[i*2 + 1]);

                if (ret < 0) {
                        tmp++;
                        if (tmp < 10) {
                                i--;
                                usleep(10*1000);
                                continue;
                        }
                        vin_err("%d : init %s fail\n", __LINE__,
				sensor_info->sensor_name);
                        return ret;
                }
                tmp = 0;
	}
	usleep(100*1000);
	tmp = 0;
	size = sizeof(ar0144AT_raw12_normal_setting_step3);
	setting_size = size/sizeof(uint16_t)/2;
	vin_info("setting_size %d\n", setting_size);
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr,
			ar0144AT_raw12_normal_setting_step3[i*2],
			ar0144AT_raw12_normal_setting_step3[i*2 + 1]);

                if (ret < 0) {
                        tmp++;
                        if (tmp < 10) {
                                i--;
                                usleep(10*1000);
                                continue;
                        }
                        vin_err("%d : init %s fail\n", __LINE__,
				sensor_info->sensor_name);
                        return ret;
		}
		tmp = 0;
	}
	usleep(100*1000);
	ret = sensor_normal_data_init(sensor_info);
	if (ret < 0) {
		vin_err("%d:init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
    int setting_size = 0, i;
    int ret = RET_OK;
    int size;
    int tmp = 0;
    /*stream on*/
    size = sizeof(ar0144AT_stream_on_setting);
    setting_size = size/sizeof(uint32_t)/2;
    vin_info("ar0144AT start setting_size %d\n", setting_size);
    for (i = 0; i < setting_size; i++) {
        ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr,
                                    ar0144AT_stream_on_setting[i*2],
                                    ar0144AT_stream_on_setting[i*2 + 1]);

        if (ret < 0) {
                tmp++;
                if (tmp < 10) {
                        i--;
                        usleep(10*1000);
                        continue;
                }
                vin_err("%d : init %s fail\n", __LINE__,
			sensor_info->sensor_name);
		return ret;
	}
	tmp = 0;
    }

    return ret;
}
int sensor_stop(sensor_info_t *sensor_info)
{
    int setting_size = 0, i;
    int ret = RET_OK;
    int size;
    int tmp = 0;
    /*stream off*/
	ae_share_flag = 0;
    size = sizeof(ar0144AT_stream_off_setting);
    setting_size = size/sizeof(uint32_t)/2;
    vin_info("ar0144AT stop setting_size %d\n", setting_size);
    for (i = 0; i < setting_size; i++) {
        ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
                                    sensor_info->sensor_addr,
                                    ar0144AT_stream_off_setting[i*2],
                                    ar0144AT_stream_off_setting[i*2 + 1]);
	if (ret < 0) {
		tmp++;
		if (tmp < 10) {
			i--;
			usleep(10*1000);
			continue;
		}
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	tmp = 0;
    }
    return ret;
}
int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	ae_share_flag = 0;
	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}
int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

sensor_module_t ar0144AT = {
	.module = "ar0144AT",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.ae_share_init = sensor_ae_share_init,
};
