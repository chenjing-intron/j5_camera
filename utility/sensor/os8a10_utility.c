/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[os8a10]:" fmt

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
#include <linux/ioctl.h>

#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/os8a10_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static int sensor_common_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[24] = {0};

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);
	return ret;
}
static void  sensor_common_data_init(sensor_info_t *sensor_info,
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

static int sensor_param_init(sensor_info_t *sensor_info,
	sensor_turning_data_t *turning_data, uint32_t fps)
{
	int ret = RET_OK;
	char init_d[3];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
		OS8A10_VAMX, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
		OS8A10_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[1];

	turning_data->sensor_data.conversion = 1;
	// turning_data->sensor_data.FSC_DOL2 = 0;hb_vio_get_data
	// turning_data->sensor_data.FSC_DOL3 = 0;

	// turning_data->sensor_data.RHS1 = 0;
	// turning_data->sensor_data.RHS2 = 0;

	turning_data->sensor_data.lane = 4;
	turning_data->sensor_data.clk = OS8A10_SYSCLKMHZ;  // uncertain
	turning_data->sensor_data.lines_per_second = 57924;
	turning_data->sensor_data.exposure_time_long_max = 2210;
	if (fps == 25) {
		if (sensor_info->sensor_mode == NORMAL_M) {
			turning_data->sensor_data.exposure_time_max = 2310;
			turning_data->sensor_data.lines_per_second = 57924;
		} else {
			turning_data->sensor_data.exposure_time_max = 1105;
			turning_data->sensor_data.exposure_time_long_max = 2210;
			turning_data->sensor_data.lines_per_second = 57924;
		}
	} else if (fps == 30) {
		if (sensor_info->sensor_mode == NORMAL_M) {
			turning_data->sensor_data.exposure_time_max = 2310;
			turning_data->sensor_data.lines_per_second = 69498;
		} else {
			turning_data->sensor_data.exposure_time_max = 1105;
			turning_data->sensor_data.exposure_time_long_max = 2210;
			turning_data->sensor_data.lines_per_second = 69498;
		}
	} else if (fps == 5) {
		if (sensor_info->sensor_mode == NORMAL_M) {
			turning_data->sensor_data.exposure_time_max = 2310;
			turning_data->sensor_data.lines_per_second = 11583;
		} else {
			turning_data->sensor_data.exposure_time_max = 1105;
			turning_data->sensor_data.exposure_time_long_max = 2210;
			turning_data->sensor_data.lines_per_second = 11583;
		}
	}
	turning_data->sensor_data.fps = fps;  // fps

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num,
		sensor_info->sensor_addr, OS8A10_X_OUTPUT_SIZE, init_d, 2);
	turning_data->sensor_data.active_width = init_d[0];  // ******self
	turning_data->sensor_data.active_width = (turning_data->sensor_data.active_width << 8) | init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
		OS8A10_Y_OUTPUT_SIZE, init_d, 2);
	turning_data->sensor_data.active_height = init_d[0];  // ******self
	turning_data->sensor_data.active_height = (turning_data->sensor_data.active_height << 8) | init_d[1];

	turning_data->sensor_data.turning_type = 6;    // gain calc

	//sensor info
	turning_data->sensor_data.analog_gain_max = 126 * 8192;
	turning_data->sensor_data.digital_gain_max = 126 * 8192;
	turning_data->sensor_data.exposure_time_min = 8;

	return ret;
}

static int sensor_normal_data_init(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t open_cnt;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data, fps);

	turning_data.normal.param_hold = 0;
	turning_data.normal.param_hold_length = 0;
	turning_data.normal.s_line = OS8A10_SHS1;
	turning_data.normal.s_line_length = 2;

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 2310;
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = OS8A10_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = OS8A10_DGAIN;
	turning_data.normal.dgain_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, os8a10_again_lut,
			sizeof(os8a10_again_lut));
		for (open_cnt =0; open_cnt < sizeof(os8a10_again_lut)/
			sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut,
			os8a10_dgain_lut, sizeof(os8a10_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(os8a10_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %s\n", sensor_info->port, strerror(errno));
		return -RET_ERROR;
	}
	if (turning_data.normal.again_lut)
		free(turning_data.normal.again_lut);
	if (turning_data.normal.dgain_lut)
		free(turning_data.normal.dgain_lut);

	return ret;
}

static int sensor_dol2_data_init(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	uint32_t open_cnt;
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_param_init(sensor_info, &turning_data, fps);
	turning_data.dol2.s_line = OS8A10_SHS2;
	turning_data.dol2.s_line_length = 2;

	turning_data.dol2.m_line = OS8A10_SHS1;
	turning_data.dol2.m_line_length = 2;

#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = 1 << 8;
	turning_data.dol2.line_p[0].offset = 0;
	turning_data.dol2.line_p[0].max = 1150;
	turning_data.dol2.line_p[1].ratio = 1 << 8;
	turning_data.dol2.line_p[1].offset = 0;
	turning_data.dol2.line_p[1].max = 2310;

	turning_data.dol2.again_control_num = 2;
	turning_data.dol2.again_control[0] = OS8A10_GAIN;
	turning_data.dol2.again_control_length[0] = 2;
	turning_data.dol2.again_control[1] = 0x350c;
	turning_data.dol2.again_control_length[1] = 2;

	turning_data.dol2.dgain_control_num = 2;
	turning_data.dol2.dgain_control[0] = OS8A10_DGAIN;
	turning_data.dol2.dgain_control_length[0] = 2;
	turning_data.dol2.dgain_control[1] = 0x350e;
	turning_data.dol2.dgain_control_length[1] = 2;
	turning_data.dol2.again_lut = malloc(256*2*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*2*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut,
			os8a10_again_lut, sizeof(os8a10_again_lut));
		for (open_cnt =0; open_cnt <
			sizeof(os8a10_again_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[open_cnt], 2);
		}
		memcpy(turning_data.dol2.again_lut + 256,
			os8a10_again_lut, sizeof(os8a10_again_lut));
		for (open_cnt =0; open_cnt <
			sizeof(os8a10_again_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[256 + open_cnt], 2);
		}
	}
	turning_data.dol2.dgain_lut = malloc(256*2*sizeof(uint32_t));
	if (turning_data.dol2.dgain_lut != NULL) {
		memset(turning_data.dol2.dgain_lut, 0xff, 256*2*sizeof(uint32_t));
		memcpy(turning_data.dol2.dgain_lut,
			os8a10_dgain_lut, sizeof(os8a10_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(os8a10_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.dgain_lut[open_cnt], 2);
		}

		memcpy(turning_data.dol2.dgain_lut + 256,
			os8a10_dgain_lut, sizeof(os8a10_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(os8a10_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.dgain_lut[256+open_cnt], 2);
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}
	if (turning_data.dol2.again_lut)
		free(turning_data.dol2.again_lut);
	if (turning_data.dol2.dgain_lut)
		free(turning_data.dol2.dgain_lut);


	return ret;
}



static int sensor_update_fps_notify_driver(sensor_info_t *sensor_info,
	uint32_t fps)
{
	int ret = 0;

	sensor_info->fps = fps;

	if (sensor_info->sensor_mode == NORMAL_M) {
		ret = sensor_normal_data_init(sensor_info, fps);
		if(ret < 0) {
			vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
		ret = sensor_dol2_data_init(sensor_info, fps);
		if(ret < 0) {
			vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
		}
	}
	return ret;
}

static int sensor_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint32_t os8a10_fps[8];

	if (sensor_info->sensor_mode == NORMAL_M) {
		if (fps == OS8A10_5FPS) {
			if (sensor_info->resolution == 2160) {  //  3840*2160
				memcpy(os8a10_fps, &os8a10_5fps_normal_setting[S2160P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				memcpy(os8a10_fps, &os8a10_5fps_normal_setting[S1520P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}
		} else if (fps == OS8A10_25FPS) {
			if (sensor_info->resolution == 2160) {  //  3840*2160
				memcpy(os8a10_fps, &os8a10_25fps_normal_setting[S2160P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				memcpy(os8a10_fps, &os8a10_25fps_normal_setting[S1520P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}
		} else {
			if (sensor_info->resolution == 2160) {  //  3840*2160
				memcpy(os8a10_fps, &os8a10_30fps_normal_setting[S2160P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				memcpy(os8a10_fps, &os8a10_30fps_normal_setting[S1520P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
		if (fps == OS8A10_25FPS) {
			if (sensor_info->resolution == 2160) {  //  3840*2160
				memcpy(os8a10_fps, &os8a10_25fps_hdr_setting[S2160P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				memcpy(os8a10_fps, &os8a10_25fps_hdr_setting[S1520P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}
		} else {
			if (sensor_info->resolution == 2160) {  //  3840*2160
				memcpy(os8a10_fps, &os8a10_30fps_hdr_setting[S2160P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				memcpy(os8a10_fps, &os8a10_30fps_hdr_setting[S1520P],
					sizeof(os8a10_fps));
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}
		}
	}

	switch (fps) {
	case OS8A10_5FPS:
		vin_info("OS8A10_5FPS\n");
		setting_size = sizeof(os8a10_fps)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			2, setting_size, os8a10_fps);
		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
		usleep(20*1000);
	break;
	case OS8A10_25FPS:
		vin_info("OS8A10_25FPS\n");
		setting_size = sizeof(os8a10_fps)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			2, setting_size, os8a10_fps);
		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
		usleep(20*1000);
	break;
	case OS8A10_30FPS:
		vin_info("OS8A10_30FPS\n");
		setting_size = sizeof(os8a10_fps)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			2, setting_size, os8a10_fps);
		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
		usleep(20*1000);
	break;
	default:
		vin_err("not suport fps type %d\n", fps);
		return -1;
	break;
	}

	return ret;
}

static int sensor_init_setting(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if (sensor_info->extra_mode) {
		int not_support = 0;

		switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			if (sensor_info->resolution == 2160) {  //  3840*2160
				if (sensor_info->fps == 1) {
					setting_size = sizeof(os8a10_2160p_raw10_1fps_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, os8a10_2160p_raw10_1fps_setting);
				} else {
					not_support = 1;
				}
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
				if (sensor_info->fps == 30) {
					setting_size = sizeof(os8a10_1080p_raw12_30fps_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, os8a10_1080p_raw12_30fps_setting);
				} else if (sensor_info->fps == 5) {
					setting_size = sizeof(os8a10_1080p_raw10_5fps_dol2_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, os8a10_1080p_raw10_5fps_dol2_setting);
				} else {
					not_support = 1;
				}
			} else {
				not_support = 1;
			}
			break;
		case DOL2_M:  //  DOl2
			if (sensor_info->resolution == 1080) {  //  1920*1080
				if (sensor_info->fps == 5) {
					setting_size = sizeof(os8a10_1080p_raw10_5fps_dol2_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, os8a10_1080p_raw10_5fps_dol2_setting);
				} else if (sensor_info->fps == 15) {
					setting_size = sizeof(os8a10_1080p_raw12_15fps_dol2_setting)/sizeof(uint32_t)/2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, os8a10_1080p_raw12_15fps_dol2_setting);
				} else {
					not_support = 1;
				}
			} else {
				not_support = 1;
			}
			break;
		default:
			not_support = 1;
			break;
		}

		if (ret == 0 && sensor_info->extra_mode != 1) {
			uint16_t vts = ((uint32_t)sensor_info->extra_mode) >> 16;
			uint16_t hts = ((uint32_t)sensor_info->extra_mode) & 0xffff;
			vin_info("%s hts=%d(0x%04x) vts=%d(0x%04x)\n", sensor_info->sensor_name, hts, hts, vts, vts);
			os8a10_extra_hts_vts_setting[1] = hts >> 8;
			os8a10_extra_hts_vts_setting[3] = hts & 0xff;
			os8a10_extra_hts_vts_setting[5] = vts >> 8;
			os8a10_extra_hts_vts_setting[7] = vts & 0xff;
			setting_size = sizeof(os8a10_extra_hts_vts_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			2, setting_size, os8a10_extra_hts_vts_setting);
		}

		if (ret < 0) {
			vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
		} else if (not_support) {
			vin_err("%s sensor_mode%d extra_mode%d %dP %dfps not support\n", sensor_info->sensor_name,
				sensor_info->sensor_mode, sensor_info->extra_mode,
				sensor_info->resolution, sensor_info->fps);
			ret = -RET_ERROR;
		}
		return ret;
	}

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			vin_info("%s init normal resolution %d\n", sensor_info->sensor_name,
				sensor_info->resolution);
			if (sensor_info->resolution == 2160) {  //  3840*2160
				setting_size = sizeof(os8a10_2160p_raw10_normal_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, os8a10_2160p_raw10_normal_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				setting_size = sizeof(os8a10_1520p_raw10_normal_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, os8a10_1520p_raw10_normal_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}

			break;
		case DOL2_M:  //  DOl2
			vin_info("%s init dol2 resolution %d\n", sensor_info->sensor_name,
				sensor_info->resolution);
			if (sensor_info->resolution == 2160) {  //  3840*2160
				setting_size = sizeof(os8a10_2160p_raw10_dol2_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, os8a10_2160p_raw10_dol2_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}

			} else if (sensor_info->resolution == 1520) {  //  2688*1520
				setting_size = sizeof(os8a10_1520p_raw10_dol2_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				2, setting_size, os8a10_1520p_raw10_dol2_setting);
				if (ret < 0) {
					vin_err("vin_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->resolution == 1080) {  //  1920*1080
			}

			break;
		default:
			break;
	}

	ret = sensor_switch_fps(sensor_info, sensor_info->fps);

	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			vin_info("%s init normal\n", sensor_info->sensor_name);
			ret = sensor_common_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_common_open_failed %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_init_setting(sensor_info);
			if (ret < 0) {
				vin_err("sensor_init_setting %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_normal_data_init(sensor_info, sensor_info->fps);
			if(ret < 0) {
				vin_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				close(sensor_info->sen_devfd);
				sensor_info->sen_devfd = -1;
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			vin_info("%s init dol2\n", sensor_info->sensor_name);
			ret = sensor_common_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_common_open_failed %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_init_setting(sensor_info);
			if (ret < 0) {
				vin_err("sensor_init_setting %s fail\n", sensor_info->sensor_name);
				return ret;
			}

			ret = sensor_dol2_data_init(sensor_info, sensor_info->fps);
			if(ret < 0) {
				vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
				close(sensor_info->sen_devfd);
				sensor_info->sen_devfd = -1;
				return ret;
			}
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;

	ret = sensor_switch_fps(sensor_info, fps);
	if (ret < 0)
		return ret;

	ret = sensor_update_fps_notify_driver(sensor_info, fps);
	if (ret < 0) {
		vin_info("update %dfps param failed\n", fps);
		return ret;
	}

	vin_info("dynamic_switch to %dfps success\n", fps);
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	setting_size = sizeof(os8a10_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	2, setting_size, os8a10_stream_on_setting);
	if(ret < 0) {
		vin_err("sensor_start %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	setting_size = sizeof(os8a10_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	2, setting_size, os8a10_stream_off_setting);
	if(ret < 0) {
		vin_err("sensor_stop %s fail\n", sensor_info->sensor_name);
	}
	return ret;
}

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->sen_devfd > 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return ret;
}

sensor_module_t os8a10 = {
	.module = "os8a10",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};
