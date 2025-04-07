/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdbool.h>
#include "../inc/cam_sensor_diag.h"
#include "../inc/sony_sen_diag_interface.h"

static int32_t no_first[CAM_MAX_NUM] = {0};

int32_t sony_errb_diag_init_setting(camera_diag_sensor_info_t *sensor_if, int32_t sensor_num)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;

	if (sensor_if == NULL) {
		vin_err("sensor diag errb init setting sen_if is NULL\n");
		return -RET_ERROR;
	}
	for (int32_t i = 0; i < sensor_num; i++) {
		setting_size = sizeof(fault_notification_mode_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_if[i].bus_num, sensor_if[i].sensor_addr,
				REG16_VAL8, setting_size, fault_notification_mode_setting);
		if (ret < 0) {
			vin_err("sen_errb %d %s fault_notification_mode_setting write error\n",
				sensor_if[i].port, sensor_if[i].sensor_name);
			return ret;
		}
	}
	return ret;
}

static void get_sony_real_voltage(camera_diag_sensor_info_t *sensor_if)
{
	char val[6] = {0};
	int32_t ret = 0, avdd = 0, dovdd = 0, dvdd = 0;
	ret = hb_vin_i2c_read_block_reg16(sensor_if->bus_num,
				sensor_if->sensor_addr, VDDH_AVDD, val, 6);
	if (ret < 0) {
        vin_err("port[%02d] %s read voltage error\n",
            sensor_if->port, sensor_if->sensor_name);
        return;
	}
	avdd = val[1] << 8 | val[0];
	dovdd = val[3] << 8 | val[2];
	dvdd = val[5] << 8 | val[4];
	vin_info("Detect port_%d %s real voltage avdd: %d mv, dodd: %d mv, dvdd: %d mv\n",
		sensor_if->port, sensor_if->sensor_name, avdd, dovdd, dvdd);
}

static int32_t sony_get_fcnt(camera_diag_sensor_info_t *sony_sensor_info)
{
	char val[4] = {0};
	int32_t ret = 0, fcnt = 0;
	ret = hb_vin_i2c_read_block_reg16(sony_sensor_info->bus_num,
            sony_sensor_info->sensor_addr, REG_FRAME_COUNT_LOW_16BIT, val, 4);
	if (ret < 0) {
		vin_err("port[%02d] %s read frame counter error\n",
			sony_sensor_info->port, sony_sensor_info->sensor_name);
		return ret;
	}
	fcnt = val[3] << 24 | val[2] << 16 | val[1] << 8 | val[0];
	return fcnt;
}

static int32_t sony_get_sensor_frame_count(camera_diag_sensor_info_t *sony_sensor_info)
{
	int32_t fcnt = 0, fcnt_init = 0;
	fcnt_init = sony_get_fcnt(sony_sensor_info);
	if (fcnt_init < 0) {
		vin_err("%d : get fcnt_init failed\n", __LINE__);
		return -RET_ERROR;
	}
	for (int i = 0; i < 4; i++) {
		usleep(500);
		fcnt = sony_get_fcnt(sony_sensor_info);
		if (fcnt < 0) {
			vin_err("%d : get fcnt failed\n", __LINE__);
			return -RET_ERROR;
		}
		if ((fcnt_init != fcnt) && ((fcnt_init + 1) != fcnt)) {
			vin_warn("port [%d] fcnt last read = %d, now read = %d, i = %d\n",
				sony_sensor_info->port, fcnt_init, fcnt, i);
			fcnt_init = fcnt;
			continue;
		} else {
			sony_sensor_info->fcnt_check.fcnt_tv.fcnt = fcnt;
			clock_gettime(CLOCK_MONOTONIC, &sony_sensor_info->fcnt_check.fcnt_tv.tv);
			vin_dbg("port [%d], fcnt = 0x%08x, tv = %ld\n", sony_sensor_info->port,
				sony_sensor_info->fcnt_check.fcnt_tv.fcnt,
				sony_sensor_info->fcnt_check.fcnt_tv.tv.tv_sec * 1000000 +
				sony_sensor_info->fcnt_check.fcnt_tv.tv.tv_nsec / 1000);
			return 0;
		}
	}
	vin_err("fcnt reg read err\n");
	return -RET_ERROR;
}

static int32_t sony_fps_check(camera_diag_sensor_info_t *sony_sen_if, int32_t index)
{
	uint64_t time_diff = 0;
	int32_t ret = RET_OK;
	int32_t fcnt_check_state;
	float fps = 0.0;
	fcnt_tv_t fcnt_last;
	if (sony_sen_if == NULL)
		return -RET_ERROR;
	float fps_setting = sony_sen_if->fps;
	fcnt_last.tv = sony_sen_if->fcnt_check.fcnt_tv.tv;
	fcnt_last.fcnt = sony_sen_if->fcnt_check.fcnt_tv.fcnt;
	fcnt_check_state = sony_sen_if->sensor_status.fps_state;
	ret = sony_get_sensor_frame_count(sony_sen_if);
	if (ret < 0) {
		vin_err("senor %s port [%d] get fcnt error\n", sony_sen_if->sensor_name,
			sony_sen_if->port);
		return ret;
	}
	if (no_first[index] == 0) {
		no_first[index] = 1;
		vin_info("fcnt_last.tv abnormal skip first fps calc !!\n");
		return 0;
	}
	time_diff =
		(sony_sen_if->fcnt_check.fcnt_tv.tv.tv_sec - fcnt_last.tv.tv_sec) * 1000000 +
		(sony_sen_if->fcnt_check.fcnt_tv.tv.tv_nsec - fcnt_last.tv.tv_nsec) / 1000;
	if (time_diff > FPS_MONITOR_PERIOD_US) {
		fps = (sony_sen_if->fcnt_check.fcnt_tv.fcnt - fcnt_last.fcnt) *
			1000000.0 / time_diff;
		vin_dbg("port[%02d] %s read fps: %f \n", sony_sen_if->port, sony_sen_if->sensor_name, fps);
		sony_sen_if->sensor_status.fps_state = 0;
		if (fps < (fps_setting - FCNT_ERR_RANGE) ||
			fps > (fps_setting + FCNT_ERR_RANGE)) {
			vin_dbg("port [%d] fps check, the setting fps is %f, "
				"while the read fps is %f\n", sony_sen_if->port, fps_setting, fps);
			vin_dbg("port [%d] fcnt_last:%d, fcnt:%d, time_diff:%ld\n",
				sony_sen_if->port, fcnt_last.fcnt,
				sony_sen_if->fcnt_check.fcnt_tv.fcnt, time_diff);
			if (fps < (fps_setting - (FCNT_ERR_RANGE + 2)) ||
				fps > (fps_setting + (FCNT_ERR_RANGE + 2))) {
				sony_sen_if->sensor_status.fps_state = 1;
			}
		}
		if (fcnt_check_state != sony_sen_if->sensor_status.fps_state) {
			vin_info("Detect port_%d %s fcnt fault state = %d, fps = %f\n",
				sony_sen_if->port, sony_sen_if->sensor_name, sony_sen_if->sensor_status.fps_state, fps);
			ret = hb_cam_diagnose(SENSOR_FCNT_ID, sony_sen_if->port + 1,
								sony_sen_if->sensor_status.fps_state);
			if (ret < 0) {
				vin_err("sony sensor diag report fcnt state: %d fail\n",
					sony_sen_if->sensor_status.fps_state);
				return ret;
			}
		}
	} else {
		sony_sen_if->fcnt_check.fcnt_tv = fcnt_last;
	}
	return 0;
}

int32_t sony_sensor_get_status(camera_diag_sensor_info_t *sony_sensor_info, int32_t index)
{
	char value[4] = {0};
    int32_t val, ret = 0;
	bool new_state = false;
	camera_diag_sensor_info_t *sony_sen_if = &sony_sensor_info[index];

	if (sony_sen_if == NULL) {
		vin_err("%d sony_sensor_info is NULL \n", __LINE__);
		return -RET_ERROR;
	}
	// temperature check
	{
		if (sony_sen_if->diag_mask.temperature_check) {
			float sensor_temp0, sensor_temp1, avg_temp;
			ret = hb_vin_i2c_read_block_reg16(sony_sen_if->bus_num,
				sony_sen_if->sensor_addr, TEMP_SEN0_OUT, value, 4);
			if (ret < 0) {
				vin_err("port[%02d] %s read temperature error\n",
					sony_sen_if->port, sony_sen_if->sensor_name);
				return ret;
			}
			sensor_temp0 = (value[1] << 8 | value[0]) / 16.0 - 50;
			sensor_temp1 = (value[3] << 8 | value[2]) / 16.0 - 50;
			avg_temp = (sensor_temp0 + sensor_temp1) / 2.0;
			if (avg_temp > SENSOR_TEMPER_MAX || avg_temp < SENSOR_TEMPER_MIN) {
				new_state = 1;
				vin_err("port [%d] temp0: %f, temp1: %f, avg_temp = %f out of range!\n",
					sony_sen_if->port, sensor_temp0, sensor_temp1, avg_temp);
			} else {
				new_state = 0;
			}
			vin_dbg("Temp detect port[%d] %s temp = %f \n", sony_sen_if->port,
				sony_sen_if->sensor_name, avg_temp);
			ret = sensor_errb_fault_report(sony_sen_if, new_state, 3, SENSOR_TEMPER_ID);
			if (ret < 0) {
				vin_err("port[%02d] %s temp state report fail\n", sony_sen_if->port, sony_sen_if->sensor_name);
			}
		}
	}
	// stream state check
	{
		bool stream_off_flag = false;
		val = hb_vin_i2c_read_reg16_data8(sony_sen_if->bus_num,
								sony_sen_if->sensor_addr, SONY_STREAM_STATE);
		if (val < 0) {
			vin_err("senor %s port [%d] read stream state error\n",
				sony_sen_if->sensor_name, sony_sen_if->port);
			return val;
		}
		stream_off_flag = (SONY_STREAMING_ON == val) ? false : true;
		ret = sensor_errb_fault_report(sony_sen_if, stream_off_flag, 0, SENSOR_STREAM_OFF_ID);
		if (ret < 0) {
			vin_err("port[%02d] %s temp state report fail\n", sony_sen_if->port, sony_sen_if->sensor_name);
		}
		if (stream_off_flag)
			return 0;
	}
	// fps check
    {
		if (sony_sen_if->diag_mask.frame_count_check) {
			ret = sony_fps_check(sony_sen_if, index);
			if (ret < 0)
				vin_warn("sony fps check fail\n");
		}
	}
	return 0;
}

int32_t sony_errb_fault_handle(camera_diag_sensor_info_t *sony_sensor_info, int32_t index)
{
	char val[4] = {0};
	bool new_state = false;
	int32_t ret = RET_OK;
	camera_diag_sensor_info_t *sony_sen_if = &sony_sensor_info[index];

	if (sony_sen_if == NULL) {
        vin_err("%d sony_sensor_info is NULL \n", __LINE__);
        return -RET_ERROR;
    }
	ret = hb_vin_i2c_read_block_reg16(sony_sen_if->bus_num, sony_sen_if->sensor_addr, SONY_FAULT_REG, val, 4);
	if (ret < 0) {
		vin_err("port_%d read fault value error\n", sony_sen_if->port);
		return ret;
	}
	if (sony_sen_if->diag_mask.voltage_check) {
		new_state = val[3] & 0x18;
		if (new_state != sony_sen_if->sensor_status.voltage_state)
			get_sony_real_voltage(sony_sen_if);
		ret = sensor_errb_fault_report(sony_sen_if, new_state, 1, SENSOR_VOLTAGE_ID);
		if (ret < 0) {
			vin_err("Line:%d port[%02d] sony fault report fail\n", __LINE__, sony_sen_if->port);
		}
	}
	if (sony_sen_if->diag_mask.row_column_id_check) {
		new_state = val[3] & BIT(2);
		ret = sensor_errb_fault_report(sony_sen_if, new_state, 4, SENSOR_ROW_COLUMN_ID);
		if (ret < 0) {
			vin_err("Line:%d port[%02d] sony fault report fail\n", __LINE__, sony_sen_if->port);
		}
	}
	if (sony_sen_if->diag_mask.pll_clock_check) {
		new_state = val[0] & BIT(0);
		ret = sensor_errb_fault_report(sony_sen_if, new_state, 5, SENSOR_PLL_CLOCK_ID);
		if (ret < 0) {
			vin_err("Line:%d port[%02d] sony fault report fail\n", __LINE__, sony_sen_if->port);
		}
	}
	if (sony_sen_if->diag_mask.ram_crc_check) {
		new_state = val[0] & BIT(7) || val[1] & 0x03;
		ret = sensor_errb_fault_report(sony_sen_if, new_state, 8, SENSOR_RAM_CRC_ID);
		if (ret < 0) {
			vin_err("Line:%d port[%02d] sony fault report fail\n", __LINE__, sony_sen_if->port);
		}
	}
	if (sony_sen_if->diag_mask.rom_crc_check) {
		new_state = val[2] & 0x0E;
		ret = sensor_errb_fault_report(sony_sen_if, new_state, 9, SENSOR_ROM_CRC_ID);
		if (ret < 0) {
			vin_err("Line:%d port[%02d] sony fault report fail\n", __LINE__, sony_sen_if->port);
		}
	}
	// vin_info("errb_test port: %d name: %s read fault value = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
	// 	sony_sen_if->port, sony_sen_if->sensor_name, val[0], val[1], val[2], val[3]);
	return 0;
}
