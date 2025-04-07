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
#include "../inc/ov_sen_diag_interface.h"

static int32_t no_first[CAM_MAX_NUM] = {0};

static int32_t ov_get_fcnt(camera_diag_sensor_info_t *ov_sensor_info)
{
    int32_t ret = 0, fcnt = 0;
    char val[4] = {0};
    ret = hb_vin_i2c_read_block_reg16(ov_sensor_info->bus_num,
                ov_sensor_info->sensor_addr, OV_VFIFO_FCNT3, val, 4);
    if (ret < 0) {
        vin_err("senor %s read frame counter error\n",
            ov_sensor_info->sensor_name);
        return ret;
    }
    fcnt = (val[0] << 8 | val[1]) << 16 | (val[2] << 8 | val[3]);
    return fcnt;
}

static int32_t ov_get_sensor_frame_count(camera_diag_sensor_info_t *ov_sensor_info)
{
    int32_t fcnt = 0, fcnt_init = 0;
    fcnt_init = ov_get_fcnt(ov_sensor_info);
    if (fcnt_init < 0) {
        vin_err("%d : get fcnt failed\n", __LINE__);
        return -RET_ERROR;
    }
    for (int i = 0; i < FCNT_RETRY_MAX; i++) {
        usleep(500);
        fcnt = ov_get_fcnt(ov_sensor_info);
        if (fcnt < 0) {
            vin_err("%d : get fcnt failed\n", __LINE__);
            return -RET_ERROR;
        }
        if ((fcnt_init != fcnt) && ((fcnt_init + 1) != fcnt)) {
            vin_warn("port [%d] fcnt last read = %d, now read = %d, i = %d\n",
                ov_sensor_info->port, fcnt_init, fcnt, i);
            fcnt_init = fcnt;
            continue;
        } else {
            ov_sensor_info->fcnt_check.fcnt_tv.fcnt = fcnt;
            clock_gettime(CLOCK_MONOTONIC, &ov_sensor_info->fcnt_check.fcnt_tv.tv);
            vin_dbg("port [%d], fcnt = %d, tv = %ld\n", ov_sensor_info->port,
                ov_sensor_info->fcnt_check.fcnt_tv.fcnt,
                ov_sensor_info->fcnt_check.fcnt_tv.tv.tv_sec * 1000000 +
                ov_sensor_info->fcnt_check.fcnt_tv.tv.tv_nsec / 1000);
            return 0;
        }
    }
    vin_err("fcnt reg read err\n");
    return -RET_ERROR;
}
static int32_t ov_voltage_check(camera_diag_sensor_info_t *ov_sen_if)
{
    int ret = RET_OK;
    char value[6] = {0};
    bool new_state = false;
    float avdd, dvdd, dovdd;
    if (ov_sen_if == NULL)
        return -RET_ERROR;

    ret = hb_vin_i2c_read_block_reg16(ov_sen_if->bus_num,
        ov_sen_if->sensor_addr, OV_AVDD_VOLTAGE, value, 6);
    if (ret < 0) {
        vin_err("port[%d] senor %s read voltage error !\n",
                ov_sen_if->port, ov_sen_if->sensor_name);
        return ret;
    }
    avdd = (value[0] << 8 | value[1]) * 6.0 / 4096;
    dovdd = (value[2] << 8 | value[3]) * 6.0 / 4096;
    dvdd = (value[4] << 8 | value[5]) * 6.0 / 4096;
    if (avdd > AVDD_MAX_VOLTAGE || avdd < AVDD_MIN_VOLTAGE || \
        dovdd > DOVDD_MAX_VOLTAGE || dovdd < DOVDD_MIN_VOLTAGE || \
        dvdd > DVDD_MAX_VOLTAGE || dvdd < DVDD_MIN_VOLTAGE) {
        new_state = 1;
    } else if (avdd < AVDD_MAX_VOLTAGE - AVDD_OFFSET && avdd > AVDD_MIN_VOLTAGE + AVDD_OFFSET && \
        dovdd < DOVDD_MAX_VOLTAGE - DOVDD_OFFSET && dovdd > DOVDD_MIN_VOLTAGE + DOVDD_OFFSET && \
        dvdd < DVDD_MAX_VOLTAGE - DVDD_OFFSET && dvdd > DVDD_MIN_VOLTAGE + DVDD_OFFSET) {
        new_state = 0;
    }
    vin_dbg("Volt detect port[%d] %s  avdd = %f, dvdd = %f, dovdd = %f\n",
        ov_sen_if->port, ov_sen_if->sensor_name, avdd, dvdd, dovdd);
    if (new_state != ov_sen_if->sensor_status.voltage_state) {
        vin_info("Detect port_%d %s volt fault state = %d, avdd = %f, dvdd = %f, dovdd = %f\n",
            ov_sen_if->port, ov_sen_if->sensor_name,
            ov_sen_if->sensor_status.voltage_state, avdd, dvdd, dovdd);
        ov_sen_if->sensor_status.voltage_state = new_state;
        ret = hb_cam_diagnose(SENSOR_VOLTAGE_ID, ov_sen_if->port + 1,
                     ov_sen_if->sensor_status.voltage_state);
        if (ret < 0) {
            vin_err("ov sensor diag report voltage state: %d fail\n",
                     ov_sen_if->sensor_status.voltage_state);
            return ret;
        }
    }
    return 0;
}

static int32_t ov_fps_check(camera_diag_sensor_info_t *ov_sen_if, int32_t index)
{
    uint64_t time_diff = 0;
    int32_t ret = RET_OK;
    int32_t fcnt_check_state;
    float fps = 0.0;
    fcnt_tv_t fcnt_last;
    if (ov_sen_if == NULL)
        return -RET_ERROR;
    float fps_setting = ov_sen_if->fps;
    fcnt_last.tv = ov_sen_if->fcnt_check.fcnt_tv.tv;
    fcnt_last.fcnt = ov_sen_if->fcnt_check.fcnt_tv.fcnt;
    fcnt_check_state = ov_sen_if->sensor_status.fps_state;
    ret = ov_get_sensor_frame_count(ov_sen_if);
    if (ret < 0) {
        vin_err("senor %s port [%d] get fcnt error\n", ov_sen_if->sensor_name,
			ov_sen_if->port);
		return ret;
	}
    if (no_first[index] == 0) {
        no_first[index] = 1;
        vin_info("fcnt_last.tv abnormal skip first fps calc !!\n");
        return 0;
    }
    time_diff =
        (ov_sen_if->fcnt_check.fcnt_tv.tv.tv_sec - fcnt_last.tv.tv_sec) * 1000000 +
        (ov_sen_if->fcnt_check.fcnt_tv.tv.tv_nsec - fcnt_last.tv.tv_nsec) / 1000;

    if (time_diff > FPS_MONITOR_PERIOD_US) {
        fps = (ov_sen_if->fcnt_check.fcnt_tv.fcnt - fcnt_last.fcnt) *
			1000000.0 / time_diff;
        vin_dbg("port[%02d] %s  read fps: %f\n", ov_sen_if->port, ov_sen_if->sensor_name, fps);
        ov_sen_if->sensor_status.fps_state = 0;
        if (fps < (fps_setting - FCNT_ERR_RANGE) ||
			fps > (fps_setting + FCNT_ERR_RANGE)) {
            vin_dbg("port [%d] fps check, the setting fps is %f, "
                "while the read fps is %f\n", ov_sen_if->port, fps_setting, fps);
            vin_dbg("port [%d] fcnt_last:%d, fcnt:%d, time_diff:%ld\n",
                ov_sen_if->port, fcnt_last.fcnt,
                ov_sen_if->fcnt_check.fcnt_tv.fcnt, time_diff);
            if (fps < (fps_setting - (FCNT_ERR_RANGE + 2)) ||
                fps > (fps_setting + (FCNT_ERR_RANGE + 2))) {
                ov_sen_if->sensor_status.fps_state = 1;
            }
        }
        if (fcnt_check_state != ov_sen_if->sensor_status.fps_state) {
            vin_info("Detect port_%d %s fcnt fault state = %d, fps = %f\n",
                ov_sen_if->port, ov_sen_if->sensor_name, ov_sen_if->sensor_status.fps_state, fps);
            ret = hb_cam_diagnose(SENSOR_FCNT_ID, ov_sen_if->port + 1,
                        ov_sen_if->sensor_status.fps_state);
            if (ret < 0) {
                vin_err("ov sensor diag report fcnt state: %d fail\n",
                        ov_sen_if->sensor_status.fps_state);
                return ret;
            }
        }
    } else {
        ov_sen_if->fcnt_check.fcnt_tv = fcnt_last;
    }
    return 0;
}

int32_t ov_sensor_get_status(camera_diag_sensor_info_t *ov_sensor_info, int32_t index)
{
    int32_t val, ret = 0;
    bool new_state = false;
    camera_diag_sensor_info_t *ov_sen_if = &ov_sensor_info[index];

    if (ov_sen_if == NULL) {
        vin_err("%d ov_sensor_info is NULL \n", __LINE__);
        return -RET_ERROR;
    }
    // voltage check
    {
        if (ov_sen_if->diag_mask.voltage_check) {
            ret = ov_voltage_check(ov_sen_if);
            if (ret < 0)
                vin_warn("ov voltage check fail\n");
        }
    }
    // temperature check
    {
        if (ov_sen_if->diag_mask.temperature_check) {
            int32_t temper = 0;
            ret = hb_vin_i2c_read_reg16_data16(ov_sen_if->bus_num,
                ov_sen_if->sensor_addr, OV_AVER_TEMPER);
            if (ret < 0) {
                vin_err("senor %s port [%d] read temper error\n",
                    ov_sen_if->sensor_name, ov_sen_if->port);
                return ret;
            }
            temper = (ret <= 0xC000) ? ret : (-1) * (ret - 0xC000);
            temper = (temper * JUNC_TEMPER_RATIO) >> 8;
            vin_dbg("port [%d] temper = %d\n", ov_sen_if->port, temper);

            if (temper > JUNC_TEMPER_MAX || temper < JUNC_TEMPER_MIN) {
                new_state = 1;
                vin_warn("port [%d] temper = %d out of range\n",
                    ov_sen_if->port, temper);
            } else {
                new_state = 0;
            }
            vin_dbg("Temp detect port[%d] %s  temp = %d\n",
                ov_sen_if->port, ov_sen_if->sensor_name, temper);
            ret = sensor_errb_fault_report(ov_sen_if, new_state, 3, SENSOR_TEMPER_ID);
            if (ret < 0) {
                vin_err("port[%02d] %s temp state report fail\n", ov_sen_if->port, ov_sen_if->sensor_name);
            }
        }
    }
    // stream state check
    {
        bool stream_off_flag = false;
        val = hb_vin_i2c_read_reg16_data8(ov_sen_if->bus_num,
			ov_sen_if->sensor_addr, OV_STREAMING);
        if (val < 0) {
            vin_err("senor %s port [%d] read stream off state error\n",
                ov_sen_if->sensor_name, ov_sen_if->port);
            return val;
        }
        stream_off_flag = !(val & BIT(0));
        ret = sensor_errb_fault_report(ov_sen_if, stream_off_flag, 0, SENSOR_STREAM_OFF_ID);
        if (ret < 0) {
            vin_err("port[%02d] %s temp state report fail\n", ov_sen_if->port, ov_sen_if->sensor_name);
        }
        if (stream_off_flag)
            return 0;
    }
    // fps check
    {
        if (ov_sen_if->diag_mask.frame_count_check) {
            ret = ov_fps_check(ov_sen_if, index);
            if (ret < 0)
                vin_warn("ov fps check fail\n");
        }
    }
    return 0;
}

int32_t ov_errb_fault_handle(camera_diag_sensor_info_t *ov_sensor_info, int32_t index)
{
    char val[3] = {0};
    bool new_state = false;
    int32_t ret = RET_OK;
    camera_diag_sensor_info_t *ov_sen_if = &ov_sensor_info[index];

    if (ov_sen_if == NULL) {
        vin_err("%d ov_sensor_info is NULL \n", __LINE__);
        return -RET_ERROR;
    }

    ret = hb_vin_i2c_read_block_reg16(ov_sen_if->bus_num, ov_sen_if->sensor_addr, OV_FAULT_REG, val, 3);
    if (ret < 0) {
        vin_err("port_%d read fault value error\n", ov_sen_if->port);
        return ret;
    }
    if (ov_sen_if->diag_mask.row_column_id_check) {
        new_state = val[0] & BIT(0) || val[1] & BIT(7);
        ret = sensor_errb_fault_report(ov_sen_if, new_state, 4, SENSOR_ROW_COLUMN_ID);
        if (ret < 0) {
            vin_err("Line:%d port[%02d] ov fault report fail\n", __LINE__, ov_sen_if->port);
        }
    }
    if (ov_sen_if->diag_mask.pll_clock_check) {
        new_state = val[2] & BIT(4);
        ret = sensor_errb_fault_report(ov_sen_if, new_state, 5, SENSOR_PLL_CLOCK_ID);
        if (ret < 0) {
            vin_err("Line:%d port[%02d] ov fault report fail\n", __LINE__, ov_sen_if->port);
        }
    }
    if (ov_sen_if->diag_mask.ram_crc_check) {
        new_state = val[0] & BIT(1);
        ret = sensor_errb_fault_report(ov_sen_if, new_state, 8, SENSOR_RAM_CRC_ID);
        if (ret < 0) {
            vin_err("Line:%d port[%02d] ov fault report fail\n", __LINE__, ov_sen_if->port);
        }
    }
    if (ov_sen_if->diag_mask.rom_crc_check) {
        new_state = val[2] & BIT(5);
        ret = sensor_errb_fault_report(ov_sen_if, new_state, 9, SENSOR_ROM_CRC_ID);
        if (ret < 0) {
            vin_err("Line:%d port[%02d] ov fault report fail\n", __LINE__, ov_sen_if->port);
        }
    }
    if (ov_sen_if->diag_mask.online_pixel_check) {
        new_state = val[2] & BIT(1);
        ret = sensor_errb_fault_report(ov_sen_if, new_state, 10 , SENSOR_ONLINE_PIXEL_ID);
        if (ret < 0) {
            vin_err("Line:%d port[%02d] ov fault report fail\n", __LINE__, ov_sen_if->port);
        }
    }
    // vin_info("errb_test port: %d name: %s read fault value = 0x%02x, 0x%02x, 0x%02x\n",
    //         ov_sen_if->port, ov_sen_if->sensor_name, val[0], val[1], val[2]);
    return 0;
}
