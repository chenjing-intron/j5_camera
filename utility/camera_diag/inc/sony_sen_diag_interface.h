/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef UTILITY_CAMERA_DIAG_INC_SONY_SEN_DIAG_INTERFACE_H_
#define UTILITY_CAMERA_DIAG_INC_SONY_SEN_DIAG_INTERFACE_H_

#include "./cam_sensor_diag.h"

// Voltage Check Info
#define VDDH_AVDD           0x1E40
#define VDDM_DOVDD          0x1E42
#define VDDL_DVDD           0x1E44

// Tempture Check Info
#define TEMP_SEN0_OUT       0x1F40
#define TEMP_SEN1_OUT       0x1F42
#define SENSOR_TEMPER_MAX   120
#define SENSOR_TEMPER_MIN   -35


// Frame Counter Info
#define REG_FRAME_COUNT_LOW_16BIT  0x7DC8
#define REG_FRAME_COUNT_HIGH_16BIT 0x7DCA

// Fault flag reg
#define SONY_FAULT_REG      0x1E4C

// Stream state
#define SONY_STREAM_STATE     0x6005
#define SONY_STREAMING_ON     0x05

uint32_t fault_notification_mode_setting[] = {
    0xBF3A, 0x55,  // avdd, row column, internal bus, register monitor set Mode 1
    0xBF3B, 0x11,  // Communication CRC, dovdd monitor set Mode 1
};

int32_t sony_sensor_get_status(camera_diag_sensor_info_t *sony_sensor_info, int32_t index);
int32_t sony_errb_info_handle(camera_diag_sensor_info_t *sony_sensor_info, int32_t index);
int32_t sony_errb_diag_init_setting(camera_diag_sensor_info_t *sensor_if, int32_t sensor_num);

#endif  // UTILITY_CAMERA_DIAG_INC_SONY_SEN_DIAG_INTERFACE_H_
