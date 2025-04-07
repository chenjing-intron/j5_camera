/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef UTILITY_CAMERA_DIAG_INC_OV_SEN_DIAG_INTERFACE_H_
#define UTILITY_CAMERA_DIAG_INC_OV_SEN_DIAG_INTERFACE_H_

#include "./cam_sensor_diag.h"

// Voltage Check Info
#define OV_AVDD_VOLTAGE     0x45A0
#define OV_DOVDD_VOLTAGE    0x45A2
#define OV_DVDD_VOLTAGE     0x45A4

#define AVDD_MAX_VOLTAGE    3.1
#define AVDD_MIN_VOLTAGE    2.7
#define DOVDD_MAX_VOLTAGE   1.98
#define DOVDD_MIN_VOLTAGE   1.62
#define DVDD_MAX_VOLTAGE    1.16
#define DVDD_MIN_VOLTAGE    1.05
#define AVDD_OFFSET         0.04
#define DOVDD_OFFSET        0.03
#define DVDD_OFFSET         0.01

// Tempture Check Info
// ov sensor temp
#define OV_AVER_TEMPER      0x4D2A
#define JUNC_TEMPER_RATIO   1000
#define JUNC_TEMPER_MAX     (125 * JUNC_TEMPER_RATIO)
#define JUNC_TEMPER_MIN     (-40 * JUNC_TEMPER_RATIO)

// Frame Counter Info
#define OV_VFIFO_FCNT3      0x4620
#define OV_VFIFO_FCNT2      0x4621
#define OV_VFIFO_FCNT1      0x4622
#define OV_VFIFO_FCNT0      0x4623

// Stream state
#define OV_STREAMING        0x0100

// Fault flag reg
#define OV_FAULT_REG        0x4f09

int32_t ov_sensor_get_status(camera_diag_sensor_info_t *ov_sensor_info, int32_t index);
int32_t ov_errb_info_handle(camera_diag_sensor_info_t *ov_sensor_info, int32_t index);

#endif  // UTILITY_CAMERA_DIAG_INC_OV_SEN_DIAG_INTERFACE_H_
