/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_CAMERA_DIAG_UTILITY_H_
#define UTILITY_CAMERA_DIAG_UTILITY_H_


#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#ifndef NODIAG
#include "./diag_lib.h"
#endif


typedef struct sst_info_s {
    uint16_t module;
	uint16_t property;
	uint16_t val;
} sst_info_t;

typedef void (*INFORM_HANDLER)(sst_info_t serdes_info);

typedef struct user_info_s {
    uint32_t user_id;
    INFORM_HANDLER cb_func;
} user_info_t;



extern int8_t hb_cam_diagnose(uint16_t module_id, uint16_t event_id, uint8_t ret);
extern int32_t hb_register_serdes_listen_callback(void *dev, INFORM_HANDLER inform_handler);
extern void hb_cam_send_diag_event(uint32_t port, uint16_t module_id, uint16_t event_id, uint8_t ret);
#ifdef __cplusplus
}
#endif

#endif


