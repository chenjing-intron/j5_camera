/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[cam_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__
#include "../inc/camera_diag_utility.h"
#include "vin_log.h"
#include "../inc/cam_common.h"
#include "../../hb_cam_utility.h"

void hb_cam_send_diag_event(uint32_t port, uint16_t module_id, uint16_t event_id, uint8_t ret)
{
	cam_event_t event_info;
	event_info.event_type = HB_CAM_EVENT_DIAG;
	event_info.event_id = event_id;
	event_info.module_id = module_id;
	event_info.status = ret;
	event_info.port = port;
	hb_cam_send_event(port, &event_info);
}

int8_t hb_cam_diagnose(uint16_t module_id, uint16_t event_id, uint8_t ret)
{
    int8_t ret_val = 0;
	vin_info("module_id %02x, event_id %02x, ret %d\n", module_id, event_id, ret);
#ifndef NODIAG
	uint8_t value = ret;
	if (value == 0) {
        ret_val = diag_send_event_stat((uint8_t)DIAG_MSG_PRIO_MID,
							    (uint16_t)module_id,
							    (uint16_t)event_id,
							    (uint8_t)DIAG_EVENT_SUCCESS);

	} else {
        ret_val = diag_send_event_stat((uint8_t)DIAG_MSG_PRIO_MID,
							    (uint16_t)module_id,
							    (uint16_t)event_id,
							    (uint8_t)DIAG_EVENT_FAIL);
	}
#endif
	if (module_id > 0xc000 && module_id < 0xc00f) {
		hb_cam_send_diag_event(event_id - 1, module_id, event_id, value);
		vin_info("hb callback report port:%d, module_id %02x, event_id %02x, fault_state %d\n",
					event_id -1, module_id, event_id, value);
	}
	return ret_val;
}

int32_t hb_register_serdes_listen_callback(void *dev, INFORM_HANDLER inform_handler)
{
	int32_t ret = 0;
#ifdef DESERIAL_DIAG
	uint32_t i = 0;
	deserial_info_t *deserial_info = (deserial_info_t *)dev;
	MUTEX_LOCK(deserial_info->listen_mutex);
	for (i = 0; i < MAX_USER; i++) {
		if (deserial_info->listner_callback[i].user_id == NO_USER) {
			break;
		}
	}
	if (i == MAX_USER) {
		MUTEX_UNLOCK(deserial_info->listen_mutex);
		return -1;
	}
	deserial_info->listner_callback[i].cb_func = inform_handler;
	deserial_info->listner_callback[i].user_id = i + 1;
	MUTEX_UNLOCK(deserial_info->listen_mutex);
#endif
	return 0;
}


void hb_unregister_serdes_listen_callback(void *dev)
{
#ifdef DESERIAL_DIAG
	uint32_t i = 0;
	deserial_info_t *deserial_info = (deserial_info_t *)dev;
	MUTEX_LOCK(deserial_info->listen_mutex);
	for (i = 0; i < MAX_USER; i++) {
		if (deserial_info->listner_callback[i].user_id != NO_USER) {
			deserial_info->listner_callback[i].cb_func = NULL;
			deserial_info->listner_callback[i].user_id = NO_USER;
		}
	}
	MUTEX_UNLOCK(deserial_info->listen_mutex);
#endif
}
