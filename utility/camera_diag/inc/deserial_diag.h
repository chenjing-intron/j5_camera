/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_CAMERA_DIAG_INC_DESERIAL_DIAG_H_
#define UTILITY_CAMERA_DIAG_INC_DESERIAL_DIAG_H_

#include <sys/prctl.h>
#include <sys/time.h>
#include <fcntl.h>

#include "../inc/cam_common.h"
#include "./camera_diag_utility.h"

#define CALC_UNIT_TIME	(1000)
#define BIT_NUM			(8)
#define ERRB_LOOP_TIME	(100000)  // us
#define ERRB_LOOP_CNT	(12)

#define MATRIXMIN_ID	(0x600)
#define MATRIXMAX_ID	(0x6ff)
#define SOLO_MIN_ID		(0x641)
#define SOLO_MAX_ID		(0x64f)
#define DUO_MIN_ID		(0x650)
#define DUO_MAX_ID		(0x65f)

/* flags for 'status' */
#define TH_STATUS_NONE 0
#define TH_STATUS_INIT 1
#define TH_STATUS_ACTIVE 2
#define TH_STATUS_PAUSE 4
#define TH_STATUS_STOP 8
#define TH_STATUS_GPIO 16
#define TH_STATUS_DONE 0x80000000

#define I2C_BUS_0		(0)
#define I2C_BUS_2		(2)
#define I2C_BUS_3		(3)
#define I2C_BUS_5		(5)

#define MASK_PORT(i)	(1 << (i))
#define LINK_ALL		(0x0f)

enum JUDGMENT_RESULT {
	NONE_RESULT = 0,
	BET_TO_FAULT = 1,
	FAULT_TO_BET = 2,
	CFG_INCORRECT = 3,
};

enum thread_status {
	THREAD_NONE = 0,
	THREAD_STOP = 1,
	THREAD_RUN = 2,
	THREAD_EXIT = 3,
};

enum EVENT_ID {
	GPIO_EVENT,
};

enum BOARD_type {
	SOLO = 1,
	DUO_A = 2,
	DUO_B = 3,
	TYPE_MAX,
};

typedef struct _shm_msg
{
	uint32_t data_id;
	uint8_t status;
} shm_msg;

typedef struct register_config_s {
	uint16_t reg;
	uint8_t val;
} register_config_t;

int8_t _common_handle_fault(void *data, uint8_t reg_val);
int8_t _unlock_clear_fault(void *des, void *data, uint8_t reg_val);
int8_t _common_report_fault(
							void *data,
							uint8_t reg_val,
							uint8_t valid_num,
							int8_t (*diag_api)(uint16_t module_id,
							uint16_t event_id, uint8_t ret));
int8_t _none_clear_fault(void *des, void *data, uint8_t reg_val);
int32_t errb_loop_thread_create(deserial_info_t *des_info);
int32_t errb_loop_thread_deinit(deserial_info_t *des_info);
void reg_register_ops(deserial_info_t *des_info, register_general_fun_t * reg_ops, uint32_t num);
void max96712_reg_register_ops(deserial_info_t *des_info);
void max96718_reg_register_ops(deserial_info_t *des_info);
int32_t deserial_poll_init(board_info_t *board_info);
int32_t deserial_poll_deinit(board_info_t *board_info);
int32_t deserial_errb_init(board_info_t *board_info);
int32_t deserial_errb_deinit(board_info_t *board_info);
int hb_wait_status(deserial_info_t *des_info , int status);
void hb_set_status(deserial_info_t *des_info, int status);
void hb_set_status_done(deserial_info_t *des_info, int status);
int32_t is_board_type(int board_id);
bool is_stop(deserial_info_t *des_info);
bool is_unlock(deserial_info_t *deserial_info, uint16_t reg_addr);
shm_msg* shm_alloc();
void shm_free(void *shm);
uint8_t fault_judging(register_bit_info_t *arg, uint8_t cur_target);
int32_t read_reg(deserial_info_t *deserial_info, uint16_t reg_addr);
int32_t write_reg(deserial_info_t *deserial_info, uint16_t reg_addr, uint8_t reg_val);
int32_t deserial_reg_init(deserial_info_t *deserial_info, register_config_t reg_cfg[], int array_num);
#endif  // UTILITY_CAMERA_DIAG_INC_DESERIAL_DIAG_H_

