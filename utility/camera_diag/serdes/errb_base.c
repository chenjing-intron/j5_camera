/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ser_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include "../../hb_cam_utility.h"
#include "../../inc/cam_common.h"
#include "../inc/deserial_diag.h"
#include "../../hb_queue.h"


int8_t _none_clear_fault(void *des, void *data, uint8_t reg_val)
{
	int ret = 0;

	return ret;
}

int hb_wait_status(deserial_info_t *des_info , int status)
{
	MUTEX_LOCK(des_info->mutex_status);
	while ((des_info->errb_thread_status & status) == 0) {
		COND_WAIT(des_info->cond_status, des_info->mutex_status);
	}
	status &= des_info->errb_thread_status;
	MUTEX_UNLOCK(des_info->mutex_status);
	return status;
}

void hb_set_status(deserial_info_t *des_info, int status)
{
	MUTEX_LOCK(des_info->mutex_status);
	des_info->errb_thread_status = status;
	COND_BROADCAST(des_info->cond_status);
	MUTEX_UNLOCK(des_info->mutex_status);
}

void hb_set_status_done(deserial_info_t *des_info, int status)
{
	MUTEX_LOCK(des_info->mutex_status);
	des_info->errb_thread_status = status | TH_STATUS_DONE;
	COND_BROADCAST(des_info->cond_status);
	MUTEX_UNLOCK(des_info->mutex_status);
}

bool is_stop(deserial_info_t *des_info)
{
	bool ret = false;
	MUTEX_LOCK(des_info->mutex_status);
	if (des_info->errb_thread_status & TH_STATUS_STOP)
		ret = true;
	MUTEX_UNLOCK(des_info->mutex_status);
	return ret;
}

static int32_t get_port_mask_of_sen(sensor_info_t *sen)
{
	int32_t mask = 0;
	if (sen && (sen->dev_port >= 0)) {
		mask = MASK_PORT(sen->dev_port);
	}
	return mask;
}

static int32_t get_entry_num_of_sen(sensor_info_t *sen)
{
	return ((sen) ? (sen->entry_num) : (-1));
}

static int32_t get_port_mask(deserial_info_t *des_info, uint16_t module_id, uint16_t event_id)
{
	int32_t dev_port, entry_num;
	int32_t port_mask = 0;
	uint16_t host, link, link_num, link_index;
	sensor_info_t *si;

	link = 0x0f & event_id;
	host = (0xff & event_id) >> 4;
	vin_info("rx %d, link %d\n", host, link-1);
	if (link == LINK_ALL) {  // all link occur error
		for (link_index = 0; link_index < DES_LINK_NUM_MAX; link_index++) {
			entry_num = get_entry_num_of_sen(des_info->sensor_info[link_index]);
			if(entry_num >= 0 && entry_num != host) {
				return -1;
			}
			port_mask |= get_port_mask_of_sen(des_info->sensor_info[link_index]);
		}
	} else {
		entry_num = get_entry_num_of_sen(des_info->sensor_info[link-1]);
		if(entry_num >= 0 && entry_num != host) {
			return -1;
		}
		port_mask |= get_port_mask_of_sen(des_info->sensor_info[link-1]);
	}
	vin_info("port_mask %d\n", port_mask);
	return port_mask;
}

static void inform_diagservice(register_bit_info_t ptr_ftb,
				int8_t (*diag_api)(uint16_t module_id, uint16_t event_id, uint8_t ret))
{
	int8_t ret = 0;
	if (diag_api) {
		ret = diag_api(ptr_ftb.module_id, ptr_ftb.event_id, ptr_ftb.fault);
		if (ret < 0) {
			vin_err("module id = %04x, event id = %04x send failed!\n",
					ptr_ftb.module_id, ptr_ftb.event_id);
		}
	}
}

static void inform_vio_module(deserial_info_t *des_info, register_bit_info_t ptr_ftb)
{
	uint32_t user_id;
	sst_info_t serdes_info;
	for (user_id = 0; user_id < MAX_USER; user_id++) {
		if (des_info->listner_callback[user_id].cb_func) {
			serdes_info.val = ptr_ftb.fault;
			serdes_info.module = ptr_ftb.module_id;
			serdes_info.property = ptr_ftb.event_id;
			des_info->listner_callback[user_id].cb_func(serdes_info);
		}
	}
}

static void inform_external_mode_byport(deserial_info_t *des_info, register_bit_info_t ptr_ftb)
{
	int32_t port_index, port_mask;
	port_mask = get_port_mask(des_info, ptr_ftb.module_id, ptr_ftb.event_id);
	if (port_mask > 0) {
		for(port_index = 0; port_index < CAM_MAX_NUM; port_index++) {
			if ((port_mask & ((uint32_t)1u << port_index)) != 0) {
				hb_cam_send_diag_event(port_index,
				ptr_ftb.module_id,
				ptr_ftb.event_id,
				ptr_ftb.fault);
			}
		}
	}
}

int8_t _common_handle_fault(void *data, uint8_t reg_val)
{
	int8_t ret = 0;
	uint8_t cur_target = 0;
	uint8_t j_result = 0;

	register_bit_info_t *ptr_ftb = NULL;
	ptr_ftb = (register_bit_info_t *)data;

	if (ptr_ftb == NULL)
		return -1;
	for (uint8_t i = 0; i < 8; i++) {
		cur_target = (reg_val >> i) & 0x01;
		if (!ptr_ftb[i].mark)
			continue;
		j_result = fault_judging(ptr_ftb + i, cur_target);
		if (j_result == CFG_INCORRECT) {
			ret = i + 1;
			break;
		} else {
			ptr_ftb[i].confirm_ret = j_result;
		}
	}
	return ret;
}


int8_t _common_report_fault(
							void *data,
							uint8_t reg_val,
							uint8_t valid_num,
							int8_t (*diag_api)(uint16_t module_id,
							uint16_t event_id, uint8_t ret))
{
	int32_t i;
	int8_t ret = 0;
	register_general_fun_t reg_ops;
	register_bit_info_t *ptr_ftb = NULL;
	deserial_info_t *deserial = (deserial_info_t *)data;
	if (!deserial->reg_ops)
		return -1;
	reg_ops = deserial->reg_ops[valid_num];
	ptr_ftb = reg_ops.reg_bit_info;

	for (i = 0; i < 8; i++) {
		if (!ptr_ftb[i].mark)
			continue;
		if (ptr_ftb[i].confirm_ret == BET_TO_FAULT ||
			ptr_ftb[i].confirm_ret == FAULT_TO_BET) {
			inform_diagservice(ptr_ftb[i], diag_api);
			inform_vio_module(deserial, ptr_ftb[i]);
			inform_external_mode_byport(deserial, ptr_ftb[i]);
			vin_info("module id = %04x, event id = %04x, result = %d, reg_val = %02x\n",
				ptr_ftb[i].module_id, ptr_ftb[i].event_id, ptr_ftb[i].fault, reg_val);
		}
	}

	return ret;
}

int8_t _common_clear_fault(void *des, void *data, uint8_t reg_val)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t cur_target = 0;
	register_bit_info_t *ptr_ftb = NULL;
	ptr_ftb = (register_bit_info_t *)data;
	deserial_info_t *deserial = (deserial_info_t *)des;
	if (!ptr_ftb || !deserial)
		return -1;

	for (i = 0; i < 8; i++) {
		cur_target = (reg_val >> i) & 0x01;
		if (!ptr_ftb[i].mark)
			continue;
		if (cur_target) {
			if (ptr_ftb[i].fault) {
				read_reg(deserial, ptr_ftb[i].fault_clear_reg);
			}
		}
	}
	return ret;
}

int8_t _unlock_clear_fault(void *des, void *data, uint8_t reg_val)
{
	int8_t ret = 0;
	register_bit_info_t *ptr_ftb = NULL;
	ptr_ftb = (register_bit_info_t *)data;
	deserial_info_t *deserial = (deserial_info_t *)des;

	if (!deserial || !ptr_ftb) {
		return -1;
	}

	if (ptr_ftb[0].confirm_ret == FAULT_TO_BET) {
		read_reg(deserial, ptr_ftb[0].fault_clear_reg);  // dec
		read_reg(deserial, ptr_ftb[0].fault_clear_reg1);  // idle
	}

	return ret;
}

static int32_t deserial_errb_polling(deserial_info_t *des_info, uint32_t loop_times)
{
	int cur_val, i, val;
	void *data = NULL;
	register_general_fun_t *des_errb_operation_fun = NULL;
	register_bit_info_t *ptr_ftb = NULL;
	des_errb_operation_fun = des_info->reg_ops;
	int count = des_info->reg_ops_num;

	if (!des_errb_operation_fun)
		return -1;

	while (loop_times--) {
		for (i = 0; i < count; ++i) {
			cur_val = hb_vin_i2c_read_reg16_data8(des_info->bus_num, \
				des_info->deserial_addr, des_errb_operation_fun[i].reg);
			if (cur_val < 0) {
				continue;
			}
			data = (void*)des_errb_operation_fun[i].reg_bit_info;
			val = des_errb_operation_fun[i].handle_fault_fun(data, cur_val);
			if (val > 0) {
				vin_info("i2c%d 0x%02x bit%d cfg incorrect\n",
						des_info->bus_num, des_errb_operation_fun[i].reg, val - 1);
			}
			des_errb_operation_fun[i].report_fault_fun(des_info, cur_val, i, hb_cam_diagnose);
			des_errb_operation_fun[i].clear_fault_fun(des_info, data, cur_val);
		}
		usleep(ERRB_LOOP_TIME);
	}
	return 0;
}

static uint8_t gpio_message_handle(deserial_info_t *des_info)
{
	uint8_t gpio_status = 1;
	shm_msg *shm_msg_gpio = NULL;

	while (1) {
		shm_msg_gpio = vin_queue_deq(des_info->q_msg);

		if (!shm_msg_gpio) {
			break;
		}

		vin_info("%s i2c%d, addr:0x%02x get msg, gpio status is %d\n",
					des_info->deserial_name, des_info->bus_num,
						des_info->deserial_addr, shm_msg_gpio->status);
		switch (shm_msg_gpio->data_id) {
		case GPIO_EVENT: {
				if (shm_msg_gpio->status == 0) {
					deserial_errb_polling(des_info, ERRB_LOOP_CNT);
					gpio_status = 0;
				} else if (shm_msg_gpio->status == 1) {
					deserial_errb_polling(des_info, ERRB_LOOP_CNT);
					gpio_status = 1;
				} else {
					// nothing
				}
			}
			break;

		default:
			break;
		}
		shm_free(shm_msg_gpio);
	}
	return gpio_status;
}


static void *des_errb_loop_thread(void *data)
{
	int status;
	uint8_t gpio_status = 1;
	deserial_info_t *des_info = NULL;
	board_info_t *board_info = NULL;
	if (!data) {
		return NULL;
	}

	des_info = (deserial_info_t *)data;
	vin_info("%s i2c%d, addr:0x%02x start, status:0x%x\n",
				des_info->deserial_name, des_info->bus_num,
					des_info->deserial_addr, des_info->errb_thread_status);
	while (1) {
		status = hb_wait_status(des_info,
					TH_STATUS_ACTIVE |
						TH_STATUS_PAUSE |
							TH_STATUS_STOP |
								TH_STATUS_GPIO);

		if (status & TH_STATUS_STOP) {
			vin_info("got STOP, %s i2c%d addr:0x%02x\n",
				des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
			break;
		}


		if (status & TH_STATUS_GPIO) {
			vin_info("got GPIO, %s i2c%d addr:0x%02x\n",
				des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
			gpio_status = gpio_message_handle(des_info);
			if (is_stop(des_info)) {
				vin_info("gpio event end, got STOP \n");
				break;
			}
			if (gpio_status == 1) {
				hb_set_status(des_info, TH_STATUS_PAUSE);
				status = hb_wait_status(des_info, TH_STATUS_GPIO);  // wake up by gpio polling thread
				vin_info("got GPIO, PAUSE -> GPIO, %s i2c%d addr:0x%02x\n",
				des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
			} else if (gpio_status == 0) {
				hb_set_status(des_info, TH_STATUS_ACTIVE);
			}
		}

		if (status & TH_STATUS_ACTIVE) {
			deserial_errb_polling(des_info, ERRB_LOOP_CNT);
		}
	}
	hb_set_status_done(des_info, TH_STATUS_STOP);
	vin_info("%s i2c%d, addr:0x%02x exit, status:0x%x\n",
				des_info->deserial_name, des_info->bus_num,
					des_info->deserial_addr, des_info->errb_thread_status);

	return NULL;
}


int32_t errb_loop_thread_create(deserial_info_t *des_info)
{
	int32_t ret = 0;

	if (des_info->deserial_addr != 0xff) {
		ret = pthread_create(&des_info->errb_pid, NULL, des_errb_loop_thread, (void *)des_info);
		if (0 == ret) {
			hb_set_status(des_info, TH_STATUS_INIT);
		} else {
			hb_set_status(des_info, TH_STATUS_NONE);
			vin_err("%s i2c%d, addr:0x%02x loop thread create fail\n",
				des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
			return -RET_ERROR;
		}
	}
	return ret;
}

int32_t errb_loop_thread_deinit(deserial_info_t *des_info)
{
	int i = 0;
	int32_t ret = RET_OK;
	if(des_info->deserial_addr != 0xff) {
		if (des_info->errb_thread_status > TH_STATUS_NONE) {
			while (des_info->errb_thread_status != (TH_STATUS_STOP|TH_STATUS_DONE)) {
				i++;
				usleep(50000);  // 50ms
				if(i == 12) {
					pthread_cancel(des_info->errb_pid);
					vin_info("%s i2c%d, addr:0x%02x cancel thread\n",
						des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
					break;
				}
			}
			pthread_join(des_info->errb_pid, NULL);
		}
	}
	return ret;
}

