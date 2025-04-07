/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ser_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include "../inc/deserial_diag.h"
#include "../../hb_cam_gpio.h"
#include "../../hb_cam_utility.h"


static register_config_t duo_a_max96712_poll_cfg[] = {
	// {0x0005, 0xc0},  // ERRB_EN(bit6, default 1)
	// {0x0029, 0x08},  // INTR6 LCRC_ERR_OEN(bit3), FSYNC_ERR_OEN bit(0)
	// {0x0025, 0x0f},  // INTR2 DEC_ERR_FLAG_A,B,C,D
	// {0x002b, 0x0f},  // INTR8 IDLE_ERR_FLAG_A,B,C,D
	// {0x002d, 0xff},  // INTR10 RT_CNT_FLAG_A,B,C,D  MAX_RT_FLAG_A,B,C,D
	// {0x0044, 0x80},  // MEM_ECC_ERR2_INT
	// {0x0506, 0x73},  // enadble linkA CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	// {0x0516, 0x73},  // enadble linkB CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	// {0x0526, 0x73},  // enadble linkC CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	// {0x0536, 0x73},  // enaable linkD CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
};
// J5A
static register_bit_info_t cfg0_40a_ftb[8] = {
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.module_id			= SERDES_LMO_MODULE,
													.event_id			= RX2_LINKA,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.module_id			= SERDES_LMO_MODULE,
													.event_id			= RX2_LINKB,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time 	= 1000,
													.fault_confirm_time = 1000,
													.module_id			= SERDES_LMO_MODULE,
													.event_id			= RX2_LINKC,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time 	= 1000,
													.fault_confirm_time = 1000,
													.module_id			= SERDES_LMO_MODULE,
													.event_id			= RX2_LINKD,
												},
											};

static register_general_fun_t duo_a_96712_operation_fun[] = {
	{
		.reg				= 0x40a,
		.val				= 0x0f,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _none_clear_fault,
		.reg_bit_info		= cfg0_40a_ftb
	},  // bit7~bit4 reserved, DEC_ERR(bit3~bit0)
};

static int32_t poll_max96712_reg_init(deserial_info_t *des_info)
{
	int32_t ret = 0;
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		// TODO(xx): Stuff
		break;
	case DUO_A:
		if ((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_max96712_poll_cfg) / sizeof(duo_a_max96712_poll_cfg[0]);
			ret = deserial_reg_init(des_info, duo_a_max96712_poll_cfg, count);
		}
		break;
	case DUO_B:
		if ((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_max96712_poll_cfg) / sizeof(duo_a_max96712_poll_cfg[0]);
			ret = deserial_reg_init(des_info, duo_a_max96712_poll_cfg, count);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int32_t poll_max96718_reg_init(deserial_info_t *des_info)
{
	int32_t ret = 0;
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		// TODO(xx): Stuff
		break;
	case DUO_A:
		if ((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_max96712_poll_cfg) / sizeof(duo_a_max96712_poll_cfg[0]);
			ret = deserial_reg_init(des_info, duo_a_max96712_poll_cfg, count);
		}
		break;
	case DUO_B:
		if ((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_max96712_poll_cfg) / sizeof(duo_a_max96712_poll_cfg[0]);
			ret = deserial_reg_init(des_info, duo_a_max96712_poll_cfg, count);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int32_t poll_feature_reg_init(board_info_t *board_info)
{
	int32_t ret = 0;
	deserial_info_t *des_info = NULL;
	uint32_t des_num = board_info->deserial_num;

	for (int32_t i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if (!des_info || des_info->ccd_pin_num == 0)
			continue;
		vin_info("%s, i2c%d, addr:0x%02x\n",
					des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
		if (strcmp(des_info->deserial_name, "max96712") == 0) {
			ret = poll_max96712_reg_init(des_info);
		} else if (strcmp(des_info->deserial_name, "max96718") == 0) {
			ret = poll_max96718_reg_init(des_info);
		}
		if (ret < 0) {
			vin_err(" poll register init failed\n");
			break;
		}
	}
	return ret;
}


static int32_t deserial_status_polling(deserial_info_t *des_info)
{
	int cur_val, i;
	void *data = NULL;
	uint8_t val = 0;
	register_general_fun_t *des_status_poll_operation_fun = NULL;
	des_status_poll_operation_fun = des_info->reg_poll_ops;

	if (!des_status_poll_operation_fun)
		return -1;
	int count = des_info->reg_poll_ops_num;

	for (i = 0; i < count; ++i) {
		cur_val = hb_vin_i2c_read_reg16_data8(des_info->bus_num, \
			des_info->deserial_addr, des_status_poll_operation_fun[i].reg);
		if (cur_val < 0) {
			continue;
		}
		data = (void*)des_status_poll_operation_fun[i].reg_bit_info;
		val = des_status_poll_operation_fun[i].val;
		des_status_poll_operation_fun[i].handle_fault_fun(data, cur_val);
		des_status_poll_operation_fun[i].report_fault_fun(data, cur_val, val, hb_cam_diagnose);
		des_status_poll_operation_fun[i].clear_fault_fun(des_info, data, cur_val);
	}
	return 0;
}


void *status_poll_thread(void *data)
{
	uint32_t des_index;
	deserial_info_t *deserial_info = NULL;
	board_info_t *board_info = (board_info_t *)data;
	uint32_t des_num = board_info->deserial_num;
	camera_component_diagnose_t *ccd = &(board_info->ccd);

	vin_info("deserial status polling thread starting!\n");
	while (ccd->poll_thread_status == THREAD_RUN) {
		for (des_index = 0; des_index < des_num; des_index++) {
			deserial_info = &(board_info->deserial_info[des_index]);
			if (deserial_info == NULL)
				continue;
			deserial_status_polling(deserial_info);
		}
		usleep(500000);  // 500ms
	}
	ccd->poll_thread_status = THREAD_EXIT;
	vin_info("deserial status polling thread exit\n");

	return NULL;
}


void max96712_register_poll_ops(deserial_info_t *des_info)
{
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(solo_96712_0_operation_fun) / sizeof(solo_96712_0_operation_fun[0]);
			// reg_register_poll_ops(des_info, solo_96712_0_operation_fun, count);
		} else if ((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(solo_96712_1_operation_fun) / sizeof(solo_96712_1_operation_fun[0]);
			// reg_register_poll_ops(des_info, solo_96712_1_operation_fun, count);
		}
		break;
	case DUO_A:
		if((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_96712_operation_fun) / sizeof(duo_a_96712_operation_fun[0]);
			reg_register_poll_ops(des_info, duo_a_96712_operation_fun, count);
		}
		break;
	case DUO_B:
		if((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(duo_b_96712_operation_fun) / sizeof(duo_b_96712_operation_fun[0]);
			// reg_register_poll_ops(des_info, duo_b_96712_operation_fun, count);
		}
		break;
	default:
		break;
	}
}

void max96718_register_poll_ops(deserial_info_t *des_info)
{
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(solo_96712_0_operation_fun) / sizeof(solo_96712_0_operation_fun[0]);
			// reg_register_poll_ops(des_info, solo_96712_0_operation_fun, count);
		} else if ((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(solo_96712_1_operation_fun) / sizeof(solo_96712_1_operation_fun[0]);
			// reg_register_poll_ops(des_info, solo_96712_1_operation_fun, count);
		}
		break;
	case DUO_A:
		if((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(duo_a_96712_operation_fun) / sizeof(duo_a_96712_operation_fun[0]);
			// reg_register_poll_ops(des_info, duo_a_96712_operation_fun, count);
		}
		break;
	case DUO_B:
		if((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			// count = sizeof(duo_b_96712_operation_fun) / sizeof(duo_b_96712_operation_fun[0]);
			// reg_register_poll_ops(des_info, duo_b_96712_operation_fun, count);
		}
		break;
	default:
		break;
	}
}

void max96712_unregister_poll_ops(deserial_info_t *des_info) {
	reg_register_ops(des_info, NULL, 0);
}

void max96718_unregister_poll_ops(deserial_info_t *des_info) {
	reg_register_ops(des_info, NULL, 0);
}

int32_t deserial_poll_init(board_info_t *board_info)
{
	int32_t i = 0;
	int32_t ret = 0;
	int32_t poll_cnt = 0;
	if (!board_info)
		return -RET_ERROR;
	uint32_t des_num = board_info->deserial_num;
	deserial_info_t *deserial_info = NULL;
    camera_component_diagnose_t *ccd = NULL;
	ccd = &(board_info->ccd);

	vin_info("deserial poll init\n");
	ccd->poll_thread_status = THREAD_NONE;
	ret = poll_feature_reg_init(board_info);
	if (ret < 0) {
		vin_err("poll feature register init failed\n");
		return ret;
	}

	for (i = 0; i < des_num; i++) {
		deserial_info = &(board_info->deserial_info[i]);
		if (deserial_info && deserial_info->ccd_poll) {
			poll_cnt++;

			if (strcmp(deserial_info->deserial_name, "max96712") == 0) {
				max96712_register_poll_ops(deserial_info);
			} else if (strcmp(deserial_info->deserial_name, "max96718") == 0) {
				max96718_register_poll_ops(deserial_info);
			}
		}
	}

	if (poll_cnt != 0) {
		ret = pthread_create(&ccd->poll_pid, NULL, status_poll_thread, (void *)board_info);
		if (ret == 0) {
			ccd->poll_thread_status = THREAD_RUN;
		} else {
			ccd->poll_thread_status = THREAD_NONE;
			vin_err("poll thread create failed %s\n", strerror(ret));
			return ret;
		}
	}

	vin_info("deserial poll end\n");
	return ret;
}


int32_t deserial_poll_deinit(board_info_t *board_info)
{
	int i = 0;
	int ret = 0;
	uint32_t des_num = 0;
	deserial_info_t *deserial_info = NULL;
	camera_component_diagnose_t *ccd = NULL;

	if (!board_info)
		return -RET_ERROR;

	ccd = &(board_info->ccd);
	des_num = board_info->deserial_num;
	if (des_num == 0) {
		vin_info("no deserializer! not need errb deinit!! des_num:%d\n", des_num);
		return 0;
	}

	if (ccd && ccd->poll_thread_status == THREAD_RUN) {
		ccd->poll_thread_status = THREAD_STOP;  // set gpio polling thread stop
		while (ccd->poll_thread_status >= THREAD_STOP
					&& ccd->poll_thread_status <= THREAD_RUN) {
			i++;
			usleep(5000);  // 5ms
			if (i == 12) {
				pthread_cancel(ccd->poll_pid);
				vin_err("cancel status polling thread, status:%d\n",
							ccd->poll_thread_status);
				break;
			}
		}
	}

	if (ccd->poll_thread_status > THREAD_NONE)
		pthread_join(ccd->poll_pid, NULL);

	for (i = 0; i < des_num; i++) {
		deserial_info = &(board_info->deserial_info[i]);
		if (deserial_info == NULL)
			return -1;

		if (strcmp(deserial_info->deserial_name, "max96712") == 0) {
			max96712_unregister_poll_ops(deserial_info);
		} else if (strcmp(deserial_info->deserial_name, "max96718") == 0) {
			max96718_unregister_poll_ops(deserial_info);
		}
	}

	vin_info("deserial poll deinit end\n");
	return ret;
}
