/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ser_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include "../../hb_cam_utility.h"
#include "../../inc/cam_common.h"
#include "../inc/deserial_diag.h"

// DUO J5A MAX96718  DEC bi1~bit0
static register_bit_info_t cfg0_1b_ftb[BIT_NUM] = {
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,  //  100~1000ms
													.fault_confirm_time	= 1000,  //  100~1000ms
													.fault_clear_reg	= 0x22,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX0_LINKA,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,  //  100~1000ms
													.fault_confirm_time	= 1000,  //  100~1000ms
													.fault_clear_reg    = 0x23,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX0_LINKB,
												},
											};

// DUO J5B MAX96718 DEC bi1~bit0
static register_bit_info_t cfg1_1b_ftb[BIT_NUM] = {
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,  //  100~1000ms
													.fault_confirm_time	= 1000,  //  100~1000ms
													.fault_clear_reg	= 0x22,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX1_LINKA,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,  //  100~1000ms
													.fault_confirm_time	= 1000,  //  100~1000ms
													.fault_clear_reg	= 0x23,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX1_LINKB,
												},
											};

// duo J5A max96718 Linka bit3
static register_bit_info_t cfg0_13_ftb[BIT_NUM] = {
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark               = 1,
													.target_fail_val	= 0,
													.fault_clear_time	= 200,  //  100~1000ms
													.fault_confirm_time	= 200,  //  100~1000ms
													.fault_clear_reg	= 0x22,
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX0_LINKA,
												},
											};

// duo J5A max96718 Linkb bit3
static register_bit_info_t cfg0_5009_ftb[BIT_NUM] = {
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark               = 1,
													.target_fail_val	= 0,
													.fault_clear_time	= 200,  //  100~1000ms
													.fault_confirm_time	= 200,  //  100~1000ms
													.fault_clear_reg	= 0x23,
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX0_LINKB,
												},
											};

// duo J5B max96718 Linka bit3
static register_bit_info_t cfg1_13_ftb[BIT_NUM] = {
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark               = 1,
													.target_fail_val	= 0,
													.fault_clear_time	= 200,  //  100~1000ms
													.fault_confirm_time	= 200,  //  100~1000ms
													.fault_clear_reg    = 0x22,
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX1_LINKA,
												},
											};

// duo J5B max96718 Linkb bit3
static register_bit_info_t cfg1_5009_ftb[BIT_NUM] = {
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark				= 0,
												},
												{
													.mark               = 1,
													.target_fail_val	= 0,
													.fault_clear_time	= 200,  //  100~1000ms
													.fault_confirm_time	= 200,  //  100~1000ms
													.fault_clear_reg	= 0x23,
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX1_LINKB,
												},
											};

static int8_t _1b_clear_fault(void *des, void *data, uint8_t reg_val)
{
	// DEC_ERR
	int ret = 0;
	uint8_t i = 0;
	uint8_t cur_target = 0;
	register_bit_info_t *ptr_ftb = (register_bit_info_t *)data;
	deserial_info_t *deserial = (deserial_info_t *)des;

	if (!deserial || !ptr_ftb) {
		return -1;
	}

	for (i = 0; i < 8; i++) {
		cur_target = (reg_val >> i) & 0x01;

		switch (i) {
		case 0: {
				if (is_unlock(deserial, 0x13) == true) {
					break;
				}

				if (cur_target) {
					if (ptr_ftb[0].fault) {  // continue two times check fault
						read_reg(deserial, ptr_ftb[0].fault_clear_reg);  // pipe0
					}
				}
			}
			break;
		case 1: {
				if (is_unlock(deserial, 0x5009) == true) {
					break;
				}
				if (cur_target) {
					if (ptr_ftb[1].fault) {  // continue two times check
						read_reg(deserial, ptr_ftb[1].fault_clear_reg);  // pipe0
					}
				}
			}
			break;
		default:
			break;
		}
	}
	return ret;
}

static register_general_fun_t duo_a_max96718_operation_fun[] = {
	{
		.reg				= 0x1b,  // DEC_ERR(bit0~bit1)
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _1b_clear_fault,
		.reg_bit_info	 	= cfg0_1b_ftb
	},
	{
		.reg				= 0x13,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg0_13_ftb
	},
	{
		.reg				= 0x5009,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun 	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg0_5009_ftb
	},
};

static register_general_fun_t duo_b_max96718_operation_fun[] = {
	{
		.reg				= 0x1b,  // DEC_ERR(bit0~bit1)
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _1b_clear_fault,
		.reg_bit_info	 	= cfg1_1b_ftb
	},
	{
		.reg				= 0x13,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg1_13_ftb
	},
	{
		.reg				= 0x5009,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun 	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg1_5009_ftb
	},
};


static register_general_fun_t solo_max96718_0_operation_fun[] = {
	{
		.reg				= 0x1b,  // DEC_ERR(bit0~bit1)
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _1b_clear_fault,
		.reg_bit_info	 	= cfg0_1b_ftb
	},
	{
		.reg				= 0x13,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg0_13_ftb
	},
};

static register_general_fun_t solo_max96718_1_operation_fun[] = {
	{
		.reg				= 0x1b,  // DEC_ERR(bit0~bit1)
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _1b_clear_fault,
		.reg_bit_info	 	= cfg1_1b_ftb
	},
	{
		.reg				= 0x13,  // bit3 linkA lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg1_13_ftb
	},
	{
		.reg				= 0x5009,  // bit3 linkb lock, 1: locked, 0: not locked
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun 	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info		= cfg1_5009_ftb
	},
};



void max96718_reg_register_ops(deserial_info_t *des_info)
{
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(solo_max96718_0_operation_fun) / sizeof(solo_max96718_0_operation_fun[0]);
			reg_register_ops(des_info, solo_max96718_0_operation_fun, count);
		} else if ((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(solo_max96718_1_operation_fun) / sizeof(solo_max96718_1_operation_fun[0]);
			reg_register_ops(des_info, solo_max96718_1_operation_fun, count);
		}
		break;
	case DUO_A:
		if((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_max96718_operation_fun) / sizeof(duo_a_max96718_operation_fun[0]);
			reg_register_ops(des_info, duo_a_max96718_operation_fun, count);
		}
		break;
	case DUO_B:
		if((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_b_max96718_operation_fun) / sizeof(duo_b_max96718_operation_fun[0]);
			reg_register_ops(des_info, duo_b_max96718_operation_fun, count);
		}
		break;
	default:
		break;
	}
}

void max96718_reg_unregister_ops(deserial_info_t *des_info) {
	reg_register_ops(des_info, NULL, 0);
}
