/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ser_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include "../inc/deserial_diag.h"
#include "../../hb_cam_utility.h"
#include "../../inc/cam_common.h"
#include "../../hb_queue.h"

// J5A
// 0x26, DEC, bit3~bit0
static register_bit_info_t cfg0_26_ftb[BIT_NUM] = {
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x35,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX2_LINKA,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x36,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX2_LINKB,
											    },
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x37,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX2_LINKC,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x38,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX2_LINKD,
												},
											};

// J5B
// 0x26, DEC, bit3~bit0
static register_bit_info_t cfg1_26_ftb[BIT_NUM] = {
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x35,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX3_LINKA,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x36,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX3_LINKB,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x37,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX3_LINKC,
												},
												{
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x38,
													.module_id			= SERDES_DEC_ERR_MODULE,
													.event_id			= RX3_LINKD,
												},
											};

// J5A
// 0x2c, IDLE, bit3~bit0
static register_bit_info_t cfg0_2c_ftb[BIT_NUM] = {
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x39,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX2_LINKA,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3a,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX2_LINKB,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3b,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX2_LINKC,
												},
												{
													.mark	            = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3c,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX2_LINKD,
												},
											 };

// J5B
// 0x2c, IDLE, bit3~bit0
static register_bit_info_t cfg1_2c_ftb[BIT_NUM] = {
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x39,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX3_LINKA,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3a,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX3_LINKB,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3b,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX3_LINKC,
												},
												{
													.mark               = 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x3c,
													.module_id			= SERDES_IDLE_ERR_MODULE,
													.event_id			= RX3_LINKD,
												},
											};


// J5A
// 0x45, MEM_ECC, bit7
static register_bit_info_t cfg0_45_ftb[BIT_NUM]  = {
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
													.mark				= 0,
												},
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
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x1250,
													.fault_clear_reg1	= 0x02,
													.module_id			= SERDES_MEM_ECC_ERR2_MODULE,
													.event_id			= RX2_LINK_ALL,
												},
											};
// J5B
// 0x45, MEM_ECC, bit7
static register_bit_info_t cfg1_45_ftb[BIT_NUM]  = {
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
													.mark				= 0,
												},
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
													.mark				= 1,
													.target_fail_val	= 1,
													.fault_clear_time	= 1000,
													.fault_confirm_time	= 1000,
													.fault_clear_reg	= 0x1250,
													.fault_clear_reg1	= 0x02,
													.module_id			= SERDES_MEM_ECC_ERR2_MODULE,
													.event_id			= RX3_LINK_ALL,
												},
											 };

// J5A Linka bit3
static register_bit_info_t cfg0_1a_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x35,  // linka dec
													.fault_clear_reg1	= 0x39,  // linka idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX2_LINKA,
												},
											};

// J5B Linka bit3
static register_bit_info_t cfg1_1a_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x35,  // linka dec
													.fault_clear_reg1	= 0x39,  // linka idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX3_LINKA,
												},
											};

// J5a Linkb bit3
static register_bit_info_t cfg0_0a_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x36,  // linkb dec
													.fault_clear_reg1	= 0x3a,  // linkb idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX2_LINKB,
												},
											};
// J5b Linkb bit3
static register_bit_info_t cfg1_0a_ftb[BIT_NUM] = {
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
													.mark				= 1,
													.target_fail_val	= 0,
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x36,  // linkb dec
													.fault_clear_reg1	= 0x3a,  // linkb idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX3_LINKB,
												},
											};

// J5A Linkc bit3
static register_bit_info_t cfg0_0b_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x37,  // linkc dec
													.fault_clear_reg1	= 0x3b,  // linkc idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX2_LINKC,
												},
											};

// J5B Linkc bit3
static register_bit_info_t cfg1_0b_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x37,  // linkc dec
													.fault_clear_reg1	= 0x3b,  // linkc idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX3_LINKC,
												},
											};

// J5A LinkD bit3
static register_bit_info_t cfg0_0c_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x38,  // linkd dec
													.fault_clear_reg1	= 0x3c,  // linkd idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX2_LINKD,
												},
											};

// J5B LinkD bit3
static register_bit_info_t cfg1_0c_ftb[BIT_NUM] = {
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
													.fault_clear_time	= 200,
													.fault_confirm_time	= 200,
													.fault_clear_reg	= 0x38,  // linkd dec
													.fault_clear_reg1	= 0x3c,  // linkd idle
													.module_id			= SERDES_UNLOCK_MODULE,
													.event_id			= RX3_LINKD,
												},
											};

static int8_t _26_handle_fault(void *data, uint8_t reg_val)
{
	int8_t ret = 0;
	uint8_t deserial_index = 0;
	uint8_t cur_target = 0;
	uint8_t link_index = 0;
	uint8_t j_result = 0;

	register_bit_info_t *ptr_ftb = NULL;
	ptr_ftb = (register_bit_info_t *)data;

	if (ptr_ftb == NULL)
		return -1;
	for (uint8_t i = 0; i < 8; i++) {
		cur_target = (reg_val >> i) & 0x01;

		switch (i) {
		case 0: {  // bit0 LinkA, left-front camera
				j_result = fault_judging(ptr_ftb, cur_target);
				ptr_ftb[0].confirm_ret = j_result;
			}
			break;

		case 1: {  // bit1 LinkB, left-back camera B口
				j_result = fault_judging(ptr_ftb + 1, cur_target);
				ptr_ftb[1].confirm_ret = j_result;
			}
			break;

		case 2: {  // bit2 LinkC
				j_result = fault_judging(ptr_ftb + 2, cur_target);
				ptr_ftb[2].confirm_ret = j_result;
			}
			break;

		case 3: {  // bit3 LinkD
				j_result = fault_judging(ptr_ftb + 3, cur_target);
				ptr_ftb[3].confirm_ret = j_result;
			}
			break;

		default:
			break;
		}
	}
	return ret;
}

static int8_t _26_2c_clear_fault(void *des, void *data, uint8_t reg_val)
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
				if (is_unlock(deserial, 0x1a) == true) {
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
				if (is_unlock(deserial, 0x0a) == true) {
					break;
				}
				if (cur_target) {
					if (ptr_ftb[1].fault) {  // continue two times check
						read_reg(deserial, ptr_ftb[1].fault_clear_reg);  // pipe0
					}
				}
			}
			break;
		case 2: {
				if (is_unlock(deserial, 0x0b) == true) {
					break;
				}
				if (cur_target) {
					if (ptr_ftb[2].fault) {  // continue two times check
						read_reg(deserial, ptr_ftb[2].fault_clear_reg);  // pipe0
					}
				}
			}
			break;
		case 3: {
				if (is_unlock(deserial, 0x0c) == true) {
					break;
				}
				if (cur_target) {
					if (ptr_ftb[3].fault) {  // continue two times check
						read_reg(deserial, ptr_ftb[3].fault_clear_reg);  // pipe0
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

static int8_t _45_clear_fault(void *des, void *data, uint8_t reg_val)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t cur_target = 0;
	register_bit_info_t *ptr_ftb = NULL;
	ptr_ftb = (register_bit_info_t *)data;

	if (ptr_ftb == NULL)
		return -1;

	deserial_info_t *deserial = (deserial_info_t *)des;

	if (!deserial) {
		vin_err("deserial ptr is null!!");
		return -1;
	}

	// ptr_ftb = cfg0_45_ftb;


	for (i = 0; i < 8; i++) {
		cur_target = (reg_val >> i) & 0x01;
		switch (i) {
		case 7: {  // bit7
				if (cur_target) {
					if (ptr_ftb[7].fault) {
						write_reg(deserial, 0x1250, 0x02);
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

static register_general_fun_t duo_a_96712_operation_fun[] = {
	{
		.reg				= 0x26,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg0_26_ftb
	},  // bit7~bit4 reserved, DEC_ERR(bit3~bit0)
	{
		.reg				= 0x2c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg0_2c_ftb
	},  // bit7~bit4 reserved, IDLE_ERR(bit3~bit0)
	{
		.reg				= 0x45,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _45_clear_fault,
		.reg_bit_info	 	= cfg0_45_ftb
	},  // bit5~bit4 reserved, MEM_ECC_ERR2(bit7), VID_PXL_CRC_ERR(bit3~bit0)
	{
		.reg				= 0x1a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_1a_ftb
	},  // bit3 linkA lock, 1: locked, 0: not locked
	{
		.reg				= 0x0a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0a_ftb
	},  // bit3 linkB lock, 1: locked, 0: not locked
	{
		.reg				= 0x0b,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0b_ftb
	},  // bit3 linkC lock, 1: locked, 0: not locked
	{
		.reg				= 0x0c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0c_ftb
	}   // bit3 linkD lock, 1: locked, 0: not locked
};

static register_general_fun_t duo_b_96712_operation_fun[] = {
	{
		.reg				= 0x26,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg1_26_ftb
	},  // bit7~bit4 reserved, DEC_ERR(bit3~bit0)
	{
		.reg				= 0x2c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg1_2c_ftb
	},  // bit7~bit4 reserved, IDLE_ERR(bit3~bit0)
	{
		.reg				= 0x45,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _45_clear_fault,
		.reg_bit_info	 	= cfg1_45_ftb
	},  // bit5~bit4 reserved, MEM_ECC_ERR2(bit7), VID_PXL_CRC_ERR(bit3~bit0)
	{
		.reg				= 0x1a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg1_1a_ftb
	},  // bit3 linkA lock, 1: locked, 0: not locked
	{
		.reg				= 0x0a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg1_0a_ftb
	},  // bit3 linkB lock, 1: locked, 0: not locked
	{
		.reg				= 0x0b,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg1_0b_ftb
	},  // bit3 linkC lock, 1: locked, 0: not locked
	{
		.reg				= 0x0c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg1_0c_ftb
	}   // bit3 linkD lock, 1: locked, 0: not locked
};


static register_general_fun_t solo_96712_0_operation_fun[] = {
	{
		.reg				= 0x26,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg0_26_ftb
	},  // bit7~bit4 reserved, DEC_ERR(bit3~bit0)
	{
		.reg				= 0x2c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg0_2c_ftb
	},  // bit7~bit4 reserved, IDLE_ERR(bit3~bit0)
	{
		.reg				= 0x45,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _45_clear_fault,
		.reg_bit_info	 	= cfg0_45_ftb
	},  // bit5~bit4 reserved, MEM_ECC_ERR2(bit7), VID_PXL_CRC_ERR(bit3~bit0)
	{
		.reg				= 0x1a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_1a_ftb
	},  // bit3 linkA lock, 1: locked, 0: not locked
	{
		.reg				= 0x0a,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0a_ftb
	},  // bit3 linkB lock, 1: locked, 0: not locked
	{
		.reg				= 0x0b,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0b_ftb
	},  // bit3 linkC lock, 1: locked, 0: not locked
	{
		.reg				= 0x0c,
		.handle_fault_fun	= _common_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _unlock_clear_fault,
		.reg_bit_info	 	= cfg0_0c_ftb
	}   // bit3 linkD lock, 1: locked, 0: not locked
};


static register_general_fun_t solo_96712_1_operation_fun[] = {
	{
		.reg				= 0x26,
		.handle_fault_fun	= _26_handle_fault,
		.report_fault_fun	= _common_report_fault,
		.clear_fault_fun	= _26_2c_clear_fault,
		.reg_bit_info	 	= cfg1_26_ftb
	},  // bit7~bit4 reserved, DEC_ERR(bit3~bit0)
};


void max96712_reg_register_ops(deserial_info_t *des_info)
{
	uint32_t count = 0;

	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(solo_96712_0_operation_fun) / sizeof(solo_96712_0_operation_fun[0]);
			reg_register_ops(des_info, solo_96712_0_operation_fun, count);
		} else if ((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(solo_96712_1_operation_fun) / sizeof(solo_96712_1_operation_fun[0]);
			reg_register_ops(des_info, solo_96712_1_operation_fun, count);
		}
		break;
	case DUO_A:
		if((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_a_96712_operation_fun) / sizeof(duo_a_96712_operation_fun[0]);
			reg_register_ops(des_info, duo_a_96712_operation_fun, count);
		}
		break;
	case DUO_B:
		if((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(duo_b_96712_operation_fun) / sizeof(duo_b_96712_operation_fun[0]);
			reg_register_ops(des_info, duo_b_96712_operation_fun, count);
		}
		break;
	default:
		break;
	}
}


void max96712_reg_unregister_ops(deserial_info_t *des_info) {
	reg_register_ops(des_info, NULL, 0);
}

