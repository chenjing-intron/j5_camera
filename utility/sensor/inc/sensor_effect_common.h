/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_
#define UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef NOCIM
#define COMP_XJ3_CAM
#endif

#define CAMERA_IOC_MAGIC   'x'
#define SENSOR_TURNING_PARAM   _IOW(CAMERA_IOC_MAGIC, 0, sensor_turning_data_t)
#ifdef COMP_XJ3_CAM
#define SENSOR_TURNING_PARAM_EX   _IOW(CAMERA_IOC_MAGIC, 7, sensor_turning_data_ex_t)
#endif


#define CAMERA_SENSOR_NAME  20

enum sensor_mode_e {
        NORMAL_M = 1,
        DOL2_M = 2,
        DOL3_M = 3,
        DOL4_M = 4,
        PWL_M = 5,
        INVALID_MOD,
};

typedef enum enum_bayer_start_e
{
	BAYER_START_R    = 0,
	BAYER_START_GR   = 1,
	BAYER_START_GB   = 2,
	BAYER_START_B    = 3,
	BAYER_START_IR   = 4,
	BAYER_START_C    = 5,
	BAYER_START_CY   = 6,

	BAYER_START_BUTT
} bayer_start_e;

typedef enum enum_bayer_pattern_e
{
	BAYER_PATTERN_RGGB    = 0,
	BAYER_PATTERN_RCCC    = 1,
	BAYER_PATTERN_RIRGB   = 2,
	BAYER_PATTERN_RGIRB   = 3,
	BAYER_PATTERN_RCCB    = 4,
	BAYER_PATTERN_RYYCY   = 5,

	BAYER_PATTERN_BUTT
} bayer_pattern_e;

typedef struct sensor_data {
	uint32_t  turning_type;  //  1:imx290 2: ar0233
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
#ifndef COMP_XJ3_CAM
	uint32_t  data_width;       // Bits per pixel.
	uint32_t  bayer_start;      // RGGB pattern start (R/Gr/Gb/B).
	uint32_t  bayer_pattern;    // CFA pattern type (RGGB/RCCC/RIrGB/RGIrB).
	uint32_t  exposure_max_bit_width; // pwl mode bits
#endif
}sensor_data_t;

#ifndef COMP_XJ3_CAM
static void inline sensor_data_bayer_fill(sensor_data_t *sd,
	uint32_t data_width, uint32_t bayer_start, uint32_t bayer_pattern) {
	sd->data_width = data_width;
	sd->bayer_start = bayer_start;
	sd->bayer_pattern = bayer_pattern;
}
static void inline sensor_data_bits_fill(sensor_data_t *sd, uint32_t exposure_max_bit_width) {
	sd->exposure_max_bit_width = exposure_max_bit_width;
}
#else
static void inline sensor_data_bayer_fill(sensor_data_t *sd,
	uint32_t data_width, uint32_t bayer_start, uint32_t bayer_pattern) {
	return;
}
static void inline sensor_data_bits_fill(sensor_data_t *sd, uint32_t exposure_max_bit_width) {
	return;
}
#endif

/* line use y = ratio * x + offset;
 * input param:
 * ratio(0,1) : 0: -1, 1: 1
 * offset:
 * max:
 */

typedef struct ctrlp_s {
	int32_t ratio;
	uint32_t offset;
	uint32_t max;
	uint32_t min;
} ctrlp_t;

/*
 * distinguish between dgain and again
 * note1: a sensor could only have again or dgain
 * note2: some sensor again/dgain in the same register, could only use again
 *        eg. imx327
 * note3: some sensor the again is stepped, could noly use again.
 * note4: again/dgain could have multiple registers,
 * note5: again [0,255], actual = 2^(again/32)
 * note6: dgain [0,255], actual = 2^(dgain/32)
 * note7: dol2/dol3/dol4 used the same again/dgain
 */

typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;
typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;

typedef struct pwl_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	uint32_t l_s_mode; //0 not use, 1 long; 2 short
	uint32_t line_num;
	ctrlp_t  line_p;
	ctrlp_t  line_p_ext[4];
	uint32_t line;
	uint32_t line_ext[4];
	uint32_t line_length;
	uint32_t line_length_ext[4];
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}pwl_t;

typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;

typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
	uint32_t apply_lut_gain;
} sensor_awb_ctrl_t;

typedef struct sensor_turning_data {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
    uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
}sensor_turning_data_t;

#ifdef COMP_XJ3_CAM
typedef struct sensor_turning_data_ex {
	uint32_t  ratio_en;
	uint32_t  ratio_value;
	uint32_t  l_line;
	uint32_t  l_line_length;
	uint32_t  lexposure_time_min;
}sensor_turning_data_ex_t;
#endif

struct MODE_SW_s {
	uint8_t WDC_OUTSEL_num;
	uint8_t WDC_THR_FRM_SEL_num;
};

enum mode_e {
	SP1_HCG_P = 0x000000,
	SP1_HCG_T = 0x010000,
	SP1_LCG_P = 0x000001,
	SP1_LCG_T = 0x010001,
	SP2_P = 0x000002,
	SP2_T = 0x010002,
	HDR = 0x000100
};

struct STA_exposure_s {
	uint32_t MODE_VMAX_num;
	uint16_t MODE_HMAX_num;
	uint32_t OFFSET_CLK_SP1H_num;
	uint32_t OFFSET_CLK_SP1L_num;
	uint8_t  FMAX_num;
};

struct SP1_exposure_s {
	uint32_t SHS1_num;
};

struct SP2_exposure_s {
	uint32_t SHS1_num;
	uint32_t SHS2_num;
};

enum exposure_sw_e {
	SP1_HCG_e_mode = 0,
	SP1_LCG_e_mode,
	SP2_1_e_mode,
	SP2_2_e_mode
};

enum gain_toal_sw_e {
	Part_mode = 0,
	Total_mode = 1
};


struct GAIN_mode_s {
	enum gain_toal_sw_e GAIN_SP1H_sw;
	enum gain_toal_sw_e GAIN_SP1L_sw;
	enum gain_toal_sw_e GAIN_SP2_sw;
};

struct GAIN_SEP_mode_s {
	uint8_t serverd:5;
	uint8_t sp2_sep_mode:1;
	uint8_t sp1l_sep_mode:1;
	uint8_t sp1h_sep_mode:1;
};

union GAIN_SEP_mode_u {
	uint8_t sep_mode_g;
	struct GAIN_SEP_mode_s sep_mode_bit;
};

struct SP1_HCG_P_gain_s {
	uint16_t AGAIN_SP1H_num;
	uint16_t PGA_GAIN_SP1H_num;
};

struct SP1_HCG_T_gain_s {
	uint16_t GAIN_SP1H_num;
};


struct SP1_LCG_P_gain_s {
	uint16_t AGAIN_SP1L_num;
	uint16_t PGA_GAIN_SP1L_num;
};

struct SP1_LCG_T_gain_s {
	uint16_t GAIN_SP1L_num;
};


struct SP2_P_gain_s {
	uint16_t AGAIN_SP1L_num;
	uint16_t PGA_GAIN_SP2_num;
};

struct GAIN_LIMIT_s {
	uint16_t AG_SP1H_LIMIT_num;
	uint16_t AG_SP1L_LIMIT_num;
};

struct GAIN_PWL_s {
        uint16_t AGAIN_SP1H_num;
        uint16_t AGAIN_SP1L_num;
        uint16_t PGA_GAIN_SP1H_num;
};

struct GAIN_NORMAL_P_u {
	struct SP2_P_gain_s sp2_p_g_num;
	struct SP1_LCG_P_gain_s sp1_p_lcg_num;
	struct SP1_HCG_P_gain_s sp1_p_hcg_num;
};

struct GAIN_NORMAL_T_u {
	struct SP1_HCG_T_gain_s sp1_t_hcg_num;
	struct SP1_LCG_T_gain_s sp1_t_lcg_num;
};

struct GAIN_DATA_s {
	struct GAIN_PWL_s pwl_d;
	struct GAIN_NORMAL_P_u gain_p_d;
	struct GAIN_NORMAL_T_u gain_t_d;
};

#ifdef __cplusplus
}
#endif

#endif // UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_

