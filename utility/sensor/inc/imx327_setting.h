/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_IMX327_SETTING_H_
#define UTILITY_SENSOR_INC_IMX327_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

#define IMX327_GAIN         	(0x3014)
#define IMX327_SHS1             (0x3020)
#define IMX327_SHS2             (0x3024)
#define IMX327_SHS3             (0x3028)
#define IMX327_VAMX	          	(0x3018)
#define IMX327_HAMX	        	(0x301C)
#define IMX327_RHS1				(0x3030)
#define IMX327_RHS2				(0x3034)
#define IMX327_FPGC				(0x3010)
#define IMX327_FPGC_1			(0x30f0)
#define IMX327_FPGC_2			(0x30f4)
#define IMX327_GAIN1			(0x30f2)
#define IMX327_GAIN2			(0x30f6)
#define IMX327_CSI_LANE_MODE    (0x3443)
#define IMX327_INCKSEL6         (0x3164)
#define IMX327_X_SIZE           (0x3472)
#define IMX327_Y_SIZE           (0x3418)
#define IMX327_PARAM_HOLD       (0x3001)

typedef enum IMX327_FPS_TYPE
{
    IMX327_10FPS = 10,
	IMX327_15FPS = 15,
	IMX327_25FPS = 25,
	IMX327_30FPS = 30
} SEN_FPS_TYPE_E;

static uint32_t imx327_stream_on_setting[] = {
	0x3002, 0x00,
	0x3000, 0x00
};

static uint32_t imx327_stream_off_setting[] = {
	0x3000, 0x01
};

static uint32_t imx327_10fps_normal_setting[] = {
	0x3018,	0x7A,	// Pair of 0x3019
	0x3019,	0x0D,	// VMAX[17:0], 0x0465 = 3450
};

static uint32_t imx327_25fps_normal_setting[] = {
	0x3018,	0x64,	// Pair of 0x3019
	0x3019,	0x05,	// VMAX[17:0], 0x0465 = 1380
};

static uint32_t imx327_15fps_normal_setting[] = {
	0x3018,	0xFC,	// Pair of 0x3019
	0x3019,	0x08,	// VMAX[17:0], 0x0465 = 1125 1150
};

static uint32_t imx327_30fps_normal_setting[] = {
	0x3018,	0x7E,	// Pair of 0x3019
	0x3019,	0x04,	// VMAX[17:0], 0x0465 = 1125 0x04b0 = 1200
};
static uint32_t imx327_25fps_dol2_setting[] = {
	0x3018,	0x64,	// Pair of 0x3019
	0x3019,	0x05,	// VMAX[17:0], 0x0465 = 1380
};
static uint32_t imx327_15fps_dol2_setting[] = {
	0x3018,	0xFC,	// Pair of 0x3019
	0x3019,	0x08,	// VMAX[17:0], 0x0465 = 2300
};
static uint32_t imx327_30fps_dol2_setting[] = {
	0x3018,	0x7E,	// Pair of 0x3019
	0x3019,	0x04,	// VMAX[17:0], 0x0465 = 1125 0x04b0 = 1150
};

static uint32_t imx327_gain_lut[] = {
	0x0,
	0x1,
	0x2,
	0x2,
	0x3,
	0x4,
	0x4,
	0x5,
	0x6,
	0x6,
	0x7,
	0x7,
	0x8,
	0x9,
	0x9,
	0xA,
	0xB,
	0xB,
	0xC,
	0xC,
	0xD,
	0xE,
	0xE,
	0xF,
	0x10,
	0x10,
	0x11,
	0x11,
	0x12,
	0x13,
	0x13,
	0x14,
	0x15,
	0x15,
	0x16,
	0x16,
	0x17,
	0x18,
	0x18,
	0x19,
	0x1A,
	0x1A,
	0x1B,
	0x1B,
	0x1C,
	0x1D,
	0x1D,
	0x1E,
	0x1F,
	0x1F,
	0x20,
	0x20,
	0x21,
	0x22,
	0x22,
	0x23,
	0x24,
	0x24,
	0x25,
	0x26,
	0x26,
	0x27,
	0x27,
	0x28,
	0x29,
	0x29,
	0x2A,
	0x2B,
	0x2B,
	0x2C,
	0x2C,
	0x2D,
	0x2E,
	0x2E,
	0x2F,
	0x30,
	0x30,
	0x31,
	0x31,
	0x32,
	0x33,
	0x33,
	0x34,
	0x35,
	0x35,
	0x36,
	0x36,
	0x37,
	0x38,
	0x38,
	0x39,
	0x3A,
	0x3A,
	0x3B,
	0x3B,
	0x3C,
	0x3D,
	0x3D,
	0x3E,
	0x3F,
	0x3F,
	0x40,
	0x40,
	0x41,
	0x42,
	0x42,
	0x43,
	0x44,
	0x44,
	0x45,
	0x45,
	0x46,
	0x47,
	0x47,
	0x48,
	0x49,
	0x49,
	0x4A,
	0x4B,
	0x4B,
	0x4C,
	0x4C,
	0x4D,
	0x4E,
	0x4E,
	0x4F,
	0x50,
	0x50,
	0x51,
	0x51,
	0x52,
	0x53,
	0x53,
	0x54,
	0x55,
	0x55,
	0x56,
	0x56,
	0x57,
	0x58,
	0x58,
	0x59,
	0x5A,
	0x5A,
	0x5B,
	0x5B,
	0x5C,
	0x5D,
	0x5D,
	0x5E,
	0x5F,
	0x5F,
	0x60,
	0x60,
	0x61,
	0x62,
	0x62,
	0x63,
	0x64,
	0x64,
	0x65,
	0x65,
	0x66,
	0x67,
	0x67,
	0x68,
	0x69,
	0x69,
	0x6A,
	0x6A,
	0x6B,
	0x6C,
	0x6C,
	0x6D,
	0x6E,
	0x6E,
	0x6F,
	0x70,
	0x70,
	0x71,
	0x71,
	0x72,
	0x73,
	0x73,
	0x74,
	0x75,
	0x75,
	0x76,
	0x76,
	0x77,
	0x78,
	0x78,
	0x79,
	0x7A,
	0x7A,
	0x7B,
	0x7B,
	0x7C,
	0x7D,
	0x7D,
	0x7E,
	0x7F,
	0x7F,
	0x80,
	0x80,
	0x81,
	0x82,
	0x82,
	0x83,
	0x84,
	0x84,
	0x85,
	0x85,
	0x86,
	0x87,
	0x87,
	0x88,
	0x89,
	0x89,
	0x8A,
	0x8A,
	0x8B,
	0x8C,
	0x8C,
	0x8D,
	0x8E,
	0x8E,
	0x8F,
	0x8F,
	0x90,
	0x91,
	0x91,
	0x92,
	0x93,
	0x93,
	0x94,
	0x95,
	0x95,
	0x96,
	0x96,
	0x97,
	0x98,
	0x98,
	0x99,
	0x9A,
	0x9A,
	0x9B,
	0x9B,
	0x9C,
	0x9D,
	0x9D,
	0x9E,
	0x9F,
	0x9F,
	0xA0,
	0xA0,
};

//sony 2lane 25fps
#if 0
static uint32_t imx327_raw12_normal_setting[] = {
      //  liner  1952*1097
	  0x3002, 0x00,   // Master Mode
	  0x3005, 0x01,   // [0:0] 0:AD-10bit	  1:AD-12bit
	  0x3007, 0x00,
	  0x3009, 0x02,
	  0x300A, 0xF0,
	  0x3011, 0x02,
	  0x3018, 0x65,	  //1125
	  0x3019, 0x04,
	  0x301C, 0xA0,   //5280 5280/2=2640
	  0x301D, 0x14,
	  0x3046, 0x01,
	  0x304B, 0x0A,
	  0x305C, 0x18,
	  0x305D, 0x03,
	  0x305E, 0x20,
	  0x305F, 0x01,
	  0x309E, 0x4A,
	  0x309F, 0x4A,
	  0x30D2, 0x19,
	  0x30D7, 0x03,
	  0x3129, 0x00,
	  0x313B, 0x61,
	  0x315E, 0x1A,
	  0x3164, 0x1A,
	  0x317C, 0x00,
	  0x31EC, 0x0E,
	  0x3405, 0x10,
	  0x3407, 0x01,
      //0x3409, 0x02,
	  0x3414, 0x0A,
	  0x3418, 0x49,
	  0x3419, 0x04,
	  0x3441, 0x0c,
	  0x3442, 0x0c,
	  0x3443, 0x01,
	  0x3444, 0x20,
	  0x3445, 0x25,
	  0x3446, 0x57,
	  0x3447, 0x00,
	  0x3448, 0x37,
	  0x3449, 0x00,
	  0x344A, 0x1F,
	  0x344B, 0x00,
	  0x344C, 0x1F,
	  0x344D, 0x00,
	  0x344E, 0x1F,
	  0x344F, 0x00,
	  0x3450, 0x77,
	  0x3451, 0x00,
	  0x3452, 0x1F,
	  0x3453, 0x00,
	  0x3454, 0x17,
	  0x3455, 0x00,
	  0x3472, 0xa0,  // 07a0 = 0x1952
	  0x3473, 0x07,
	  0x3480, 0x49
};
#endif
#if 0
//sony 4lane 25fps
static uint32_t imx327_raw12_normal_setting[] = {
      //  liner  1952*1097
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x20,
	0x3007, 0x00,
	0x3407, 0x03,


}
#endif

#if 0
//mark 2lane 25fps
static uint32_t imx327_raw12_normal_setting[] = {
   		0x3000, 0x01,	// Standby
		0x3003, 0x01,	// SW_RESET
		0x3002, 0x00,	// Master Mode
		0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
		0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
		0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
		0x300a, 0xf0,	// [7:0] BlkLevel
		0x3011, 0x02,	// Set to 0x02
		0x3018,	0x65,	// Pair of 0x3019
		0x3019,	0x04,	// VMAX[17:0], 0x0465 = 1125
		0x301c, 0xA0,	// Pair of 301d
		0x301d, 0x14,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS
		0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
		0x304b, 0x0A,	// XVS[1:0]=VSYNC Output, XHS[3:2]=HSYNC Output. ?? Why we need to set this ??
		0x305c, 0x18,	// INCKSEL1 ?
		0x305d, 0x03,	// INCKSEL2 ?
		0x305e, 0x20,	// INCKSEL3 ?
		0x305f, 0x01,	// INCKSEL4 ?
		0x309e, 0x4A,	// Set to 0x4A
		0x309f, 0x4A,	// Set to 0x4A
		0x30d2, 0x19,	// Set to 0x19
		0x30d7,	0x03,	// Set to 0x03
		0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
		0x313b, 0x61,	// Set to 0x61
		0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
		0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
		0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
		0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
		0x3405, 0x10,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
		0x3407, 0x01,	// PHY_LANE_NUM[1:0], 0:x, 1:2Lane, 3:x, 3:4Lane
		0x3414, 0x0a,	// OPB_SIZE_V[5:0]
		0x3418, 0x49,	// Pair or 0x3419
		0x3419, 0x04,	// Y_OUT_SIZE[12:0],  0x449 = 1097
		0x3441, 0x0c,	// Pair of 0x3442
		0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
		0x3443, 0x01,	// CSI_LANE_MODE[1:0], 0:x, 1:2Lane, 3:x, 3:4Lane
		0x3444, 0x20,	// Pair of 0x3445
		0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
		0x3446, 0x57,
		0x3447, 0x00,
		0x3448, 0x37,
		0x3449, 0x00,
		0x344a, 0x1f,
		0x344b, 0x00,
		0x344c, 0x1f,
		0x344d, 0x00,
		0x344e, 0x1f,
		0x344f, 0x00,
		0x3450, 0x77,
		0x3451, 0x00,
		0x3452, 0x1f,
		0x3453, 0x00,
		0x3454, 0x17,
		0x3455, 0x00,
		0x3472, 0xa0,	// Pair od 0x3473
		0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07a0 = 1952
		0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};
#endif

#if 1
//mark 4lane 30fps
static uint32_t imx327_1097p_linear_30fps_setting[] = {
      //  liner  1952*1097
		0x3000, 0x01,	// Standby
		0x3003, 0x01,	// SW_RESET
		0x3002, 0x00,	// Master Mode
		0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
		0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
		0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
		0x300a, 0xf0,	// [7:0] BlkLevel
		0x3011, 0x02,	// Set to 0x02
		0x3018,	0x7E,	// Pair of 0x3019
		0x3019,	0x04,	// VMAX[17:0], 0x0465 = 1125 0x04b0 = 1200  0x47e=1150
		0x301c, 0xD0,	// Pair of 301d  13a1 HMAX=4304  = 2linelength = 2152 > 0x36=0x858
		0x301d, 0x10,	// HMAX[15:9], <<1080p>> 0x1130:30FPS HMAX=4126  = 2linelength = 2063
		0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
		0x304b, 0x0A,	// XVS[1:0]=VSYNC Output, XHS[3:2]=HSYNC Output. ?? Why we need to set this
		0x305c, 0x18,	// INCKSEL1 ?
		0x305d, 0x03,	// INCKSEL2 ?
		0x305e, 0x20,	// INCKSEL3 ?
		0x305f, 0x01,	// INCKSEL4 ?
		0x309e, 0x4A,	// Set to 0x4A
		0x309f, 0x4A,	// Set to 0x4A
		0x30d2, 0x19,	// Set to 0x19
		0x30d7,	0x03,	// Set to 0x03
		0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
		0x313b, 0x61,	// Set to 0x61
		0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
		0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
		0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
		0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
		0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
		0x3407, 0x03,	// PHY_LANE_NUM[1:0]
		0x3414, 0x0a,	// OPB_SIZE_V[5:0]
		0x3418, 0x49,	// Pair or 0x3419
		0x3419, 0x04,	// Y_OUT_SIZE[12:0],  0x449 = 1097
		0x3441, 0x0c,	// Pair of 0x3442
		0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
		0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
		0x3444, 0x20,	// Pair of 0x3445
		0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
		0x3446, 0x47,
		0x3447, 0x00,
		0x3448, 0x1f,
		0x3449, 0x00,
		0x344a, 0x17,
		0x344b, 0x00,
		0x344c, 0x0f,
		0x344d, 0x00,
		0x344e, 0x17,
		0x344f, 0x00,
		0x3450, 0x47,
		0x3451, 0x00,
		0x3452, 0x0f,
		0x3453, 0x00,
		0x3454, 0x0f,
		0x3455, 0x00,
		0x3472, 0xa0,	// Pair od 0x3473
		0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07a0 = 1952
		0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};
#endif

#if 1
//mark 4lane 25fps
static uint32_t imx327_1097p_linear_25fps_setting[] = {
      //  liner  1952*1097
		0x3000, 0x01,	// Standby
		0x3003, 0x01,	// SW_RESET
		0x3002, 0x00,	// Master Mode
		0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
		0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
		0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
		0x300a, 0xf0,	// [7:0] BlkLevel
		0x3011, 0x02,	// Set to 0x02
		0x3018,	0x64,	// Pair of 0x3019
		0x3019, 0x05,	// VMAX[17:0], 0x0465 = 1125 1200 0x47e=1150
		0x301c, 0xD0,	// Pair of 301d  13a1 HMAX=5164 linelength = 2582
		0x301d, 0x10,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS  HMAX=4950 linelength = 2475
		0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
		0x304b, 0x0A,	// XVS[1:0]=VSYNC Output, XHS[3:2]=HSYNC Output. ?? Why we need to set this
		0x305c, 0x18,	// INCKSEL1 ?
		0x305d, 0x03,	// INCKSEL2 ?
		0x305e, 0x20,	// INCKSEL3 ?
		0x305f, 0x01,	// INCKSEL4 ?
		0x309e, 0x4A,	// Set to 0x4A
		0x309f, 0x4A,	// Set to 0x4A
		0x30d2, 0x19,	// Set to 0x19
		0x30d7,	0x03,	// Set to 0x03
		0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
		0x313b, 0x61,	// Set to 0x61
		0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
		0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
		0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
		0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
		0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
		0x3407, 0x03,	// PHY_LANE_NUM[1:0]
		0x3414, 0x0a,	// OPB_SIZE_V[5:0]
		0x3418, 0x49,	// Pair or 0x3419
		0x3419, 0x04,	// Y_OUT_SIZE[12:0],  0x449 = 1097
		0x3441, 0x0c,	// Pair of 0x3442
		0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
		0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
		0x3444, 0x20,	// Pair of 0x3445
		0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
		0x3446, 0x47,
		0x3447, 0x00,
		0x3448, 0x1f,
		0x3449, 0x00,
		0x344a, 0x17,
		0x344b, 0x00,
		0x344c, 0x0f,
		0x344d, 0x00,
		0x344e, 0x17,
		0x344f, 0x00,
		0x3450, 0x47,
		0x3451, 0x00,
		0x3452, 0x0f,
		0x3453, 0x00,
		0x3454, 0x0f,
		0x3455, 0x00,
		0x3472, 0xa0,	// Pair od 0x3473
		0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07a0 = 1952
		0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};
#endif

#if 0
//mark 4lane 10fps
static uint32_t imx327_raw12_normal_setting[] = {
      //  liner  1952*1097
		0x3000, 0x01,	// Standby
		0x3003, 0x01,	// SW_RESET
		0x3002, 0x00,	// Master Mode
		0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
		0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
		0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
		0x300a, 0xf0,	// [7:0] BlkLevel
		0x3011, 0x02,	// Set to 0x02
		0x3018,	0x98,	// Pair of 0x3019
		0x3019,	0x06,	// VMAX[17:0], 0x0465 = 1125
		0x301c, 0xF0,	// Pair of 301d
		0x301d, 0x1E,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS
		0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
		0x304b, 0x0A,	// XVS[1:0]=VSYNC Output, XHS[3:2]=HSYNC Output. ?? Why we need to set this
		0x305c, 0x18,	// INCKSEL1 ?
		0x305d, 0x03,	// INCKSEL2 ?
		0x305e, 0x20,	// INCKSEL3 ?
		0x305f, 0x01,	// INCKSEL4 ?
		0x309e, 0x4A,	// Set to 0x4A
		0x309f, 0x4A,	// Set to 0x4A
		0x30d2, 0x19,	// Set to 0x19
		0x30d7,	0x03,	// Set to 0x03
		0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
		0x313b, 0x61,	// Set to 0x61
		0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
		0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
		0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
		0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
		0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
		0x3407, 0x03,	// PHY_LANE_NUM[1:0]
		0x3414, 0x0a,	// OPB_SIZE_V[5:0]
		0x3418, 0x49,	// Pair or 0x3419
		0x3419, 0x04,	// Y_OUT_SIZE[12:0],  0x449 = 1097
		0x3441, 0x0c,	// Pair of 0x3442
		0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
		0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
		0x3444, 0x20,	// Pair of 0x3445
		0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
		0x3446, 0x47,
		0x3447, 0x00,
		0x3448, 0x1f,
		0x3449, 0x00,
		0x344a, 0x17,
		0x344b, 0x00,
		0x344c, 0x0f,
		0x344d, 0x00,
		0x344e, 0x17,
		0x344f, 0x00,
		0x3450, 0x47,
		0x3451, 0x00,
		0x3452, 0x0f,
		0x3453, 0x00,
		0x3454, 0x0f,
		0x3455, 0x00,
		0x3472, 0xa0,	// Pair od 0x3473
		0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07a0 = 1952
		0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};
#endif


#if 0 // HAPS 4lane 25fps
static uint32_t imx327_raw12_normal_setting[] = {
      //  liner  1952*1097
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x3011, 0x02,
	0x3018, 0xB0,
	0x3019, 0x04,
	0x301c, 0xA0,
	0x301d, 0x14,
	0x3046, 0x01,
	0x304b, 0x0a,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x309e, 0x4a,
	0x309f, 0x4a,
	0x30d2, 0x19,
	0x30d7, 0x03,
	0x3129, 0x00,
	0x313b, 0x61,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x31ec, 0x0e,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00, //
	0x3418, 0x49,
	0x3419, 0x04,
	0x3441, 0x0c,
	0x3442, 0x0c,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0xa0,
	0x3473, 0x07,
	0x3480, 0x49,
	//0x3002, 0x00,
	//0x3000, 0x00
};
#endif
#if 0
static uint32_t imx327_raw12_normal_setting[] = {
      //  liner  1952*1097
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x3011, 0x02,
	0x3018, 0xf9,
	0x3019, 0x15,  // VMAX = 5625
	0x301c, 0x90,
	0x301d, 0x33,  // HMAX = 13200
	0x3046, 0x01,
	0x304b, 0x0a,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x309e, 0x4a,
	0x309f, 0x4a,
	0x30d2, 0x19,
	0x30d7, 0x03,
	0x3129, 0x00,
	0x313b, 0x61,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x31ec, 0x0e,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00,  // 0x0a
	0x3418, 0x49,
	0x3419, 0x04,
	0x3441, 0x0c,
	0x3442, 0x0c,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0xa0,
	0x3473, 0x07,
	0x3480, 0x49
};
#endif
/*sony dol3*/
static uint32_t imx327_3609p_dol3_15fps_setting[] = {
		0x3000, 0x01,
   		0x3002, 0x00,
		0x3005, 0x01,
		0x3405, 0x10,
		0x3106, 0x33,
		0x3007, 0x00,
		0x3407, 0x03,
		0x3009, 0x01,
		0x300A, 0xF0,
		0x300C, 0x21,
		0x3011, 0x02,
		0x3414, 0x00,   // OB
		0x3415, 0x00,
		0x3018, 0x7E,
		0x3019, 0x04,  // VMAX 1150 framelength = 4vmax = 4600
		0x3418, 0x19,
		0x3419, 0x0E,
		0x301C, 0x68,
		0x301D, 0x08,  // HMAX 2152 = linelenght
		0x3020, 0x05,
		0x3021, 0x00,
		0x3024, 0x0B,
		0x3025, 0x01,
		0x3028, 0x93,
		0x3029, 0x01,
		0x3129, 0x00,
		0x3030, 0x06,
		0x3031, 0x01,
		0x3034, 0x1C,
		0x3035, 0x01,
		0x313B, 0x61,
		0x3441, 0x0C,
		0x3442, 0x0C,
		0x3443, 0x03,
		0x3444, 0x20,
		0x3045, 0x05,
		0x3445, 0x25,
		0x3046, 0x01,
		0x3446, 0x57,
		0x3447, 0x00,
		0x3448, 0x37,
		0x3449, 0x00,
		0x344A, 0x1F,
		0x304B, 0x0A,
		0x344B, 0x00,
		0x344C, 0x1F,
		0x344D, 0x00,
		0x344E, 0x1F,
		0x344F, 0x00,
		0x3450, 0x77,
		0x3451, 0x00,
		0x3452, 0x1F,
		0x3453, 0x00,
		0x3454, 0x17,
		0x3455, 0x00,
		0x305C, 0x18,
		0x305D, 0x03,
		0x305E, 0x20,
		0x315E, 0x1A,
		0x305F, 0x01,
		0x3164, 0x1A,
		0x3472, 0xA0,
		0x3473, 0x07,
		0x347B, 0x23,
		0x317C, 0x00,
		0x3480, 0x49,
		0x309E, 0x4A,
		0x309F, 0x4A,
		0x30D2, 0x19,
		0x30D7, 0x03,
		0x31EC, 0x0E,
		0x3010, 0x61
};

//sony 4lane Dol2 1952x2228
static uint32_t imx327_2228p_dol2_25fps_setting[] = {
		0x3000, 0x01,
   		0x3002, 0x00,
		0x3005, 0x01,
		0x3405, 0x10,
		0x3106, 0x11,
		0x3007, 0x00,
		0x3407, 0x03,
		0x3009, 0x01,
		0x300A, 0xF0,
		0x300C, 0x11,
		0x3011, 0x02,
		0x3414, 0x00, //
		0x3415, 0x00,
		0x3018, 0x64, // VMAX 1380
		0x3019, 0x05,
		0x3418, 0xB4,
		0x3419, 0x08,
		0x301C, 0x68,
		0x301D, 0x08, // HMAX 2152 = linelength
		0x3020, 0x02,  //
		0x3021, 0x00,
		0x3024, 0x49,
		0x3025, 0x08,
		0x3129, 0x00,
		0x3030, 0x0B,
		0x3031, 0x00,
		0x313B, 0x61,
		0x3441, 0x0C,
		0x3442, 0x0C,
		0x3443, 0x03,
		0x3444, 0x20,
		0x3045, 0x05,
		0x3445, 0x25,
		0x3046, 0x01,
		0x3446, 0x57,
		0x3447, 0x00,
		0x3448, 0x37,
		0x3449, 0x00,
		0x344A, 0x1F,
		0x304B, 0x0A,
		0x344B, 0x00,
		0x344C, 0x1F,
		0x344D, 0x00,
		0x344E, 0x1F,
		0x344F, 0x00,
		0x3450, 0x77,
		0x3451, 0x00,
		0x3452, 0x1F,
		0x3453, 0x00,
		0x3454, 0x17,
		0x3455, 0x00,
		0x305C, 0x18,
		0x305D, 0x03,
		0x305E, 0x20,
		0x315E, 0x1A,
		0x305F, 0x01,
		0x3164, 0x1A,
		0x3472, 0xA0,  //1952
		0x3473, 0x07,
		0x347B, 0x23,
		0x317C, 0x00,
		0x3480, 0x49,
		0x309E, 0x4A,
		0x309F, 0x4A,
		0x30D2, 0x19,
		0x30D7, 0x03,
		0x31EC, 0x0E,
		0x3010, 0x61,
};

//sony 4lane Dol2 1952x2228
static uint32_t imx327_2228p_dol2_30fps_setting[] = {
		0x3000, 0x01,
		0x3002, 0x00,
		0x3005, 0x01,
		0x3405, 0x10,
		0x3106, 0x11,
		0x3007, 0x00,
		0x3407, 0x03,
		0x3009, 0x01,
		0x300A, 0xF0,
		0x300C, 0x11,
		0x3011, 0x02,
		0x3414, 0x00,
		0x3415, 0x00,
		0x3018, 0x7E, // VMAX 1150
		0x3019, 0x04,
		0x3418, 0xB4,
		0x3419, 0x08,
		0x301C, 0x68,
		0x301D, 0x08, // HMAX 2152 = linelength must > 2136 手册dol appnote page35
		0x3020, 0x03,
		0x3021, 0x00,
		0x3024, 0x49,
		0x3025, 0x08,
		0x3129, 0x00,
		0x3030, 0x0B,
		0x3031, 0x00,
		0x313B, 0x61,
		0x3441, 0x0C,
		0x3442, 0x0C,
		0x3443, 0x03,
		0x3444, 0x20,
		0x3045, 0x05,
		0x3445, 0x25,
		0x3046, 0x01,
		0x3446, 0x57,
		0x3447, 0x00,
		0x3448, 0x37,
		0x3449, 0x00,
		0x344A, 0x1F,
		0x304B, 0x0A,
		0x344B, 0x00,
		0x344C, 0x1F,
		0x344D, 0x00,
		0x344E, 0x1F,
		0x344F, 0x00,
		0x3450, 0x77,
		0x3451, 0x00,
		0x3452, 0x1F,
		0x3453, 0x00,
		0x3454, 0x17,
		0x3455, 0x00,
		0x305C, 0x18,
		0x305D, 0x03,
		0x305E, 0x20,
		0x315E, 0x1A,
		0x305F, 0x01,
		0x3164, 0x1A,
		0x3472, 0xA0,  //1952
		0x3473, 0x07,
		0x347B, 0x23,
		0x317C, 0x00,
		0x3480, 0x49,
		0x309E, 0x4A,
		0x309F, 0x4A,
		0x30D2, 0x19,
		0x30D7, 0x03,
		0x31EC, 0x0E,
		0x3010, 0x61,
};

static uint32_t imx327_720p_linear_25fps_setting[] = {
	0x3000, 0x01,
	0x3003, 0x00,
	0x3005, 0x01,
	0x3405, 0x10,
	0x3007, 0x10,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x3011, 0x02,
	0x3414, 0x04,
	0x3018, 0x08,
	0x3418, 0xD9,
	0x3019, 0x07,
	0x3419, 0x02,
	0x301C, 0xE4,
	0x301D, 0x0C,
	0x3129, 0x00,
	0x313B, 0x61,
	0x3441, 0x0C,
	0x3442, 0x0C,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x4F,
	0x3447, 0x00,
	0x3448, 0x2F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x17,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x57,
	0x3451, 0x00,
	0x3452, 0x17,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x305C, 0x20,
	0x305D, 0x00,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x1C,
	0x3473, 0x05,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_720p_linear_30fps_setting[] = {
	0x3000, 0x01,
	0x3003, 0x00,
	0x3005, 0x01,
	0x3405, 0x10,
	0x3007, 0x10,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x3011, 0x02,
	0x3414, 0x04,
	0x3018, 0xDC,
	0x3418, 0xD9,
	0x3019, 0x05,
	0x3419, 0x02,
	0x301C, 0xE4,
	0x301D, 0x0C,
	0x3129, 0x00,
	0x313B, 0x61,
	0x3441, 0x0C,
	0x3442, 0x0C,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x4F,
	0x3447, 0x00,
	0x3448, 0x2F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x17,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x57,
	0x3451, 0x00,
	0x3452, 0x17,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x305C, 0x20,
	0x305D, 0x00,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x1C,
	0x3473, 0x05,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_720p_dol2_25fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x10,
	0x3105, 0x11,
	0x3007, 0x10,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x300C, 0x11,
	0x3011, 0x02,
	0x3414, 0x00,   // OB
	0x3415, 0x00,
	0x3018, 0x08,
	0x3418, 0xBA,
	0x3019, 0x07,
	0x3419, 0x05,
	0x301C, 0xE4,
	0x301D, 0x0C,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3024, 0x7B,
	0x3025, 0x05,
	0x3129, 0x00,
	0x3030, 0x09,
	0x3031, 0x00,
	0x313B, 0x61,
	0x3441, 0x0C,
	0x3442, 0x0C,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3045, 0x05,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x4F,
	0x3447, 0x00,
	0x3448, 0x2F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x3048, 0x0A,
	0x3448, 0x00,
	0x344C, 0x17,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x57,
	0x3451, 0x00,
	0x3452, 0x17,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x305C, 0x20,
	0x305D, 0x00,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x20,
	0x3473, 0x05,
	0x347B, 0x23,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_720p_dol2_30fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x10,
	0x3106, 0x11,
	0x3007, 0x10,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x300C, 0x11,
	0x3011, 0x02,
	0x3414, 0x00,  // OB
	0x3415, 0x00,
	0x3018, 0xDC,
	0x3418, 0xBA,
	0x3019, 0x05,
	0x3419, 0x05,
	0x301C, 0xE4,
	0x301D, 0x0C,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3024, 0x7B,
	0x3025, 0x05,
	0x3129, 0x00,
	0x3030, 0x09,
	0x3031, 0x00,
	0x313B, 0x61,
	0x3441, 0x0C,
	0x3442, 0x0C,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3045, 0x05,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x4F,
	0x3447, 0x00,
	0x3448, 0x2F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x17,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x57,
	0x3451, 0x00,
	0x3452, 0x17,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x305C, 0x20,
	0x305D, 0x00,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x20,
	0x3473, 0x05,
	0x347B, 0x23,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_1080p_linear_25fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x20,
	0x3007, 0x40,
	0x3407, 0x03,
	0x3009, 0x02,
	0x300A, 0xF0,
	0x3011, 0x02,
	0x3414, 0x04,
	0x3018, 0x46,
	0x3418, 0x38,
	0x3019, 0x05,
	0x3419, 0x04,
	0x301C, 0x60,
	0x301D, 0x22,
	0x3129, 0x00,
	0x303A, 0x06,
	0x313B, 0x61,
	0x303C, 0x08,
	0x313D, 0x00,
	0x303E, 0x38,
	0x303F, 0x04,
	0x3040, 0x0C,
	0x3041, 0x00,
	0x3441, 0x0C,
	0x3042, 0x80,
	0x3442, 0x0C,
	0x3043, 0x07,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x0F,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0F,
	0x3453, 0x00,
	0x3454, 0x0F,
	0x3455, 0x00,
	0x305C, 0x18,
	0x305D, 0x03,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x80,
	0x3473, 0x07,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_1080p_linear_30fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x20,
	0x3007, 0x40,
	0x3407, 0x03,
	0x3009, 0x02,
	0x300A, 0xF0,
	0x3011, 0x02,
	0x3414, 0x04,
	0x3018, 0x65,
	0x3418, 0x38,
	0x3019, 0x04,
	0x3419, 0x04,
	0x301C, 0x60,
	0x301D, 0x22,
	0x3129, 0x00,
	0x303A, 0x06,
	0x313B, 0x61,
	0x303C, 0x08,
	0x313D, 0x00,
	0x303E, 0x38,
	0x303F, 0x04,
	0x3040, 0x0C,
	0x3041, 0x00,
	0x3441, 0x0C,
	0x3042, 0x80,
	0x3442, 0x0C,
	0x3043, 0x07,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1F,
	0x3449, 0x00,
	0x344A, 0x17,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x0F,
	0x344D, 0x00,
	0x344E, 0x17,
	0x344F, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0F,
	0x3453, 0x00,
	0x3454, 0x0F,
	0x3455, 0x00,
	0x305C, 0x18,
	0x305D, 0x03,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x80,
	0x3473, 0x07,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_1080p_dol2_25fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3405, 0x10,
	0x3106, 0x11,
	0x3007, 0x40,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x300C, 0x11,
	0x3011, 0x02,
	0x3414, 0x00,  // OB
	0x3415, 0x00,
	0x3018, 0x46,
	0x3418, 0x26,
	0x3019, 0x05,
	0x3419, 0x09,
	0x301C, 0x98,
	0x301D, 0x08,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3024, 0xab,
	0x3025, 0x00,
	0x3129, 0x00,
	0x3030, 0xa1,
	0x3031, 0x00,
	0x303A, 0x06,
	0x313B, 0x61,
	0x303C, 0x08,
	0x303D, 0x00,
	0x303E, 0x38,
	0x303F, 0x04,
	0x3040, 0x0C,
	0x3041, 0x00,
	0x3441, 0x0C,
	0x3042, 0x80,
	0x3442, 0x0C,
	0x3043, 0x07,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3045, 0x05,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x57,
	0x3447, 0x00,
	0x3448, 0x37,
	0x3449, 0x00,
	0x344A, 0x1F,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x1F,
	0x344D, 0x00,
	0x344E, 0x1F,
	0x344F, 0x00,
	0x3450, 0x77,
	0x3451, 0x00,
	0x3452, 0x1F,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x315C, 0x18,
	0x315D, 0x03,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x84,
	0x3473, 0x07,
	0x347B, 0x23,
	0x317C, 0x00,
	0x3480, 0x49,
	0x319E, 0x4A,
	0x319F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};

static uint32_t imx327_1080p_dol2_30fps_setting[] = {
	0x3000, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3410, 0x10,
	0x3106, 0x11,
	0x3007, 0x40,
	0x3407, 0x03,
	0x3009, 0x01,
	0x300A, 0xF0,
	0x300C, 0x11,
	0x3011, 0x02,
	0x3414, 0x00,  // OB
	0x3415, 0x00,
	0x3018, 0x65,
	0x3418, 0x26,
	0x3019, 0x04,
	0x3419, 0x09,
	0x301C, 0x98,
	0x301D, 0x08,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3024, 0xAB,
	0x3025, 0x00,
	0x3129, 0x00,
	0x3030, 0xA1,
	0x3031, 0x00,
	0x303A, 0x06,
	0x313B, 0x61,
	0x303C, 0x08,
	0x303D, 0x00,
	0x303E, 0x38,
	0x303F, 0x04,
	0x3040, 0x0C,
	0x3041, 0x00,
	0x3441, 0x0C,
	0x3042, 0x80,
	0x3442, 0x0C,
	0x3043, 0x07,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3045, 0x05,
	0x3445, 0x25,
	0x3046, 0x01,
	0x3446, 0x57,
	0x3447, 0x00,
	0x3448, 0x37,
	0x3449, 0x00,
	0x344A, 0x1F,
	0x304B, 0x0A,
	0x344B, 0x00,
	0x344C, 0x1F,
	0x344D, 0x00,
	0x344E, 0x1F,
	0x344F, 0x00,
	0x3450, 0x77,
	0x3451, 0x00,
	0x3452, 0x1F,
	0x3453, 0x00,
	0x3454, 0x17,
	0x3455, 0x00,
	0x305C, 0x18,
	0x305D, 0x03,
	0x305E, 0x20,
	0x315E, 0x1A,
	0x305F, 0x01,
	0x3164, 0x1A,
	0x3472, 0x84,
	0x3473, 0x07,
	0x347B, 0x23,
	0x317C, 0x00,
	0x3480, 0x49,
	0x309E, 0x4A,
	0x309F, 0x4A,
	0x30D2, 0x19,
	0x30D7, 0x03,
	0x31EC, 0x0E,
};


#if 0// fpga
static uint32_t imx327_raw12_dol2_setting[] = {
    //  1080P @30fps DOL
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x11,
	0x3011, 0x02,
	0x3018, 0xf9,
	0x3019, 0x15,
	0x301c, 0x90,
	0x301d, 0x33,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3024, 0x49,
	0x3025, 0x08,
	0x3030, 0x0b,
	0x3031, 0x00,
	0x3045, 0x05,
	0x3046, 0x01,
	0x304b, 0x0a,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x309e, 0x4a,
	0x309f, 0x4a,
	0x30d2, 0x19,
	0x30d7, 0x03,
	0x3106, 0x11,
	0x3129, 0x00,
	0x313b, 0x61,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x31ec, 0x0e,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00,
	0x3415, 0x00,
	0x3418, 0xb4,
	0x3419, 0x08,  // 2228
	0x3441, 0x0c,
	0x3442, 0x0c,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0xa0,
	0x3473, 0x07, //1952
	0x347b, 0x23,
	0x3480, 0x49
};
#endif
#ifdef __cplusplus
}
#endif

#endif  // UTILITY_SENSOR_INC_IMX327_SETTING_H_

