/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_IMX290_SETTING_H_
#define UTILITY_SENSOR_INC_IMX290_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

#define IMX290_GAIN				(0x3014)
#define IMX290_SHS1             (0x3020)
#define IMX290_SHS2             (0x3024)
#define IMX290_SHS3             (0x3028)
#define IMX290_VAMX	            (0x3018)
#define IMX290_HAMX	            (0x301C)
#define IMX290_RHS1		        (0x3030)
#define IMX290_RHS2		        (0x3034)
#define IMX290_FPGC		        (0x3010)
#define IMX290_FPGC_1		    (0x30f0)
#define IMX290_FPGC_2		    (0x30f4)
#define IMX290_GAIN1		    (0x30f2)
#define IMX290_GAIN2		    (0x30f6)
#define IMX290_CSI_LANE_MODE    (0x3443)
#define IMX290_INCKSEL6         (0x3164)
#define IMX290_X_SIZE           (0x3472)
#define IMX290_Y_SIZE           (0x3418)


static uint32_t imx290_stream_on_setting[] = {
	0x3002, 0x00,
	0x3000, 0x00
};

static uint32_t imx290_stream_off_setting[] = {
	0x3000, 0x01
};

static uint32_t imx290_raw12_normal_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018, 0x65,
	0x3019, 0x0e,
	0x301c, 0xA0,
	0x301d, 0x3c,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6, 0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
	0x3418, 0x38,
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
	0x3472, 0x80,   // Pair od 0x3473
	0x3473, 0x07,   // X_OUT_SIZE[12:0]	0x07a0 = 1952
	0x3480, 0x49,   // INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};

static uint32_t imx290_raw12_dol2_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x11,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018, 0x65,
	0x3019, 0x0E,
	0x301c, 0xA0,
	0x301d, 0x14,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x49,
	0x3025, 0x08,
	0x3026, 0x00,
	0x3030, 0x0B,
	0x3031, 0x00,
	0x3032, 0x00,
	0x3045, 0x05,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6, 0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
	0x3418, 0x9C,
	0x3419, 0x08,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49
};

static uint32_t imx290_raw12_dol3_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x21,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018, 0x65,
	0x3019, 0x30,
	0x301c, 0xA0,
	0x301d, 0x14,
	0x3020, 0x05,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x0B,
	0x3025, 0x01,
	0x3026, 0x00,
	0x3028, 0x93,
	0x3029, 0x01,
	0x302a, 0x00,
	0x3030, 0x06,
	0x3031, 0x01,
	0x3032, 0x00,
	0x3034, 0x1c,
	0x3035, 0x01,
	0x3036, 0x00,
	0x3045, 0x05,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6, 0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00,
	0x3415, 0x00,
	0x3418, 0x55,
	0x3419, 0x11,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49
};

static uint32_t imx290_extra_hts_vts_setting[] = {
	0x381c, 0x00,
	0x381d, 0x00,
	0x3818, 0x00,
	0x3819, 0x00,
};

static uint32_t imx290_1280x720_raw10_25fps_normal_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x00,
	0x3007, 0x10,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0xEE,
	0x3019,	0x02,
	0x301c, 0xF0,
	0x301d, 0x1E,
	0x3046, 0x00,
	0x305c, 0x20,
	0x305d, 0x00,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x1D,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x12,
	0x317e, 0x00,
	0x31ec, 0x37,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x04,
	0x3418, 0xD9,
	0x3419, 0x02,
	0x3441, 0x0a,
	0x3442, 0x0a,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x17,
	0x3449, 0x00,
	0x344a, 0x0f,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x0f,
	0x344f, 0x00,
	0x3450, 0x2b,
	0x3451, 0x00,
	0x3452, 0x0b,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0x1c,
	0x3473, 0x05,
	0x3480, 0x49,
};

static uint32_t imx290_1280x720_raw12_25fps_normal_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x10,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0xEE,
	0x3019,	0x02,
	0x301c, 0xF0,
	0x301d, 0x1E,
	0x3046, 0x00,
	0x305c, 0x20,
	0x305d, 0x00,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x04,
	0x3418, 0xD9,
	0x3419, 0x02,
	0x3441, 0x0c,
	0x3442, 0x0c,
	0x3443, 0x03,
	0x3444, 0x20,
	0x3445, 0x25,
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x17,
	0x3449, 0x00,
	0x344a, 0x0f,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x0f,
	0x344f, 0x00,
	0x3450, 0x2b,
	0x3451, 0x00,
	0x3452, 0x0b,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0x1c,
	0x3473, 0x05,
	0x3480, 0x49,
};

static uint32_t imx290_1920x1080_raw10_25fps_normal_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x00,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x0e,
	0x301c, 0xA0,
	0x301d, 0x3c,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x1D,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x12,
	0x317e, 0x00,
	0x31ec, 0x37,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
	0x3418, 0x49,
	0x3419, 0x04,
	0x3441, 0x0a,
	0x3442, 0x0a,
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
};

static uint32_t imx290_1920x1080_raw10_25fps_dol2_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x00,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x11,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x04,
	0x301c, 0xA0,
	0x301d, 0x14,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x49,
	0x3025, 0x08,
	0x3026, 0x00,
	0x3030, 0x0B,
	0x3031, 0x00,
	0x3032, 0x00,
	0x3045, 0x05,
	0x3046, 0x00,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x1D,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x12,
	0x317e, 0x00,
	0x31ec, 0x37,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
	0x3415, 0x00,
	0x3418, 0x9C,
	0x3419, 0x08,
	0x3441, 0x0a,
	0x3442, 0x0a,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49,
};

static uint32_t imx290_1920x1080_raw10_25fps_dol3_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x00,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x21,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x04,
	0x301c, 0xA0,
	0x301d, 0x14,
	0x3020, 0x05,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x0B,
	0x3025, 0x01,
	0x3026, 0x00,
	0x3028, 0x93,
	0x3029, 0x01,
	0x302a, 0x00,
	0x3030, 0x06,
	0x3031, 0x01,
	0x3032, 0x00,
	0x3034, 0x1c,
	0x3035, 0x01,
	0x3036, 0x00,
	0x3045, 0x05,
	0x3046, 0x00,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x1D,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x12,
	0x317e, 0x00,
	0x31ec, 0x37,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
	0x3415, 0x00,
	0x3418, 0x55,
	0x3419, 0x11,
	0x3441, 0x0a,
	0x3442, 0x0a,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49,
};

static uint32_t imx290_1920x1080_raw12_25fps_normal_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x0e,
	0x301c, 0xA0,
	0x301d, 0x3c,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x0a,
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
};

static uint32_t imx290_1920x1080_raw12_25fps_dol2_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x11,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x0E,
	0x301c, 0xA0,
	0x301d, 0x3c,
	0x3020, 0x02,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x49,
	0x3025, 0x08,
	0x3026, 0x00,
	0x3030, 0x0B,
	0x3031, 0x00,
	0x3032, 0x00,
	0x3045, 0x05,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00,
	0x3415, 0x00,
	0x3418, 0x9C,
	0x3419, 0x08,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49,
};

static uint32_t imx290_1920x1080_raw12_25fps_dol3_setting[] = {
	0x3000, 0x01,
	0x3003, 0x01,
	0x3002, 0x00,
	0x3005, 0x01,
	0x3007, 0x00,
	0x3009, 0x02,
	0x300a, 0xf0,
	0x300c, 0x21,
	0x300f, 0x00,
	0x3010, 0x21,
	0x3012, 0x64,
	0x3016, 0x09,
	0x3018,	0x65,
	0x3019,	0x30,
	0x301c, 0xA0,
	0x301d, 0x3c,
	0x3020, 0x05,
	0x3021, 0x00,
	0x3022, 0x00,
	0x3024, 0x0B,
	0x3025, 0x01,
	0x3026, 0x00,
	0x3028, 0x93,
	0x3029, 0x01,
	0x302a, 0x00,
	0x3030, 0x06,
	0x3031, 0x01,
	0x3032, 0x00,
	0x3034, 0x1c,
	0x3035, 0x01,
	0x3036, 0x00,
	0x3045, 0x05,
	0x3046, 0x01,
	0x305c, 0x18,
	0x305d, 0x03,
	0x305e, 0x20,
	0x305f, 0x01,
	0x3070, 0x02,
	0x3071, 0x11,
	0x309b, 0x10,
	0x309c, 0x22,
	0x30a2, 0x02,
	0x30a6,	0x20,
	0x30a8, 0x20,
	0x30aa, 0x20,
	0x30ac, 0x20,
	0x30b0, 0x43,
	0x3119, 0x9E,
	0x311c, 0x1E,
	0x311e, 0x08,
	0x3128, 0x05,
	0x3129, 0x00,
	0x313d, 0x83,
	0x3150, 0x03,
	0x315e, 0x1a,
	0x3164, 0x1a,
	0x317c, 0x00,
	0x317e, 0x00,
	0x31ec, 0x0E,
	0x32b8, 0x50,
	0x32b9, 0x10,
	0x32ba, 0x00,
	0x32bb, 0x04,
	0x32c8, 0x50,
	0x32c9, 0x10,
	0x32ca, 0x00,
	0x32cb, 0x04,
	0x332c, 0xd3,
	0x332d, 0x10,
	0x332e, 0x0d,
	0x3358, 0x06,
	0x3359, 0xe1,
	0x335a, 0x11,
	0x3360, 0x1e,
	0x3361, 0x61,
	0x3362, 0x10,
	0x33b0, 0x50,
	0x33b2, 0x1a,
	0x33b3, 0x04,
	0x3405, 0x20,
	0x3407, 0x03,
	0x3414, 0x00,
	0x3415, 0x00,
	0x3418, 0x55,
	0x3419, 0x11,
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
	0x3472, 0xA0,
	0x3473, 0x07,
	0x347b, 0x23,
	0x3480, 0x49,
};

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_SENSOR_INC_IMX290_SETTING_H_
