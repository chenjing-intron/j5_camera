/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_ISX031S_SETTING_H_
#define UTILITY_SENSOR_INC_ISX031S_SETTING_H_

// config_index bit[8~9] for camera trig mode
#define BIT(i)		(1 << (i))
#define DUAL_LANE           BIT(2)
#define DES_STREAMOFF		BIT(5)
// trig_source 1: mcu; 0: j3
#define TRIG_SOURCE         BIT(7)
#define TRIG_MODE           (0x3 << 8)
// shutter trigger-based sync, exposure sync
#define TRIG_SHUTTER        BIT(8)
// external pulse-based sync, read out sync
#define TRIG_EXTERNAL       BIT(9)

#define POC_DISABLE         BIT(14)
#define DEFAULT_SER_ADDR        0x80
#define HAOMO_SER_ADDR          0x84

#define REG_DATALANE_PORTA_9296 0x44A
#define REG_DATALANE_PORTB_9296 0x48A

#define REG_STREAM_ON           0x08A0
#define FORCE_CSI_OUT_96712     BIT(7)

#define REG_DPHY_0_DATA_RATE_96712 0x415
#define REG_DPHY_1_DATA_RATE_96712 0x418
#define REG_DPHY_2_DATA_RATE_96712 0x41B
#define REG_DPHY_3_DATA_RATE_96712 0x41E
#define DATA_RATE_BASE_96712       0x20

#define MAX9296_MFP_NUM 12u
#define MAX9296_MFP_OFFSET 3u

#define MAX96712_MFP_NUM 16u
#define MAX96712_MFP_LOOP 5u
#define MAX96712_MFP_OFFSET 0x10

static uint8_t max96712_lane_colck_reverse[] = {
	0x04, 0x52, 0x08, 0xA5, 0x24,
	0x04, 0x52, 0x08, 0xA6, 0x24,
};

static uint8_t serializer_pipez_setting_hmisx01[] = {
	0x04, 0x84, 0x03, 0x18, 0x5e,
	0x04, 0x84, 0x02, 0xbe, 0x10,
	0x04, 0x84, 0x02, 0xbf, 0x20,
	0x04, 0x84, 0x02, 0xc7, 0x10,
};

static uint8_t serializer_pipez_setting[] = {
	0x04, 0x80, 0x02, 0xd6, 0x00,
	0x04, 0x80, 0x02, 0xbe, 0x10,
	0x04, 0x80, 0x02, 0xbf, 0x60,

	0x04, 0x80, 0x00, 0x02, 0x43,       // enable Video transmit Channel Z
	0x04, 0x80, 0x03, 0x30, 0x00,       // Set SER to 1x4 mode
	0x04, 0x80, 0x03, 0x31, 0x33,       // Set 4 Lanes for SER
	0x04, 0x80, 0x03, 0x08, 0x64,       // Enable info lines: PORT B && Pipe Z
	0x04, 0x80, 0x03, 0x11, 0x40,       // Start video from Port B && Pipe Z

	// Set 9295A pipe z stream ID
	0x04, 0x80, 0x03, 0x18, 0x5e,       // Pipe_Z YUV
	0x04, 0x80, 0x02, 0xd6, 0x10,
	// 0x04, 0x80, 0x00, 0x02, 0x03,
};

static uint8_t serializer_pipez_setting_hk[] = {
	0x04, 0x84, 0x02, 0xd6, 0x00,
	0x04, 0x84, 0x02, 0xbe, 0x10,
	0x04, 0x84, 0x02, 0xbf, 0x60,

	0x04, 0x84, 0x00, 0x02, 0x43,       // enable Video transmit Channel Z
	0x04, 0x84, 0x03, 0x30, 0x00,       // Set SER to 1x4 mode
	0x04, 0x84, 0x03, 0x31, 0x33,       // Set 4 Lanes for SER
	0x04, 0x84, 0x03, 0x08, 0x64,       // Enable info lines: PORT B && Pipe Z
	0x04, 0x84, 0x03, 0x11, 0x40,       // Start video from Port B && Pipe Z

	// Set 9295A pipe z stream ID
	0x04, 0x84, 0x03, 0x18, 0x5e,       // Pipe_Z YUV
	0x04, 0x84, 0x02, 0xd6, 0x10,
};

uint32_t max96717_max9295_trigger_mfp7[] = {
	0x02D3, 0x84,  // pullup,push-pull,id = 7
	0x02D4, 0x67,  // id = 7
	0x02D5, 0x07,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en

	// sd P2.1 isx031 mfp7 & mfp8 two kinds of camera module
	0x02D7, 0x67,  // pullup,push-pull,id = 8
	0x02D8, 0x07,  // id = 8
	0x02D6, 0xf4,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en
};

uint32_t max96717f_trigger_mfp7[] = {
	0x02D3, 0x84,  // pullup,push-pull,id = 7
	0x02D4, 0x67,  // id = 7
	0x02D5, 0x07,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en
};

uint32_t max96717_max9295_trigger_mfp8[] = {
	0x02D7, 0x67,  // pullup,push-pull,id = 7
	0x02D8, 0x07,  // id = 7
	0x02D6, 0xf4,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en
};

uint32_t max9296_trigger_mfp7[] = {
	0x02C6, 0xa7,  // pulldown,push-pull,id = 7
	0x02C7, 0x07,  // id = 7
	0x02C5, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
};

uint32_t max9296_trigger_mfp8[] = {
	0x02C9, 0xa7,  // mfp8,pulldown,push-pull,id = 7
	0x02Ca, 0x07,  // mfp8,id = 7
	0x02C8, 0xeb,  // mfp8,1M,High prio,Jitter,output 0,GMSL2 tx
};

uint32_t max96712_trigger_mfp5[] = {
	0x0311, 0xa7,  // pulldown,push-pull,id = 7
	0x0312, 0x07,  // id = 7
	0x0310, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0347, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0348, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x037d, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x037e, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03b4, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03b5, 0x07,  // linkD,disable GMSL2 rx, id = 7
};

uint32_t max96712_trigger_mfp7[] = {
	0x0317, 0xa7,  // pulldown,push-pull,id = 7
	0x0318, 0x07,  // id = 7
	0x0316, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x034d, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x034e, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x0384, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x0385, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03ba, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03bb, 0x07,  // linkD,disable GMSL2 rx, id = 7
};

uint16_t max96712_trigger_mfp14[] = {
	0x032d, 0xa7,  // pulldown,push-pull,id = 7
	0x032e, 0x07,  // id = 7
	0x032c, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0364, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0365, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x039a, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x039b, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03d1, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03d2, 0x07,  // linkD,disable GMSL2 rx, id = 7
};

uint32_t isx031_trigger_shutter_mode_setting[] = {
	0x8AF0, 0x02,  // shutter trigger-based
	0xBF14, 0x02,  // SG_MODE_APL
};
uint32_t isx031_trigger_external_mode_setting[] = {
	0x8AF0, 0x01,  // shutter trigger-based  SG_MODE_ =
	0xBF14, 0x01,  // SG_MODE_APL
};

static uint32_t max96712_stream_on_setting[] = {
	0x08A0, 0x84,  	// MIPI output enable
};

static uint32_t max96712_stream_off_setting[] = {
	0x08A0, 0x00,  	// MIPI output enable
};

static uint32_t max9296_stream_on_setting[] = {
	0x0313, 0x42,
};

static uint32_t max9296_stream_off_setting[] = {
	0x0313, 0x00,
};

static uint32_t isx031_stream_on_setting[] = {
	0x8A01, 0x80,
};

static uint32_t isx031_stream_off_setting[] = {
	0x8A01, 0x00,
};

static uint32_t isx031_vmax_setting[] = {
	0x8A70, 0x0000,   // fps setting
};

static uint32_t isx031_pattern_mode_setting[] = {
	0xBE14, 0x01,  // DIF_PG_EN_
	0xBF60, 0x01,  // DIF_PG_EN_APL
};

#endif  // UTILITY_SENSOR_INC_ISX031S_SETTING_H_

