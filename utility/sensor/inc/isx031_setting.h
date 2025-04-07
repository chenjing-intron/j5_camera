/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_INC_ISX031_YUV422_SETTING_4LANE_H_
#define UTILITY_INC_ISX031_YUV422_SETTING_4LANE_H_

// config_index bit[8~9] for camera trig mode
#define BIT(i)		(1 << (i))
#define DUAL_LANE           BIT(2)
#define TEST_PATTERN        BIT(3)
// trig_source 1: mcu; 0: j3
#define TRIG_SOURCE         BIT(7)
#define TRIG_MODE           (0x3 << 8)
// shutter trigger-based sync, exposure sync
#define TRIG_SHUTTER        BIT(8)
// external pulse-based sync, read out sync
#define TRIG_EXTERNAL       BIT(9)

#define POC_DISABLE         BIT(14)

#define DES_PORT_NUM_MAX 4

#define LINK_ALL                0xFF
#define LINK_NONE               0x00

#define REG_LINK_SET_96712      0x0003
#define LINK_ALL_96712          0xAA
#define LINK_NONE_96712         0xFF

#define REG_LINK_SET_9296       0x0010
#define LINK_ALL_9296           0x23
#define LINK_NONE_9296          0x20

#define REG_TXRATE_96712_AB     0x0010
#define REG_TXRATE_96712_CD     0x0011

#define REG_TXRATE_9296         0x0001

#define TXRATE_6G               0x2
#define TXRATE_3G               0x1

#define REG_ALIAS_ID_SER        0x0000
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

#define ISX031_VMAX_OFFSET	0x8a70
#define ISX031_VMAX			0x8a74
#define ISX031_DEF_VMAX		0x06d6

uint32_t poc_init_setting[] = {
	0x01, 0x00,
	0x01, 0x1f,
};

static uint8_t max96712_lane_colck_reverse[] = {
	0x04, 0x54, 0x08, 0xA5, 0x24,
	0x04, 0x54, 0x08, 0xA6, 0x24,
};

static uint32_t max9296_init_setting[] = {
	0x0313, 0x00,
	0x0001, 0x01,
	0x0320, 0x28,
};

static uint32_t max96717_init_setting[] = {
	0x02D6,  0x00,
	0x02be,  0x10,
	0x0057,  0x12,
	0x005b,  0x11,
	0x0318,  0x5e,
	0x02D6,  0x10,
};

static uint8_t max96712_quad_init_setting_base[2][100] = {
	{
		0x04, 0x54, 0x00, 0x18, 0x0F,    // data path reset
		0x00, 0x80,
		// 0x00, 0xFF,

		// pipe Z in link B to video pipe 1, pipe Z in link A to video pipe 0
		0x04, 0x54, 0x00, 0xF0, 0x62,
		// pipe Z in link D to video pipe 3, pipe Z in link C to video pipe 2
		0x04, 0x54, 0x00, 0xF1, 0xEA,

		0x04, 0x54, 0x00, 0xF4, 0x0F,    // Enable Pipe 0~3

		0x04, 0x54, 0x08, 0xA2, 0xF4,    // Enable MIPI PHY0~3

		0x04, 0x54, 0x00, 0x03, 0xAA,    // Enable all 4 Links in GMSL2 mode

		0x04, 0x54, 0x00, 0x04, 0xFF,    // Enables all video transmission channels
#if 0	
		0x04, 0x54, 0x00, 0x10, 0x11,

		0x04, 0x54, 0x00, 0x11, 0x11,
#endif
	},
	{
		0x04, 0x54, 0x00, 0x18, 0x0F,    // data path reset
		0x00, 0x80,
		// 0x00, 0xFF,

		// pipe Z in link B to video pipe 1, pipe Z in link A to video pipe 0
		0x04, 0x54, 0x00, 0xF0, 0x62,
		// pipe Z in link D to video pipe 3, pipe Z in link C to video pipe 2
		0x04, 0x54, 0x00, 0xF1, 0xEA,

		// pipe Z in link B to video pipe 5, pipe Z in link A to video pipe 4
		0x04, 0x54, 0x00, 0xF2, 0x62,
		// pipe Z in link D to video pipe 7, pipe Z in link C to video pipe 6
		0x04, 0x54, 0x00, 0xF3, 0xEA,


		0x04, 0x54, 0x00, 0xF4, 0xFF,    // Enable Pipe 0~7

		0x04, 0x54, 0x08, 0xA2, 0xF4,    // Enable MIPI PHY0~3

		0x04, 0x54, 0x00, 0x03, 0xAA,    // Enable all 4 Links in GMSL2 mode
	},
};

static uint8_t max96712_polarity_pilot_gwm[] = {
		0x04, 0x54, 0x08, 0xA5, 0x02,    // polarity setting for Pilot_gwm A1
};

#if 0
static uint8_t max96712_quad_init_setting_base[] = {
	0x04, 0x52, 0x00, 0x18, 0x0F,     // data path reset
	0x00, 0x80,
	// 0x00, 0xFF,

// pipe Z in link B to video pipe 1, pipe Z in link A to video pipe 0
	0x04, 0x52, 0x00, 0xF0, 0x62,
// pipe Z in link D to video pipe 3, pipe Z in link C to video pipe 2
	0x04, 0x52, 0x00, 0xF1, 0xEA,

// pipe Z in link B to video pipe 5, pipe Z in link A to video pipe 4
	0x04, 0x52, 0x00, 0xF2, 0x62,
// pipe Z in link D to video pipe 7, pipe Z in link C to video pipe 6
	0x04, 0x52, 0x00, 0xF3, 0xEA,

// Enable Pipe 0~7
	0x04, 0x52, 0x00, 0xF4, 0xFF,
// Enable MIPI PHY0~3
	0x04, 0x52, 0x08, 0xA2, 0xF4,
// Enable all 4 Links in GMSL2 mode
	0x04, 0x52, 0x00, 0x03, 0xAA,

	0x04, 0x52, 0x08, 0xA5, 0x02,

	0x04, 0x52, 0x00, 0x04, 0x00,
};
#endif

static uint8_t max96712_lane_setting[5][500] = {
	{
	},
	{
		0x04, 0x54, 0x08, 0xA3, 0x44,    // Map data lanes for PHY 1
		0x04, 0x54, 0x08, 0xA4, 0x44,    // Map data lanes for PHY 0

		0x04, 0x54, 0x09, 0x0A, 0x40,    // 2 lanes
		0x04, 0x54, 0x09, 0x4A, 0x40,
		0x04, 0x54, 0x09, 0x8A, 0x40,
		0x04, 0x54, 0x09, 0xCA, 0x40,
		0x04, 0x54, 0x08, 0xA0, 0x01,    // 4 x 2 mode

		0x04, 0x54, 0x09, 0x0B, 0x07,	 // Map source 0~2 for Link A
		0x04, 0x54, 0x09, 0x2D, 0x00,	 // Map source to controller 1
		0x04, 0x54, 0x09, 0x0D, 0x1E,	 // src vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0E, 0x1E,	 // dst vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0F, 0x00,	 // src frame start
		0x04, 0x54, 0x09, 0x10, 0x00,	 // dst frame start
		0x04, 0x54, 0x09, 0x11, 0x01,	 // src frame end
		0x04, 0x54, 0x09, 0x12, 0x01,	 // dst frame end

		0x04, 0x54, 0x09, 0x4B, 0x07,    // Map source 0~2 for Link B
		0x04, 0x54, 0x09, 0x6D, 0x00,
		0x04, 0x54, 0x09, 0x4D, 0x1E,
		0x04, 0x54, 0x09, 0x4E, 0x5E,    // vc = 1
		0x04, 0x54, 0x09, 0x4F, 0x00,
		0x04, 0x54, 0x09, 0x50, 0x40,
		0x04, 0x54, 0x09, 0x51, 0x01,
		0x04, 0x54, 0x09, 0x52, 0x41,

		0x04, 0x54, 0x09, 0x8B, 0x07,    // Map source 0~2 for Link C
		0x04, 0x54, 0x09, 0xAD, 0x55,
		0x04, 0x54, 0x09, 0x8D, 0x1E,
		0x04, 0x54, 0x09, 0x8E, 0x1E,    // vc = 0
		0x04, 0x54, 0x09, 0x8F, 0x00,
		0x04, 0x54, 0x09, 0x90, 0x00,
		0x04, 0x54, 0x09, 0x91, 0x01,
		0x04, 0x54, 0x09, 0x92, 0x01,

		0x04, 0x54, 0x09, 0xCB, 0x07,    // Map source 0~2 for Link D
		0x04, 0x54, 0x09, 0xED, 0x55,
		0x04, 0x54, 0x09, 0xCD, 0x1E,
		0x04, 0x54, 0x09, 0xCE, 0x5E,    // vc = 1
		0x04, 0x54, 0x09, 0xCF, 0x00,
		0x04, 0x54, 0x09, 0xD0, 0x40,
		0x04, 0x54, 0x09, 0xD1, 0x01,
		0x04, 0x54, 0x09, 0xD2, 0x41,
	},
	{
	},
	{
		0x04, 0x54, 0x08, 0xA3, 0xE4,    // Map data lanes for PHY 1
		0x04, 0x54, 0x08, 0xA4, 0xE4,    // Map data lanes for PHY 0

		0x04, 0x54, 0x09, 0x0A, 0xC0,    // 4 lanes
		0x04, 0x54, 0x09, 0x4A, 0xC0,
		0x04, 0x54, 0x09, 0x8A, 0xC0,
		0x04, 0x54, 0x09, 0xCA, 0xC0,
		0x04, 0x54, 0x08, 0xA0, 0x04,    // 2x4 mode

		0x04, 0x54, 0x09, 0x0B, 0x07,	 // Map source 0~2 for Link A
		0x04, 0x54, 0x09, 0x2D, 0x15,	 // Map source to controller 1
		0x04, 0x54, 0x09, 0x0D, 0x1E,	 // src vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0E, 0x1E,	 // dst vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0F, 0x00,	 // src frame start
		0x04, 0x54, 0x09, 0x10, 0x00,	 // dst frame start
		0x04, 0x54, 0x09, 0x11, 0x01,	 // src frame end
		0x04, 0x54, 0x09, 0x12, 0x01,	 // dst frame end

		0x04, 0x54, 0x09, 0x4B, 0x07,	 // Map source 0~2 for Link B
		0x04, 0x54, 0x09, 0x6D, 0x15,
		0x04, 0x54, 0x09, 0x4D, 0x1E,
		0x04, 0x54, 0x09, 0x4E, 0x5E,	 // vc = 1
		0x04, 0x54, 0x09, 0x4F, 0x00,
		0x04, 0x54, 0x09, 0x50, 0x40,
		0x04, 0x54, 0x09, 0x51, 0x01,
		0x04, 0x54, 0x09, 0x52, 0x41,

		0x04, 0x54, 0x09, 0x8B, 0x07,	 // Map source 0~2 for Link C
		0x04, 0x54, 0x09, 0xAD, 0x15,
		0x04, 0x54, 0x09, 0x8D, 0x1E,
		0x04, 0x54, 0x09, 0x8E, 0x9E,	 // vc = 2
		0x04, 0x54, 0x09, 0x8F, 0x00,
		0x04, 0x54, 0x09, 0x90, 0x80,
		0x04, 0x54, 0x09, 0x91, 0x01,
		0x04, 0x54, 0x09, 0x92, 0x81,

		0x04, 0x54, 0x09, 0xCB, 0x07,	 // Map source 0~2 for Link D
		0x04, 0x54, 0x09, 0xED, 0x15,
		0x04, 0x54, 0x09, 0xCD, 0x1E,
		0x04, 0x54, 0x09, 0xCE, 0xDE,	 // vc = 3
		0x04, 0x54, 0x09, 0xCF, 0x00,
		0x04, 0x54, 0x09, 0xD0, 0xC0,
		0x04, 0x54, 0x09, 0xD1, 0x01,
		0x04, 0x54, 0x09, 0xD2, 0xC1,
	},
	{
		0x04, 0x54, 0x08, 0xA3, 0xE4,    // Map data lanes for PHY 1
		0x04, 0x54, 0x08, 0xA4, 0xE4,    // Map data lanes for PHY 0

		0x04, 0x54, 0x09, 0x0A, 0xC0,    // 4 lanes
		0x04, 0x54, 0x09, 0x4A, 0xC0,
		0x04, 0x54, 0x09, 0x8A, 0xC0,
		0x04, 0x54, 0x09, 0xCA, 0xC0,
		0x04, 0x54, 0x08, 0xA0, 0x04,    // 2x4 mode

		0x04, 0x54, 0x09, 0x0B, 0x07,	 // Map source 0~2 for Link A
		0x04, 0x54, 0x09, 0x2D, 0x2a,	 // Map source to controller 1
		0x04, 0x54, 0x09, 0x0D, 0x1E,	 // src vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0E, 0x1E,	 // dst vc && datatype, vc = 0, YUV422
		0x04, 0x54, 0x09, 0x0F, 0x00,	 // src frame start
		0x04, 0x54, 0x09, 0x10, 0x00,	 // dst frame start
		0x04, 0x54, 0x09, 0x11, 0x01,	 // src frame end
		0x04, 0x54, 0x09, 0x12, 0x01,	 // dst frame end

		0x04, 0x54, 0x09, 0x4B, 0x07,	 // Map source 0~2 for Link B
		0x04, 0x54, 0x09, 0x6D, 0x2a,
		0x04, 0x54, 0x09, 0x4D, 0x1E,
		0x04, 0x54, 0x09, 0x4E, 0x5E,	 // vc = 1
		0x04, 0x54, 0x09, 0x4F, 0x00,
		0x04, 0x54, 0x09, 0x50, 0x40,
		0x04, 0x54, 0x09, 0x51, 0x01,
		0x04, 0x54, 0x09, 0x52, 0x41,

		0x04, 0x54, 0x09, 0x8B, 0x07,	 // Map source 0~2 for Link C
		0x04, 0x54, 0x09, 0xAD, 0x2a,
		0x04, 0x54, 0x09, 0x8D, 0x1E,
		0x04, 0x54, 0x09, 0x8E, 0x9E,	 // vc = 2
		0x04, 0x54, 0x09, 0x8F, 0x00,
		0x04, 0x54, 0x09, 0x90, 0x80,
		0x04, 0x54, 0x09, 0x91, 0x01,
		0x04, 0x54, 0x09, 0x92, 0x81,

		0x04, 0x54, 0x09, 0xCB, 0x07,	 // Map source 0~2 for Link D
		0x04, 0x54, 0x09, 0xED, 0x2a,
		0x04, 0x54, 0x09, 0xCD, 0x1E,
		0x04, 0x54, 0x09, 0xCE, 0xDE,	 // vc = 3
		0x04, 0x54, 0x09, 0xCF, 0x00,
		0x04, 0x54, 0x09, 0xD0, 0xC0,
		0x04, 0x54, 0x09, 0xD1, 0x01,
		0x04, 0x54, 0x09, 0xD2, 0xC1,

		// 0x04,0x54,0x09,0x71,0x8F,
		// 0x04,0x54,0x09,0xb1,0x8F,

		0x04, 0x54, 0x0A, 0x0B, 0x07,	 // Map source 0~2 for Link
		0x04, 0x54, 0x0A, 0x2D, 0x15,	 // Map source to controller 2
		0x04, 0x54, 0x0A, 0x0D, 0x1E,	 // src vc && datatype, vc = 0, RAW12
		0x04, 0x54, 0x0A, 0x0E, 0x1E,	 // dst vc && datatype, vc = 0, RAW12
		0x04, 0x54, 0x0A, 0x0F, 0x00,	 // src frame start
		0x04, 0x54, 0x0A, 0x10, 0x00,	 // dst frame start
		0x04, 0x54, 0x0A, 0x11, 0x01,	 // src frame end
		0x04, 0x54, 0x0A, 0x12, 0x01,	 // dst frame end

		0x04, 0x54, 0x0A, 0x4B, 0x07,	 // Map source 0~2 for Link B
		0x04, 0x54, 0x0A, 0x6D, 0x15,
		0x04, 0x54, 0x0A, 0x4D, 0x1E,
		0x04, 0x54, 0x0A, 0x4E, 0x5E,	 // vc = 1
		0x04, 0x54, 0x0A, 0x4F, 0x00,
		0x04, 0x54, 0x0A, 0x50, 0x40,
		0x04, 0x54, 0x0A, 0x51, 0x01,
		0x04, 0x54, 0x0A, 0x52, 0x41,

		0x04, 0x54, 0x0A, 0x8B, 0x07,	 // Map source 0~2 for Link C
		0x04, 0x54, 0x0A, 0xAD, 0x15,
		0x04, 0x54, 0x0A, 0x8D, 0x1E,
		0x04, 0x54, 0x0A, 0x8E, 0x9E,	 // vc = 2
		0x04, 0x54, 0x0A, 0x8F, 0x00,
		0x04, 0x54, 0x0A, 0x90, 0x80,
		0x04, 0x54, 0x0A, 0x91, 0x01,
		0x04, 0x54, 0x0A, 0x92, 0x81,

		0x04, 0x54, 0x0A, 0xCB, 0x07,	 // Map source 0~2 for Link D
		0x04, 0x54, 0x0A, 0xED, 0x15,
		0x04, 0x54, 0x0A, 0xCD, 0x1E,
		0x04, 0x54, 0x0A, 0xCE, 0xDE,	 // vc = 3
		0x04, 0x54, 0x0A, 0xCF, 0x00,
		0x04, 0x54, 0x0A, 0xD0, 0xC0,
		0x04, 0x54, 0x0A, 0xD1, 0x01,
		0x04, 0x54, 0x0A, 0xD2, 0xC1,
	},
};

static uint8_t max96712_config_soft_rst_n_0[] = {
// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x04, 0x54, 0x1C, 0x00, 0xF4,
	0x04, 0x54, 0x1D, 0x00, 0xF4,
	0x04, 0x54, 0x1E, 0x00, 0xF4,
	0x04, 0x54, 0x1F, 0x00, 0xF4,
};

static uint8_t max96712_config_soft_rst_n_1[] = {
// Release reset to DPLL (config_soft_rst_n = 1) after changing the rate
	0x04, 0x54, 0x1C, 0x00, 0xF5,
	0x04, 0x54, 0x1D, 0x00, 0xF5,
	0x04, 0x54, 0x1E, 0x00, 0xF5,
	0x04, 0x54, 0x1F, 0x00, 0xF5,
};

#if 1
static uint8_t max96712_mipi_rate_setting_1g[] = {
// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x04, 0x54, 0x1C, 0x00, 0xF4,
	0x04, 0x54, 0x1D, 0x00, 0xF4,
	0x04, 0x54, 0x1E, 0x00, 0xF4,
	0x04, 0x54, 0x1F, 0x00, 0xF4,

// Set Data rate to be 2000Mbps/lane
	0x04, 0x54, 0x04, 0x15, 0x2A,
	0x04, 0x54, 0x04, 0x18, 0x2A,
	0x04, 0x54, 0x04, 0x1B, 0x2A,
	0x04, 0x54, 0x04, 0x1E, 0x2A,

// Release reset to DPLL (config_soft_rst_n = 1)
	0x04, 0x54, 0x1C, 0x00, 0xF5,
	0x04, 0x54, 0x1D, 0x00, 0xF5,
	0x04, 0x54, 0x1E, 0x00, 0xF5,
	0x04, 0x54, 0x1F, 0x00, 0xF5,
};

static uint8_t max96712_mipi_rate_setting_2g[] = {
// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x04, 0x54, 0x1C, 0x00, 0xF4,
	0x04, 0x54, 0x1D, 0x00, 0xF4,
	0x04, 0x54, 0x1E, 0x00, 0xF4,
	0x04, 0x54, 0x1F, 0x00, 0xF4,

// Set Data rate to be 2000Mbps/lane
	0x04, 0x54, 0x04, 0x15, 0x34,
	0x04, 0x54, 0x04, 0x18, 0x34,
	0x04, 0x54, 0x04, 0x1B, 0x34,
	0x04, 0x54, 0x04, 0x1E, 0x34,

// Release reset to DPLL (config_soft_rst_n = 1)
	0x04, 0x54, 0x1C, 0x00, 0xF5,
	0x04, 0x54, 0x1D, 0x00, 0xF5,
	0x04, 0x54, 0x1E, 0x00, 0xF5,
	0x04, 0x54, 0x1F, 0x00, 0xF5,
};
#endif

static uint8_t max9296_dual_init_setting_base[] = {
	// Disable MAX9296A MIPI output, CSI_OUT_EN=0
	0x04, 0x90, 0x03, 0x13, 0x00,

// ------------------ MAX9296 Settings -----------------------
	0x04, 0x90, 0x03, 0x30, 0x04,       // 2x4 mode

	// Software override for BPP, VC and DT
	0x04, 0x90, 0x03, 0x16, 0x5E,
	0x04, 0x90, 0x03, 0x17, 0x7E,
	0x04, 0x90, 0x03, 0x18, 0x7A,
	0x04, 0x90, 0x03, 0x19, 0x90,
	0x04, 0x90, 0x03, 0x1A, 0x40,

	// 0x04, 0x90, 0x03, 0x14, 0x10,      // pipeY VC=1 pipeX VC=0
	// 0x04, 0x90, 0x03, 0x15, 0x10,      // pipeU VC=1 pipeZ VC=0

	// 0x04, 0x90, 0x04, 0x4A, 0xD0,      // Four lane output from MIPI Port A
	0x04, 0x90, 0x03, 0x20, 0x6C,      // Set MIPI speed 1200Mbps, PHY1 soft override
	0x04, 0x90, 0x03, 0x23, 0x6C,      // Set MIPI speed 1200Mbps, PHY2 soft override

	// Send YUV422, FS, and FE from Pipe X to Controller 1
	0x04, 0x90, 0x04, 0x0B, 0x07,     // Enable 3 Mappings
	0x04, 0x90, 0x04, 0x2D, 0x15,     // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	// For the following MSB 2 bits = VC, LSB 6 bits =DT
	0x04, 0x90, 0x04, 0x0D, 0x1E,     // SRC  0b00011110, DT = 0x1E VC=0
	0x04, 0x90, 0x04, 0x0E, 0x1E,     // DEST 0b01011110, DT = 0x1E VC=0
	0x04, 0x90, 0x04, 0x0F, 0x00,     // SRC  DT = Frame Start
	0x04, 0x90, 0x04, 0x10, 0x00,     // DEST DT = Frame Start
	0x04, 0x90, 0x04, 0x11, 0x01,     // SRC  DT = Frame End
	0x04, 0x90, 0x04, 0x12, 0x01,     // DEST DT = Frame End

	// Send YUV422, FS, and FE from Pipe Y to Controller 1
	0x04, 0x90, 0x04, 0x4B, 0x07,     // Enable 3 Mappings
	0x04, 0x90, 0x04, 0x6D, 0x15,     // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	// For the following MSB 2 bits = VC, LSB 6 bits =DT
	0x04, 0x90, 0x04, 0x4D, 0x1E,     // SRC  0b00011110, DT = 0x1E VC=0
	0x04, 0x90, 0x04, 0x4E, 0x5E,     // DEST 0b01011110, DT = 0x1E VC=1
	0x04, 0x90, 0x04, 0x4F, 0x00,     // SRC  DT = Frame Start
	0x04, 0x90, 0x04, 0x50, 0x40,     // DEST DT = Frame Start
	0x04, 0x90, 0x04, 0x51, 0x01,     // SRC  DT = Frame End
	0x04, 0x90, 0x04, 0x52, 0x41,     // DEST DT = Frame End

	// Send YUV422, FS, and FE from Pipe Z to Controller 2
	0x04, 0x90, 0x04, 0x8B, 0x07,     // Enable 3 Mappings
	0x04, 0x90, 0x04, 0xAD, 0x2A,     // Destionation Controller = Controller 2. Controller 2 sends data to MIPI Port B
	// For the following MSB 2 bits = VC, LSB 6 bits = DT
	0x04, 0x90, 0x04, 0x8D, 0x1E,     // SRC  0b00011110, DT = 0x1E VC=0
	0x04, 0x90, 0x04, 0x8E, 0x1E,     // DEST 0b00011110, DT = 0x1E
	0x04, 0x90, 0x04, 0x8F, 0x00,     // SRC  DT = Frame Start
	0x04, 0x90, 0x04, 0x90, 0x00,     // DEST DT = Frame Start
	0x04, 0x90, 0x04, 0x91, 0x01,     // SRC  DT = Frame End
	0x04, 0x90, 0x04, 0x92, 0x01,     // DEST DT = Frame End

	// Send YUV422, FS, and FE from Pipe U to Controller 2
	0x04, 0x90, 0x04, 0xCB, 0x07,     // Enable 3 Mappings
	0x04, 0x90, 0x04, 0xED, 0x2A,     // Destionation Controller = Controller 2. Controller 2 sends data to MIPI Port B
	// For the following MSB 2 bits = VC, LSB 6 bits = DT
	0x04, 0x90, 0x04, 0xCD, 0x1E,     // SRC  0b00011110, DT = 0x1E VC=0
	0x04, 0x90, 0x04, 0xCE, 0x5E,     // DEST 0b00011110, DT = 0x1E
	0x04, 0x90, 0x04, 0xCF, 0x00,     // SRC  DT = Frame Start
	0x04, 0x90, 0x04, 0xD0, 0x40,     // DEST DT = Frame Start
	0x04, 0x90, 0x04, 0xD1, 0x01,     // SRC  DT = Frame End
	0x04, 0x90, 0x04, 0xD2, 0x41,     // DEST DT = Frame End

	0x04, 0x90, 0x00, 0x50, 0x02,     // 9296 pipeX (ID0), map to pipeZ_SIOA
	0x04, 0x90, 0x00, 0x51, 0x03,     // 9296 pipeY (ID1), map to pipeZ_SIOB
	0x04, 0x90, 0x00, 0x52, 0x02,     // 9296 pipeZ (ID2), map to pipeZ_SIOA
	0x04, 0x90, 0x00, 0x53, 0x03,     // 9296 pipeU (ID3), map to pipeZ_SIOB
};

static uint8_t max9296_dual_setting_patch[] = {
	0x04, 0x80, 0x00, 0x5B, 0x13,

	0x04, 0x80, 0x00, 0x6B, 0x16,
	0x04, 0x80, 0x00, 0x73, 0x17,
	0x04, 0x80, 0x00, 0x7B, 0x36,
	0x04, 0x80, 0x00, 0x83, 0x36,
	0x04, 0x80, 0x00, 0x93, 0x36,
	0x04, 0x80, 0x00, 0x9B, 0x36,
	0x04, 0x80, 0x00, 0xA3, 0x36,
	0x04, 0x80, 0x00, 0xAB, 0x36,
	0x04, 0x80, 0x00, 0x8B, 0x36,
	0x00, 0x32,
};

static uint8_t alias_id_setting[CAM_MAX_NUM][25] = {
	{
		0x04, 0x80, 0x00, 0x00, 0x82,
		0x04, 0x82, 0x00, 0x42, 0x42,
		0x04, 0x82, 0x00, 0x43, 0xA0,
		0x04, 0x82, 0x00, 0x44, 0x22,
		0x04, 0x82, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x84,
		0x04, 0x84, 0x00, 0x42, 0x44,
		0x04, 0x84, 0x00, 0x43, 0xA0,
		0x04, 0x84, 0x00, 0x44, 0x24,
		0x04, 0x84, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x86,
		0x04, 0x86, 0x00, 0x42, 0x46,
		0x04, 0x86, 0x00, 0x43, 0xA0,
		0x04, 0x86, 0x00, 0x44, 0x26,
		0x04, 0x86, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x88,
		0x04, 0x88, 0x00, 0x42, 0x48,
		0x04, 0x88, 0x00, 0x43, 0xA0,
		0x04, 0x88, 0x00, 0x44, 0x28,
		0x04, 0x88, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x8a,
		0x04, 0x8a, 0x00, 0x42, 0xC4,
		0x04, 0x8a, 0x00, 0x43, 0x8a,
		0x04, 0x8a, 0x00, 0x44, 0x2a,
		0x04, 0x8a, 0x00, 0x45, 0x34,
	},
};

static uint8_t alias_id_setting_hmisx031[CAM_MAX_NUM][25] = {
	{
		0x04, 0x84, 0x00, 0x00, 0x82,
		0x04, 0x82, 0x00, 0x42, 0xC4,
		0x04, 0x82, 0x00, 0x43, 0x82,
		0x04, 0x82, 0x00, 0x44, 0x22,
		0x04, 0x82, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x84, 0x00, 0x00, 0x84,
		0x04, 0x84, 0x00, 0x42, 0xC4,
		0x04, 0x84, 0x00, 0x43, 0x84,
		0x04, 0x84, 0x00, 0x44, 0x24,
		0x04, 0x84, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x84, 0x00, 0x00, 0x86,
		0x04, 0x86, 0x00, 0x42, 0xC4,
		0x04, 0x86, 0x00, 0x43, 0x86,
		0x04, 0x86, 0x00, 0x44, 0x26,
		0x04, 0x86, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x84, 0x00, 0x00, 0x88,
		0x04, 0x88, 0x00, 0x42, 0xC4,
		0x04, 0x88, 0x00, 0x43, 0x88,
		0x04, 0x88, 0x00, 0x44, 0x28,
		0x04, 0x88, 0x00, 0x45, 0x34,
	},
	{
		0x04, 0x84, 0x00, 0x00, 0x8a,
		0x04, 0x8a, 0x00, 0x42, 0xC4,
		0x04, 0x8a, 0x00, 0x43, 0x8a,
		0x04, 0x8a, 0x00, 0x44, 0x2a,
		0x04, 0x8a, 0x00, 0x45, 0x34,
	},
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
	0x04, 0x80, 0x00, 0x02, 0x03,
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

uint32_t max96717_max9295_trigger_mfp7_mfp8[] = {
	0x02D3, 0x84,  // pullup,push-pull,id = 7
	0x02D4, 0x67,  // id = 7
	0x02D5, 0x07,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en

	// sd P2.1 isx031 mfp7 & mfp8 two kinds of camera module
	0x02D7, 0x67,  // pullup,push-pull,id = 8
	0x02D8, 0x07,  // id = 8
	0x02D6, 0xf4,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en
};

uint32_t max96717_max9295_trigger_mfp8[] = {
	0x02D7, 0x67,  // pullup,push-pull,id = 7
	0x02D8, 0x07,  // id = 7
	0x02D6, 0xf4,  // 1M,High prio,Jitter,ouput 1,GMSL2 rx,ouput en
};

static uint16_t max9296_trigger_mfp[] = {
	0x02B1, 0xa7,  // pulldown,push-pull,id = 7
	0x02B2, 0x07,  // id = 7
	0x02B0, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
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

uint16_t max9296_trigger_mfp5[] = {
	0x0003, 0x40,  // swap pin assignments
	0x02C0, 0xa7,  // mfp5,pulldown,push-pull,id = 7
	0x02C1, 0x07,  // mfp5,id = 7
	0x02bf, 0xeb,  // mfp5,1M,High prio,Jitter,output 0,GMSL2 tx
};

static uint16_t max96712_trigger_setting_mfp[] = {
// MFP0
	0x0301, 0xa7,  // pulldown,push-pull,id = 7
	0x0302, 0x07,  // id = 7
	0x0300, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0337, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0338, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x036D, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x036E, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03A4, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03A5, 0x07,  // linkD,disable GMSL2 rx, id = 7
// MFP1
	0x0304, 0xa7,  // pulldown,push-pull,id = 7
	0x0305, 0x07,  // id = 7
	0x0303, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x033A, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x033B, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x0371, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x0372, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03A7, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03A8, 0x07,  // linkD,disable GMSL2 rx, id = 7
// MFP2
	0x0307, 0xa7,  // pulldown,push-pull,id = 7
	0x0308, 0x07,  // id = 7
	0x0306, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x033D, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x033E, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x0374, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x0375, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03AA, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03AB, 0x07,  // linkD,disable GMSL2 rx, id = 7
// MFP3
	0x030A, 0xa7,  // pulldown,push-pull,id = 7
	0x030B, 0x07,  // id = 7
	0x0309, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0341, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0342, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x0377, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x0378, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03AD, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03AE, 0x07,  // linkD,disable GMSL2 rx, id = 7
// MFP4
	0x030D, 0xa7,  // pulldown,push-pull,id = 7
	0x030E, 0x07,  // id = 7
	0x030C, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0344, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0345, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x037A, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x037B, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03B1, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03B2, 0x07,  // linkD,disable GMSL2 rx, id = 7
};

static uint8_t serializer_errb_mfp5_setting[CAM_MAX_NUM][10] = {
	{
		// sensor1 errb -> 96717 mfp5
		0x04, 0x82, 0x02, 0xcd, 0x83,
		0x04, 0x82, 0x02, 0xce, 0x82,  // id = 2
	},
	{
		// sensor2 errb -> 96717 mfp5
		0x04, 0x84, 0x02, 0xcd, 0x83,
		0x04, 0x84, 0x02, 0xce, 0x83,  // id = 3
	},
	{
		// sensor3 errb -> 96717 mfp5
		0x04, 0x86, 0x02, 0xcd, 0x83,
		0x04, 0x86, 0x02, 0xce, 0x84,  // id = 4
	},
	{
		// sensor4 errb -> 96717 mfp5
		0x04, 0x88, 0x02, 0xcd, 0x83,
		0x04, 0x88, 0x02, 0xce, 0x85,  // id = 5
	},
};

static uint8_t max96712_errb_mfp_mapping_setting[] = {
	// sensor2 mapping 96712 mfp0
	0x04, 0x54, 0x03, 0x00, 0x84,
	0x04, 0x54, 0x03, 0x01, 0x60,
	0x04, 0x54, 0x03, 0x02, 0x22,  // id = 2

	// sensor2 mapping 96712 mfp5
	0x04, 0x54, 0x03, 0x10, 0x80,
	0x04, 0x54, 0x03, 0x47, 0x00,
	0x04, 0x54, 0x03, 0x48, 0x23,  // id = 3

	// sensor2 mapping 96712 mfp7
	0x04, 0x54, 0x03, 0x16, 0x80,
	0x04, 0x54, 0x03, 0x84, 0x00,
	0x04, 0x54, 0x03, 0x85, 0x24,  // id = 4

	// sensor2 mapping 96712 mfp9
	0x04, 0x54, 0x03, 0x1c, 0x80,
	0x04, 0x54, 0x03, 0xc1, 0x00,
	0x04, 0x54, 0x03, 0xc2, 0x25,  // id = 5
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

uint16_t max96712_trigger_mfp4[] = {
	0x030D, 0xa7,  // pulldown,push-pull,id = 7
	0x030E, 0x07,  // id = 7
	0x030C, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x0344, 0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
	0x0345, 0x07,  // linkB,disable GMSL2 rx, id = 7
	0x037A, 0xe7,  // linkC,High prio,Jitter,GMSL2 tx,id = 7
	0x037B, 0x07,  // linkC,disable GMSL2 rx, id = 7
	0x03B1, 0xe7,  // linkD,High prio,Jitter,GMSL2 tx,id = 7
	0x03B2, 0x07,  // linkD,disable GMSL2 rx, id = 7
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

#endif  // UTILITY_INC_ISX031_YUV422_SETTING_4LANE_H_

