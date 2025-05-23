/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef UTILITY_SENSOR_INC_OV2311_SETTING_H_
#define UTILITY_SENSOR_INC_OV2311_SETTING_H_

#define POC_RETRY_POLICY 1
#define RETRY_POC_TIMES 3

#define REG_LINKA_SET_96718     0x0001
#define REG_LINKB_SET_96718     0x0003
#define LINK_ALL_9296_ANY       0x31
#define REG_LINK_SET_9296       0x0010
#define LINK_ALL_9296           0x23
#define LINK_NONE_9296          0x20
#define DES_PORT_NUM_MAX        4
#define LINK_ALL                0xFF
#define LINK_NONE               0x00
#define REG_LINK_SET_96712      0x0003
#define LINK_ALL_96712          0xAA
#define LINK_NONE_96712         0xFF
#define BIT(i)		(1 << (i))

static uint8_t alias_id_setting[CAM_MAX_NUM][25] = {
	{
		0x04, 0x80, 0x00, 0x00, 0x82,
		0x04, 0x82, 0x00, 0x42, 0xA2,
		0x04, 0x82, 0x00, 0x43, 0xA0,
		0x04, 0x82, 0x00, 0x44, 0x22,
		0x04, 0x82, 0x00, 0x45, 0x20,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x84,
		0x04, 0x84, 0x00, 0x42, 0xA4,
		0x04, 0x84, 0x00, 0x43, 0xA0,
		0x04, 0x84, 0x00, 0x44, 0x24,
		0x04, 0x84, 0x00, 0x45, 0x20,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x86,
		0x04, 0x86, 0x00, 0x42, 0xA6,
		0x04, 0x86, 0x00, 0x43, 0xA0,
		0x04, 0x86, 0x00, 0x44, 0x26,
		0x04, 0x86, 0x00, 0x45, 0x20,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x88,
		0x04, 0x88, 0x00, 0x42, 0xA8,
		0x04, 0x88, 0x00, 0x43, 0xA0,
		0x04, 0x88, 0x00, 0x44, 0x28,
		0x04, 0x88, 0x00, 0x45, 0x20,
	},
	{
		0x04, 0x80, 0x00, 0x00, 0x8a,
		0x04, 0x8a, 0x00, 0x42, 0xAA,
		0x04, 0x8a, 0x00, 0x43, 0xA0,
		0x04, 0x8a, 0x00, 0x44, 0x2a,
		0x04, 0x8a, 0x00, 0x45, 0x6C,
	},
};

static uint8_t max9296_dual_setting_patch[] = {
	0x04, 0x80, 0x00, 0x5B, 0x13,
	0x04, 0x80, 0x00, 0x57, 0x11,

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

static uint8_t serializer_pipex_setting[] = {
	/* DVP configuration for ov2311 */
	0x04, 0x80, 0x01, 0xb0, 0x04,
	0x04, 0x80, 0x01, 0xb1, 0x05,
	0x04, 0x80, 0x01, 0xb2, 0x06,
	0x04, 0x80, 0x01, 0xb3, 0x07,
	0x04, 0x80, 0x01, 0xb4, 0x08,
	0x04, 0x80, 0x01, 0xb5, 0x09,
	0x04, 0x80, 0x01, 0xb6, 0x0a,
	0x04, 0x80, 0x01, 0xb7, 0x0b,

	0x04, 0x80, 0x00, 0x07, 0xF7,	/* MAX9295 Parallel video enabled */
	0x04, 0x80, 0x03, 0x11, 0xf0,	/* csi-b -> xyzu */
	0x04, 0x80, 0x03, 0x08, 0x6f,	/* csi-b */

	/* Set 9295A pipe x stream ID */
  	0x04, 0x80, 0x00, 0x53, 0x02,   /* pipe x -> stream 2 0x02 */

	0x04, 0x80, 0x01, 0x00, 0x60,	/* Line-CRC enabled, HS,VS,DE encoding on */
	0x04, 0x80, 0x01, 0x01, 0x4A,	/* Color bits per pixel */
};

uint8_t ov2311_power_on_setting[] = {
	0x04, 0x50, 0x00, 0x01, 0x1f,	/* enable output */
};

uint8_t ov2311_power_off_setting[] = {
	0x04, 0x50, 0x00, 0x01, 0x10,	/* disable output */
};

static uint8_t max96712_dms_parellel_enable[] = {
	// 0x04, 0x82, 0x00, 0x07, 0xF7, 	// Parellel enable
	0x04, 0x52, 0x04, 0x0B, 0x62,  	// MIPI output enable
};

static uint8_t max96712_j5b_dms_out_setting[] = {
	0x04, 0x52, 0x08, 0xA2, 0xC4,
	0x04, 0x52, 0x08, 0xA2, 0xF4,
};

static uint8_t max96712_dms_init_setting_4lane[] = {
	0x04, 0x52, 0x00, 0x13, 0x40,     // chip reset
	0x00, 0x64,

	0x04, 0x52, 0x04, 0x0B, 0x00,     // CSI output disabled
	// 0x04, 0x52, 0x04, 0x06, 0xF0,     // close all links
	0x04, 0x52, 0x00, 0x10, 0x11,     // Link A
	// 0x04, 0x52, 0x00, 0x11, 0x11,     // Link A
	0x04, 0x52, 0x00, 0x18, 0x0F,     // data path reset avtive
	0x00, 0x32,
	0x04, 0x52, 0x00, 0x18, 0x00,     // data path reset release
	0x00, 0x64,
	0x04, 0x52, 0x00, 0x06, 0xF1,     // Link A
	0x00, 0x32,
	0x04, 0x80, 0x00, 0x00, 0x82,

	0x04, 0x52, 0x00, 0x06, 0xF1,   // Enable LINKA Links in GMSL2 mode
	0x00, 0x64,
	/* DVP configuration for ov2311  LINKA pipe X */
	0x04, 0x82, 0x01, 0xb0, 0x04,
	0x04, 0x82, 0x01, 0xb1, 0x05,
	0x04, 0x82, 0x01, 0xb2, 0x06,
	0x04, 0x82, 0x01, 0xb3, 0x07,
	0x04, 0x82, 0x01, 0xb4, 0x08,
	0x04, 0x82, 0x01, 0xb5, 0x09,
	0x04, 0x82, 0x01, 0xb6, 0x0a,
	0x04, 0x82, 0x01, 0xb7, 0x0b,
	0x04, 0x82, 0x00, 0x02, 0x13, 	// Turn on pipe x only
	0x04, 0x82, 0x00, 0x53, 0x02,   /* pipe x -> stream 2 0x02 */
	0x04, 0x82, 0x01, 0x00, 0x60,	/* Line-CRC enabled, HS,VS,DE encoding on */
	0x04, 0x82, 0x01, 0x01, 0x4A,	/* Color bits per pixel */

    // pipe Z in link B to video pipe 1, pipe Z in link A to video pipe 0
	0x04, 0x52, 0x00, 0xF0, 0x62,
	// pipe Z in link D to video pipe 3, pipe Z in link C to video pipe 2
	0x04, 0x52, 0x00, 0xF1, 0xEA,

	// pipe Z in link B to video pipe 5, pipe Z in link A to video pipe 4
	0x04, 0x52, 0x00, 0xF2, 0x62,

	0x04, 0x52, 0x04, 0x0E, 0x1E,    // 0b00(DT1_H)-011110(DT0), Set DT of Pipe0(0x1E)
	0x04, 0x52, 0x04, 0x2E, 0x1E,    // 0b00(DT1_H)-011110(DT0), Set DT of Pipe4(0x1E)
	0x04, 0x52, 0x04, 0x0B, 0x40,    // 0b01000-000, Set bpp of pipe0(0x08)
	0x04, 0x52, 0x04, 0x2B, 0x40,    // 0b01000-000, Set bpp of pipe4(0x08)
	// YUV MUXED MODE for pipe 0,2
	0x04, 0x52, 0x04, 0x1A, 0x10,    // Set pipe0 YUV MUX mode
	0x04, 0x52, 0x04, 0x3A, 0x10,    // Set pipe4 YUV MUX mode

	0x04, 0x52, 0x00, 0xF4, 0x11,    // Enable Pipe 0 and pipe4
	0x04, 0x52, 0x08, 0xA2, 0xF4,    // Enable MIPI PHY0~3

	// open dms to J5B
	0x04, 0x52, 0x0A, 0x0B, 0x07,	 // Map source 0~2 for Link A
	0x04, 0x52, 0x0A, 0x2D, 0x15,	 // Map source to controller 1
	0x04, 0x52, 0x0A, 0x0D, 0x1E,	 // src vc && datatype, vc = 0, RAW12
	0x04, 0x52, 0x0A, 0x0E, 0x1E,	 // dst vc && datatype, vc = 0, RAW12
	0x04, 0x52, 0x0A, 0x0F, 0x00,	 // src frame start
	0x04, 0x52, 0x0A, 0x10, 0x00,	 // dst frame start
	0x04, 0x52, 0x0A, 0x11, 0x01,	 // src frame end
	0x04, 0x52, 0x0A, 0x12, 0x01,	 // dst frame end

	0x04, 0x52, 0x08, 0xA0, 0x04,    // 2x4 mode
	0x04, 0x52, 0x08, 0xA3, 0xE4,    // Map data lanes for PHY 1
	0x04, 0x52, 0x08, 0xA4, 0xE4,    // Map data lanes for PHY 0
	0x04, 0x52, 0x09, 0x0A, 0xC0,    // 4 lanes
	0x04, 0x52, 0x09, 0x4A, 0xC0,
	0x04, 0x52, 0x09, 0x8A, 0xC0,
	0x04, 0x52, 0x09, 0xCA, 0xC0,

// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x04, 0x52, 0x1C, 0x00, 0xF4,
	0x04, 0x52, 0x1D, 0x00, 0xF4,
	0x04, 0x52, 0x1E, 0x00, 0xF4,
	0x04, 0x52, 0x1F, 0x00, 0xF4,

// Set Data rate to be 2000Mbps/lane
	0x04, 0x52, 0x04, 0x15, 0x34,     // enable software-override for pipe0
	// 0x04, 0x52, 0x04, 0x18, 0x34,
	0x04, 0x52, 0x04, 0x1B, 0x74,    // enable software-override for pipe 4
	// 0x04, 0x52, 0x04, 0x1E, 0x34,

// Release reset to DPLL (config_soft_rst_n = 1)
	0x04, 0x52, 0x1C, 0x00, 0xF5,
	0x04, 0x52, 0x1D, 0x00, 0xF5,
	0x04, 0x52, 0x1E, 0x00, 0xF5,
	0x04, 0x52, 0x1F, 0x00, 0xF5,
	0x04, 0x82, 0x00, 0x07, 0xF7, 	// Parellel enable
};

uint8_t ov2311_init_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf1,	/* reset 9296 */
	0x00, 0x5F,	/* delay */

	0x04, 0x90, 0x00, 0x01, 0x01,	/* change to 3Gbps */
	0x04, 0x90, 0x00, 0x10, 0x21,	/* link A */
	0x04, 0x90, 0x03, 0x13, 0x00,	/* disable MAX9296A MIPI output */
	0x00, 0x5F,
	0x00, 0x5F,	/* delay */

	/* DVP configuration for ov2311 */
	0x04, 0x80, 0x01, 0xb0, 0x04,
	0x04, 0x80, 0x01, 0xb1, 0x05,
	0x04, 0x80, 0x01, 0xb2, 0x06,
	0x04, 0x80, 0x01, 0xb3, 0x07,
	0x04, 0x80, 0x01, 0xb4, 0x08,
	0x04, 0x80, 0x01, 0xb5, 0x09,
	0x04, 0x80, 0x01, 0xb6, 0x0a,
	0x04, 0x80, 0x01, 0xb7, 0x0b,

	0x04, 0x80, 0x00, 0x07, 0xF7,	/* MAX9295 Parallel video enabled */
	0x04, 0x80, 0x03, 0x11, 0xf0,	/* csi-b -> xyzu */
	0x04, 0x80, 0x03, 0x08, 0x6f,	/* csi-b */

	/* Set 9295A pipe x stream ID */
  	0x04, 0x80, 0x00, 0x53, 0x02,   /* pipe x -> stream 2 0x02 */

	0x04, 0x80, 0x01, 0x00, 0x60,	/* Line-CRC enabled, HS,VS,DE encoding on */
	0x04, 0x80, 0x01, 0x01, 0x4A,	/* Color bits per pixel */

  	0x04, 0x90, 0x03, 0x16, 0x5E,	/* pipe X datatype 0x1e */
	0x04, 0x90, 0x03, 0x17, 0x7E,	/* pipe Y datatype 0x1e */
	0x04, 0x90, 0x03, 0x19, 0x48,  	/* soft bpp Y = 0x8 for mux */
	0x04, 0x90, 0x03, 0x1D, 0xa0,	/* enable pipe Y soft override */
	0x04, 0x90, 0x03, 0x20, 0x2E,	/* Set MIPI phy1 speed 1400Mbps */
	0x04, 0x90, 0x03, 0x22, 0x20,	/* enable pipe Y yuv422 8/10bit muxed mode */

	0x04, 0x90, 0x03, 0x30, 0x04, 	/* Set as 2x4 mode, Default value */
	0x04, 0x90, 0x04, 0x4A, 0xD0, 	/* Set the lane count as 4 lanes */

	/* Send YUV422, FS, and FE from Pipe Y to Controller 1 */
	0x04, 0x90, 0x04, 0x4B, 0x07,	/* Enable 3 Mappings */
	0x04, 0x90, 0x04, 0x6D, 0x15,	/* Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A*/
	/* For the following MSB 2 bits = VC, LSB 6 bits =DT */
	0x04, 0x90, 0x04, 0x4D, 0x1E,	/* SRC  0b00011110,DT = 0x1E (YUV422-8bit) VC=0 */
	0x04, 0x90, 0x04, 0x4E, 0x1E,	/* DEST 0b00011110,DT = 0x1E,(YUV422-8bit) 0x1E */
	0x04, 0x90, 0x04, 0x4F, 0x00,	/* SRC  DT = Frame Start */
	0x04, 0x90, 0x04, 0x50, 0x00,	/* DEST DT = Frame Start */
	0x04, 0x90, 0x04, 0x51, 0x01,	/* SRC  DT = Frame End */
	0x04, 0x90, 0x04, 0x52, 0x01,	/* DEST DT = Frame End */

	0x04, 0x90, 0x00, 0x50, 0x00,	/* 9296 pipe X receive stream id 0 from 9295 */
  	0x04, 0x90, 0x00, 0x51, 0x02,	/* 9296 pipe Y receive stream id 2 from 9295 */
  	0x04, 0x90, 0x00, 0x52, 0x01,	/* 9296 pipe Z receive stream id 1 from 9295 */
  	0x04, 0x90, 0x00, 0x53, 0x03,	/* 9296 pipe U receive stream id 3 from 9295 */
};

uint8_t ov2311_stream_on_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x02,	/* enable CSI output */
};

uint8_t ov2311_stream_off_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x00,	/* enable mipi output */
};

#endif  // UTILITY_SENSOR_INC_OV2311_SETTING_H_
