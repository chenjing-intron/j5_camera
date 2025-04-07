/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_SENSOR_COMMON_H_
#define UTILITY_SENSOR_INC_SENSOR_COMMON_H_

#define REG_TXRATE_96712     0x0010

#define REG_TXRATE_96718_A     	0x0001
#define REG_TXRATE_96718_B     	0x0004
#define REG_LINKA_SET_96718     0x0001
#define REG_LINKB_SET_96718     0x0003

#define REG_TXRATE_9296         0x0001

#define TXRATE_6G               0x2
#define TXRATE_3G               0x1
#define LINK_ALL                0xFF
#define LINK_NONE               0x00
#define REG_ALIAS_ID_SER        0x0000
#define REG_LINK_SET_96712      0x0003
#define LINK_ALL_96712          0xAA
#define LINK_NONE_96712         0xFF

#define REG_LINK_SET_9296       0x0010
#define LINK_ALL_9296           0x23
#define LINK_NONE_9296          0x20
#define LINK_ALL_9296_ANY       0x31
#define DES_PORT_NUM_MAX 4

#define MAX9296_MFP_NUM         12u
#define MAX9296_MFP_OFFSET      3u
#define MAX96712_MFP_NUM        16u
#define MAX96712_MFP_LOOP       5u
#define MAX96712_MFP_OFFSET     0x10
#define POC_RETRY_POLICY        1
#define OVERRIDE_96718_9296_ZU 	0x0320
#define OVERRIDE_96718_Y 	    0x031D
#define MUXED_96718_9296 	    0x0322
#define OVERRIDE_96712_PIPE01 	0x0415
#define OVERRIDE_96712_PIPE23 	0x0418
#define MUXED_96712 	        0x041A

#define MAX9296_RESET_REG		0x10
#define MAX9296_RESET_VAL		0xf3
#define MAX96712_RESET_REG		0x13
#define MAX96712_RESET_VAL		0x40
#define MAX9296_GMSL_MASK       0x3
#define MAX96712_GMSL_LINK_SHIFT	4
#define DATATYPE_YUV	            0X1E

#define MAX9296_ONE_SHOT_RESET_REG		0x10
#define MAX96718_ONE_SHOT_RESETA_REG	0x10
#define MAX96718_ONE_SHOT_RESETB_REG	0x12
#define MAX96712_ONE_SHOT_RESET_REG		0x18

#define BIT(i)              (1 << (i))
#define TEST_PATTERN_SERDES BIT(0)
#define TEST_PATTERN        BIT(1)
#define FPS_DIV             BIT(2)
#define DPHY_PORTB          BIT(3)
#define DPHY_COPY           BIT(4)
#define TRIG_STANDARD       BIT(8)
#define TRIG_SHUTTER_SYNC   BIT(9)
#define FORCE_3G            BIT(20)

#define ERRCHPWRUP_EN       BIT(2)

uint8_t common_rx_rate_switch(sensor_info_t *sensor_info, uint8_t data_rate);
int write_j5_register(int bus, uint8_t *pdata, int setting_size);
uint8_t common_link_switch(sensor_info_t *sensor_info, uint8_t link_port);
int32_t deserial_setting(sensor_info_t *sensor_info);
void loop_udelay(const uint64_t x);
int32_t poc_linked_first(int32_t bus, int32_t poc_addr);
void setting_modify(uint8_t *pdata, int32_t size, uint8_t addr, uint16_t reg, uint16_t v);
int32_t sensor_setting_array(int32_t bus, uint32_t i2c_addr, int32_t reg_width,
		int32_t setting_size, uint16_t *cam_setting);
int32_t sensor_poweron(sensor_info_t *sensor_info);
int32_t sensor_serdes_stream_on(sensor_info_t *sensor_info);
int32_t sensor_serdes_stream_off(sensor_info_t *sensor_info);

static uint16_t maxdes_errchpwrup_en[] = {
	0x1449,
	0x1549,
	0x1649,
	0x1749,
};
static uint16_t max9296_max96718_start_setting[] = {
	// 0x0313, 0x62,  	// MIPI output enable
	0x0330, 0x84,
};

static uint16_t max9296_max96718_stop_setting[] = {
	0x0330, 0x04,
};

static uint16_t max96712_start_setting[] = {
	0x08A0, 0x84,  	// MIPI csi force enable
};

static uint16_t max96712_stop_setting[] = {
	0x08A0, 0x04,  	// MIPI csi not force
};

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

static uint8_t max9296_max96718_init_setting_base[] = {
#ifndef POC_RETRY_POLICY
	0x03, 0x50, 0x01, 0x00,
	0x00, 0xff,
	0x03, 0x50, 0x01, 0x1f,
	0x00, 0xff,
#endif
	0x04, 0x90, 0x00, 0x10, 0x21,         // Reset data path linkA
	0x04, 0x90, 0x00, 0x12, 0x24,         // Reset data path linkB
	0x00, 0x32,
	0x04, 0x90, 0x03, 0x13, 0x00,

	// 0x04, 0x90, 0x04, 0x4A, 0xD0,	  // Four lane output from MIPI Port A
	0x04, 0x90, 0x03, 0x20, 0x34,		  // Port A: 2G
	0x04, 0x90, 0x03, 0x23, 0x34,		  // Port B: 2G
	// 0x04, 0x90, 0x00, 0x51, 0x01,	  // 9296 pipeY (ID1), map to pipeY
	// 0x04, 0x90, 0x00, 0x52, 0x02,	  // 9296 pipeZ (ID2), map to pipeZ

	0x04, 0x90, 0x03, 0x25, 0x80,         // ignore frist frame
};

static uint8_t max96712_init_setting_base[] = {
	0x04, 0x52, 0x00, 0x18, 0x0F,	  // data path reset avtive
	0x00, 0x80,

	0x04, 0x52, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x52, 0x15, 0x49, 0x75,
	0x04, 0x52, 0x16, 0x49, 0x75,
	0x04, 0x52, 0x17, 0x49, 0x75,
// pipe Z in link B to video pipe 1, pipe Z in link A to video pipe 0
	0x04, 0x52, 0x00, 0xF0, 0x62,
// pipe Z in link D to video pipe 3, pipe Z in link C to video pipe 2
	0x04, 0x52, 0x00, 0xF1, 0xEA,
	0x04, 0x52, 0x00, 0xF4, 0x0F,	 // Enable Pipe 0~3
	0x04, 0x52, 0x08, 0xA0, 0x04,	 // 2x4 mode
	0x04, 0x52, 0x08, 0xA3, 0xE4,	 // Map data lanes for PHY 1
	0x04, 0x52, 0x08, 0xA4, 0xE4,	 // Map data lanes for PHY 0

	0x04, 0x52, 0x09, 0x0A, 0xC0,	 // 4 lanes
	0x04, 0x52, 0x09, 0x4A, 0xC0,
	0x04, 0x52, 0x09, 0x8A, 0xC0,
	0x04, 0x52, 0x09, 0xCA, 0xC0,
	0x04, 0x52, 0x08, 0xA2, 0x34,	 // Enable MIPI PHY0~1
// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x04, 0x52, 0x1C, 0x00, 0xF4,
	0x04, 0x52, 0x1D, 0x00, 0xF4,
	0x04, 0x52, 0x1E, 0x00, 0xF4,
	0x04, 0x52, 0x1F, 0x00, 0xF4,

// Set Data rate to be 2000Mbps/lane
	0x04, 0x52, 0x04, 0x15, 0x34,
	0x04, 0x52, 0x04, 0x18, 0x34,
	0x04, 0x52, 0x04, 0x1B, 0x34,
	0x04, 0x52, 0x04, 0x1E, 0x34,

// Release reset to DPLL (config_soft_rst_n = 1)
	0x04, 0x52, 0x1C, 0x00, 0xF5,
	0x04, 0x52, 0x1D, 0x00, 0xF5,
	0x04, 0x52, 0x1E, 0x00, 0xF5,
	0x04, 0x52, 0x1F, 0x00, 0xF5,
	// 0x04, 0x52, 0x04, 0x0B, 0x62,
};

static uint8_t max96718_all_maplink_setting[] = {
	0x04, 0x50, 0x04, 0x8B, 0x07,	  // Map source 0~2 for Link A
	0x04, 0x50, 0x04, 0xAD, 0x15,	  // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
	0x04, 0x50, 0x04, 0x8F, 0x00,	  // SRC  DT = Frame Start
	0x04, 0x50, 0x04, 0x90, 0x00,	  // DEST DT = Frame Start
	0x04, 0x50, 0x04, 0x91, 0x01,	  // SRC  DT = Frame End
	0x04, 0x50, 0x04, 0x92, 0x01,	  // DEST DT = Frame End

	0x04, 0x50, 0x04, 0x4B, 0x07,     // Map source 0~2 for Link B
	0x04, 0x50, 0x04, 0x6D, 0x15,     // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	0x04, 0x50, 0x04, 0x4F, 0x00,     // SRC  DT = Frame Start
	0x04, 0x50, 0x04, 0x50, 0x00,     // DEST DT = Frame Start
	0x04, 0x50, 0x04, 0x51, 0x01,     // SRC  DT = Frame End
	0x04, 0x50, 0x04, 0x52, 0x01,     // DEST DT = Frame End
};

static uint8_t max96718_all_dt_vc_setting[] = {
	0x04, 0x50, 0x04, 0x8D, 0x2C,	  // SRC  0b00011110, DT = 0x2C VC=0
	0x04, 0x50, 0x04, 0x8E, 0x2C,	  // DEST 0b00011110, DT = 0x2C
	0x04, 0x50, 0x04, 0x4D, 0x2C,     // SRC  0b00011110, DT = 0x2C VC=0
	0x04, 0x50, 0x04, 0x4E, 0x2C,     // DEST 0b01011110, DT = 0x2C VC=0
};

static uint8_t max9296_dt_vc_setting[2][10] = {
	{
		0x04, 0x90, 0x04, 0x8D, 0x2C,	  // SRC  0b00011110, DT = 0x2C VC=0
		0x04, 0x90, 0x04, 0x8E, 0x2C,	  // DEST 0b00011110, DT = 0x2C
	},
	{
		0x04, 0x90, 0x04, 0xCD, 0x2C,     // SRC  0b00011110, DT = 0x2C VC=0
		0x04, 0x90, 0x04, 0xCE, 0x6C,     // DEST 0b01011110, DT = 0x2C VC=1
	},
};

static uint8_t max9296_maplink_setting[2][30] = {
	{
		// Send YUV422, FS, and FE from Pipe Z to Controller 1
		0x04, 0x90, 0x04, 0x8B, 0x07,	  // Enable 3 Mappings
		0x04, 0x90, 0x04, 0xAD, 0x15,	  // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
		0x04, 0x90, 0x04, 0x8F, 0x00,	  // SRC  DT = Frame Start
		0x04, 0x90, 0x04, 0x90, 0x00,	  // DEST DT = Frame Start
		0x04, 0x90, 0x04, 0x91, 0x01,	  // SRC  DT = Frame End
		0x04, 0x90, 0x04, 0x92, 0x01,	  // DEST DT = Frame End
	},
	{
		// Send RAW12, FS, and FE from Pipe U to Controller 1
		0x04, 0x90, 0x04, 0xCB, 0x07,     // Enable 3 Mappings
		0x04, 0x90, 0x04, 0xED, 0x15,     // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
		0x04, 0x90, 0x04, 0xCF, 0x00,     // SRC  DT = Frame Start
		0x04, 0x90, 0x04, 0xD0, 0x40,     // DEST DT = Frame Start
		0x04, 0x90, 0x04, 0xD1, 0x01,     // SRC  DT = Frame End
		0x04, 0x90, 0x04, 0xD2, 0x41,     // DEST DT = Frame End
	},
};

static uint8_t max96718_maplink_setting[2][30] = {
	{
		// Send YUV422, FS, and FE from Pipe Z to Controller 1
		0x04, 0x50, 0x04, 0x8B, 0x07,	  // Enable 3 Mappings
		0x04, 0x50, 0x04, 0xAD, 0x15,	  // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
		0x04, 0x50, 0x04, 0x8F, 0x00,	  // SRC  DT = Frame Start
		0x04, 0x50, 0x04, 0x90, 0x00,	  // DEST DT = Frame Start
		0x04, 0x50, 0x04, 0x91, 0x01,	  // SRC  DT = Frame End
		0x04, 0x50, 0x04, 0x92, 0x01,	  // DEST DT = Frame End
	},
	{
		// Send RAW12, FS, and FE from Pipe Y to Controller 1
		0x04, 0x50, 0x04, 0x4B, 0x07,     // Enable 3 Mappings
		0x04, 0x50, 0x04, 0x6D, 0x15,     // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
		0x04, 0x50, 0x04, 0x4F, 0x00,     // SRC  DT = Frame Start
		0x04, 0x50, 0x04, 0x50, 0x40,     // DEST DT = Frame Start
		0x04, 0x50, 0x04, 0x51, 0x01,     // SRC  DT = Frame End
		0x04, 0x50, 0x04, 0x52, 0x41,     // DEST DT = Frame End
	},
};

static uint8_t max96718_dt_vc_setting[2][10] = {
	{
		0x04, 0x50, 0x04, 0x8D, 0x2C,	  // SRC  0b00011110, DT = 0x2C VC=0
		0x04, 0x50, 0x04, 0x8E, 0x2C,	  // DEST 0b00011110, DT = 0x2C
	},
	{
		0x04, 0x50, 0x04, 0x4D, 0x2C,     // SRC  0b00011110, DT = 0x2C VC=0
		0x04, 0x50, 0x04, 0x4E, 0x6C,     // DEST 0b01011110, DT = 0x2C VC=1
	},
};

static uint8_t max96712_all_dt_vc_setting[] = {
	0x04, 0x52, 0x09, 0x0D, 0x2C,    // src vc && datatype, vc = 0, RAW12
	0x04, 0x52, 0x09, 0x0E, 0x2C,    // dst vc && datatype, vc = 0, RAW12

	0x04, 0x52, 0x09, 0x4D, 0x2C,
	0x04, 0x52, 0x09, 0x4E, 0x2C,    // vc = 0

	0x04, 0x52, 0x09, 0x8D, 0x2C,
	0x04, 0x52, 0x09, 0x8E, 0x2C,    // vc = 0

	0x04, 0x52, 0x09, 0xCD, 0x2C,
	0x04, 0x52, 0x09, 0xCE, 0x2C,    // vc = 0
};

static uint8_t max96712_dt_vc_setting[4][10] = {
	{
		0x04, 0x52, 0x09, 0x0D, 0x2C,    // src vc && datatype, vc = 0, RAW12
		0x04, 0x52, 0x09, 0x0E, 0x2C,    // dst vc && datatype, vc = 0, RAW12
	},
	{
		0x04, 0x52, 0x09, 0x4D, 0x2C,
		0x04, 0x52, 0x09, 0x4E, 0x6C,    // vc = 1
	},
	{
		0x04, 0x52, 0x09, 0x8D, 0x2C,
		0x04, 0x52, 0x09, 0x8E, 0xAC,    // vc = 2
	},
	{
		0x04, 0x52, 0x09, 0xCD, 0x2C,
		0x04, 0x52, 0x09, 0xCE, 0xEC,    // vc = 3
	},
};

static uint8_t max96712_all_maplink_setting[] = {
	0x04, 0x52, 0x09, 0x0B, 0x07,    // Map source 0~2 for Link A
	0x04, 0x52, 0x09, 0x2D, 0x15,    // Map source to controller 1
	0x04, 0x52, 0x09, 0x0F, 0x00,    // src frame start
	0x04, 0x52, 0x09, 0x10, 0x00,    // dst frame start
	0x04, 0x52, 0x09, 0x11, 0x01,    // src frame end
	0x04, 0x52, 0x09, 0x12, 0x01,    // dst frame end

	0x04, 0x52, 0x09, 0x4B, 0x07,    // Map source 0~2 for Link B
	0x04, 0x52, 0x09, 0x6D, 0x15,
	0x04, 0x52, 0x09, 0x4F, 0x00,
	0x04, 0x52, 0x09, 0x50, 0x00,
	0x04, 0x52, 0x09, 0x51, 0x01,
	0x04, 0x52, 0x09, 0x52, 0x01,

	0x04, 0x52, 0x09, 0x8B, 0x07,    // Map source 0~2 for Link C
	0x04, 0x52, 0x09, 0xAD, 0x15,
	0x04, 0x52, 0x09, 0x8F, 0x00,
	0x04, 0x52, 0x09, 0x90, 0x00,
	0x04, 0x52, 0x09, 0x91, 0x01,
	0x04, 0x52, 0x09, 0x92, 0x01,

	0x04, 0x52, 0x09, 0xCB, 0x07,    // Map source 0~2 for Link D
	0x04, 0x52, 0x09, 0xED, 0x15,
	0x04, 0x52, 0x09, 0xCF, 0x00,
	0x04, 0x52, 0x09, 0xD0, 0x00,
	0x04, 0x52, 0x09, 0xD1, 0x01,
	0x04, 0x52, 0x09, 0xD2, 0x01,
};

static uint8_t max96712_maplink_setting[4][30] = {
	{
		0x04, 0x52, 0x09, 0x0B, 0x07,    // Map source 0~2 for Link A
		0x04, 0x52, 0x09, 0x2D, 0x15,    // Map source to controller 1
		0x04, 0x52, 0x09, 0x0F, 0x00,    // src frame start
		0x04, 0x52, 0x09, 0x10, 0x00,    // dst frame start
		0x04, 0x52, 0x09, 0x11, 0x01,    // src frame end
		0x04, 0x52, 0x09, 0x12, 0x01,    // dst frame end
	},
	{
		0x04, 0x52, 0x09, 0x4B, 0x07,    // Map source 0~2 for Link B
		0x04, 0x52, 0x09, 0x6D, 0x15,
		0x04, 0x52, 0x09, 0x4F, 0x00,
		0x04, 0x52, 0x09, 0x50, 0x40,
		0x04, 0x52, 0x09, 0x51, 0x01,
		0x04, 0x52, 0x09, 0x52, 0x41,
	},
	{
		0x04, 0x52, 0x09, 0x8B, 0x07,    // Map source 0~2 for Link C
		0x04, 0x52, 0x09, 0xAD, 0x15,
		0x04, 0x52, 0x09, 0x8F, 0x00,
		0x04, 0x52, 0x09, 0x90, 0x80,
		0x04, 0x52, 0x09, 0x91, 0x01,
		0x04, 0x52, 0x09, 0x92, 0x81,
	},
	{
		0x04, 0x52, 0x09, 0xCB, 0x07,    // Map source 0~2 for Link D
		0x04, 0x52, 0x09, 0xED, 0x15,
		0x04, 0x52, 0x09, 0xCF, 0x00,
		0x04, 0x52, 0x09, 0xD0, 0xC0,
		0x04, 0x52, 0x09, 0xD1, 0x01,
		0x04, 0x52, 0x09, 0xD2, 0xC1,
	},
};

static uint8_t max9296_datatype_bpp_init_setting[] = {
	0x04, 0x90, 0x03, 0x17, 0x70,	/* pipe Z datatype 0x1e */
	0x04, 0x90, 0x03, 0x18, 0x7A,
	0x04, 0x90, 0x03, 0x19, 0x40,  	/* soft bpp z = 0x8 for mux */
	0x04, 0x90, 0x03, 0x1a, 0x20,
};

static uint8_t max96718_datatype_bpp_init_setting[] = {
	0x04, 0x50, 0x03, 0x16, 0x40,
	0x04, 0x50, 0x03, 0x17, 0x7E,	/* pipe Z datatype 0x1e */
	0x04, 0x50, 0x03, 0x18, 0x7A,
	0x04, 0x50, 0x03, 0x19, 0x48,  	/* soft bpp z/y = 0x8 for mux */
};

static uint8_t max96712_datatype_bpp_init_setting[] = {
	0x04, 0x52, 0x04, 0x0E, 0x5E,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe0(0x1E)
	0x04, 0x52, 0x04, 0x0F, 0x7E,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe1(0x1E)
	0x04, 0x52, 0x04, 0x10, 0x7A,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe2~3(0x1E)
	0x04, 0x52, 0x04, 0x0B, 0x40,    // 0b01000-000, Set bpp of pipe0(0x08)
	0x04, 0x52, 0x04, 0x11, 0x48,    // 0b01000-000, Set bpp of pipe1(0x08)
	0x04, 0x52, 0x04, 0x12, 0x20,    // 0b01000-000, Set bpp of pipe2~3(0x08)
};

static uint8_t max9296_alllink_yuv_mode_setting[] = {
	0x04, 0x90, 0x03, 0x20, 0x74,   // enable software-override for pipez
	0x04, 0x90, 0x03, 0x22, 0x40,	/* enable pipe Z yuv422 8/10bit muxed mode */
	0x04, 0x90, 0x03, 0x17, 0x7E,	/* pipe Z datatype 0x1e */
	0x04, 0x90, 0x03, 0x18, 0x7A,
	0x04, 0x90, 0x03, 0x19, 0x40,  	/* soft bpp z = 0x8 for mux */
};

static uint8_t max96718_alllink_yuv_mode_setting[] = {
	0x04, 0x50, 0x03, 0x20, 0x74,   // enable software-override for pipez
	0x04, 0x50, 0x03, 0x22, 0x60,	/* enable pipe Y/Z yuv422 8/10bit muxed mode */
	0x04, 0x50, 0x03, 0x16, 0x40,
	0x04, 0x50, 0x03, 0x17, 0x7E,	/* pipe Z datatype 0x1e */
	0x04, 0x50, 0x03, 0x18, 0x7A,
	0x04, 0x50, 0x03, 0x19, 0x48,  	/* soft bpp z/y = 0x8 for mux */
	0x04, 0x50, 0x03, 0x1D, 0xB4,
	0x04, 0x50, 0x00, 0x51, 0x02,
};

static uint8_t max96712_alllink_yuv_mode_setting[] = {
	0x04, 0x52, 0x04, 0x1A, 0xF0,	 // Set pipe0~3 YUV MUX mode
	0x04, 0x52, 0x04, 0x0E, 0x5E,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe0(0x1E)
	0x04, 0x52, 0x04, 0x0F, 0x7E,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe1(0x1E)
	0x04, 0x52, 0x04, 0x10, 0x7A,	 // 0b00(DT1_H)-011110(DT0), Set DT of Pipe2~3(0x1E)
	0x04, 0x52, 0x04, 0x0B, 0x40,    // 0b01000-000, Set bpp of pipe0(0x08)
	0x04, 0x52, 0x04, 0x11, 0x48,    // 0b01000-000, Set bpp of pipe1(0x08)
	0x04, 0x52, 0x04, 0x12, 0x40,    // 0b01000-000, Set bpp of pipe2~3(0x08)
	0x04, 0x52, 0x04, 0x15, 0xf4,    // enable software-override for pipe0~1
	0x04, 0x52, 0x04, 0x18, 0xf4,    // enable software-override for pipe2~3
};

static uint8_t max9296_add_max96718_init_setting[] = {
	0x04, 0x90, 0x01, 0x61, 0x09,
};

static uint32_t max9296_phy_portb_init_setting[] = {
	0x0051, 0x01,     // PipeY Stream ID = 1
	0x0052, 0x02,     // PipeZ Stream ID = 2
};

static uint32_t max9296_phy_portall_init_setting[] = {
	0x0051, 0x01,     // PipeY Stream ID = 1
	0x0052, 0x01,     // PipeZ Stream ID = 1
};

static uint8_t max96718_porta_out_setting[] = {
	0x04, 0x90, 0x04, 0x6D, 0x55,  // Pipe Y to dphy1.
	0x04, 0x90, 0x04, 0xAD, 0x55,  // Pipe Z to dphy1.
};

static uint8_t max96718_portb_out_setting[] = {
	0x04, 0x90, 0x04, 0x6D, 0xaa,  // Pipe Y to dphy2.
	0x04, 0x90, 0x04, 0xAD, 0xaa,  // Pipe Z to dphy2.
};

static uint8_t max96712_phy_portb_init_setting[] = {
	0x04, 0x52, 0x08, 0xA2, 0xC4,    // Enable MIPI PHY2~3
	0x04, 0x52, 0x09, 0x2D, 0x2A,    // Map pipe0 to controller 2
	0x04, 0x52, 0x09, 0x6D, 0x2A,    // Map pipe1 to controller 2
	0x04, 0x52, 0x09, 0xAD, 0x2A,    // Map pipe2 to controller 2
	0x04, 0x52, 0x09, 0xED, 0x2A,    // Map pipe3 to controller 2
};

static uint8_t max96712_phy_cpA2B_init_setting[] = {
	0x04, 0x52, 0x08, 0xA2, 0xF4,    // Enable MIPI PHY0~3
	0x04, 0x52, 0x08, 0xA9, 0xE8,    // Enable phy1 cp to phy3
};

static uint8_t max96712_phy_cpB2A_init_setting[] = {
	0x04, 0x52, 0x08, 0xA2, 0xF4,    // Enable MIPI PHY0~3
	0x04, 0x52, 0x08, 0xA9, 0xB8,    // Enable phy3 cp to phy1
};

static uint16_t max9296_trigger_mfp[] = {
	0x02B1, 0xa7,  // pulldown,push-pull,id = 7
	0x02B2, 0x07,  // id = 7
	0x02B0, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
};

static uint16_t max9296_trigger_mfp5[] = {
	0x0003, 0x40,  // pulldown,push-pull,id = 7
	0x02C0, 0xa7,  // pulldown,push-pull,id = 7
	0x02C1, 0x07,  // id = 7
	0x02bf, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
};

static uint16_t max96718_trigger_setting_mfp[] = {
	0x02B0, 0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
	0x02B1, 0xa7,  // pulldown,push-pull,id = 7
	0x02B2, 0x07,  // id = 7
	0x52B0, 0xeb,  // prio,Jitter,output 0,GMSL2 tx for linkB
	0x52B1, 0xa7,  // id = 7
	0x52B2, 0x07,  // id = 7
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

static uint16_t max96712_trigger_setting_mfp14[] = {
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

#endif  // UTILITY_SENSOR_INC_SENSOR_COMMON_H_

