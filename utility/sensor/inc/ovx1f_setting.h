/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_OVX1F_SETTING_H_
#define UTILITY_SENSOR_INC_OVX1F_SETTING_H_

#define REG_LINK_SET_96712      0x0003
#define LINK_96712_GMSL1        0x00
#define LINK_RESET_REG          0x18
#define LINK_ENABLE_REG         0x06
#define DES_PORT_NUM_MAX 4

#define REG_ALIAS_ID_SER        0x0000
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x29)
#define DEFAULT_SENSOR_ADDR		(0x36)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)


#define REG_DPHY_0_DATA_RATE_96712 0x415
#define REG_DPHY_1_DATA_RATE_96712 0x418
#define REG_DPHY_2_DATA_RATE_96712 0x41B
#define REG_DPHY_3_DATA_RATE_96712 0x41E
#define DATA_RATE_BASE_96712       0xE0

// config_index bit[8~11] reserved for camera trig mode
#define BIT(i)		(1 << (i))
#define TEST_PATTERN_SERDES	BIT(0)
#define TEST_PATTERN		BIT(1)
#define FPS_DIV				BIT(2)
#define DPHY_PORTB			BIT(3)
#define DPHY_COPY			BIT(4)
#define DES_STREAMOFF		BIT(5)

#define TRIG_STANDARD       BIT(8)
#define TRIG_SHUTTER_SYNC   BIT(9)

static uint32_t max96712_quad_init_setting_base[] = {
        // begin preset registers
	0x0013, 0x75,   // activate chip reset

	0x1449, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x1549, 0x75,
	0x1649, 0x75,
	0x1749, 0x75,

	0x040B, 0x00,   // disable mipi output
	0x0006, 0x00,   // disable all links

	0x0B06, 0xEF,
	0x0C06, 0xEF,
	0x0D06, 0xEF,
	0x0E06, 0xEF,

	0x0B07, 0x84,   // GMSL1 port 0 DBL=1
	0x0C07, 0x84,   // GMSL1 port 1 DBL=1
	0x0D07, 0x84,   // GMSL1 port 2 DBL=1
	0x0E07, 0x84,   // GMSL1 port 3 DBL=1

	// disable HS/VS processing
	0x0B0F, 0x01,
	0x0C0F, 0x01,
	0x0D0F, 0x01,
	0x0E0F, 0x01,

	0x041a, 0xf0,   // yuv mux mode

	0x08A0, 0x04,   // 2x4 mode
// 	0x08A2, 0xf0,   // enable 2 output phy
	0x00f4, 0x0f,   // enable 4 pipeline
// 	0x08A4, 0xE4,   // Map data lanes for PHY 0

// 	0x090A, 0xC0,   // phy0 lane no. at 2
	0x094A, 0xC0,
	0x08A3, 0xE4,
	0x08A4, 0xE4,
//  0x09CA, 0xC0,
	0x08A2, 0xF4,   // enable 4 mipi phy, set tlpx 106.7ns

	0x040B, 0x40,   // BPP for pipe lane 0 set as 1E (YUV422)
	0x040C, 0x00,   // VC for pipe line 0/1 set as 0
	0x040D, 0x00,   // VC for pipe line 2/3 set 0
	0x040E, 0x5E,   // dt for pipe line 0/1 set yuv422
	0x040F, 0x7E,   // dt for pipe line 1/2 set yuv422
	0x0410, 0x7A,   // dt for pipe line 2/3 set yuv422
	0x0411, 0x48,   // BPP for pipe line 1/2
	0x0412, 0x20,   // BPP for pipe line 2/3


// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x1C00, 0xF4,
	0x1D00, 0xF4,
	0x1E00, 0xF4,
	0x1F00, 0xF4,

	0x0415, 0xe8,   // enable softwareoverrideVC/DT/BPP for pipe line 0/1
	0x0418, 0xe8,   // enable softwareoverrideVC/DT/BPP for pipe line 2/3
	0x041B, 0x28,   // disable softwareoverrideVC/DT/BPP for pipe line 4/5
	0x041E, 0x28,   // disable softwareoverrideVC/DT/BPP for pipe line 6/7

// Release reset to DPLL (config_soft_rst_n = 1)
	0x1C00, 0xF5,
	0x1D00, 0xF5,
	0x1E00, 0xF5,
	0x1F00, 0xF5,

	0x090B, 0x07,   // enable internal pipe0 vc/dt to 0/2
	0x092D, 0x15,   // internal pipe0 route mipi control1
	0x090D, 0x1E,   // source data type(1E) and VC(0)
	0x090E, 0x1E,   // destination dt(1E) and VC (0)
	0x090F, 0x00,   // source dt(00,frame start) and vc (0)
	0x0910, 0x00,   // destination dt(00,frame start) and vc (0)
	0x0911, 0x01,   // source dt(01, frame end) and VC (0)
	0x0912, 0x01,   // destination dt(01,frame end) and VC(0)

	0x094B, 0x07,   // enable 3 mapping for pipeline 1
	0x096D, 0x15,   // csi2 controller 1
	0x094D, 0x1E,   // source vc = 0
	0x094E, 0x5E,   // des vc = 1
	0x094F, 0x00,   // source vc = 0
	0x0950, 0x40,   // des vc = 1
	0x0951, 0x01,   // source vc = 1
	0x0952, 0x41,   // des vc = 1

	0x098B, 0x07,   // enable 3 mapping for pipeline 2
	0x09AD, 0x15,   // csi2 controller 1
	0x098D, 0x1E,   // source vc = 0
	0x098E, 0x9E,   // des vc = 1
	0x098F, 0x00,   // source vc = 0
	0x0990, 0x80,   // des vc = 1
	0x0991, 0x01,   // source vc = 1
	0x0992, 0x81,   // des vc = 1

	0x09CB, 0x07,   // enable 3 mapping for pipeline 3
	0x09ED, 0x15,   // csi2 controller 1
	0x09CD, 0x1E,   // source vc = 0
	0x09CE, 0xDE,   // des vc = 1
	0x09CF, 0x00,   // source vc = 0
	0x09D0, 0xc0,   // des vc = 1
	0x09D1, 0x01,   // source vc = 1
	0x09D2, 0xc1,   // des vc = 1

	0x0B08, 0x01,
	0x0C08, 0x01,
	0x0D08, 0x01,
	0x0E08, 0x01,

	0x0018, 0x0F,   // one-shot link reset
	// 0x0006, 0x0F,   // enable internal pipe 0
};

static uint8_t alias_id_setting[][10] = {
	{
		0x00, 0x82,   // Set Ser new Address
		0x09, 0x22,   // I2C Source A Set Sensor new Address
		0x0a, 0x6c,   // I2C Desination A
		0x0b, 0x34,   // I2C Source B Set Ser Common Address
		0x0c, 0x82,   // I2C Desination B
	},
	{
		0x00, 0x84,
		0x09, 0x24,
		0x0a, 0x6c,
		0x0b, 0x34,
		0x0c, 0x84,
	},
	{
		0x00, 0x86,
		0x09, 0x26,
		0x0a, 0x6c,
		0x0b, 0x34,
		0x0c, 0x86,
	},
	{
		0x00, 0x88,
		0x09, 0x28,
		0x0a, 0x6c,
		0x0b, 0x34,
		0x0c, 0x88,
	},
};

uint32_t max96712_gmsl1_trigger_mfp14[] = {
    0x0B08, 0xf1,
    0x0C08, 0xf1,
    0x0D08, 0xf1,
    0x0E08, 0xf1,
};

static uint32_t serializer_gmsl1_setting[] = {
	// serializer 96701
	0x04, 0x47,   // 0x47
	0x07, 0x84,
	// crossbar_0~crossbar_de
	0x20, 0x10,   // invert mux output, mux outputs data from input 7
	0x21, 0x11,
	0x22, 0x12,
	0x23, 0x13,
	0x24, 0x14,
	0x25, 0x15,
	0x26, 0x16,   // invert mux output, mux outputs data from input 1
	0x27, 0x17,   // invert mux output, mux outputs data from input 0

	0x28, 0x40,   // force mux output low, mux outputs data from input 0
	0x29, 0x40,
	0x2a, 0x40,
	0x2b, 0x40,
	0x2c, 0x40,
	0x2d, 0x40,
	0x2e, 0x40,
	0x2f, 0x40,   // force mux output low, mux outputs data from input 0

	0x30, 0x00,   // input 7
	0x31, 0x01,   // input 6
	0x32, 0x02,   // input 5
	0x33, 0x03,   // input 4
	0x34, 0x04,   // mux outputs data from input 3
	0x35, 0x05,   // mux outputs data from input 2
	0x36, 0x06,   // input mapped to mux output, mux outputs data from input 1
	0x37, 0x07,   // input mapped to mux output, mux outputs data from input 0

	0x38, 0x40,   // force mux output low,mux outputs data from input 0
	0x39, 0x40,
	0x3a, 0x40,
	0x3b, 0x40,
	0x3c, 0x40,
	0x3d, 0x40,   // force mux output low,mux outputs data from input 0
	0x3e, 0x40,   // force mux output low,mux outputs data from input 0
	0x3f, 0x0c,   // mux sync signal din12
	0x40, 0x0d,   // mux sync signal din13
	0x41, 0x0c,   // mux sync signal din12

	0x67, 0xc4,   // align at each rising edge of HS
	0x04, 0x87,   // enable serialization / reverse-forward transmitter
	0x0f, 0xbf,   // GPO enable
};

uint32_t poc_init_setting[] = {
	0x01, 0x00,
	0x01, 0x1f,
};

static uint16_t max96712_start_setting[] = {
	0x040B, 0x42,  	// MIPI output enable
};

static uint16_t max96712_stop_setting[] = {
	0x040B, 0x40,  	// MIPI output enable
};

#endif  // UTILITY_SENSOR_INC_OVX1F_SETTING_H_
