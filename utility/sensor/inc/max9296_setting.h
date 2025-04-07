/*
 *    COPYRIGHT NOTICE
 *   Copyright 2019 Horizon Robotics, Inc.
 *    All rights reserved.
 */

#ifndef __HB_MAX9296_SETTING_H__
#define __HB_MAX9296_SETTING_H__

#ifdef __cplusplus
extern "C" {
#endif

#define BYPASS_96718_SYNC_REG	    (0x178)
#define BYPASS_96178_SYNC_VAL	    (0x80)  // default value: 0x00

uint8_t max9296_dummy_reset_preinit_setting[] = {
	// reset poc.
	0x03, 0x50, 0x01, 0x00,
	0x00, 0xff,
	0x03, 0x50, 0x01, 0x1f,
	0x00, 0xff,
	0x00, 0xff,
	0x04, 0x90, 0x00, 0x10, 0xf3,
	0x00, 0xff,
	0x00, 0xff,
};

uint8_t max96718_dummy_reset_preinit_setting[] = {
	// reset
	// 0x04, 0x90, 0x00, 0x10, 0xf3,
	// 0x00, 0xff,
	// 0x00, 0xff,
};

uint8_t max96718_porta_out_setting[] = {
	0x04, 0x90, 0x04, 0x6D, 0x55,  // Pipe Y to dphy1.
	0x04, 0x90, 0x04, 0xAD, 0x55,  // Pipe Z to dphy1.
};

uint8_t max96718_portb_out_setting[] = {
	0x04, 0x90, 0x04, 0x6D, 0xaa,  // Pipe Y to dphy2.
	0x04, 0x90, 0x04, 0xAD, 0xaa,  // Pipe Z to dphy2.
};

uint8_t max9295a_dummy_init_setting[] = {
	0x00, 0x64,       // Just doing delay has no practical meaning
};

uint8_t max9295e_dummy_A2Y_B2Z_init_setting[] = {
	0x00, 0xff,
	0x04, 0x80, 0x00, 0x10, 0xF0,		// Link Mode: Auto.
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0x63,		// Enable pipes Y Z
	0x04, 0x80, 0x03, 0x30, 0x86,		// CSI enable: PORTA+PORTB
	0x04, 0x80, 0x03, 0x08, 0x7C,		// Pipes X Y pull clock from port A, Z U pull clock from port B
	0x04, 0x80, 0x03, 0x11, 0xC3,		// Pipes X Y pull data from port A, Z U pull data from port B

	0x04, 0x80, 0x03, 0x0B, 0x01,		// Pipe Y pulls VC0
	0x04, 0x80, 0x03, 0x0C, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6C,		// Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5E,		// Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0D, 0x01,		// Pipe Z pulls VC0
	0x04, 0x80, 0x03, 0x0E, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6C,		// Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5E,		// Pipe Z pulls YUV422(DT 0x1E)
};

uint8_t max9295e_dummy_A2Y_init_setting[] = {
	0x04, 0x80, 0x00, 0x10, 0xf0,
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0x23,  // ENABLE pipe Y

	0x04, 0x80, 0x03, 0x30, 0x84,  // CSI enable: PORTA
	0x04, 0x80, 0x03, 0x08, 0x50,  // Pipes X Y Z U pull clock from port A
	0x04, 0x80, 0x03, 0x11, 0x06,  // Pipes Y Z pull data from port A

	0x04, 0x80, 0x03, 0x0b, 0x01,  // Pipe Y pulls VC0
	0x04, 0x80, 0x03, 0x0c, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6c,  // Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5e,  // Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0d, 0x01,  // Pipe Z pulls VC0
	0x04, 0x80, 0x03, 0x0e, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6c,  // Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5e,  // Pipe Z pulls YUV422(DT 0x1E)
};

uint8_t max9295e_dummy_bypass_A2Y_init_setting[] = {
	0x04, 0x80, 0x00, 0x10, 0x80,
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0x23,
	0x00, 0x64,

	0x04, 0x80, 0x03, 0x30, 0x84,
	0x04, 0x80, 0x03, 0x08, 0x5C,
	0x04, 0x80, 0x03, 0x11, 0x03,

	0x04, 0x80, 0x03, 0x09, 0x01,
	0x04, 0x80, 0x03, 0x0a, 0x00,

	0x04, 0x80, 0x03, 0x0b, 0x02,  // Pipe Y pulls VC1
	0x04, 0x80, 0x03, 0x0c, 0x00,

	0x04, 0x80, 0x03, 0x15, 0x80,  // independent VS enabled

    // disable HEARTBEAT MODE PIPE X Y Z U
	0x04, 0x80, 0x01, 0x02, 0x0e,
	0x04, 0x80, 0x01, 0x0A, 0x0e,
	0x04, 0x80, 0x01, 0x12, 0x0e,
	0x04, 0x80, 0x01, 0x1A, 0x0e,
};

uint8_t max9295e_dummy_bypass_A2YZ_init_setting[] = {
	0x04, 0x80, 0x00, 0x02, 0x63,
	0x00, 0x64,

	0x04, 0x80, 0x03, 0x30, 0x84,
	0x04, 0x80, 0x03, 0x08, 0x50,
	0x04, 0x80, 0x03, 0x11, 0x06,

	0x04, 0x80, 0x03, 0x09, 0x04,
	0x04, 0x80, 0x03, 0x0a, 0x00,

	0x04, 0x80, 0x03, 0x0b, 0x01,  // Pipe Y pulls VC0
	0x04, 0x80, 0x03, 0x0c, 0x00,

	0x04, 0x80, 0x03, 0x0D, 0x02,  // Pipe Z pulls VC1
	0x04, 0x80, 0x03, 0x0E, 0x00,

	0x04, 0x80, 0x03, 0x15, 0x80,  // independent VS enabled

    // disable HEARTBEAT MODE PIPE X Y Z U
	0x04, 0x80, 0x01, 0x02, 0x0e,
	0x04, 0x80, 0x01, 0x0A, 0x0e,
	0x04, 0x80, 0x01, 0x12, 0x0e,
	0x04, 0x80, 0x01, 0x1A, 0x0e,
};

uint8_t max96718_dummy_bypass_portA_Y2VC1_init_setting[] = {
	0x04, 0x50, 0x00, 0x10, 0x80,
	0x00, 0x64,
	0x04, 0x50, 0x03, 0x13, 0x60,  // mipi csi out disable

	0x04, 0x50, 0x04, 0x4B, 0x07,  // Enable 3 mappings for Pipe Y
	0x04, 0x50, 0x04, 0x6D, 0x15,
	0x04, 0x50, 0x04, 0x4D, 0x6C,  // Map RAW12, VC1
	0x04, 0x50, 0x04, 0x4E, 0x6C,
	0x04, 0x50, 0x04, 0x4F, 0x40,  // Map frame start, VC1
	0x04, 0x50, 0x04, 0x50, 0x40,
	0x04, 0x50, 0x04, 0x51, 0x41,  // Map frame end, VC1
	0x04, 0x50, 0x04, 0x52, 0x41,

	// Seq_miss_en fixed to 0 for CSI Dis_pkt_det fixed to 1 for CSI
	0x04, 0x50, 0x01, 0x12, 0x23,
	0x04, 0x50, 0x01, 0x24, 0x23,

	0x04, 0x50, 0x03, 0x25, 0x80,  // wait for a new frame
	0x04, 0x50, 0x03, 0x20, 0x2E,
	0x04, 0x50, 0x03, 0x23, 0x2E,

	0x04, 0x50, 0x01, 0x61, 0x01,
};

uint8_t max96718_dummy_bypass_portB_Y2VC1_init_setting[] = {
	0x04, 0x50, 0x00, 0x10, 0x80,
	0x00, 0x64,
	0x04, 0x50, 0x03, 0x13, 0x60,  // mipi csi out disable

	0x04, 0x50, 0x04, 0x4B, 0x07,  // Enable 3 mappings for Pipe Y
	0x04, 0x50, 0x04, 0x6D, 0x2a,
	0x04, 0x50, 0x04, 0x4D, 0x6C,  // Map RAW12, VC1
	0x04, 0x50, 0x04, 0x4E, 0x6C,
	0x04, 0x50, 0x04, 0x4F, 0x40,  // Map frame start, VC1
	0x04, 0x50, 0x04, 0x50, 0x40,
	0x04, 0x50, 0x04, 0x51, 0x41,  // Map frame end, VC1
	0x04, 0x50, 0x04, 0x52, 0x41,

	// Seq_miss_en fixed to 0 for CSI Dis_pkt_det fixed to 1 for CSI
	0x04, 0x50, 0x01, 0x12, 0x23,
	0x04, 0x50, 0x01, 0x24, 0x23,

	0x04, 0x50, 0x03, 0x25, 0x80,  // wait for a new frame
	0x04, 0x50, 0x03, 0x20, 0x2E,
	0x04, 0x50, 0x03, 0x23, 0x2E,

	0x04, 0x50, 0x01, 0x61, 0x01,
};

uint8_t max96718_dummy_bypass_Y2VC0_Z2VC1_init_setting[] = {
	0x04, 0x50, 0x03, 0x13, 0x60,  // mipi csi out disable

	0x04, 0x50, 0x03, 0x20, 0x32,
	0x04, 0x50, 0x03, 0x23, 0x32,
	0x04, 0x50, 0x03, 0x25, 0x80,

	0x04, 0x50, 0x04, 0x4B, 0x07,  // Enable 3 mappings for Pipe Y
	0x04, 0x50, 0x04, 0x6D, 0x15,
	0x04, 0x50, 0x04, 0x4D, 0x2C,  // Map RAW12, VC0
	0x04, 0x50, 0x04, 0x4E, 0x2C,
	0x04, 0x50, 0x04, 0x4F, 0x00,  // Map frame start, VC0
	0x04, 0x50, 0x04, 0x50, 0x00,
	0x04, 0x50, 0x04, 0x51, 0x01,  // Map frame end, VC0
	0x04, 0x50, 0x04, 0x52, 0x01,

	0x04, 0x50, 0x04, 0x8B, 0x07,  // Enable 3 mappings for Pipe Z
	0x04, 0x50, 0x04, 0xAD, 0x15,
	0x04, 0x50, 0x04, 0x8D, 0x6C,  // Map RAW12, VC1
	0x04, 0x50, 0x04, 0x8E, 0x6C,
	0x04, 0x50, 0x04, 0x8F, 0x40,  // Map frame start, VC1
	0x04, 0x50, 0x04, 0x90, 0x40,
	0x04, 0x50, 0x04, 0x91, 0x41,  // Map frame end, VC1
	0x04, 0x50, 0x04, 0x92, 0x41,

	0x04, 0x50, 0x01, 0x61, 0x11,

	// Seq_miss_en fixed to 0 for CSI Dis_pkt_det fixed to 1 for CSI
	0x04, 0x50, 0x01, 0x12, 0x23,
	0x04, 0x50, 0x01, 0x24, 0x23,
};


uint8_t max96718_dummy_Y2VC0_init_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf3,
	0x00, 0xff,
	0x00, 0xff,
	0x04, 0x90, 0x14, 0x49, 0xF5,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0xF5,
	0x04, 0x90, 0x00, 0x12, 0x24,		// reset LINKB
	0x04, 0x90, 0x00, 0x10, 0x23,		// choose LINKA+LINKB and rest LINKA

	// LINKA: MAX9295 E1 - i2c.
	0x04, 0x90, 0x00, 0x01, 0x02,
	0x04, 0x90, 0x00, 0x03, 0x57,
	0x00, 0x32,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe y
	0x04, 0x90, 0x04, 0x4D, 0x2C,
	0x04, 0x90, 0x04, 0x4E, 0x2C,
	0x04, 0x90, 0x04, 0x4F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x50, 0x1E,
	0x04, 0x90, 0x04, 0x51, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x52, 0x00,
	0x04, 0x90, 0x04, 0x53, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x54, 0x01,

	// 0x04, 0x90, 0x04, 0x6D, 0xaa,		// Pipe y to Port B.

	0x04, 0x90, 0x01, 0x61, 0x01,		// pipey -> linka -> pipey
};

uint8_t max96718_dummy_Z2VC0_init_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf3,
	0x00, 0xff,
	0x00, 0xff,
	0x04, 0x90, 0x14, 0x49, 0xF5,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0xF5,

	0x04, 0x90, 0x00, 0x12, 0x24,		// reset LINKB
	0x04, 0x90, 0x00, 0x10, 0x23,		// choose LINKA+LINKB and rest LINKA

	// LINKB: MAX9295 E2 - i2c.
	0x04, 0x90, 0x00, 0x01, 0x12,
	0x04, 0x90, 0x00, 0x03, 0x53,
	0x00, 0x32,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x8E, 0x2C,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x90, 0x1E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x92, 0x00,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x94, 0x01,

	// 0x04, 0x90, 0x04, 0xAD, 0xaa,		// Pipe Z to Port B.
	// 0x04, 0x90, 0x04, 0x6D, 0xaa,		// Pipe y to Port B.

	0x04, 0x90, 0x01, 0x61, 0x28,		// pipey -> linkb -> pipeZ
};

uint8_t max96718_dummy_Y2VC0_single_init_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf1,		// reset, auto link
	0x00, 0xff,
	0x04, 0x90, 0x14, 0x49, 0xF5,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0xF5,

	0x04, 0x90, 0x0f, 0x00, 0x01,		// Enable LinkA

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe y
	0x04, 0x90, 0x04, 0x4D, 0x2C,
	0x04, 0x90, 0x04, 0x4E, 0x2C,
	0x04, 0x90, 0x04, 0x4F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x50, 0x1E,
	0x04, 0x90, 0x04, 0x51, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x52, 0x00,
	0x04, 0x90, 0x04, 0x53, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x54, 0x01,

	// 0x04, 0x90, 0x04, 0x6D, 0xaa,		// Pipe y to Port B.
	0x04, 0x90, 0x01, 0x61, 0x01,		// pipey -> linka -> pipey
};

uint8_t max9295e_dummy_A2YX_B2ZU_init_setting[] = {
	0x00, 0xff,
	0x04, 0x80, 0x00, 0x10, 0xF0,		// Link Mode: Auto.
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0xF3,		// Enable pipes X Y Z U
	0x04, 0x80, 0x03, 0x30, 0x86,		// CSI enable: PORTA+PORTB
	0x04, 0x80, 0x03, 0x08, 0x7C,		// Pipes X Y pull clock from port A, Z U pull clock from port B
	0x04, 0x80, 0x03, 0x11, 0xC3,		// Pipes X Y pull data from port A, Z U pull data from port B

	0x04, 0x80, 0x03, 0x09, 0x02,		// Pipe X pulls VC1
	0x04, 0x80, 0x03, 0x0A, 0x00,
	0x04, 0x80, 0x03, 0x14, 0x6C,		// Pipe X pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x15, 0x5E,		// Pipe X pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0B, 0x01,		// Pipe Y pulls VC0
	0x04, 0x80, 0x03, 0x0C, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6C,		// Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5E,		// Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0D, 0x01,		// Pipe Z pulls VC0
	0x04, 0x80, 0x03, 0x0E, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6C,		// Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5E,		// Pipe Z pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0F, 0x02,		// Pipe U pulls VC1
	0x04, 0x80, 0x03, 0x10, 0x00,
	0x04, 0x80, 0x03, 0x1A, 0x6C,		// Pipe U pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x1B, 0x5E,		// Pipe U pulls YUV422(DT 0x1E)
};

uint8_t max9295e_dummy_A2XYZU_init_setting[] = {
	0x00, 0xff,
	0x04, 0x80, 0x00, 0x10, 0xF0,		// Link Mode: Auto.
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0xF3,		// Enable pipes X Y Z U
	0x04, 0x80, 0x03, 0x30, 0x84,		// CSI enable: PORTA
	0x04, 0x80, 0x03, 0x08, 0x50,		// Pipes X Y Z U pull clock from port A
	0x04, 0x80, 0x03, 0x11, 0x0F,		// Pipes X Y Z U pull data from port A

	0x04, 0x80, 0x03, 0x09, 0x01,		// Pipe X pulls VC0
	0x04, 0x80, 0x03, 0x0A, 0x00,
	0x04, 0x80, 0x03, 0x14, 0x6C,		// Pipe X pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x15, 0x5E,		// Pipe X pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0B, 0x02,		// Pipe Y pulls VC1
	0x04, 0x80, 0x03, 0x0C, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6C,		// Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5E,		// Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0D, 0x04,		// Pipe Z pulls VC2
	0x04, 0x80, 0x03, 0x0E, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6C,		// Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5E,		// Pipe Z pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0F, 0x08,		// Pipe U pulls VC3
	0x04, 0x80, 0x03, 0x10, 0x00,
	0x04, 0x80, 0x03, 0x1A, 0x6C,		// Pipe U pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x1B, 0x5E,		// Pipe U pulls YUV422(DT 0x1E)
};

uint8_t max9295e_dummy_B2XYZU_init_setting[] = {
	0x00, 0xff,
	0x04, 0x80, 0x00, 0x10, 0xF0,		// Link Mode: Auto.
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0xF3,		// Enable pipes X Y Z U
	0x04, 0x80, 0x03, 0x30, 0x85,		// CSI enable: PORTB
	0x04, 0x80, 0x03, 0x08, 0x6F,		// Pipes X Y Z U pull clock from port B
	0x04, 0x80, 0x03, 0x11, 0xF0,		// Pipes X Y Z U pull data from port B

	0x04, 0x80, 0x03, 0x09, 0x01,		// Pipe X pulls VC0
	0x04, 0x80, 0x03, 0x0A, 0x00,
	0x04, 0x80, 0x03, 0x14, 0x6C,		// Pipe X pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x15, 0x5E,		// Pipe X pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0B, 0x02,		// Pipe Y pulls VC1
	0x04, 0x80, 0x03, 0x0C, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6C,		// Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5E,		// Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0D, 0x04,		// Pipe Z pulls VC2
	0x04, 0x80, 0x03, 0x0E, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6C,		// Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5E,		// Pipe Z pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0F, 0x08,		// Pipe U pulls VC3
	0x04, 0x80, 0x03, 0x10, 0x00,
	0x04, 0x80, 0x03, 0x1A, 0x6C,		// Pipe U pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x1B, 0x5E,		// Pipe U pulls YUV422(DT 0x1E)
};

uint8_t max9295e_dummy_A2YXA_B2ZUB_init_setting[] = {
	0x00, 0xff,
	0x04, 0x80, 0x00, 0x10, 0xF0,		// Link Mode: Auto.
	0x00, 0x64,
	0x04, 0x80, 0x00, 0x02, 0xF3,		// Enable pipes X Y Z U
	0x04, 0x80, 0x03, 0x30, 0x86,		// CSI enable: PORTA+PORTB
	0x04, 0x80, 0x03, 0x08, 0x7C,		// Pipes X Y pull clock from port A, Z U pull clock from port B
	0x04, 0x80, 0x03, 0x11, 0xC3,		// Pipes X Y pull data from port A, Z U pull data from port B

	0x04, 0x80, 0x03, 0x09, 0x02,		// Pipe X pulls VC1
	0x04, 0x80, 0x03, 0x0A, 0x00,
	0x04, 0x80, 0x03, 0x14, 0x6C,		// Pipe X pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x15, 0x5E,		// Pipe X pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0B, 0x01,		// Pipe Y pulls VC0
	0x04, 0x80, 0x03, 0x0C, 0x00,
	0x04, 0x80, 0x03, 0x16, 0x6C,		// Pipe Y pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x17, 0x5E,		// Pipe Y pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0D, 0x01,		// Pipe Z pulls VC0
	0x04, 0x80, 0x03, 0x0E, 0x00,
	0x04, 0x80, 0x03, 0x18, 0x6C,		// Pipe Z pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x19, 0x5E,		// Pipe Z pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x03, 0x0F, 0x02,		// Pipe U pulls VC1
	0x04, 0x80, 0x03, 0x10, 0x00,
	0x04, 0x80, 0x03, 0x1A, 0x6C,		// Pipe U pulls RAW12(DT 0x2C)
	0x04, 0x80, 0x03, 0x1B, 0x5E,		// Pipe U pulls YUV422(DT 0x1E)

	0x04, 0x80, 0x00, 0x53, 0x10,		// Pipe X -> LinkA: ID0.
	0x04, 0x80, 0x00, 0x57, 0x11,		// Pipe Y -> LinkA: ID1.
	0x04, 0x80, 0x00, 0x5B, 0x21,		// Pipe Z -> LinkB: ID1.
	0x04, 0x80, 0x00, 0x5F, 0x20,		// Pipe U -> LinkB: ID0.
};

uint8_t max9296_max9295e_link_setting[] = {
	0x04, 0x80, 0x00, 0x10, 0x00,		// Link Mode: Dual.
	0x04, 0x90, 0x00, 0x10, 0x20,		// Link Mode: Dual & reset link.
	0x00, 0xFF,
};

uint8_t max9296_dummy_YZ2VC0_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	// pipi Y default to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x8E, 0x2C,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x90, 0x1E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x92, 0x00,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x94, 0x01,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.
};

uint8_t max9296_dummy_Y2VC0_Z2VC1_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	// pipi Y default to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x8E, 0x6C,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x90, 0x5E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x92, 0x40,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x94, 0x41,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.
};

uint8_t max9296_dummy_YZ2VC0_XU2VC1_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	// pipi Y default to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x8E, 0x2C,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x90, 0x1E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x92, 0x00,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x94, 0x01,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.

	0x04, 0x90, 0x04, 0x0B, 0x0F,		// Enable 4 mappings for Pipe X
	0x04, 0x90, 0x04, 0x0D, 0x6C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x0E, 0x6C,
	0x04, 0x90, 0x04, 0x0F, 0x5E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x10, 0x5E,
	0x04, 0x90, 0x04, 0x11, 0x40,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x12, 0x40,
	0x04, 0x90, 0x04, 0x13, 0x41,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x14, 0x41,
	0x04, 0x90, 0x04, 0x2D, 0x55,		// Pipe X to dphy1.

	0x04, 0x90, 0x04, 0xCB, 0x0F,		// Enable 4 mappings for Pipe U
	0x04, 0x90, 0x04, 0xCD, 0x6C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0xCE, 0x6C,
	0x04, 0x90, 0x04, 0xCF, 0x5E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0xD0, 0x5E,
	0x04, 0x90, 0x04, 0xD1, 0x40,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0xD2, 0x40,
	0x04, 0x90, 0x04, 0xD3, 0x41,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0xD4, 0x41,
	0x04, 0x90, 0x04, 0xED, 0x55,		// Pipe U to dphy1.
};

uint8_t max9296_dummy_XYZU2VC0123_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x0B, 0x0F,		// Enable 4 mappings for Pipe X
	0x04, 0x90, 0x04, 0x0D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x0E, 0x2C,
	0x04, 0x90, 0x04, 0x0F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x10, 0x1E,
	0x04, 0x90, 0x04, 0x11, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x12, 0x00,
	0x04, 0x90, 0x04, 0x13, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x14, 0x01,
	0x04, 0x90, 0x04, 0x2D, 0x55,		// Pipe X to dphy1.

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe Y
	0x04, 0x90, 0x04, 0x4D, 0x6C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x4E, 0x6C,
	0x04, 0x90, 0x04, 0x4F, 0x5E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x50, 0x5E,
	0x04, 0x90, 0x04, 0x51, 0x40,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x52, 0x40,
	0x04, 0x90, 0x04, 0x53, 0x41,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x54, 0x41,
	0x04, 0x90, 0x04, 0x6D, 0x55,		// Pipe Y to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0xAC,		// Map RAW12, VC2
	0x04, 0x90, 0x04, 0x8E, 0xAC,
	0x04, 0x90, 0x04, 0x8F, 0x9E,		// Map YUV422, VC2
	0x04, 0x90, 0x04, 0x90, 0x9E,
	0x04, 0x90, 0x04, 0x91, 0x80,		// Map frame start, VC2
	0x04, 0x90, 0x04, 0x92, 0x80,
	0x04, 0x90, 0x04, 0x93, 0x81,		// Map frame end, VC2
	0x04, 0x90, 0x04, 0x94, 0x81,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.

	0x04, 0x90, 0x04, 0xCB, 0x0F,		// Enable 4 mappings for Pipe U
	0x04, 0x90, 0x04, 0xCD, 0xEC,		// Map RAW12, VC3
	0x04, 0x90, 0x04, 0xCE, 0xEC,
	0x04, 0x90, 0x04, 0xCF, 0xDE,		// Map YUV422, VC3
	0x04, 0x90, 0x04, 0xD0, 0xDE,
	0x04, 0x90, 0x04, 0xD1, 0xC0,		// Map frame start, VC3
	0x04, 0x90, 0x04, 0xD2, 0xC0,
	0x04, 0x90, 0x04, 0xD3, 0xC1,		// Map frame end, VC3
	0x04, 0x90, 0x04, 0xD4, 0xC1,
	0x04, 0x90, 0x04, 0xED, 0x55,		// Pipe U to dphy1.
};

uint8_t max9296_dummy_XYxZU2VC0123_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x03, 0x20, 0x2F,		// 1500Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x2F,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x0B, 0x0F,		// Enable 4 mappings for Pipe X
	0x04, 0x90, 0x04, 0x0D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x0E, 0x2C,
	0x04, 0x90, 0x04, 0x0F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x10, 0x1E,
	0x04, 0x90, 0x04, 0x11, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x12, 0x00,
	0x04, 0x90, 0x04, 0x13, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x14, 0x01,
	0x04, 0x90, 0x04, 0x2D, 0x55,		// Pipe X to dphy1.

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe Y
	0x04, 0x90, 0x04, 0x4D, 0x6C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x4E, 0x6C,
	0x04, 0x90, 0x04, 0x4F, 0x5E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x50, 0x5E,
	0x04, 0x90, 0x04, 0x51, 0x40,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x52, 0x40,
	0x04, 0x90, 0x04, 0x53, 0x41,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x54, 0x41,
	0x04, 0x90, 0x04, 0x6D, 0x55,		// Pipe Y to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC2
	0x04, 0x90, 0x04, 0x8E, 0xAC,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC2
	0x04, 0x90, 0x04, 0x90, 0x9E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC2
	0x04, 0x90, 0x04, 0x92, 0x80,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC2
	0x04, 0x90, 0x04, 0x94, 0x81,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.

	0x04, 0x90, 0x04, 0xCB, 0x0F,		// Enable 4 mappings for Pipe U
	0x04, 0x90, 0x04, 0xCD, 0x6C,		// Map RAW12, VC3
	0x04, 0x90, 0x04, 0xCE, 0xEC,
	0x04, 0x90, 0x04, 0xCF, 0x5E,		// Map YUV422, VC3
	0x04, 0x90, 0x04, 0xD0, 0xDE,
	0x04, 0x90, 0x04, 0xD1, 0x40,		// Map frame start, VC3
	0x04, 0x90, 0x04, 0xD2, 0xC0,
	0x04, 0x90, 0x04, 0xD3, 0x41,		// Map frame end, VC3
	0x04, 0x90, 0x04, 0xD4, 0xC1,
	0x04, 0x90, 0x04, 0xED, 0x55,		// Pipe U to dphy1.
};

uint8_t max9296_dummy_Y2VC0123_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x00, 0x50, 0x01,		// PipeX <- Y.
	0x04, 0x90, 0x00, 0x51, 0x01,		// PipeY <- Y.
	0x04, 0x90, 0x00, 0x52, 0x01,		// PipeZ <- Y.
	0x04, 0x90, 0x00, 0x53, 0x01,		// PipeU <- Y.

	0x04, 0x90, 0x03, 0x20, 0x34,		// 2000Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x34,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x0B, 0x0F,		// Enable 4 mappings for Pipe X
	0x04, 0x90, 0x04, 0x0D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x0E, 0x2C,
	0x04, 0x90, 0x04, 0x0F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x10, 0x1E,
	0x04, 0x90, 0x04, 0x11, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x12, 0x00,
	0x04, 0x90, 0x04, 0x13, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x14, 0x01,
	0x04, 0x90, 0x04, 0x2D, 0x55,		// Pipe X to dphy1.

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe Y
	0x04, 0x90, 0x04, 0x4D, 0x2C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x4E, 0x6C,
	0x04, 0x90, 0x04, 0x4F, 0x1E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x50, 0x5E,
	0x04, 0x90, 0x04, 0x51, 0x00,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x52, 0x40,
	0x04, 0x90, 0x04, 0x53, 0x01,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x54, 0x41,
	0x04, 0x90, 0x04, 0x6D, 0x55,		// Pipe Y to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC2
	0x04, 0x90, 0x04, 0x8E, 0xAC,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC2
	0x04, 0x90, 0x04, 0x90, 0x9E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC2
	0x04, 0x90, 0x04, 0x92, 0x80,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC2
	0x04, 0x90, 0x04, 0x94, 0x81,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.

	0x04, 0x90, 0x04, 0xCB, 0x0F,		// Enable 4 mappings for Pipe U
	0x04, 0x90, 0x04, 0xCD, 0x2C,		// Map RAW12, VC3
	0x04, 0x90, 0x04, 0xCE, 0xEC,
	0x04, 0x90, 0x04, 0xCF, 0x1E,		// Map YUV422, VC3
	0x04, 0x90, 0x04, 0xD0, 0xDE,
	0x04, 0x90, 0x04, 0xD1, 0x00,		// Map frame start, VC3
	0x04, 0x90, 0x04, 0xD2, 0xC0,
	0x04, 0x90, 0x04, 0xD3, 0x01,		// Map frame end, VC3
	0x04, 0x90, 0x04, 0xD4, 0xC1,
	0x04, 0x90, 0x04, 0xED, 0x55,		// Pipe U to dphy1.
};

uint8_t max9296_dummy_Z2VC0123_init_setting[] = {
	// MAX9296A Setup
	0x04, 0x90, 0x14, 0x49, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x04, 0x90, 0x15, 0x49, 0x75,

	0x04, 0x90, 0x03, 0x13, 0x60,		// mipi csi out disable
	0x04, 0x90, 0x03, 0x30, 0x04,		// 2x4 mode
	0x04, 0x90, 0x04, 0x4A, 0xD0,		// 4 lanes on port A

	0x04, 0x90, 0x00, 0x50, 0x02,		// PipeX <- Z.
	0x04, 0x90, 0x00, 0x51, 0x02,		// PipeY <- Z.
	0x04, 0x90, 0x00, 0x52, 0x02,		// PipeZ <- Z.
	0x04, 0x90, 0x00, 0x53, 0x02,		// PipeU <- Z.

	0x04, 0x90, 0x03, 0x20, 0x34,		// 2000Mbps/lane on port A
	0x04, 0x90, 0x03, 0x23, 0x34,
	0x04, 0x90, 0x03, 0x25, 0x80,

	0x04, 0x90, 0x04, 0x0B, 0x0F,		// Enable 4 mappings for Pipe X
	0x04, 0x90, 0x04, 0x0D, 0x2C,		// Map RAW12, VC0
	0x04, 0x90, 0x04, 0x0E, 0x2C,
	0x04, 0x90, 0x04, 0x0F, 0x1E,		// Map YUV422, VC0
	0x04, 0x90, 0x04, 0x10, 0x1E,
	0x04, 0x90, 0x04, 0x11, 0x00,		// Map frame start, VC0
	0x04, 0x90, 0x04, 0x12, 0x00,
	0x04, 0x90, 0x04, 0x13, 0x01,		// Map frame end, VC0
	0x04, 0x90, 0x04, 0x14, 0x01,
	0x04, 0x90, 0x04, 0x2D, 0x55,		// Pipe X to dphy1.

	0x04, 0x90, 0x04, 0x4B, 0x0F,		// Enable 4 mappings for Pipe Y
	0x04, 0x90, 0x04, 0x4D, 0x2C,		// Map RAW12, VC1
	0x04, 0x90, 0x04, 0x4E, 0x6C,
	0x04, 0x90, 0x04, 0x4F, 0x1E,		// Map YUV422, VC1
	0x04, 0x90, 0x04, 0x50, 0x5E,
	0x04, 0x90, 0x04, 0x51, 0x00,		// Map frame start, VC1
	0x04, 0x90, 0x04, 0x52, 0x40,
	0x04, 0x90, 0x04, 0x53, 0x01,		// Map frame end, VC1
	0x04, 0x90, 0x04, 0x54, 0x41,
	0x04, 0x90, 0x04, 0x6D, 0x55,		// Pipe Y to dphy1.

	0x04, 0x90, 0x04, 0x8B, 0x0F,		// Enable 4 mappings for Pipe Z
	0x04, 0x90, 0x04, 0x8D, 0x2C,		// Map RAW12, VC2
	0x04, 0x90, 0x04, 0x8E, 0xAC,
	0x04, 0x90, 0x04, 0x8F, 0x1E,		// Map YUV422, VC2
	0x04, 0x90, 0x04, 0x90, 0x9E,
	0x04, 0x90, 0x04, 0x91, 0x00,		// Map frame start, VC2
	0x04, 0x90, 0x04, 0x92, 0x80,
	0x04, 0x90, 0x04, 0x93, 0x01,		// Map frame end, VC2
	0x04, 0x90, 0x04, 0x94, 0x81,
	0x04, 0x90, 0x04, 0xAD, 0x55,		// Pipe Z to dphy1.

	0x04, 0x90, 0x04, 0xCB, 0x0F,		// Enable 4 mappings for Pipe U
	0x04, 0x90, 0x04, 0xCD, 0x2C,		// Map RAW12, VC3
	0x04, 0x90, 0x04, 0xCE, 0xEC,
	0x04, 0x90, 0x04, 0xCF, 0x1E,		// Map YUV422, VC3
	0x04, 0x90, 0x04, 0xD0, 0xDE,
	0x04, 0x90, 0x04, 0xD1, 0x00,		// Map frame start, VC3
	0x04, 0x90, 0x04, 0xD2, 0xC0,
	0x04, 0x90, 0x04, 0xD3, 0x01,		// Map frame end, VC3
	0x04, 0x90, 0x04, 0xD4, 0xC1,
	0x04, 0x90, 0x04, 0xED, 0x55,		// Pipe U to dphy1.
};

uint8_t max9296_dummy_start_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x62,  			// MIPI output enable.
};

uint8_t max9296_dummy_stop_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x60,  			// MIPI output disable.
};

uint8_t max9295_disable_heartbeat[] = {
	0x04, 0x80, 0x01, 0x02, 0x0e,		// disable HEARTBEAT MODE PIPEX Y Z U
	0x04, 0x80, 0x01, 0x0A, 0x0e,
	0x04, 0x80, 0x01, 0x12, 0x0e,
	0x04, 0x80, 0x01, 0x1A, 0x0e,
};

uint8_t max9296_disable_heartbeat[] = {
	0x04, 0x90, 0x01, 0x00, 0x23,		// disable HEARTBEAT MODE PIPEX Y Z U
	0x04, 0x90, 0x01, 0x12, 0x23,
	0x04, 0x90, 0x01, 0x24, 0x23,
	0x04, 0x90, 0x01, 0x36, 0x23,
};

#ifdef __cplusplus
}
#endif
#endif
