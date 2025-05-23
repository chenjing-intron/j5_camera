/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef UTILITY_SENSOR_INC_OV9284_SETTING_H_
#define UTILITY_SENSOR_INC_OV9284_SETTING_H_

uint8_t ov9284_power_on_setting[] = {
	0x04, 0x50, 0x00, 0x01, 0x1f,	/* enable output */
};

uint8_t ov9284_power_off_setting[] = {
	0x04, 0x50, 0x00, 0x01, 0x10,	/* disable output */
};

uint8_t ov9284_init_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf1,	/* reset 9296 */
	0x00, 0x5F,	/* delay */

	0x04, 0x90, 0x00, 0x01, 0x01,	/* change to 3Gbps */
	0x04, 0x90, 0x00, 0x10, 0x21,	/* link A */
	0x04, 0x90, 0x03, 0x13, 0x00,	/* disable MAX9296A MIPI output */
	0x00, 0x5F,
	0x00, 0x5F,	/* delay */

	/* DVP configuration for ov9284 */
	0x04, 0x80, 0x00, 0x6B, 0x16,
	0x04, 0x80, 0x00, 0x73, 0x17,
	0x04, 0x80, 0x00, 0x7B, 0x36,
	0x04, 0x80, 0x00, 0x83, 0x36,
	0x04, 0x80, 0x00, 0x93, 0x36,
	0x04, 0x80, 0x00, 0x9B, 0x36,
	0x04, 0x80, 0x00, 0xA3, 0x36,
	0x04, 0x80, 0x00, 0xAB, 0x36,
	0x04, 0x80, 0x00, 0x8B, 0x36,

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

uint8_t ov9284_stream_on_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x02,	/* enable CSI output */
};

uint8_t ov9284_stream_off_setting[] = {
	0x04, 0x90, 0x03, 0x13, 0x00,	/* enable mipi output */         
};

#endif  // UTILITY_SENSOR_INC_OV9284_SETTING_H_
