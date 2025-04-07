/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef HBRE_CAMERA_UTILITY_SENSOR_INC_RGB_SETTING_H_
#define HBRE_CAMERA_UTILITY_SENSOR_INC_RGB_SETTING_H_



uint8_t RGB_stream_on_setting[] = {
	0x04,0xd6,0x08,0xa0,0x84,       //force all mipi output
	0x04,0xd6,0x04,0x0b,0x42,       //force all mipi output
};

uint8_t RGB_stream_off_setting[] = {
	0x04,0xd6,0x04,0x0b,0x00,	// enable mipi output           
};
#endif  // HBRE_CAMERA_UTILITY_SENSOR_INC_OV2311_IMS290_SETTING_H_
