#ifndef UTILITY_SENSOR_INC_DS913_SETTING_H_
#define UTILITY_SENSOR_INC_DS913_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t ds913_ov10635_init_setting[] = {
	0x1, 0x33, // delay 500 us
	0x3, 0xc5, // delay 10us
	0xd, 0x55  // delay 10us
};

#ifdef __cplusplus
}
#endif
#endif // UTILITY_SENSOR_INC_DS913_SETTING_H_
