
#ifndef __HB_OV13855_SETTING_H__
#define __HB_OV13855_SETTING_H__

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t ds914_init_setting[] = {
        0x01, 0x04 , //delay 500 us
        0x03, 0xed , //delay 10us

        0x6 , 0xb0 , // write
        0x7 , 0xe2 , // write
        0x8 , 0x60 , // write
        0x10, 0x6a , // write
        0x1c, 0x3   // write
};

#ifdef __cplusplus
}
#endif
#endif