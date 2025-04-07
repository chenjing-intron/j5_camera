/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_CAM_UTILITY_H__
#define __HB_CAM_UTILITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "../inc/cam_common.h"
#include "../inc/vin_log.h"
#define GPIO_HIGH	1
#define GPIO_LOW	0
#define REG_WIDTH_16bit 16
#define REG_WIDTH_8bit 8
#define I2C_BUS  0
#define SPI_BUS  1

#define REG16_VAL16 3
#define REG16_VAL8  2
#define REG8_VAL8   1

// pre board id
#define	BOARD_ID_MATRIXDUO_A	(0x631)
#define	BOARD_ID_MATRIXDUO_B	(0x632)
#define	BOARD_ID_MATRIXDSOLO	(0x641)
// a sample board id
#define BOARD_ID_MATRIXDUO_A_V2	(0X651)
#define	BOARD_ID_MATRIXDUO_B_V2	(0X652)
#define	BOARD_ID_MATRIXDSOLO_V2	(0X642)
// b sample board id
#define BOARD_ID_MATRIXDUO_A_V3	(0X653)
#define BOARD_ID_MATRIXDUO_B_V3	(0X654)
#define BOARD_ID_MATRIXSOLO_V3	(0X643)

#define DELAY_FLAG              (0xFFFF)

typedef struct _x2_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x2_camera_i2c_t;

typedef struct _x2_camera_spi_t {
	uint64_t tx_buf;
	uint64_t rx_buf;
	uint32_t reg;
	uint32_t reg_size;
	uint32_t len;
	uint32_t data;
	uint32_t speed_hz;
	uint32_t bits_per_word;
} x2_camera_spi_t;

extern void VIN_DOFFSET(uint32_t *x, uint32_t n);
extern int vin_power_ctrl(unsigned int gpio, int on_off);
extern int vin_i2c_read16(int bus, int reg_width, int sensor_addr, uint32_t reg_addr);
extern int vin_i2c_read8(int bus, int reg_width, int sensor_addr, uint32_t reg_addr);
extern int vin_i2c_write16(int bus, int reg_width, int sensor_addr, uint32_t reg_addr, uint16_t value);
extern int vin_i2c_write8(int bus, int reg_width, int sensor_addr, uint32_t reg_addr, uint8_t value);
extern int vin_i2c_write_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size);
extern int vin_i2c_read_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size);
extern int32_t vin_i2c_bit_write8(uint32_t bus, uint32_t i2c_addr, uint32_t reg_addr,
				  uint32_t reg_width, int32_t bit_mask, uint32_t value);
extern int32_t vin_i2c_bit_array_write8(uint32_t bus, uint32_t i2c_addr,  uint32_t reg_width,
					int setting_size, uint32_t *cam_setting);
extern int vin_write_array(int bus, uint32_t i2c_addr, int reg_width, int setting_size, uint32_t *cam_setting);
extern int vin_cam_control_isp(int *gpio_pin, int *gpio_level, int enable);
extern int hb_vin_deserial_init(deserial_info_t *deserial_info);
extern int hb_vin_deserial_deinit(deserial_info_t *deserial_info);
extern int hb_vin_cam_utility(int cam_ctl, sensor_info_t *sensor_info);
extern int hb_vin_deserial_start_physical(deserial_info_t *deserial_info);
extern int32_t hb_vin_deserial_start(deserial_info_t *deserial_info);
extern int hb_vin_cam_ae_share_init(uint32_t port, uint32_t flag);
extern int vin_spi_write_block(int bus, char *buffer, int size);
extern int vin_spi_read_block(int bus, char *buffer, int size);
extern int vin_get_board_id(void);
extern int32_t hb_cam_send_event(uint32_t port, cam_event_t *event_info);

//extern int camera_i2c_write_block(sensor_info_t *sensor_info, int subdev, char *buffer, int size)

#ifdef __cplusplus
}
#endif
#endif
