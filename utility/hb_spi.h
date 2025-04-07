/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_HB_SPI_H_
#define UTILITY_HB_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

extern int32_t hb_vin_spi_write_block(int32_t fd, const int8_t *buf, int32_t count);
extern int32_t hb_vin_spi_read_block(int32_t fd, const int8_t *buf, int32_t count);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_SPI_H_
