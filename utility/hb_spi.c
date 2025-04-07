/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[hb_spi]:[%s][%d]" fmt, __func__, __LINE__

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "inc/cam_common.h"
#include "hb_spi.h"
#include "vin_log.h"


int32_t hb_vin_spi_read_block(int32_t fd, const int8_t *buf, int32_t count) /* PRQA S 3219 */
{
    int32_t ret;

	struct spi_ioc_transfer rx = {
			.rx_buf = (uint64_t)buf,    /* PRQA S 0306 */
			.tx_buf = 0,
			.len = (uint32_t)count,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &rx); /* PRQA S ALL */
	if (ret < 1) {
		vin_err("can't read spi message\n"); /* PRQA S ALL */
		return -RET_ERROR;
	}
	return ret;
}

int32_t hb_vin_spi_write_block(int32_t fd, const int8_t *buf, int32_t count) /* PRQA S 3219 */
{
	int32_t ret;

	struct spi_ioc_transfer tx = {
			.tx_buf = (uint64_t)buf,    /* PRQA S 0306 */
			.rx_buf = 0,
			.len = (uint32_t)count,
	};
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tx); /* PRQA S ALL */
	if (ret < 1) {
		vin_err("can't write spi message\n"); /* PRQA S ALL */
		return -RET_ERROR;
	}

	return ret;
}
