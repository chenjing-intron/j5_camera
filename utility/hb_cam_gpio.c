/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[vin_gpio]:[%s][%d]" fmt, __func__, __LINE__

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "hb_cam_gpio.h"
#include "inc/cam_common.h"

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF	64
#define WRITE_GPIO_TO_OUT_SIZE	4
#define WRITE_GPIO_TO_IN_SIZE	3
#define WRITE_GPIO_LEVEL_SIZE	2    // Sizeof write the gpio to high/low level

/****************************************************************
 * gpio_export
 ****************************************************************/
int32_t vin_gpio_export(uint32_t gpio)
{
	int32_t ret;
	int32_t fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY); /* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/export\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	ret = write(fd, buf, len);    /* PRQA S ALL */
	if (ret != len) {
		vin_err("write fail! export gpio fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
		(void)close(fd);
		return -1;
	}
	(void)close(fd);

	return RET_OK;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int32_t vin_gpio_unexport(uint32_t gpio)
{
	int32_t ret;
	int32_t fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);/* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/export\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	ret = write(fd, buf, len);    /* PRQA S ALL */
	if (ret != len) {
		vin_err("write fail! unexport gpio fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
		(void)close(fd);
		return -1;
	}
	(void)close(fd);
	return RET_OK;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int32_t vin_gpio_set_dir(uint32_t gpio, uint32_t out_flag)
{
	int32_t ret;
	int32_t fd;
	char buf[MAX_BUF];

	(void)snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);/* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/direction\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	if (out_flag == 1u) {
		ret = write(fd, "out", WRITE_GPIO_TO_OUT_SIZE);    /* PRQA S ALL */
		if (ret != 4) {
			vin_err("write fail! set gpio out fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
			(void)close(fd);
			return -1;
		}
	} else {
		ret = write(fd, "in", WRITE_GPIO_TO_IN_SIZE);    /* PRQA S ALL */
		if (ret != 3) {    /* PRQA S ALL */
			vin_err("write fail! set gpio in fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
			(void)close(fd);
			return -1;
		}
	}

	(void)close(fd);
	return RET_OK;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int32_t vin_gpio_set_value(uint32_t gpio, uint32_t value)
{
	int32_t ret;
	int32_t fd;
	char buf[MAX_BUF];

	(void)snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);/* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/set-value\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	if (value == 1u) {
		ret = write(fd, "1", WRITE_GPIO_LEVEL_SIZE);    /* PRQA S ALL */
		if (ret != 2) {    /* PRQA S ALL */
			vin_err("write gpio fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
			(void)close(fd);
			return -1;
		}
	} else {
		ret = write(fd, "0", WRITE_GPIO_LEVEL_SIZE);    /* PRQA S ALL */
		if (ret != 2) {    /* PRQA S ALL */
			vin_err("write export gpio fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
			(void)close(fd);
			return -1;
		}
	}

	(void)close(fd);
	return RET_OK;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int32_t vin_gpio_get_value(uint32_t gpio, uint32_t *value)    /* PRQA S 1503 */
{
	int32_t ret;
	int32_t fd;
	char buf[MAX_BUF];
	char ch;

	(void)snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);/* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/get-value\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	ret = read(fd, &ch, 1);    /* PRQA S ALL */
	if (ret < 0) {
		vin_err("read gpio status fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
		(void)close(fd);
		return -1;
	}

	if (ch != '0') {    /* PRQA S 3123 */
		*value = 1;
	} else {
		*value = 0;
	}

	(void)close(fd);
	return RET_OK;
}

/****************************************************************
 * gpio_set_edge
 ****************************************************************/
int32_t vin_gpio_set_edge(uint32_t gpio, const char *edge)    /* PRQA S 1503 */
{
	ssize_t ret;
	int32_t fd;
	char buf[MAX_BUF];

	(void)snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);/* PRQA S 0339 */ /* fcntl macro */
	if (fd < 0) {
		vin_err("gpio/set-edge\n");/* PRQA S ALL */ /* print api */
		return fd;
	}

	ret = write(fd, edge, strlen(edge) + 1u);    /* PRQA S 0315 */
	if (ret != strlen(edge) + 1) {    /* PRQA S 1823 */
		vin_err("write gpio edge fail! ret = %d\n", ret);/* PRQA S ALL */ /* print api */
		(void)close(fd);
		return -1;
	}
	(void)close(fd);
	return RET_OK;
}
