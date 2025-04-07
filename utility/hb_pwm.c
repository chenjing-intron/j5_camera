/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt)		"[hb_pwm]:" fmt

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <sys/ioctl.h>

#include "../utility/hb_pwm.h"

#define BSIZE_ATTR   64u
#define BSIZE_PATH   128
#define MAX_PWM_NUM  8
#define LPWM_NUM     4
#define BUFF_SIZE  	32
#define LPWM_TIME_CALC_UNIT_1000  1000
#define LPWM_0 		0
#define LPWM_1 		1
#define LPWM_2 		2
#define LPWM_3 		3
#define LPWM_DEVICE_NODE_LENGTH 10
#define LPWM_SW_TRIGGER_MODE 8

#define LPWM_PATH "/dev/hobot-lpwm"

struct lpwm_cdev_state {
	uint32_t lpwm_num;
	uint32_t period;
	uint32_t duty_cycle;
	bool enabled;
};

#define LPWM_CDEV_MAGIC 'L'
#define LPWM_CDEV_INIT      _IOW(LPWM_CDEV_MAGIC, 0x12, unsigned int)
#define LPWM_CDEV_DEINIT    _IOW(LPWM_CDEV_MAGIC, 0x13, unsigned int)
#define LPWM_CDEV_CONF_SINGLE    _IOW(LPWM_CDEV_MAGIC, 0x14, struct lpwm_cdev_state)
#define LPWM_CDEV_ENABLE_SINGLE  _IOW(LPWM_CDEV_MAGIC, 0x15, unsigned int)
#define LPWM_CDEV_DISABLE_SINGLE _IOW(LPWM_CDEV_MAGIC, 0x16, unsigned int)
#define LPWM_CDEV_GET_STATE      _IOWR(LPWM_CDEV_MAGIC, 0x19, struct lpwm_cdev_state)
#define LPWM_CDEV_OFFSET_CONF    _IOW(LPWM_CDEV_MAGIC, 0x20, int)
#define LPWM_CDEV_OFFSET_STATE   _IOR(LPWM_CDEV_MAGIC, 0x21, int)
#define LPWM_CDEV_TRIG_ALL_ENABLE     _IO(LPWM_CDEV_MAGIC, 0x17)
#define LPWM_CDEV_TRIG_ALL_DISABLE    _IO(LPWM_CDEV_MAGIC, 0x18)
#define LPWM_CDEV_TRIG_SET_SOURCE    _IOW(LPWM_CDEV_MAGIC, 0x22, int)
#define LPWM_CDEV_TRIG_GET_SOURCE    _IOR(LPWM_CDEV_MAGIC, 0x23, int)

/*
 * export all usable pwm in sysfs
 * @pwmchip_path can be null or any path, e.g. /dev/lpwm_cdev
 */
static int32_t hb_pwm_init(lpwm_info_t *lpwm_info)
{
	uint32_t i;
	int32_t ret;
	char name[32];

	if (lpwm_info->fd > 0)
		return 0;

	snprintf(name, sizeof(name), "/dev/hobot-lpwm%d", lpwm_info->lpwm_index);
	lpwm_info->fd = open(name, O_RDWR | O_CLOEXEC);
	if (lpwm_info->fd < 0) {
		vin_err("Failed to open path %s\n", name);
		return -HB_VIN_LPWM_NODE_OPEN_FAIL;
	}

	for (i = 0; i < LPWM_NUM; i++) {
		ret = ioctl(lpwm_info->fd, LPWM_CDEV_INIT, &i);	/* PRQA S 4513 */
		if (ret < 0) {
			close(lpwm_info->fd);
			lpwm_info->fd = 0;
			vin_err("Failed to get lpwm%u \n", i);
			return -HB_VIN_LPWM_INIT_FAIL;
		}
	}
	vin_info("%s: open %s sucessfully\n", __func__, name);

	return ret;
}

/*
 * disable and unexport all pwms
 */
int32_t hb_vin_lpwm_deinit(lpwm_info_t *lpwm_info)
{
	uint32_t i;
	int32_t ret = 0;

	if (lpwm_info->fd <= 0)
		return -HB_VIN_LPWM_NODE_OPEN_FAIL;

	for (i = 0; i < LPWM_NUM; i++) {
		ret = ioctl(lpwm_info->fd, LPWM_CDEV_DEINIT, &i);	/* PRQA S 4513 */
		if (ret < 0) {
			vin_err("Failed to free lpwm%u \n", i);
		}
	}
	close(lpwm_info->fd);
	lpwm_info->fd = 0;
	vin_info("%s: done\n", __func__);

	return ret;
}

/*
 * config a pwm
 * @pwm: index of the pwm
 * @period_us, 10^9/period_ns is the frequency of PWM, range: [2us 1000000us]
 * @duty_us, duty_cycle of pwm, aka high level time of in a pwm period, range [1us 4000us]
 */
static int32_t hb_pwm_config_single(int32_t fd, uint32_t pwm_ch, int32_t period_us, int32_t duty_us)
{
	int32_t ret;
	struct lpwm_cdev_state cdev_state;

	if (pwm_ch >= LPWM_NUM) {
		vin_err("lpwm %d is out of range\n", pwm_ch);
		return -HB_VIN_LPWM_PARAM_FAIL;
	}

	if (period_us < 2 || (period_us > 1000000)) {
		vin_err("lpwm only support period in [2us 1000000us]\n");
		return -HB_VIN_LPWM_PARAM_FAIL;
	}

	if (duty_us < 1 || (duty_us > 4000)) {
		vin_err("lpwm only support duty_cycle in [1us~4000us]\n");
		return -HB_VIN_LPWM_PARAM_FAIL;
	}

	cdev_state.lpwm_num = pwm_ch;
	cdev_state.duty_cycle = (uint32_t)(duty_us * 1000);
	cdev_state.period = (uint32_t)(period_us * 1000);

	ret = ioctl(fd, LPWM_CDEV_CONF_SINGLE, &cdev_state);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("Failed to config lpwm%u \n", pwm_ch);
		return -HB_VIN_LPWM_CONFIG_FAIL;
	}

	return ret;
}

static int32_t hb_lpwm_config_offset(int32_t fd, uint32_t *lpwm_offset)
{
	int32_t ret;
	uint32_t i;

	for (i = 0; i < LPWM_NUM; i++) {
		if (lpwm_offset[i] >= 1000000) {
			vin_err("%s: wrong lpwm offset = %d\n", __func__, lpwm_offset[i]);
			return -HB_VIN_LPWM_PARAM_FAIL;
		}
	}
	ret = ioctl(fd, LPWM_CDEV_OFFSET_CONF, lpwm_offset);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("Failed to config lpwm offsets \n");
		return -HB_VIN_LPWM_CONFIG_OFFSET_FAIL;
	}

	return ret;
}

/*
 * @enable, 1 to enable, 0 to disable
 */
static int32_t lpwm_enable_single(int32_t fd, uint32_t pwm, int32_t enable)
{
	int32_t ret;

	if (pwm >= LPWM_NUM) {
		vin_err("lpwm %d is out of range\n", pwm);
		return -HB_VIN_LPWM_PARAM_FAIL;
	}

	if (fd <= 0)
		return -HB_VIN_LPWM_NODE_OPEN_FAIL;

	if (enable)
		ret = ioctl(fd, LPWM_CDEV_ENABLE_SINGLE, &pwm);	/* PRQA S 4513 */
	else
		ret = ioctl(fd, LPWM_CDEV_DISABLE_SINGLE, &pwm);	/* PRQA S 4513 */

	if (ret < 0) {
		vin_err("Failed to enable/disable lpwm%u \n", pwm);
		return -HB_VIN_LPWM_ENABLE_SINGLE_FAIL;
	}

	return ret;
}

static int32_t lpwm_enable_all(int32_t fd)
{
	int32_t ret;

	if (fd <= 0) {
		vin_err("%s please check fd first\n", __func__);
		return -HB_VIN_LPWM_PARAM_FAIL;
	}
	ret = ioctl(fd, LPWM_CDEV_TRIG_ALL_ENABLE);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("%s error, ret = %d\n", __func__, ret);
		ret = -HB_VIN_LPWM_ENABLE_ALL_FAIL;
	}

	return ret;
}

static int32_t lpwm_disable_all(int32_t fd)
{
	int32_t ret;

	if (fd <= 0) {
		vin_err("%s please check fd first\n", __func__);
		return -HB_VIN_LPWM_NODE_OPEN_FAIL;
	}
	ret = ioctl(fd, LPWM_CDEV_TRIG_ALL_DISABLE);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("%s error, ret = %d\n", __func__, ret);
		ret = -HB_VIN_LPWM_DISABLE_ALL_FAIL;
	}

	return ret;
}

static int32_t hb_pwm_enable_single(int32_t fd, uint32_t pwm)
{
	return lpwm_enable_single(fd, pwm, 1);
}

static int32_t hb_pwm_disable_single(int32_t fd, uint32_t pwm)
{
	return lpwm_enable_single(fd, pwm, 0);
}

/*
 * lpwm signal won't run before sw/pps triggered
 */
static int32_t hb_lpwm_set_trigger_mode(int32_t fd, uint32_t mode)
{
	int32_t ret;

	ret = ioctl(fd, LPWM_CDEV_TRIG_SET_SOURCE, &mode);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("%s error, ret = %d\n", __func__, ret);
		ret = -HB_VIN_LPWM_SET_MODE_FAIL;
	}

	return ret;
}

/*
 * hb_vin_lpwm_config configures all 4 lpwms
 * offset_us: should pass a offset[4] array, for offset of each lpwm channel
 * offset_us[i] is in [1us 40960us], step is 10us
 * period_us: period of lpwm signal, same for all lpwm channels
 * period_us is in [10us 40960us], step is 10us, e.g. 30fps is 33330
 * duty_us: high level time in each pwm period, same for all lpwm channels
 * duty_us is in [10us 160us], step is 10us
 */
int32_t hb_vin_lpwm_config(lpwm_info_t *lpwm_info)
{
	uint32_t i;
	int32_t ret;

	ret = hb_pwm_init(lpwm_info);
	if (ret < 0)
		return ret;

	for(i = 0; i < LPWM_NUM; i++) {
		ret = hb_pwm_config_single(lpwm_info->fd, i, lpwm_info->period_us[i], lpwm_info->duty_us[i]);
		if (ret < 0)
			goto config_err;
	}
	ret = hb_lpwm_config_offset(lpwm_info->fd, (uint32_t *)(lpwm_info->offset_us));
	if (ret < 0)
		goto config_err;

	if (lpwm_info->trigger_mode == 8)
		lpwm_info->trigger_mode = 4;
	ret = hb_lpwm_set_trigger_mode(lpwm_info->fd, (uint32_t)lpwm_info->trigger_mode);
	if (ret < 0)
		goto config_err;

	return ret;

config_err:
	hb_vin_lpwm_deinit(lpwm_info);
	return ret;
}


int32_t hb_vin_lpwm_start(const lpwm_info_t *lpwm_info)
{
	int32_t i = 0;
	int32_t ret = 0;

	if (lpwm_info->lpwm_enable == ((1 << LPWM_NUM) - 1)) {
		ret = lpwm_enable_all(lpwm_info->fd);
	} else {
		for (i = 0; i < LPWM_NUM; i++) {
			if ((uint32_t)(lpwm_info->lpwm_enable) & ((uint32_t)1 << (uint32_t)i)) {
				ret = hb_pwm_enable_single(lpwm_info->fd, (uint32_t)i);
				if (ret < 0) {
					vin_err("Failed to enable lpwm%d\n", i);
					return ret;
				}
			}
		}
	}

	return ret;
}

int32_t hb_vin_lpwm_stop(const lpwm_info_t *lpwm_info)
{
	int32_t i = 0;
	int32_t ret = 0;

	if (lpwm_info->lpwm_enable == ((1 << LPWM_NUM) - 1)) {
		ret = lpwm_disable_all(lpwm_info->fd);
	} else {
		for (i = 0; i < LPWM_NUM; i++) {
			if ((uint32_t)(lpwm_info->lpwm_enable) & ((uint32_t)1 << (uint32_t)i)) {
				ret = hb_pwm_disable_single(lpwm_info->fd, (uint32_t)i);
				if (ret < 0) {
					vin_err("Failed to disable lpwm%d\n", i);
					return ret;
				}
			}
		}
	}
	return ret;
}
